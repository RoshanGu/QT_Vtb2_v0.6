#include <map>
#include <cassert>
#include <cstring>

#include <lemon/smart_graph.h>
#include <lemon/time_measure.h>
#include <lemon/matching.h>

using namespace lemon;

// The granularity (starting from bit0) for write data masking
// (so that writing trace data does not affect trigger lookup)
#define RAM_DOUT_MASK 	1
// The data width of each RAM
#define RAM_DATA_WIDTH 4	
// The address width of each RAM
#define RAM_ADDR_WIDTH 	14

typedef SmartBpGraph BpGraph;

int main (int argc, char * const argv[]) 
{
	BpGraph bpg;
	BpGraph::EdgeMap<int> weight(bpg);
	BpGraph::RedNodeMap<int> rnode2ipin(bpg);

	assert(argc == 6);
	const char *overlay_file = argv[1];
	const int trigger_width = atoi(argv[2]);
	const int total_vnets = atoi(argv[3]);
	const float trace_fraction = atof(argv[4]);
	const int seed = atoi(argv[5]);

	FILE *fp = fopen(overlay_file, "r");
	char line[32768];
	// LEMON bipartite graphs nominally call the two set of nodes
	// red and blue
	typedef BpGraph::RedNode RedNode;
	typedef BpGraph::BlueNode BlueNode;
	typedef std::map<int,BlueNode> int2bnode;
	int2bnode vnet2bnode;
	int ipin = 0;

	// Read the overlay file, line by line
	while (fgets(line, 32768, fp)) {
		// Extracting each comma-separated token
		char *pch = strtok(line, ",");
		// If the line is non empty, means that is one or more nets that connect to it
		if (pch) {
			RedNode in;
			in = bpg.addRedNode();
			rnode2ipin.set(in, ipin);
			// Line contains vnet numbers of all nets that reach this ipin
			while (pch && *pch != '\n') {
				const int vnet = atoi(pch); 
				// Create a node in graph only if one hasn't been created before
				BlueNode vn;
				if (vnet2bnode.count(vnet) == 0)
				{
					vn = bpg.addBlueNode();
					vnet2bnode[vnet] = vn;
				}
				else
				{
					vn = vnet2bnode[vnet];
				}
				bpg.addEdge(in, vn);
				pch = strtok(NULL, ",");
			}
		}
		++ipin;
	}
	fclose(fp);

	const int total_ipins = ipin;
	const int num_vnets = vnet2bnode.size();
	assert(num_vnets == bpg.blueNum());
	const int num_ipins = bpg.redNum();
	assert(num_ipins <= total_ipins);

	// Build a vector of all trace-able/reachable vnets
	// (i.e. those for which a blue node exists)
	std::vector<int> vnets;
	vnets.reserve(total_vnets);
	for (int2bnode::const_iterator it = vnet2bnode.begin(); it != vnet2bnode.end(); ++it)
	{
		vnets.push_back(it->first);
	}
	// The difference of reachable and total are nets that are not
	// traceable/reachable, so assign them a "-1" vnet number
	std::sort(vnets.begin(), vnets.end());
	for (int i = vnets.size(); i < total_vnets; ++i)
	{
		vnets.push_back(-1);
	}

	std::vector<int> vnets_pop;
	vnets_pop.reserve(total_vnets);
	std::vector< std::map<int,int> > iram2ipins(total_ipins/RAM_DATA_WIDTH);
	std::vector<BpGraph::IncEdgeIt> triggerIt(trigger_width);
	
	std::cout << " \033[1;31m ***** Maximum Weighted Bipartite Matching ***** \033[0m" << std::endl;
	std::cerr << "# Total RAMs: " << iram2ipins.size() << std::endl;
	const int trace_capacity = total_ipins*(RAM_DATA_WIDTH-RAM_DOUT_MASK)/RAM_DATA_WIDTH;
	const int trace_width = trace_capacity*trace_fraction;
	std::cerr << "# Used/Total IPINs: " << num_ipins << "/" << total_ipins << std::endl;
	std::cerr << "# TraceWidth/Trace capacity: " << trace_width << "/" << trace_capacity << std::endl;
	std::cerr << "# TriggerMatched,TraceMatched,TraceInvalid,Runtime" <<  std::endl;
	
	sprintf(line, "%s.match", overlay_file);
	std::ofstream fs(line);

	Timer t(false);

	vnets_pop.assign(vnets.begin(), vnets.end());
	int invalidTrace = 0;

	// NB: srand(0) == srand(1)
	srand(seed+1);

	// Randomly pick a set of valid trigger signals
	for (int i = 0; i < trigger_width; ++i)
	{
		const int j = i + (rand() % (num_vnets-i));
		const int vnet = vnets_pop[j];
		assert(vnet >= 0);
		const BlueNode vn = vnet2bnode[vnet];
		triggerIt[i] = BpGraph::IncEdgeIt(bpg,vn);
		std::swap(vnets_pop[i],vnets_pop[j]);
	}

	// Now go through each trigger signal in turn, and allow it to claim a 
	// potential ipin on a trace-buffer, if ipins remain
	// NB: This is because of the restriction that each trace-buffer can
	// only support RAM_ADDR_WIDTH trigger signals, whereas it can support
	// (RAM_DATA_WIDTH-RAM_DOUT_MASK) trace signals
	bool lfp = false;
	while(!lfp)
	{
		lfp = true;
		for (int i = 0; i < trigger_width; ++i)
		{
			for (BpGraph::IncEdgeIt &it = triggerIt[i]; it != INVALID; ++it)
			{
				const RedNode rn = bpg.asRedNode(bpg.runningNode(it));
				const int ipin = rnode2ipin[rn];
				const int iram = ipin / RAM_DATA_WIDTH;
				std::map<int,int> &ipins = iram2ipins[iram];
				const int ipin_mod = (ipin % RAM_DATA_WIDTH) % RAM_ADDR_WIDTH;
				std::map<int,int>::const_iterator jt = ipins.find(ipin_mod);
				assert(weight[it] == 0);
				// If no ipins have been claimed
				if (jt != ipins.end())
				{
					if (jt->second == ipin) {
						// Give it a weight more than all trace signals
						// NB: The net effect is that trigger signals are always
						// given priority over trace signals
						weight[it] = total_ipins+1;
						lfp = false;
					}
				}
				else 
				{
					// If not all ipins have been claimed
					if (ipins.size() < RAM_ADDR_WIDTH)
					{
						// Give it a weight more than all trace signals
						// NB: The net effect is that trigger signals are always
						// given priority over trace signals
						weight[it] = total_ipins+1;
						ipins[ipin_mod] = ipin;
						lfp = false;
						++it;
						break;
					}
				}
			}
		}
	}

	// Choose some trace signals (ignoring those signals selected for triggering, 
	// but allowing the picking of potentially untraceable nets)
	for (int i = trigger_width; i < trigger_width+trace_width; ++i)
	{
		const int j = i + (rand() % (total_vnets-i));
		const int vnet = vnets_pop[j];
		std::swap(vnets_pop[i],vnets_pop[j]);
		// vnet is untraceable 
		if (vnet == -1)
		{
			invalidTrace++;
			continue;
		}
		const BlueNode vn = vnet2bnode[vnet];
		for (BpGraph::IncEdgeIt it(bpg, vn); it != INVALID; ++it)
		{
			const RedNode in = bpg.asRedNode(bpg.runningNode(it));
			const int ipin = rnode2ipin[in];
			if ((ipin % RAM_DATA_WIDTH) >= RAM_DOUT_MASK)
			{
				// Give it a weight of just 1
				weight[it] = 1;
			}
		}
	}

	t.restart();
	// Use lemon bipartite matching algorithms
	MaxWeightedMatching<BpGraph> bm(bpg, weight);
	bm.run();
	t.stop();

	const int matchedW = bm.matchingWeight();
	const int numTrig = matchedW / (total_ipins+1);
	const int numTrace = matchedW % (total_ipins+1);

	const MaxWeightedMatching<BpGraph>::MatchingMap &mm = bm.matchingMap();

	// Clear all edge weights 
	int matched = 0;
	for (int i = 0; i < trigger_width+trace_width; ++i)
	{
		const int vnet = vnets_pop[i];
		// Untraceable vnet
		if (vnet == -1)
			continue;
		const BlueNode vn = vnet2bnode[vnet];

		//for (BpGraph::IncEdgeIt it(bpg, vn); it != INVALID; ++it)
		//{
		//	weight[it] = 0;
		//}

		const BpGraph::Edge e = mm[vn];
		// Unmatched net, or if it's a don't care net
		if (e == INVALID)
			continue;
		assert(weight[e] > 0);
		const RedNode in = bpg.redNode(e);
		const int ipin = rnode2ipin[in];
		fs << vnet << "-" << ipin/RAM_DATA_WIDTH << "." << ipin%RAM_DATA_WIDTH << "\n";
		matched++;
	}
	fs << std::endl;

	// Check all RAMs, and clear their ipins
	for (std::vector< std::map<int,int> >::iterator it = iram2ipins.begin(); it != iram2ipins.end(); ++it)
	{
		assert(it->size() <= RAM_ADDR_WIDTH);
		it->clear();
	}

	std::cout << numTrig << "," << numTrace << "," << invalidTrace << "," << t.userTime() << std::endl;
	assert(matched == numTrace+numTrig);
	fs.close();
	std::cout << " \033[1;31m Map Stage Ended \033[0m" << std::endl;

	return 0;
}
