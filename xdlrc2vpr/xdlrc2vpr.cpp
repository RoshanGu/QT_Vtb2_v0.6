// Torc - Copyright 2011 University of Southern California.  All Rights Reserved.
// $HeadURL: https://svn.east.isi.edu/torc/trunk/src/torc/architecture/DDB.cpp $
// $Id: DDB.cpp 1436 2013-05-06 20:47:54Z nsteiner $

// This program is free software: you can redistribute it and/or modify it under the terms of the 
// GNU General Public License as published by the Free Software Foundation, either version 3 of the 
// License, or (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See 
// the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License along with this program.  If 
// not, see <http://www.gnu.org/licenses/>.

// *******************************************************************************
// EH: This file is an adaptation of ../torc/src/torc/architecture/VprExporter.cpp
// *******************************************************************************

#include "xdlrc2vpr.hpp"
#include "torc/architecture/OutputStreamHelpers.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <zlib.h>

namespace torc {
namespace architecture {

	using namespace std;
	bool operator!=(const Tilewire& lhs, const Tilewire& rhs) {
		return !(lhs == rhs);
	}

	const boost::regex Xdlrc2Vpr::re_clk 	= boost::regex("(HCLK|CMT|CLK|GCLK).*");
	const boost::regex Xdlrc2Vpr::re_124 	= boost::regex("[NESW][NESWLR](\\d)BEG(_[NS])?\\d");
	const boost::regex Xdlrc2Vpr::re_L 	= boost::regex("L(H|V|VB)(_L)?\\d+");
	const boost::regex Xdlrc2Vpr::re_CLB	= boost::regex("CLBL[LM]_(M|LL|L)_[ABCD]([1-6]|Q|MUX)?");
	const boost::regex Xdlrc2Vpr::re_BYP 	= boost::regex("BYP(_ALT)?\\d");
	const boost::regex Xdlrc2Vpr::re_BOUNCE_NS = boost::regex("(BYP|FAN)_BOUNCE_[NS]3_\\d");
	const boost::regex Xdlrc2Vpr::re_FAN 	= boost::regex("FAN(_ALT)?\\d");
	const boost::regex Xdlrc2Vpr::re_BRAM_DI = boost::regex("BRAM_(FIFO36|FIFO18|RAMB18)_DIP?[AB]DIP?[LU]?\\d+");
	const boost::regex Xdlrc2Vpr::re_BRAM_AD = boost::regex("BRAM_(FIFO36|FIFO18|RAMB18)_ADDR(ARD|BWR)ADDR[LU]?\\d+");
	const boost::regex Xdlrc2Vpr::re_BRAM_WE = boost::regex("BRAM_(FIFO36|FIFO18|RAMB18)_WE(A|BWE)[LU]?\\d+");
	const boost::regex Xdlrc2Vpr::re_DSP 	= boost::regex("DSP_[01]_(A|B)\\d+");

	int Xdlrc2Vpr::getIswitch(const ExtendedWireInfo &ewiSrc, const Tilewire &twSnk) {
		boost::cmatch what;
		const bool srcIsClk = boost::regex_match(ewiSrc.mWireName, re_clk);
		int iswitch = DEFAULT_SWITCH_INDEX;

		TilewireVector tilewires;
		mDDB.expandSegment(twSnk, tilewires);

		bool snkIsClk = false;
		for (const Tilewire &tw : tilewires) {
			ExtendedWireInfo ewiSnk(mDDB, tw);

			if (!include(ewiSnk))
				continue;

			const char *sinkWireName = ewiSnk.mWireName;
			snkIsClk = snkIsClk || boost::regex_match(ewiSnk.mWireName, re_clk);

			if (boost::regex_match(sinkWireName, what, re_124)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				string s(what[1]);

				switch (s[0]) {
					case '1': iswitch = L1_SWITCH_INDEX; break;
					case '2': iswitch = L2_SWITCH_INDEX; break;
					case '4': iswitch = L4_SWITCH_INDEX; break;
					case '6': iswitch = L6_SWITCH_INDEX; break;
					default: throw;
				}
			}
			else if (boost::regex_match(sinkWireName, what, re_L)) {
				assert(!srcIsClk);
				string d(what[1]);
				if (boost::starts_with(mDDB.getDeviceName(), "xc6")) {
					assert(iswitch == DEFAULT_SWITCH_INDEX || iswitch == L16_SWITCH_INDEX);
					iswitch = L16_SWITCH_INDEX;
				}
				else throw;
			}
			else if (boost::regex_match(sinkWireName, what, re_CLB)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				string s(what[2]);
				switch (s[0]) {
					case '1': iswitch = A1_SWITCH_INDEX; break;
					case '2': iswitch = A2_SWITCH_INDEX; break;
					case '3': iswitch = A3_SWITCH_INDEX; break;
					case '4': iswitch = A4_SWITCH_INDEX; break;
					case '5': iswitch = A5_SWITCH_INDEX; break;
					case '6': iswitch = A6_SWITCH_INDEX; break;
					case '\0':
					case 'Q':
					case 'M': iswitch = LUT_SWITCH_INDEX; break;
					default: throw;
				}
			}
			else if (boost::regex_match(sinkWireName, what, re_BYP)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				iswitch = BYP_SWITCH_INDEX;
			}
			else if (boost::regex_match(sinkWireName, what, re_BYP_B)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX || BYP_B_SWITCH_INDEX);
				iswitch = BYP_B_SWITCH_INDEX;
			}
			else if (boost::regex_match(sinkWireName, what, re_FAN)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				iswitch = FAN_SWITCH_INDEX;
			}
			else if (boost::regex_match(sinkWireName, what, re_BRAM_DI)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				iswitch = BRAM_DI_SWITCH_INDEX;
			}
			else if (boost::regex_match(sinkWireName, what, re_BRAM_AD)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				iswitch = BRAM_AD_SWITCH_INDEX;
			}
			else if (boost::regex_match(sinkWireName, what, re_BRAM_WE)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				iswitch = BRAM_WE_SWITCH_INDEX;
			}
			else if (boost::regex_match(sinkWireName, what, re_DSP)) {
				assert(!srcIsClk);
				assert(iswitch == DEFAULT_SWITCH_INDEX);
				string s(what[1]);
				switch (s[0]) {
					case 'A': iswitch = DSP_A_SWITCH_INDEX; break;
					case 'B': iswitch = DSP_B_SWITCH_INDEX; break;
					default: throw;
				}
			}
		}
		if (srcIsClk != snkIsClk) {
			if (srcIsClk) {
				assert(iswitch == DEFAULT_SWITCH_INDEX || iswitch == CLK2GEN_SWITCH_INDEX);
				iswitch = CLK2GEN_SWITCH_INDEX;
			}
		}

		return iswitch;
	}

	void Xdlrc2Vpr::operator()(void) {

		//bool debug = true;

		cout << mDDB;
		cerr << "sizeof(short) = " << sizeof(short) << endl;
		cerr << "sizeof(int) = " << sizeof(int) << endl;
		cerr << "sizeof(float) = " << sizeof(float) << endl;
		cerr << "sizeof(t_rr_type) = " << sizeof(t_rr_type) << endl;
		cerr << "sizeof(void*) = " << sizeof(void*) << endl;
		cerr << "sizeof(t_rr_node) = " << sizeof(t_rr_node) << endl;

		const int num_vpr_nodes = rr_node.size();

		TileCount tileCount = mTiles.getTileCount();
		map<Tilewire,int> anchor2inode;
		for(TileIndex tileIndex(0); tileIndex < tileCount; tileIndex++) {
			// iterate over every wire in the tile
			const TileInfo& tileInfo = mTiles.getTileInfo(tileIndex);
			TileTypeIndex tileTypeIndex = tileInfo.getTypeIndex();
			WireCount wireCount = mTiles.getWireCount(tileTypeIndex);
			for(WireIndex wireIndex(0); wireIndex < wireCount; wireIndex++) {
				// take a quick exit if we've already looked at this segment
				Tilewire currentTilewire(tileIndex, wireIndex);

				// look up the segment information for this tilewire
				TilewireVector tilewires;
				mDDB.expandSegment(currentTilewire, tilewires);
				// if a segment is not real in this device, it won't have any tilewires
				if (tilewires.empty()) continue;
				// if this isn't the anchor, ignore it
				if (currentTilewire != tilewires.front()) continue;

				// determine whether to include this segment or not
				auto tp = tilewires.cbegin(), te = tilewires.cend();
				for (; tp < te; tp++) {
					ExtendedWireInfo ewi(mDDB, *tp);
					if (include(ewi))
						break;
				}
				if (tp == te) continue;

				bool b = anchor2inode.insert(make_pair(currentTilewire, num_nodes)).second;
				assert(b);
				++num_nodes;
			}
		}
		cerr << "Found " << anchor2inode.size() << " total segments compared to expected "
			<< mSegments.getTotalSegmentCount() << endl;

		/* First, setup the nodes file pointer, writing into a separate file.
		 * This is necessary and intentional because if we want to write node
		 * AND switch data (both of which must be contiguous, and not interleaved) 
		 * as we go along, we either need to store it all in memory and then 
		 * write it all at the very end, or to write them into separate files.
		 * A convenient property of gzip streams is that they can be concatenated
		 * into a single file without penalty; thus we can concat the streams
		 * into the format expected:   header_and_shim + nodes + edge_switch
		 * NB: The Makefile is responsible for doing this 'cat'-ing! */
		stringstream ss;
		ss << mDDB.getDeviceName() << mPackageName << mPostfix << ".nodes.gz";
		gzFile fpNodes = gzopen(ss.str().c_str(), "wb6");
		ss.str("");
		/* Second, setup the edge/switch data into a separate file */
		ss << mDDB.getDeviceName() << mPackageName << mPostfix << ".edge_switch.gz";
		gzFile fpEdgeSwitch = gzopen(ss.str().c_str(), "wb6");
		gzwrite(fpEdgeSwitch, edge_switch.data(), edge_switch.size());
		edge_switch.clear();
		edge_switch.shrink_to_fit();

		cerr << "Before: " << num_vpr_nodes << "/" << gztell(fpEdgeSwitch) << endl;

		auto p = anchor2inode.cbegin();
		auto e = anchor2inode.cend();
		ArcVector arcVector;
		TilewireVector tilewires;
		vector<short> switches;
		std::set<int> sinks;
		ExtendedWireInfo ewi(mDDB);

		ss.str("");
		ss << mDDB.getDeviceName() << mPackageName << mPostfix << ".tws";
		ofstream fInode2Tw(ss.str().c_str(), std::ios::binary);
		//ss << ".txt";
		//ofstream fInode2TwText(ss.str().c_str());
		fInode2Tw.write(reinterpret_cast<const char*>(&num_vpr_nodes), sizeof(int));

		while(p != e) {
			t_rr_node vpr_rr_node;

			// Set coordinates to maximum
			vpr_rr_node.xlow = numeric_limits<decltype(vpr_rr_node.xlow)>::max();
			vpr_rr_node.ylow = numeric_limits<decltype(vpr_rr_node.ylow)>::max();
			vpr_rr_node.xhigh = numeric_limits<decltype(vpr_rr_node.xhigh)>::min();
			vpr_rr_node.yhigh = numeric_limits<decltype(vpr_rr_node.yhigh)>::min();

			// Set default cost index
			vpr_rr_node.cost_index = DEFAULT_COST_INDEX;

			// iterate over every segment wire to extract attributes
			const Tilewire& twAnchor = p->first;
			const int& inodeAnchor = p->second;
			++p;
			tilewires.clear();
			mDDB.expandSegment(twAnchor, tilewires);
			auto tp = tilewires.cbegin(), te = tilewires.cend();
			for(; tp < te; ++tp) {
				ewi = *tp;
				if (!include(ewi))
					continue;

				short row, col;
				std::tie(row,col) = ewi2rc(ewi);
				vpr_rr_node.xlow = min(vpr_rr_node.xlow, col);
				vpr_rr_node.ylow = min(vpr_rr_node.ylow, row);
				vpr_rr_node.xhigh = max(vpr_rr_node.xhigh, col);
				vpr_rr_node.yhigh = max(vpr_rr_node.yhigh, row);

				// TODO: Function
				// Compute cost_index
				boost::match_results<const char*> what;
				if (boost::regex_match(ewi.mWireName, re_clk)) {
					if (vpr_rr_node.cost_index != CLOCK_COST_INDEX) {
						assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
						vpr_rr_node.cost_index = CLOCK_COST_INDEX;
					}
				} 
				else if (boost::regex_match(ewi.mWireName, what, re_124)) {
					string s(what[1]);
					switch (s[0]) {
						case '1': 
							if (vpr_rr_node.cost_index != L1_COST_INDEX) {
								assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
								vpr_rr_node.cost_index = L1_COST_INDEX; 
							}
							break;
						case '2': 
							if (vpr_rr_node.cost_index != L2_COST_INDEX) {
								assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
								vpr_rr_node.cost_index = L2_COST_INDEX; 
							}
							break;
						case '4': 
							if (vpr_rr_node.cost_index != L4_COST_INDEX) {
								assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
								vpr_rr_node.cost_index = L4_COST_INDEX; 
							}
							break;
						case '6': 
							if (vpr_rr_node.cost_index != L6_COST_INDEX) {
								assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
								vpr_rr_node.cost_index = L6_COST_INDEX; 
							}
							break;
						default: throw;
					}
					//break;
				} 
				else if (boost::regex_match(ewi.mWireName, what, re_L)) {
					string d(what[1]);
					if (boost::starts_with(mDDB.getDeviceName(), "xc6")) {
						if (vpr_rr_node.cost_index != L16_COST_INDEX) {
							assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
							vpr_rr_node.cost_index = L16_COST_INDEX;
						}
					}
					else throw;
				}
				// Treat BYP_BOUNCE_N3_\\d and FAN_BOUNCE_S3_\\d segments as a L1 wire
				else if (boost::regex_match(ewi.mWireName, what, re_BOUNCE_NS)) {
					if (vpr_rr_node.cost_index != L1_COST_INDEX) {
						assert(vpr_rr_node.cost_index == DEFAULT_COST_INDEX);
						vpr_rr_node.cost_index = L1_COST_INDEX; 
					}
				}
			}
			assert(vpr_rr_node.xlow != numeric_limits<decltype(vpr_rr_node.xlow)>::max());
			assert(vpr_rr_node.xhigh != numeric_limits<decltype(vpr_rr_node.xhigh)>::max());
			assert(vpr_rr_node.ylow != numeric_limits<decltype(vpr_rr_node.ylow)>::min());
			assert(vpr_rr_node.yhigh != numeric_limits<decltype(vpr_rr_node.yhigh)>::min());

			if (vpr_rr_node.cost_index == DEFAULT_COST_INDEX) {
				static set<string> wires;
				ewi = twAnchor;
				if (!wires.count(ewi.mWireName)) {
					cerr << "No cost index: " << ewi << endl;
					wires.insert(ewi.mWireName);
				}
			}

			// Horizontal wires (and 0-length wires?)
			const int rowRange = vpr_rr_node.xhigh - vpr_rr_node.xlow;
			const int colRange = vpr_rr_node.yhigh - vpr_rr_node.ylow;
			if(rowRange >= colRange) { 
				vpr_rr_node.type = CHANX; 
			}
			// Vertical wires
			else /*if(node.mColRange == 0)*/ { 
				vpr_rr_node.type = CHANY; 
				vpr_rr_node.cost_index += INV_COST_INDEX_OFFSET;
			}
			// Diagonal wires
			/*else { 
				vpr_rr_node.type = CHANX;
			}*/

			ewi = twAnchor;

#if 0
			// ptc_num and net_num are not used during routing,
			// claim them for wire/tile to help debugging
			vpr_rr_node.ptc_num = twAnchor.getWireIndex();
			vpr_rr_node.net_num = twAnchor.getTileIndex();
#else
			vpr_rr_node.ptc_num = vpr_rr_node.net_num = 0;
#endif

			// no intrinsic cost, and current occupancy is zero
			vpr_rr_node.occ = 0;
			// the capacity of every XDLRC wire is one
			vpr_rr_node.capacity = 1;
			// does VPR really use fanin information?  we can provide this if it's important
			vpr_rr_node.fan_in = 0;
			// ignore direction for now
			vpr_rr_node.direction = (e_direction)-1;

			vpr_rr_node.edges = reinterpret_cast<decltype(vpr_rr_node.edges)>
				(gztell(fpEdgeSwitch));

			// Compute edges
			arcVector.clear();
			mDDB.expandSegmentSinks(twAnchor, arcVector, DDB::eExpandDirectionNone, true, true, 
				true, true);
			auto ap = arcVector.cbegin(), ae = arcVector.cend();
			sinks.clear();
			while(ap < ae) {
				const Arc& arc = *ap++;
				const Tilewire& twSrc = arc.getSourceTilewire();
				const Tilewire& twSink = arc.getSinkTilewire();

				// Extract the anchor
				tilewires.clear();
				mDDB.expandSegment(twSink, tilewires);
				auto it = anchor2inode.find(tilewires.front());
				if (it == anchor2inode.end()) continue;
				const int& inodeSink = it->second;

				ExtendedWireInfo ewiSrc(mDDB, twSrc);
				ExtendedWireInfo ewiSnk(mDDB, twSink);
				static boost::cmatch what;
				static boost::regex bufg_i("(CMT|CLK)_BUFG_BUFGCTRL\\d+_I0");
				static boost::regex bufg_o("(CMT|CLK)_BUFG_BUFGCTRL\\d+_O");
				if (boost::regex_match(ewiSrc.mWireName, bufg_i) && boost::regex_match(ewiSnk.mWireName, bufg_o))
					continue;

				if (sinks.find(inodeSink) == sinks.end()) {
					gzwrite(fpEdgeSwitch, &inodeSink, sizeof(inodeSink));

					short iswitch;
					// For (BYP|FAN)_BOUNCE_[NS]3_\\d arcs, give it the
					// same delay as a L1 wire
					if (boost::regex_match(ewiSrc.mWireName, re_BOUNCE_NS)) {
						iswitch = L1_SWITCH_INDEX;
					}
					else 
						iswitch = getIswitch(ewiSrc, twSink);
					switches.push_back(iswitch);


					sinks.insert(inodeSink);
				}
			}

			stitchInode(twAnchor, inodeAnchor, vpr_rr_node, fpEdgeSwitch, switches);

			vpr_rr_node.num_edges = switches.size();

			vpr_rr_node.switches = reinterpret_cast<short*>(gztell(fpEdgeSwitch));
			gzwrite(fpEdgeSwitch, switches.data(), sizeof(short) * switches.size());
			switches.clear();

			// we have no resistance or capacitance data
			vpr_rr_node.R = vpr_rr_node.C = 0;
			// all I know is that most wires are not bidirectional
			vpr_rr_node.direction = INC_DIRECTION; // what does INC_DIRECTION mean?
			// all XDLRC wires have a single driver
			vpr_rr_node.drivers = SINGLE;
			// what exactly do the following two fields mean?
			vpr_rr_node.num_wire_drivers = 0; // unless it's important to provide this
			vpr_rr_node.num_opin_drivers = 0; // unless it's important to provide this
			// VPR will use this to trace back after routing
			vpr_rr_node.prev_node = vpr_rr_node.prev_edge = /*vpr_rr_node.net_num =*/ OPEN;
			// we aren't populating graph or timing information
			vpr_rr_node.pb_graph_pin = 0;
			vpr_rr_node.tnode = 0;
			// we currently provide no packing cost hints
			vpr_rr_node.pack_intrinsic_cost = 0;
			vpr_rr_node.z = 0;

			/* Set prev_node for CLB LUT and X inputs, as well as BYP_B\\d */
			static const boost::regex re_clblut("CLBL[LM]_(L|LL|M)_[ABCD][1-6]");
			static const boost::regex re_clbx("CLBL[LM]_(L|LL|M)_[ABCD]X");
			if (boost::regex_match(ewi.mWireName, re_clblut)
					|| boost::regex_match(ewi.mWireName, re_clbx)
					|| boost::regex_match(ewi.mWireName, re_BYP_B)) {
				arcVector.clear();
				mDDB.expandSegmentSources(twAnchor, arcVector, DDB::eExpandDirectionNone, true, true, 
						true, true);
				assert(arcVector.size() == 1);

				const Arc& arc = arcVector.front();
				const Tilewire& twSrc = arc.getSourceTilewire();

				// Extract the anchor
				tilewires.clear();
				mDDB.expandSegment(twSrc, tilewires);
				auto it = anchor2inode.find(tilewires.front());
				assert(it != anchor2inode.end());
				const int& inodeSrc = it->second;

				vpr_rr_node.prev_node = inodeSrc;
			}

			gzwrite(fpNodes, &vpr_rr_node, sizeof(vpr_rr_node));

			fInode2Tw.write(reinterpret_cast<const char*>(&twAnchor), sizeof(twAnchor));
			//fInode2TwText << inodeAnchor << "," << twAnchor << std::endl;
		}

		fInode2Tw.close();
		//fInode2TwText.close();

		cerr << "After: " << num_nodes << "/" << gztell(fpEdgeSwitch) << endl;

		/* Lastly, write the header and shim information into a third separate file;
		 * The Makefile is responsible for */
		ss.str("");
		ss << mDDB.getDeviceName() << mPackageName << mPostfix << ".header_and_shim.gz";
		gzFile fpHeaderShim = gzopen(ss.str().c_str(), "wb6");

		int t;
		// Write the size of the t_rr_node
		t = sizeof(t_rr_node);
		gzwrite(fpHeaderShim, &t, sizeof(int));
		// Write the number of nodes
		t = num_nodes;
		gzwrite(fpHeaderShim, &t, sizeof(int));

		// Process the deferred edge/switch data
		for (const auto &it : inode2EdgeSwitch) {
			const auto &inode = it.first;
			const auto &edges = get<0>(it.second);
			const auto &switches = get<1>(it.second);
			assert(inode < num_vpr_nodes);
			assert(edges.size() == switches.size());
			rr_node[inode].num_edges = edges.size();
			rr_node[inode].edges = reinterpret_cast<int*>(gztell(fpEdgeSwitch));
			gzwrite(fpEdgeSwitch, edges.data(), edges.size() * sizeof(int));
			rr_node[inode].switches = reinterpret_cast<short*>(gztell(fpEdgeSwitch));
			gzwrite(fpEdgeSwitch, switches.data(), switches.size() * sizeof(short));
		}

		// Write the size of the edge switch data
		t = gztell(fpEdgeSwitch);
		gzwrite(fpHeaderShim, &t, sizeof(int));
		// Write the VPR shim
		gzwrite(fpHeaderShim, rr_node.data(), sizeof(t_rr_node)*rr_node.size());

		gzclose(fpHeaderShim);
		gzclose(fpNodes);
		gzclose(fpEdgeSwitch);
	}

	void Xdlrc2Vpr::loadVpr(const string &fn) {
		ifstream fs(fn, ifstream::binary);
		if (!fs) {
			std::cout << "loadVpr() failed: '" << fn << "' not found!" << std::endl;
			exit(1);
		}
		int size, edgeLength;
		fs.read(reinterpret_cast<char*>(&size), sizeof(int));
		assert(size == sizeof(t_rr_node));
		fs.read(reinterpret_cast<char*>(&num_nodes), sizeof(int));
		fs.read(reinterpret_cast<char*>(&edgeLength), sizeof(int));
		rr_node.resize(num_nodes);
		edge_switch.resize(edgeLength);
		type.resize(num_nodes);
		fs.read(reinterpret_cast<char*>(rr_node.data()), sizeof(t_rr_node)*num_nodes);
		fs.read(edge_switch.data(), edgeLength);
		fs.read(reinterpret_cast<char*>(type.data()), sizeof(int)*num_nodes);

		int edgeOffset = 0;
		int inode = 0;
		for (auto &n : rr_node) {
			int num_edges = n.num_edges;
			n.edges = reinterpret_cast<int*>(edgeOffset);
			edgeOffset += sizeof(int)*num_edges;
			n.switches = reinterpret_cast<short*>(edgeOffset);
			edgeOffset += sizeof(short)*num_edges;
			const bool inserted = ptc2inode.insert( std::make_pair(
						std::make_tuple(type[inode], (int)n.xlow, (int)n.ylow, n.type, (int)n.ptc_num),
						inode) ).second;
			inode++;
			n.prev_node = -1;
			assert(inserted);
		}
		assert(edgeOffset == edgeLength);

		fs.close();
	}

	void Xdlrc2Vpr::loadPtc2Name(const string &fn) {
		ifstream fs(fn);
		if (!fs) {
			std::cout << "loadPtc2Name() failed: '" << fn << "' not found!" << std::endl;
			exit(1);
		}
		fs.ignore(1024, '\n');

		int pb_type, ptc;
		std::string s_type, wirename;
		const boost::regex re("CLBL[LM]_(L|M|LL)_[ABCD]6");

		while (fs >> pb_type >> ptc >> s_type >> wirename) {
			t_rr_type type;
			if (s_type == "IPIN") type = IPIN;
			else if (s_type == "OPIN") type = OPIN;
			else throw;

			if (!(wirename == "GND_WIRE" || wirename == "VCC_WIRE" || boost::regex_match(wirename, re))) {
				assert(name2ptc.find(wirename) == name2ptc.end());
			}
			name2ptc.insert( std::make_pair( 
						wirename,
						std::make_tuple(pb_type,ptc,type) ));
		}

		fs.close();
	}

	string Xdlrc2Vpr::getStitchWirename(const Tilewire &tw) {
		const ExtendedWireInfo ewi(mDDB, tw);
		const string wirename(ewi.mWireName);
		return wirename;
	}

	short Xdlrc2Vpr::getStitchY(const ExtendedWireInfo& ewi, const short& x, const short& y) {
		const string wirename(ewi.mWireName);
		
		if (mDDB.getDeviceName() == "xc6vlx240t") {
			if (boost::starts_with(ewi.mTileTypeName, "INT")
					&& (wirename == "GND_WIRE" || wirename == "VCC_WIRE")) {

				// Collapse all into root of hard block
				switch(x) {
					// DSP
					case 8:
					case 13:
					case 28:
					case 33:
					case 65:
					case 70:
					case 85:
					case 90:

					// BRAM
					case 5:
					case 16:
					case 25:
					case 36:
					case 62:
					case 73:
					case 82:
					case 93:
					case 101:
						return ((y-1) / 5) * 5 + 1;
				}
			}
		}


		return y;
	}

	void Xdlrc2Vpr::stitchInode(const Tilewire& twAnchor, const int &inodeXilinx, 
			t_rr_node &rrnXilinx, gzFile fpEdgeSwitch, std::vector<short>& switches) {
		TilewireVector twVec;
		mDDB.expandSegment(twAnchor, twVec);
		assert(twVec.front() == twAnchor);
		string wirename;

		ExtendedWireInfo ewi(mDDB, twAnchor);
		for (const Tilewire& tw : twVec) {
			ewi = tw;
			if (!include(ewi)) continue;
			const string &w = getStitchWirename(tw);
			auto it = name2ptc.equal_range(w);
			if (it.first != it.second) {
				assert(wirename.empty());
				wirename = w;
			}
		}
		if (wirename.empty()) return;
		auto it = name2ptc.equal_range(wirename);

		ewi = twAnchor;
		const short& x = rrnXilinx.xlow;
		const short& y = getStitchY(ewi, x, rrnXilinx.ylow);

		// Go through all ptc-s
		auto kt = it.first, ke = it.second;
		bool found = false;
		for (; kt != ke; ++kt) {
			int pb_type, ptc;
			t_rr_type type;
			tie(pb_type,ptc,type) = kt->second;
			// Find the inode
			auto jt = ptc2inode.find( std::make_tuple(pb_type,x,y,type,ptc) );
			if (jt == ptc2inode.end()) continue;

			const int& inodeVpr = jt->second;
			switch(type) {
				case IPIN: 
					gzwrite(fpEdgeSwitch, &inodeVpr, sizeof(inodeVpr));
					switches.push_back(DEFAULT_SWITCH_INDEX);
					++rrnXilinx.num_edges;

					assert(rr_node[inodeVpr].prev_node == OPEN);
					rr_node[inodeVpr].prev_node = inodeXilinx;
					break;
				case OPIN: 
					{
					assert(rr_node[inodeVpr].num_edges == 0);

					auto lt = inode2EdgeSwitch.find(inodeVpr);
					if (lt == inode2EdgeSwitch.end())
						lt = inode2EdgeSwitch.insert(make_pair(inodeVpr,
											make_tuple(vector<int>(), vector<short>())))
							.first;

					vector<int>& edges = get<0>(lt->second);
					edges.push_back(inodeXilinx);

					vector<short>& switches = get<1>(lt->second);
					switches.push_back(DEFAULT_SWITCH_INDEX);
					}
					break; 
				default: 
					throw;
			}
			found = true;
		}

		if (!found) {
			cerr << "For anchor " << ewi << ", did not find VPR node @"
				<< " (" << x << "," << y << ")" << endl;
		}
	}

} // namespace architecture
} // namespace torc

int main(int argc, char **argv)
{
	using namespace torc::architecture;
	if (argc != 4) {
		std::cout << "Usage: xdlrc2vpr <device+package> <width> <height>" << std::endl;
		std::cout << "e.g. : xdlrc2vpr xc6vlx240tff1156 102 240" << std::endl;
		return 1;
	}
	const boost::regex re("(xc.*)((ff|ffg|clg)\\d+)(_.*)?");
	boost::cmatch what;
	const bool rt = boost::regex_match(argv[1], what, re);
	if (!rt) {
		std::cout << "<device+package>: '" << argv[2] << "' does not regex: '" << re.str() << "'" << std::endl;
		return 1;
	}

	DDB ddb(what[1],what[2]);
	const int width = boost::lexical_cast<int>(argv[2]);
	const int height = boost::lexical_cast<int>(argv[3]);
	Xdlrc2Vpr x2v(ddb, what[2], what[4], width, height);
	x2v();
}
