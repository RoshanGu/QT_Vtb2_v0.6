#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <float.h>
#include <signal.h>

#include "vpr_types.h"
#include "globals.h"
#include "route_common.h"
#include "check_netlist.h"
#include "check_route.h"
#include "cluster_legality.h"
#include "heapsort.h"
#include "rr_graph_util.h"
#include "rr_graph2.h"
#include "mst.h"
#include "route_export.h"

#include "inc_trace.h"
#include "inc_route.h"
#include "inc_route_bfs.h"
#include "inc_route_dir.h"
#include "inc_misc.h"
#include "inc_collapse.h"

/**
 * Finds the index of the "memory" type (returned value)
 * and populates the data_pin and data_width params
 */
static int 
inc_find_mem(int *data_pin, int *data_width)
{
	int i;

	/* Go through all the types looking for the name "memory" */
	for (i = 0; i < num_types; i++) 
	{
		//printf(" --->>> type name for memory %s \n", type_descriptors[i].name );
		if (strcmp(type_descriptors[i].name, "RAMB36E1") == 0) 
		{
			int j, type, pin;
			type = i;
			pin = 0;
			/* Go through each port looking for the name "data" */
			for (j = 0; j < type_descriptors[i].pb_type->num_ports; j++)
			{
				//printf(" ---> PORT names for memory %s \n", type_descriptors[i].pb_type->ports[j].name );
				if (strcmp(type_descriptors[i].pb_type->ports[j].name, "DIADI")  == 0 || strcmp(type_descriptors[i].pb_type->ports[j].name, "DIPADIP")  == 0
				|| strcmp(type_descriptors[i].pb_type->ports[j].name, "DIBDI")  == 0 || strcmp(type_descriptors[i].pb_type->ports[j].name, "DIPBDIP")  == 0)
				{
					*data_pin = pin;
					*data_width = type_descriptors[i].pb_type->ports[j].num_pins;
				}
				else
					pin += type_descriptors[i].pb_type->ports[j].num_pins;
			}
			return type;
		}
	}
	printf(ERRTAG "\"memory\" type not found!");
	exit(1);
}

/**
 * Wrapper function for alloc_and_load_rr_graph_for_pb_garph_node()
 * which clobbers the rr_node global
 */
static void 
inc_alloc_and_load_rr_graph_for_pb_graph_node(	
		INP t_pb_graph_node *pb_graph_node, 
		INP const t_arch* arch, 
		int mode, 
		t_rr_node *local_rr_node) 
{
	t_rr_node *global_rr_node;

	global_rr_node = rr_node;
	rr_node = local_rr_node;
	alloc_and_load_rr_graph_for_pb_graph_node(pb_graph_node, arch, mode);
	rr_node = global_rr_node;
}


/**
 * Place a new block into the circuit, returning its index
 */
static int 
inc_place_block(short x, short y, int type, const t_arch *arch)
{
	int iblk;
	int i;
	iblk = num_blocks;
	num_blocks++;
	block = (struct s_block*)realloc(block, num_blocks*sizeof(struct s_block));
	block[iblk].name = NULL;
	block[iblk].x = x;
	block[iblk].y = y;
	block[iblk].z = 0;
	block[iblk].type = &type_descriptors[type];
	block[iblk].pb = (t_pb *) calloc(1, sizeof(t_pb));
	block[iblk].pb->pb_graph_node = block[iblk].type->pb_graph_head;
	block[iblk].pb->rr_graph = (t_rr_node *) calloc(block[iblk].type->pb_graph_head->total_pb_pins, sizeof(t_rr_node));
	inc_alloc_and_load_rr_graph_for_pb_graph_node(block[iblk].pb->pb_graph_node, arch, 0, block[iblk].pb->rr_graph);
	block[iblk].nets = (int *) malloc(block[iblk].type->num_pins*sizeof(int));
	for (i = 0; i < block[iblk].type->num_pins; i++)
	{
		block[iblk].nets[i] = OPEN;
	}

	assert(grid[block[iblk].x][block[iblk].y].usage < grid[block[iblk].x][block[iblk].y].type->capacity);
	grid[block[iblk].x][block[iblk].y].blocks[grid[block[iblk].x][block[iblk].y].usage] = iblk;
	grid[block[iblk].x][block[iblk].y].usage++;

	return iblk;
}

/**
 * Reclaims all spare memory blocks as trace-buffers
 * Populate the tb parameter with their data
 */
int 
inc_reclaim_tbs(t_trace_buffer **tb, const t_arch *arch)
{
	int mem_type, data_pin, data_width;
	int i, num_tb;

	mem_type = inc_find_mem(&data_pin, &data_width);
	num_tb = 0;

	/* Search the FPGA grid for unused memory blocks,
	 * and place a trace-buffer into those */
	for (i = 0; i <= nx+1; i++) 
	{
		int j;
		for (j = 0; j <= ny+1; j++)
		{
			if (grid[i][j].offset == 0 && grid[i][j].type->index == mem_type)
			{
				if (grid[i][j].usage == 0)
				{
					*tb = (t_trace_buffer *) realloc(*tb, sizeof(**tb) * (num_tb+1));
					(*tb)[num_tb].x = i; 
					(*tb)[num_tb].y = j; 
					(*tb)[num_tb].iblk = num_blocks;
					(*tb)[num_tb].data_pin = data_pin;
					(*tb)[num_tb].usage = 0;
					(*tb)[num_tb].capacity = data_width;
					(*tb)[num_tb].sink_inodes = NULL;
					inc_place_block(i, j, mem_type, arch);
					num_tb++;
				}
			}
		}
	}

	if (num_tb == 0)
	{
		printf(ERRTAG "No free memory blocks to reclaim as trace-buffers!");
		exit(1);
	}
	return num_tb;
}

/**
 * Mark all trace-buffer "data" nodes as targets
 * (NB: I'm using target_flag as an indicator for which
 * trace-buffer it belongs to)
 * Adapted from mark_ends() 
 * */
static void 
inc_mark_targets(int num_tb, t_trace_buffer *tb)
{
	int inode;
	int i, j;
	for (i = 0; i < num_tb; i++) 
	{
		for (j = 0; j < tb[i].capacity; j++) 
		{
			inode = get_rr_node_index(tb[i].x, tb[i].y, SINK, 
					j+tb[i].data_pin, rr_node_indices);
			assert(rr_node_route_inf[inode].target_flag == 0);
			rr_node_route_inf[inode].target_flag = tb[i].iblk;
		}
	}
}

/**
 * Find the block corresponding to the x/y coordinates given
 */
static int 
inc_find_block(short x, short y)
{
	int i = 0;
	for (i = 0; i < num_blocks; i++)
	{
		if (block[i].x == x && block[i].y == y)
		{
			return i;
		}
	}
	return -1;
}

/**
 * Connect the specified net to the block/pin specified 
 */
static void 
inc_connect_net(int inet, int iblk, int ptc)
{
	int isink;
	int node1, node2;

	assert(iblk >= 0);

	clb_net[inet].num_sinks++;
	isink = clb_net[inet].num_sinks;
#if 0
	if (clb_net[inet].node_block[isink] != OPEN)
		assert(clb_net[inet].node_block[isink] == iblk);
	else
		clb_net[inet].node_block[isink] = iblk;
#endif
	clb_net[inet].node_block = (int *) realloc(clb_net[inet].node_block, (isink+1)*sizeof(int));
	clb_net[inet].node_block[isink] = iblk;
	assert(clb_net[inet].node_block_port == NULL);
	clb_net[inet].node_block_pin = (int *) realloc(clb_net[inet].node_block_pin, (isink+1)*sizeof(int));
	clb_net[inet].node_block_pin[isink] = ptc;

	/*	assert(block[iblk].nets[ptc] == OPEN);*/
	block[iblk].nets[ptc] = inet;

	/*	assert(block[iblk].pb->rr_graph[ptc].net_num == OPEN);*/
	block[iblk].pb->rr_graph[ptc].net_num = clb_to_vpack_net_mapping[inet];

	/* Use mode zero (which in sample_arch.xml is the widest) */
	/* node1 -- interconnect */
	node1 = block[iblk].pb->rr_graph[ptc].edges[0];
	/*	assert(node1 == block[iblk].pb->rr_graph[ptc].pb_graph_pin->output_edges[0]->output_pins[0]->pin_count_in_cluster);*/
	/*	assert(block[iblk].pb->rr_graph[node1].prev_node == OPEN);*/
	block[iblk].pb->rr_graph[node1].prev_node = ptc;
	/*	assert(block[iblk].pb->rr_graph[node1].prev_edge == OPEN);*/
	block[iblk].pb->rr_graph[node1].prev_edge = 0;
	/*	assert(block[iblk].pb->rr_graph[node1].net_num == OPEN);*/
	block[iblk].pb->rr_graph[node1].net_num = clb_to_vpack_net_mapping[inet];
	/*	assert(block[iblk].pb->rr_graph[node1].num_edges == 1);*/

	/* node2 -- sink */
	node2 = block[iblk].pb->rr_graph[node1].edges[0];
	/*	assert(block[iblk].pb->rr_graph[node2].type == SINK);*/
	/*	assert(block[iblk].pb->rr_graph[node2].prev_node == OPEN);*/
	block[iblk].pb->rr_graph[node2].prev_node = node1;
	/*	assert(block[iblk].pb->rr_graph[node2].prev_edge == OPEN);*/
	block[iblk].pb->rr_graph[node2].prev_edge = 0;
	/*	assert(block[iblk].pb->rr_graph[node2].net_num == OPEN);*/
	block[iblk].pb->rr_graph[node2].net_num = clb_to_vpack_net_mapping[inet];
}

/**
 * Connect all trace nets to their trace_tail sinks 
 */
static void 
inc_connect_trace(
		int *trace_nets, 
		int num_trace_nets, 
		int old_num_nets)
{
	int i, j;
	int inet, sink_inode, inode;
	int iblk, ptc;
	int isink;

	for (i = 0; i < num_trace_nets; i++) 
	{
		struct s_trace *tptr;

		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		tptr = trace_head[inet];
		while (tptr)
		{
			sink_inode = tptr->index;
			if (rr_node[sink_inode].type == SINK &&
					rr_node[sink_inode].inc_occ > 0)
			{
				sink_inode = tptr->index;
				assert(rr_node[sink_inode].type == SINK);
				iblk = inc_find_block(rr_node[sink_inode].xlow, rr_node[sink_inode].ylow);
				ptc = rr_node[sink_inode].ptc_num;
				inc_connect_net(inet, iblk, ptc);

				isink = clb_net[inet].num_sinks;

				/* Update net_rr_terminals if its empty, or if many-to-many 
				 * flexibility is enabled */
				net_rr_terminals[inet] = (int *) realloc(net_rr_terminals[inet], (isink+1)*sizeof(int *));
				net_rr_terminals[inet][isink] = sink_inode;

				/* Fix wrong class error in check_source() 
				 * for newly formed (global) inets */
				if (inet >= old_num_nets)
				{
					int x, y;
					t_type_ptr type;
					inode = trace_head[inet]->index;
					x = rr_node[inode].xlow;
					y = rr_node[inode].ylow;
					type = grid[x][y].type;
					for (j = 0; j < type->num_pins; j++)
					{
						if (type->pin_class[j] == rr_node[inode].ptc_num)
						{
							clb_net[inet].node_block_pin[0] = j;
							break;
						}
					}
					assert(j < type->num_pins);
				}
			}
			tptr = tptr->next;
		}
	}
}

/**
 * Check that the OPIN of inet is legitimate
 */
static boolean 
inc_feasible_outpin(	
		int inet,
		int old_num_nets)
{
	int iblk, ipin, vnet, ptc;
	int ible, nbles;
	int inode, source_inode;
	int ninpins, noutpins;
	iblk = clb_net[inet].node_block[0];
	ipin = clb_net[inet].node_block_pin[0];

	vnet = block[iblk].pb->rr_graph[ipin].net_num;
	assert(vpack_to_clb_net_mapping[vnet] == inet);

	assert(block[iblk].pb->pb_graph_node->pb_type->num_pb == 1);
	assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 1);
	assert(block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children == 1);
	nbles = block[iblk].pb->pb_graph_node->pb_type->modes[0].pb_type_children[0].num_pb;
	assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->num_input_ports == 1);
	ninpins = block[iblk].pb->child_pbs[0][0].pb_graph_node->num_input_pins[0];
	assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->num_output_ports == 1);
	noutpins = block[iblk].pb->child_pbs[0][0].pb_graph_node->num_output_pins[0];

	assert(inet >= old_num_nets);
	/* FIXME: Do for all inodes in net */
	source_inode = trace_head[inet]->index;

	for (ible = 0; ible < nbles; ible++)
	{
		int used_inputs;
		bool fractured;

		assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_input_ports == 1);
		assert(ninpins == block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_input_pins[0]);

		/* Count how many used inputs there are in its BLE */
		used_inputs = 0;
		for (ipin = 0; ipin < ninpins; ipin++)
		{
			ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->input_pins[0][ipin]
				.pin_count_in_cluster;
			vnet = block[iblk].pb->rr_graph[ptc].net_num;
			if (vnet != OPEN)
			{
				used_inputs++;
			}
		}
		/* If we are not using all the input pins, then this net can 
		 * be considered fractured ... because in the k6_N10_memDepth16384...
		 * arch file both fractured 5LUTs use the same inputs */
		/* FIXME: Make this more generic */
		fractured = used_inputs < ninpins;

		/* Find the CLB OPIN that this inet originates from */
		inode = OPEN;
		for (ipin = 0; ipin < noutpins; ipin++)
		{
			/* BLE OPIN ptc */
			ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
				.pin_count_in_cluster;
			vnet = block[iblk].pb->rr_graph[ptc].net_num;
			/* If this net is a local net (has no global net, or has a global net which we created for tracing) */
			if (vnet == OPEN || clb_to_vpack_net_mapping[inet] == vnet || vpack_to_clb_net_mapping[vnet] == OPEN || vpack_to_clb_net_mapping[vnet] >= old_num_nets) 
			{
				assert(block[iblk].pb->pb_graph_node->num_output_ports == 1);
				/* CLB OPIN */
				ptc = block[iblk].pb->pb_graph_node->output_pins[0][ible*noutpins+ipin].pin_count_in_cluster;

				/* Get the rr_node index of CLB_OPIN SOURCE */
				inode = get_rr_node_index(block[iblk].x, block[iblk].y, 
						SOURCE, block[iblk].type->pin_class[ptc], rr_node_indices);

				if (source_inode == inode)
				{
					break;
				}
			}
		}

		/* If this is the BLE */
		if (source_inode == inode)
		{
			int ineligible;
			ineligible = 0;
			/* Make sure there's at most one ineligible output pin for
			 * fractured, but none for unfractured */
			for (ipin = 0; ipin < noutpins; ipin++)
			{
				ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
					.pin_count_in_cluster;
				vnet = block[iblk].pb->rr_graph[ptc].net_num;

				if (!(vnet == OPEN || clb_to_vpack_net_mapping[inet] == vnet || vpack_to_clb_net_mapping[vnet] == OPEN || vpack_to_clb_net_mapping[vnet] >= old_num_nets))
				{
					ineligible++;
				}
			}

			if ((fractured && ineligible > 1) || (!fractured && ineligible > 0))
			{
				return FALSE;
			}
			return TRUE;
		}
	}
	assert(ible < nbles);
	return TRUE;
}


/**
 * Check if just the incremental routing is feasible 
 * Also, track how many nets and nodes were overused
 * Adapted from feasible_routing()
 */
static bool 
inc_feasible_routing(
		int num_trace_nets, 
		int *trace_nets, 
		struct s_trace **old_trace_tail, 
		int *opins_overuse,
		int *fanout_overuse,
		int old_num_nets,
		int **inode2fanouts)
{
	int i, inet, inode;
	struct s_trace *tptr;
	boolean net_overuse;
	*opins_overuse = 0;

	/* Go through each trace net in turn */
	for (i = 0; i < num_trace_nets; i++)
	{
		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		/* Find when the incremental trace starts */
		if (old_trace_tail[inet])
			tptr = old_trace_tail[inet]->next->next;
		else
			tptr = trace_head[inet];

		/* Check that each incremental node does
		 * not exceed occupancy */
		net_overuse = FALSE;
		while (tptr)
		{
			inode = tptr->index;
			if (rr_node[inode].occ > rr_node[inode].capacity)
			{
				assert(rr_node[inode].type == SOURCE || rr_node[inode].type == OPIN);
				net_overuse = TRUE;
			}
			tptr = tptr->next;
		}

		/* If this is a new global net, then
		 * check that its OPIN is feasible too */
		if (inet >= old_num_nets)
		{
			if (!inc_feasible_outpin(inet, old_num_nets))
			{
				net_overuse = TRUE;
			}
		}

		if (net_overuse)
			(*opins_overuse)++;
	}

	/* Check edges overused */
	*fanout_overuse = 0;
	for (inode = 0; inode < num_rr_nodes; inode++)
	{
		/* Allow multiple fan-out if only one occupancy */
		if (inode2fanouts[inode])
		{
			int iedge, fanout_used;

			fanout_used = 0;
			for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
			{
				if (inode2fanouts[inode][iedge] > 0) fanout_used++;
			}

			if (fanout_used > 1)
			{
				assert(rr_node[inode].inc_occ > 1);
				/*printf("inode %d uses %d output edges!\n", inode, fanout_used);*/
				*fanout_overuse += fanout_used - 1;
			}
		}
	}

	/* To be extra safe, double-check using the vanilla 
	 * feasible_routing() */
	assert(feasible_routing() == (*opins_overuse == 0));

	return (*opins_overuse == 0) && (*fanout_overuse == 0);
}

/**
 * inc_update_cost() penalizes incremental nets
 * that have infeasible OPINs 
 */
static void 
inc_update_cost(
		int num_trace_nets, 
		int *trace_nets, 
		int old_num_nets,
		float pres_fac,
		float acc_fac,
		int **inode2fanouts)
{
	int i, inet, inode;
	for (i = 0; i < num_trace_nets; i++)
	{
		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		/* If this is a new global net */
		if (inet >= old_num_nets)
		{
			inode = trace_head[inet]->index;
			if (!inc_feasible_outpin(inet, old_num_nets))
			{
				/* Penalize as if it was over capacity by one */
				rr_node_route_inf[inode].acc_cost += acc_fac;
			}
		}
	}

	for (inode = 0; inode < num_rr_nodes; inode++)
	{
		/* Allow multiple fan-out if only one occupancy */
		if (inode2fanouts[inode])
		{
			int iedge, fanout_used;

			fanout_used = 0;
			for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
			{
				if (inode2fanouts[inode][iedge] > 0) fanout_used++;
			}

			if (fanout_used > 1)
			{
				assert(rr_node[inode].inc_occ > 1);

				/* Same penalty as pathfinder_update_one_cost() */
				rr_node_route_inf[inode].acc_cost += 
					(fanout_used - 1) * acc_fac;
				rr_node_route_inf[inode].pres_cost +=
					1. + fanout_used * pres_fac;
			}
		}
		/* For trace-buffer sinks, set their cost to the incremental occupancy
		 * to try and encourage nets to spread out a bit? */
		else if (rr_node[inode].type == SINK)
		{
			if (rr_node[inode].inc_occ > 0)
			{
				rr_node_route_inf[inode].pres_cost = rr_node[inode].inc_occ;
			}
		}
	}

}

/**
 * Undoes what inc_add_clb_net() does 
 */
void 
inc_remove_clb_net(int inet)
{
	int iblk, ptc;

	/* Restore all data structures */
	free_traceback(inet);

	/*assert(trace_head[inet] == NULL);*/
	/*assert(trace_tail[inet] == NULL);*/
	assert(vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[inet]] == inet);
	vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[inet]] = OPEN;
	iblk = clb_net[inet].node_block[0];
	ptc = clb_net[inet].node_block_pin[0];
	assert(block[iblk].pb->rr_graph[ptc].net_num = clb_to_vpack_net_mapping[inet]);
	block[iblk].pb->rr_graph[ptc].net_num = OPEN;
	assert(block[iblk].pb->rr_graph[ptc].prev_node != OPEN);
	block[iblk].pb->rr_graph[ptc].prev_node = OPEN;
	assert(block[iblk].pb->rr_graph[ptc].prev_edge != OPEN);
	block[iblk].pb->rr_graph[ptc].prev_edge = OPEN;
}

/**
 * Resolve congestion by iteratively (according to inet) 
 * discarding each net if it has any over-used resources,
 * until a valid solution remains 
 */
static void 
inc_resolve_congestion(
		int num_trace_nets, 
		int *trace_nets, 
		struct s_trace **old_trace_tail,
		int old_num_nets,
		struct s_router_opts *router_opts,
		int *num_failed_nets,
		int *num_failed_seg,
		int **inode2fanouts,
		t_ivec **clb_opins_used_locally,
		float pres_fac)
{
	int i, inet, inode;
	bool global;

	/* Determine which inode fanout edge to keep */
	int *keep_inode_iedge = (int *) calloc(num_rr_nodes, sizeof(int));
	for (inode = 0; inode < num_rr_nodes; inode++)
	{
		int iedge, iedge_max;
		if (inode2fanouts[inode] == NULL) continue;

		/* Find the first edge with the max fanout */
		iedge_max = 0;
		for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
		{
			if (inode2fanouts[inode][iedge] > inode2fanouts[inode][iedge_max])
				iedge_max = iedge;
		}
		keep_inode_iedge[inode] = iedge_max;
	}

	/* Go through each trace net */
	for (i = 0; i < num_trace_nets; i++)
	{
		int ndiscarded;
		struct s_trace *tptr, *tptr_last_srcsnk;
		bool opin_overuse, discard_seg;

		inet = trace_nets[i];
		if (inet == OPEN)
			continue;

		global = inet < old_num_nets;
		if (global)
		{
			tptr = old_trace_tail[inet]->next->next;
			tptr_last_srcsnk = old_trace_tail[inet];
		}
		else
		{
			tptr = trace_head[inet];
			tptr_last_srcsnk = tptr;
		}
		assert(tptr);

		/* Find out if net is overused */
		opin_overuse = (!global && !inc_feasible_outpin(inet, old_num_nets));
		assert(opin_overuse == FALSE);
		discard_seg = FALSE;
		ndiscarded = 0;

		/* Remove rr_node.occ */
		pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
		/* Remove rr_node.inc_occ */
		if (global)
		{
			if (old_trace_tail[inet]->next)
				inc_update_one_cost(old_trace_tail[inet]->next->next, -1, inode2fanouts, pres_fac);
		}
		else
		{
			inc_update_one_cost(trace_head[inet], -1, inode2fanouts, pres_fac);
		}

		if (!opin_overuse)
		{
			while (tptr)
			{
				inode = tptr->index;
				assert(inode != OPEN);

				/* If SOURCE or OPIN is overused */
				if ((rr_node[inode].occ+1) > rr_node[inode].capacity)
				{
					assert(rr_node[inode].type == SOURCE || rr_node[inode].type == OPIN);
					opin_overuse = TRUE;
					break;
				}

				/* If this node has >1 fanouts, then discard this net
				 * only if it's not the maximum */
				if (inode2fanouts[inode] && !discard_seg)
				{
					int iedge, inode_next;
					assert(tptr->next);
					inode_next = tptr->next->index;
					for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
					{
						if (rr_node[inode].edges[iedge] == inode_next) break;
					}
					assert(iedge < rr_node[inode].num_edges);
					assert(inode2fanouts[inode][iedge] >= 0);

					/* If this edge isn't the one we want to keep,
					 * discard it */
					if (iedge != keep_inode_iedge[inode])
					{
						discard_seg = TRUE;
					}
				}

				if (rr_node[inode].type == SINK)
				{
					struct s_trace *t;
					if (discard_seg)
					{
						struct s_trace *tn;
						/* Now free the traceback between last src/snk
						 * and the current snk (inclusive) */
						t = tptr_last_srcsnk->next;
						tn = t->next;
						do {
							free_trace_data(t);
							t = tn;
							tn = t->next;
						} while (t != tptr);
						free_trace_data(tptr);

						/* Set the last src/snk next pointer to the 
						 * next pointer of this snk */
						tptr_last_srcsnk->next = tn;
						tptr = tn;
						/*discard_seg = FALSE;*/
						ndiscarded++;
					}
					else
					{
						tptr_last_srcsnk = tptr;
						tptr = tptr->next;
					}
					if (tptr == NULL) break;
					assert(rr_node[tptr->index].type != SOURCE && rr_node[tptr->index].type != SINK);

					/* Check that tptr exists in this traceback,
					 * otherwise, discard that too */
					t = trace_head[inet];
					discard_seg = TRUE;
					while(t && t != tptr)
					{
						if (tptr->index == t->index)
						{
							discard_seg = FALSE;
							break;
						}
						t = t->next;
					}
				}
				tptr = tptr->next;
			}
		} /* if overuse */

		/* If net is overused in any way */
		if (opin_overuse || ndiscarded > 0)
		{
			if (opin_overuse)
			{
				fprintf(stdout, "Abandoning trace_nets[%d]: vnet=%d inet=%d due to infeasible OPIN!\n", 
						i, clb_to_vpack_net_mapping[inet], inet);
				ndiscarded = router_opts->inc_connectivity;
			}
			else
			{
				fprintf(stdout, "Discarding %d segments(s) from trace_nets[%d]: vnet=%d inet=%d due to fanout congestion!\n", 
						ndiscarded, i, clb_to_vpack_net_mapping[inet], inet);
			}

			if (global)
			{
				/* Only remove the incremental (and not the user) trace */
				if (opin_overuse && old_trace_tail[inet]->next)
				{
					inc_free_traceback(inet, old_trace_tail);
					assert(trace_tail[inet] == old_trace_tail[inet]);
					assert(trace_tail[inet]->next == NULL);
				}

				if (opin_overuse || !old_trace_tail[inet]->next)
				{
					trace_nets[i] = OPEN;
					(*num_failed_nets)++;
					*num_failed_seg += router_opts->inc_connectivity;
				}
			}
			else
			{
				if (opin_overuse || !trace_head[inet]->next)
				{
					inc_remove_clb_net(inet);

					trace_nets[i] = OPEN;
					(*num_failed_nets)++;
				}
			}
			*num_failed_seg += ndiscarded;
		}

		/* Add rr_node.occ */
		pathfinder_update_one_cost(trace_head[inet], 1, pres_fac);
		if (global)
		{
			if (old_trace_tail[inet]->next)
				inc_update_one_cost(old_trace_tail[inet]->next->next, 1, inode2fanouts, pres_fac);
		}
		else
		{
			inc_update_one_cost(trace_head[inet], 1, inode2fanouts, pres_fac);
		}
	}

	/* Double check that everything is feasible now */
	recompute_occupancy_from_scratch(clb_opins_used_locally);
	assert(feasible_routing());

	free(keep_inode_iedge);
}

/**
 * Convert a vpack (local) net into a new clb (global) net
 */
static int 
inc_add_clb_net(int vnet, const int new_inet)
{
	int iblk, ptc, inode, i, clb_inet;
	/* Create a new clb_net element */
	clb_net = (s_net *) realloc(clb_net, (new_inet+1)*sizeof(struct s_net));
	clb_net[new_inet].name = (char *) malloc((strlen(vpack_net[vnet].name)+1)*sizeof(char));
	strcpy(clb_net[new_inet].name, vpack_net[vnet].name);
	clb_net[new_inet].num_sinks = 0;
	clb_net[new_inet].node_block = (int *) malloc(sizeof(int));
	clb_net[new_inet].node_block[0] = iblk = vpack_net[vnet].node_block[0];
	assert(iblk != OPEN);
	clb_net[new_inet].node_block_port = NULL;
	clb_net[new_inet].node_block_pin = (int *) malloc(sizeof(int));
	clb_net[new_inet].node_block_pin[0] = ptc = vpack_net[vnet].node_block_pin[0];
	assert(ptc != OPEN);
	clb_net[new_inet].is_global = FALSE;
	clb_net[new_inet].is_const_gen = FALSE;
	/* Create a new clb_to_vpack_mapping element */
	clb_to_vpack_net_mapping = (int *) realloc(clb_to_vpack_net_mapping, (new_inet+1)*sizeof(int));
	clb_to_vpack_net_mapping[new_inet] = vnet;

	/* CLB_OPIN ptc */
	assert(block[iblk].pb->rr_graph[ptc].net_num == OPEN);
	block[iblk].pb->rr_graph[ptc].net_num = vnet;
	assert(block[iblk].pb->rr_graph[ptc].pb_graph_pin->num_input_edges == 1);
	assert(block[iblk].pb->rr_graph[ptc].pb_graph_pin->input_edges[0]->num_input_pins == 1);
	inode = block[iblk].pb->rr_graph[ptc].pb_graph_pin->input_edges[0]
		->input_pins[0]->pin_count_in_cluster;
	assert(block[iblk].pb->rr_graph[ptc].prev_node == OPEN);
	block[iblk].pb->rr_graph[ptc].prev_node = inode;
	assert(block[iblk].pb->rr_graph[inode].net_num == vnet);

	/* Enable the edge that connects the previous inode (BLE_OPIN) to this CLB_OPIN */
	for (i = 0; i < block[iblk].pb->rr_graph[inode].num_edges; i++)
	{
		if (block[iblk].pb->rr_graph[inode].edges[i] == ptc)
			break;
	}
	assert(i < block[iblk].pb->rr_graph[inode].num_edges);
	assert(block[iblk].pb->rr_graph[ptc].prev_edge == OPEN);
	block[iblk].pb->rr_graph[ptc].prev_edge = i;

	clb_inet = new_inet;

	return clb_inet;
}

/**
 * Add all non global vnets to the trace_nets array
 */
int 
inc_setup_trace(int **trace_nets, int *new_num_nets)
{
	int inet, vnet, num_trace_nets;

	*trace_nets = (int *) malloc(sizeof(int));
	*new_num_nets = num_nets;
	num_trace_nets = 0;

	for (vnet = 0; vnet < num_logical_nets; vnet++)
	{
		int iblk;

		inet = vpack_to_clb_net_mapping[vnet];
		if (inet == OPEN)
			iblk = vpack_net[vnet].node_block[0];
		else
			iblk = clb_net[inet].node_block[0];

		if ((inet != OPEN && clb_net[inet].is_global) || iblk == OPEN)
			continue;

		/* If net is local, make it global */
		if (vpack_to_clb_net_mapping[vnet] == OPEN)
		{
			inet = inc_add_clb_net(vnet, *new_num_nets);
			(*new_num_nets)++;
			vpack_to_clb_net_mapping[vnet] = inet;

			assert(clb_net[inet].num_sinks == 0);
			clb_net[inet].node_block = (int *) realloc(clb_net[inet].node_block, 2*sizeof(int));
			clb_net[inet].node_block[1] = OPEN;

			/* Create a new net_rr_terminals if appropriate 
			 * (during post-map) */
			if (net_rr_terminals)
			{
				int /*iblk,*/ ptc, inode;
				/*iblk = clb_net[inet].node_block[0];*/
				assert(iblk == clb_net[inet].node_block[0]);
				ptc = clb_net[inet].node_block_pin[0];
				inode = get_rr_node_index(block[iblk].x, block[iblk].y, 
						SOURCE, block[iblk].type->pin_class[ptc], rr_node_indices);

				net_rr_terminals = (int **) realloc(net_rr_terminals, (*new_num_nets)*sizeof(int *));
				net_rr_terminals[inet] = (int *) malloc(2*sizeof(int));
				net_rr_terminals[inet][0] = inode;
				net_rr_terminals[inet][1] = OPEN;
			}
		}
		else
		{
			int num_sinks;

			inet = vpack_to_clb_net_mapping[vnet];
			num_sinks = clb_net[inet].num_sinks;

			clb_net[inet].node_block = (int *) realloc(clb_net[inet].node_block, (num_sinks+2)*sizeof(int));
			clb_net[inet].node_block[num_sinks+1] = OPEN;

			if (net_rr_terminals)
			{
				int *tmp;

				/* Code below doesn't work because net_rr_terminals[inet] was 
				 * allocated using my_chunk_alloc(); instead copy the entire array */
				/* net_rr_terminals[inet] = realloc(net_rr_terminals[inet], (clb_net[inet].num_sinks+1)*sizeof(int)); */
				tmp = (int*)malloc((num_sinks+2)*sizeof(int));
				memcpy(tmp, net_rr_terminals[inet], (num_sinks+1)*sizeof(int));
				net_rr_terminals[inet] = tmp;
				net_rr_terminals[inet][num_sinks+1] = OPEN;
			}
		}

		/* Add to trace_nets[] */
		*trace_nets = (int *) realloc(*trace_nets, (num_trace_nets+1) * sizeof(int));
		(*trace_nets)[num_trace_nets] = inet;
		num_trace_nets++;
	}
	return num_trace_nets;
}

/** 
 * Recompute a new inode2fanout structure, and check that it is identical 
 * to the one we had before
 */
static void 
inc_check_inode2fanouts_and_verify(
		int *trace_nets,
		int num_trace_nets,
		struct s_trace **old_trace_tail,
		int **_inode2fanouts)
{
	int inode, itrace;
	int **inode2fanouts;
	inode2fanouts = (int **) calloc(num_rr_nodes, sizeof(int*));

	for (itrace = 0; itrace < num_trace_nets; itrace++)
	{
		int inet;
		struct s_trace *tptr;

		inet = trace_nets[itrace];
		if (inet == OPEN) continue;

		tptr = old_trace_tail[inet];
		/* If this is a local-turned-global net, 
		 * then use trace_head instead */
		if (tptr == NULL) tptr = trace_head[inet];
		assert(tptr);

		while(tptr)
		{
			inode = tptr->index;
			assert(inode != OPEN);

			if (rr_node[inode].type == CHANX || rr_node[inode].type == CHANY)
			{
				int next_inode, iedge;
				assert(tptr->next);
				next_inode = tptr->next->index;
				assert(next_inode != OPEN);

				if (inode2fanouts[inode] == NULL)
					inode2fanouts[inode] = (int *) calloc(rr_node[inode].num_edges, sizeof(int));

				for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
				{
					if (rr_node[inode].edges[iedge] == next_inode) break;
				}
				assert(iedge < rr_node[inode].num_edges);

				inode2fanouts[inode][iedge]++;
			}

			if (rr_node[inode].type == SINK)
			{
				tptr = tptr->next;
				if (tptr == NULL) break;
			}

			tptr = tptr->next;
		}
	}

	for (inode = 0; inode < num_rr_nodes; inode++)
	{
		if (inode2fanouts[inode] == NULL)
		{
			if (_inode2fanouts[inode])
			{
				int iedge;
				for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
				{
					assert(_inode2fanouts[inode][iedge] == 0);
				}
			}
			else
				assert(_inode2fanouts[inode] == NULL);
		}
		else
		{
			int iedge;
			for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
			{
				assert(_inode2fanouts[inode][iedge] == inode2fanouts[inode][iedge]);
			}
		}
		free(inode2fanouts[inode]);
	}
	free(inode2fanouts);
}

/**
 * Try and iteratively route the trace nets
 * Adapted from try_<algo>_route()
 */
static bool 
inc_try_trace(
		int *trace_nets,
		const int num_trace_nets, 
		struct s_trace **old_trace_tail,
		const int old_num_nets,
		struct s_router_opts *router_opts, 
		struct s_file_name_opts *FileNameOpts,
		struct s_det_routing_arch det_routing_arch,
		t_timing_inf timing_inf,
		t_trace_buffer *tb,
		int num_tb,
		t_ivec **clb_opins_used_locally,
		float *criticality)
{
	int inet;
	float pres_fac;
	bool success;
	int i, j;
	clock_t begin, end;
	int opins_overuse, fanout_overuse;
	int num_failed_nets, num_failed_seg;
	int inode;

	int **inode2fanouts;

	printf("Performing incremental-routing...\n");
	begin = clock();

	/* Reset accumulated costs stored in rr_node_route_inf */
	inc_reset_rr_node_route_structs();

	inode2fanouts = (int **) calloc(num_rr_nodes, sizeof(int*));
	pres_fac = router_opts->first_iter_pres_fac;

	num_failed_nets = 0;
	num_failed_seg = 0;

	if (router_opts->inc_router_algorithm == READ_ROUTE)
	{
		int itrace;

		printf("Confirming Router Algorithm: Reading from %s.\n", router_opts->inc_route_file);
		inc_read_route(router_opts->inc_route_file);

		for (itrace = 0; itrace < num_trace_nets; itrace++)
		{
			bool global;

			inet = trace_nets[itrace];
			assert(inet != OPEN);

			global = inet < old_num_nets;


			if (global)
			{
				if (old_trace_tail[inet]->next == NULL)
				{
					assert(old_trace_tail[inet] == trace_tail[inet]);
					trace_nets[itrace] = OPEN;
					num_failed_nets++;
				}
				else
				{
					pathfinder_update_one_cost(old_trace_tail[inet]->next->next, 1, pres_fac);
					inc_update_one_cost(old_trace_tail[inet]->next->next, 1, inode2fanouts, pres_fac);
				}
			}
			else
			{
				if (trace_head[inet] == NULL)
				{
					assert(old_trace_tail[inet] == NULL);
					assert(trace_tail[inet] == NULL);

					inc_remove_clb_net(inet);
					trace_nets[itrace] = OPEN;
					num_failed_nets++;
				}
				else
				{
					pathfinder_update_one_cost(trace_head[inet], 1, pres_fac);
					inc_update_one_cost(trace_head[inet], 1, inode2fanouts, pres_fac);
				}
			}
		}

		recompute_occupancy_from_scratch(clb_opins_used_locally);

		inc_check_inode2fanouts_and_verify(trace_nets,
				num_trace_nets,
				old_trace_tail,
				inode2fanouts);

		printf("Incremental-routing successful for %d/%d nets\n", 
				num_trace_nets - num_failed_nets, num_trace_nets);
		success = TRUE;
	}
	else /* (router_opts->inc_router_algorithm != READ_ROUTE) */
	{
		float bend_cost = router_opts->bend_cost;

		/* For every routing iteration */
		for (i = 1; i <= router_opts->max_router_iterations; i++) 
		{
			int nsuccess = 0;
			/* For every trace net */
			for (j = 0; j < num_trace_nets; j++)
			{
				bool global;
				inet = trace_nets[j];
				if (inet == OPEN)
					continue;
				global = inet < old_num_nets;

				/* Remove rr_node.occ */
				pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
				/* Remove rr_node.inc_occ */
				if (global)
				{
					if (old_trace_tail[inet]->next)
						inc_update_one_cost(old_trace_tail[inet]->next->next, -1, inode2fanouts, pres_fac);
				}
				else
				{
					inc_update_one_cost(trace_head[inet], -1, inode2fanouts, pres_fac);
					free_traceback(inet);
				}

				switch(router_opts->inc_router_algorithm)
				{
					case BREADTH_FIRST:
						printf("BREADTH_FIRST not currently supported!");
						exit(1);
						success = inc_breadth_first_route_net(inet, bend_cost, old_trace_tail, old_num_nets, router_opts);
						break;
						/* Difference between (non) timing-driven directed search is that for
						 * non timing-driven, all net criticalities are zero */
					case DIRECTED_SEARCH:
					case TIMING_DRIVEN:
						success = inc_directed_search_route_net(inet, pres_fac, router_opts->astar_fac, bend_cost, 
								old_trace_tail, old_num_nets, router_opts, num_tb, tb, criticality[j], inode2fanouts);
						break;
					default:
						assert(FALSE);
				}

				if (success)
				{
					/* Add rr_node.occ */
					pathfinder_update_one_cost(trace_head[inet], 1, pres_fac);
					/* Add rr_node.inc_occ */
					nsuccess += 1;
					if (global)
						inc_update_one_cost(old_trace_tail[inet]->next->next, 1, inode2fanouts, pres_fac);
					else
						inc_update_one_cost(trace_head[inet], 1, inode2fanouts, pres_fac);
				}
				else
				{
					/* Routing this net is completely impossible: 
					 * if best effort is not enabled bail out here */
					if (!router_opts->inc_best_effort)
					{
						goto end;
					}
					/* Should only be impossible on the first iteration!?! */
					assert(i == 1);

					if (global)
					{
						if (trace_tail[inet] != old_trace_tail[inet])
						{
							/* TODO: At least one connection made... can salvage! */
							inc_free_traceback(inet, old_trace_tail);
						}
						/* Add rr_node.occ */
						assert(trace_tail[inet] == old_trace_tail[inet]);
						pathfinder_update_one_cost(trace_head[inet], 1, pres_fac);
						assert(rr_node[trace_head[inet]->index].occ == 1);
						assert(rr_node[trace_head[inet]->index].inc_occ == 0);
					}
					else
					{
						if (trace_head[inet] == NULL)
						{
							inc_remove_clb_net(inet);
						}
						else
						{
							assert(rr_node[trace_head[inet]->index].occ == 0);
							assert(rr_node[trace_head[inet]->index].inc_occ == 0);
							/* TODO: Salvage */
							inc_remove_clb_net(inet);
						}
						assert(trace_head[inet] == NULL);
						assert(trace_tail[inet] == NULL);
					}

					/* Give up tracing this net permanently */
					trace_nets[j] = OPEN;
					num_failed_nets++;
				}
			}


			/* Check if incremental trace is feasible */
			success = inc_feasible_routing(num_trace_nets, trace_nets, old_trace_tail, 
					&opins_overuse, &fanout_overuse, old_num_nets, inode2fanouts);
			if (success) goto end;

			if (i == 1)
				pres_fac = router_opts->initial_pres_fac;
			else
				pres_fac *= router_opts->pres_fac_mult;

			pres_fac = std::min(pres_fac, static_cast<float> (HUGE_POSITIVE_FLOAT / 1e5));

			pathfinder_update_cost(pres_fac, router_opts->acc_fac);
			inc_update_cost(num_trace_nets, trace_nets, old_num_nets, 
					pres_fac, router_opts->acc_fac, inode2fanouts);
			fprintf(stdout, "Iteration %d: %d/%d successfully traced but found %d opins over-used, and %d nodes with >1 incremental fanout!\n", 
					i, nsuccess, num_trace_nets, opins_overuse, fanout_overuse);
			fflush(stdout);
		}
		/* Remove one here because for loop would have incremented i to beyond max_router_iterations */
		i--;

		/* If congestion wasn't full resolvable after the set number of iterations,
		 * then use inc_resolve_congestion() which iteratively discards until
		 * legal solution is found */
		fprintf(stdout, "Congestion unresolved after %d iteration(s). Abandon ship...\n", i);
		inc_resolve_congestion(num_trace_nets, trace_nets, old_trace_tail, old_num_nets, router_opts,
				&num_failed_nets, &num_failed_seg, inode2fanouts, clb_opins_used_locally, pres_fac);
		success = inc_feasible_routing(num_trace_nets, trace_nets, old_trace_tail, 
				&opins_overuse, &fanout_overuse, old_num_nets, inode2fanouts);
		assert(success);

end:	
		printf("Incremental-routing ran for %d iteration(s).\n", i);
	}

	if (success)
	{
		if (router_opts->inc_router_algorithm != READ_ROUTE)
		{
			char fn[1024];
			sprintf(fn, "%s.overlay.route", FileNameOpts->CircuitName);
			inc_print_route(fn, old_trace_tail);
		}
		else
		{
			char fn[1024];
			sprintf(fn, "%s.overlay.route_rewrite", FileNameOpts->CircuitName);
			inc_print_route(fn, old_trace_tail);
		}

		/* If a match file has been specified, collapse the
		 * overlay network to the matches given */
		if (router_opts->inc_match_file)
		{
			inc_collapse_match_from_file(router_opts->inc_match_file, 
					trace_nets, num_trace_nets, tb, num_tb, 
					old_num_nets, old_trace_tail, pres_fac,
					inode2fanouts, clb_opins_used_locally);
		}

		/* Make sure there are no holes in clb_net;
		 * holes created by abandoning impossible nets 
		 * during the routing procedure, or during
		 * inc_resolve_congestion() */
		for (i = old_num_nets; i < num_nets; i++)
		{
			if (trace_head[i] == NULL)
			{
				assert(trace_tail[i] == NULL);
				while(trace_head[num_nets-1] == NULL)
				{
					assert(trace_tail[num_nets-1] == NULL);
					num_nets--;
				}
				if (num_nets == i)
					continue;
				assert(num_nets > i);
				assert(trace_head[num_nets-1] != NULL);
				assert(trace_tail[num_nets-1] != NULL);

				trace_head[i] = trace_head[num_nets-1];
				trace_tail[i] = trace_tail[num_nets-1];
				trace_head[num_nets-1] = NULL;
				trace_tail[num_nets-1] = NULL;
				clb_net[i] = clb_net[num_nets-1];
				assert(vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[num_nets-1]] == num_nets-1);
				vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[num_nets-1]] = i;
				clb_to_vpack_net_mapping[i] = clb_to_vpack_net_mapping[num_nets-1];
				clb_to_vpack_net_mapping[num_nets-1] = OPEN;
				assert(vpack_to_clb_net_mapping[clb_to_vpack_net_mapping[i]] == i);
				net_rr_terminals[i] = net_rr_terminals[num_nets-1];
				net_rr_terminals[num_nets-1] = NULL;

				for (j = 0; j < num_trace_nets; j++)
				{
					if (trace_nets[j] == (num_nets-1))
					{
						trace_nets[j] = i;
						break;
					}
				}
				assert(j < num_trace_nets);
				num_nets--;
			}
		}

		for (i = old_num_nets; i < num_nets; i++)
		{
			assert(trace_head[i] != NULL);
			assert(trace_tail[i] != NULL);
		}

		inc_check_inode2fanouts_and_verify(trace_nets,
				num_trace_nets,
				old_trace_tail,
				inode2fanouts);

		/* Check that all nets are traced */
		for (i = 0; i < num_trace_nets; i++)
		{
			inet = trace_nets[i];
			if (inet == OPEN)
				continue;
			assert(inet < num_nets);
			assert(trace_head[inet] != NULL);
			assert(trace_tail[inet] != NULL);
		}

		/* Connect them up */
		inc_connect_trace(trace_nets, num_trace_nets, old_num_nets);

		check_route(router_opts->route_type,
				det_routing_arch.num_switch,
				clb_opins_used_locally);

		/* Double check that all wiring on the user net has capacity one too */
		for (i = 0; i < old_num_nets; i++)
		{
			struct s_trace *tptr;
			tptr = trace_head[i];
			while(tptr != old_trace_tail[i])
			{
				inode = tptr->index;
				if (rr_node[inode].type == CHANX || rr_node[inode].type == CHANY)
				{
					assert(rr_node[inode].capacity == 1);
				}
				tptr = tptr->next;
			}

		}
	}

	for (inode = 0; inode < num_rr_nodes; inode++)
	{
		free(inode2fanouts[inode]);
	}
	free(inode2fanouts);

	end = clock();
#ifdef CLOCKS_PER_SEC
	printf("Incremental-routing took %g seconds\n", (float)(end - begin) / CLOCKS_PER_SEC);
#else
	printf("Incremental-routing took %g seconds\n", (float)(end - begin) / CLK_PER_SEC);
#endif

	if (success)
	{
		if (router_opts->inc_router_algorithm != READ_ROUTE) 
		{
			printf("Incremental-routing successful for %d/%d nets (%d/%d segments)\n", 
					num_trace_nets - num_failed_nets, num_trace_nets,
					(num_trace_nets * router_opts->inc_connectivity) - num_failed_seg,
					num_trace_nets * router_opts->inc_connectivity);
		}
	}
	else
	{
		printf("Incremental-routing failed!\n");
	}
	fflush(stdout);

	return success;
}

static
int **inc_tb_to_nets(	
		int *trace_nets,
		int num_trace_nets,
		t_trace_buffer *tb,
		int num_tb,
		int ****_tb_nets)
{
	int itrace, itb, inode;
	int ***tb_nets, **num_tb_nets;

	tb_nets = (int ***) calloc(num_tb, sizeof(int**));
	num_tb_nets = (int **) calloc(num_tb, sizeof(int*));
	for (itb = 0; itb < num_tb; itb++)
	{
		const int tb_width = tb[itb].capacity;
		tb_nets[itb] = (int **) calloc(tb_width, sizeof(int*));
		num_tb_nets[itb] = (int *) calloc(tb_width, sizeof(int));
	}

	/* First scan through all trace-backs looking for
	 * incrementally-connected SINKs */
	for (itrace = 0; itrace < num_trace_nets; itrace++)
	{
		int inet;
		struct s_trace *tptr;
		inet = trace_nets[itrace];
		if (inet == OPEN) 
			continue;
		tptr = trace_head[inet];
		while (tptr)
		{
			inode = tptr->index;
			if (rr_node[inode].type == SINK &&
					rr_node[inode].inc_occ > 0)
			{
				for (itb = 0; itb < num_tb; itb++)
				{
					if (rr_node[inode].xlow == tb[itb].x &&
							rr_node[inode].ylow == tb[itb].y)
					{
						/* If found, add this net to the table */
						int ipin, num;
						ipin = rr_node[inode].ptc_num - tb[itb].data_pin;
						num = num_tb_nets[itb][ipin];
						tb_nets[itb][ipin] = (int *) realloc(tb_nets[itb][ipin], (num+1)*sizeof(int));
						tb_nets[itb][ipin][num] = clb_to_vpack_net_mapping[inet];
						num_tb_nets[itb][ipin]++;
					}
				}
			}
			tptr = tptr->next;
		}
	}

	*_tb_nets = tb_nets;
	return num_tb_nets;
}

/**
 * Write out the *.overlay file, which specifies the vnets that
 * can reach every RAM ipin 
 */
static void 
inc_write_overlay(	
		char *fn,
		int num_tb,
		t_trace_buffer *tb,
		int ***tb_nets,
		int **num_tb_nets)
{
	FILE *fp;
	int itb;

	fp = fopen(fn, "w");
	assert(fp);

	/* Calculate the number of times each net is connected */
	for (itb = 0; itb < num_tb; itb++)
	{
		int ipin;
		for (ipin = 0; ipin < tb[itb].capacity; ipin++)
		{
			int nets, iedge;
			if (num_tb_nets[itb] != NULL)
			{
				nets = num_tb_nets[itb][ipin];
				if (nets > 0) 
				{
					for (iedge = 0; iedge < nets; iedge++)
					{
						int vnet;
						vnet = tb_nets[itb][ipin][iedge];
						/*fprintf(fp, "%s,", vpack_net[vnet].name);*/
						fprintf(fp, "%d,", vnet);
					}
				}
			}
			fprintf(fp, "\n");
		}
	}
	fclose(fp);
}


/** 
 *************** Main entry point for inserting overlay network ********************
 * */
bool 
inc_trace(
		int *trace_nets, 
		int num_trace_nets,
		int new_num_nets,
		t_trace_buffer *tb,
		int num_tb,
		struct s_router_opts *router_opts, 
		struct s_det_routing_arch det_routing_arch,
		t_timing_inf timing_inf,
		struct s_file_name_opts *FileNameOpts,
		t_ivec **clb_opins_used_locally,
		float **net_slack,
		float T_crit)
{
	bool success;
	int old_num_nets;
	float *criticality;
	int inet, itrace;
	struct s_trace **old_trace_tail;

	alloc_and_load_rr_node_route_structs();

	old_num_nets = num_nets;
	num_nets = new_num_nets;


	/* Copy the existing trace_tail structure  */
	old_trace_tail = (struct s_trace **)malloc(num_nets * sizeof(struct s_trace *));
	memcpy(old_trace_tail, trace_tail, old_num_nets * sizeof(struct s_trace *));

	/* Reallocate trace_head and trace_tail structures */
	trace_head = (s_trace**) realloc(trace_head, num_nets*sizeof(struct s_trace*));
	trace_tail = (s_trace**) realloc(trace_tail, num_nets*sizeof(struct s_trace*));

	/* Zero out all the new trace back structures */
	for (inet = old_num_nets; inet < num_nets; inet++)
	{
		trace_head[inet] = NULL;
		trace_tail[inet] = NULL;
		old_trace_tail[inet] = NULL;
	}

	criticality = (float *) calloc(num_trace_nets, sizeof(float));
	/* Compute the criticality of each trace net for TIMING_DRIVEN,
	 * otherwise criticality = 0. */
	if (router_opts->inc_router_algorithm == TIMING_DRIVEN)
	{
		for (itrace = 0; itrace < num_trace_nets; itrace++)
		{
			int num_sinks;
			inet = trace_nets[itrace];
			num_sinks = clb_net[inet].num_sinks;

			/* Adapted from timing_driven_route_net() */
			if (inet < old_num_nets)
			{
				int ipin;
				for (ipin = 1; ipin <= num_sinks; ipin++)
				{
					float pin_crit;
					pin_crit = std::max(router_opts->max_criticality - net_slack[inet][ipin] / T_crit, static_cast<float> (0.));
					pin_crit = pow(pin_crit, router_opts->criticality_exp);
					pin_crit = std::min(pin_crit, router_opts->max_criticality);
					criticality[itrace] = std::max(criticality[itrace], pin_crit);
				}
			}
			else
			{
				int iblk, ptc;
				t_tnode *ttnode;
				float pin_crit, slack;
				assert(num_sinks == 0);
				iblk = clb_net[inet].node_block[0];
				/* CLB OPIN ptc */
				ptc = clb_net[inet].node_block_pin[0];
				/* BLE OPIN ptc */
				ptc = block[iblk].pb->rr_graph[ptc].prev_node;
				assert(ptc != OPEN);
				ttnode = block[iblk].pb->rr_graph[ptc].tnode;
				slack = ttnode->T_req - ttnode->T_arr;

				pin_crit = std::max(router_opts->max_criticality - slack / T_crit, static_cast<float> (0.));
				pin_crit = pow(pin_crit, router_opts->criticality_exp);
				pin_crit = std::min(pin_crit, router_opts->max_criticality);
				criticality[itrace] = pin_crit;
			}
		}
	}

	/* Mark all available memories as targets */
	inc_mark_targets(num_tb, tb);

	/* Start tracing */
	success = inc_try_trace(trace_nets,
			num_trace_nets,
			old_trace_tail,
			old_num_nets,
			router_opts, 
			FileNameOpts, 
			det_routing_arch, 
			timing_inf, 
			tb, 
			num_tb,
			clb_opins_used_locally,
			criticality);

	/* Write out overlay map -- which nets are connected 
	 * to which pins (determined by line number) */
	if (success && !router_opts->inc_match_file)
	{
		int itb;
		int ***tb_nets, **num_tb_nets;
		char fn[1024];

		num_tb_nets = inc_tb_to_nets(	trace_nets,
				num_trace_nets,
				tb,
				num_tb,
				&tb_nets);
		sprintf(fn, "%s.overlay", FileNameOpts->CircuitName);
		inc_write_overlay(	fn,
				num_tb,
				tb,
				tb_nets, 
				num_tb_nets);

		for (itb = 0; itb < num_tb; itb++)
		{
			int ipin;
			for (ipin = 0; ipin < tb[itb].capacity; ipin++)
			{
				free(tb_nets[itb][ipin]);
			}
			free(tb_nets[itb]);
			free(num_tb_nets[itb]);
		}
		free(tb_nets);
		free(num_tb_nets);
	}

	free_rr_node_route_structs();
	free(criticality);
	free(old_trace_tail);

	return success;
}
