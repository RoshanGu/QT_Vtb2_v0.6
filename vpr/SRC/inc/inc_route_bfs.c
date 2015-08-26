#include <stdio.h>
#include "vpr_types.h"
#include "globals.h"
#include "mst.h"
#include "route_common.h"
#include "route_breadth_first.h"
#include "inc_trace.h"
#include "inc_route.h"
#include "inc_route_bfs.h"
#include <assert.h>

/** 
 * Adapted from breadth_first_expand_neighbours 
 */
	static void
inc_breadth_first_expand_neighbours(int inode,
		float pcost,
		int inet,
		float bend_cost)
{

	/* Puts all the rr_nodes adjacent to inode on the heap.  rr_nodes outside   *
	 * the expanded bounding box specified in route_bb are not added to the     *
	 * heap.  pcost is the path_cost to get to inode.                           */

	int iconn, to_node, num_edges;
	t_rr_type from_type, to_type;
	float tot_cost;

	num_edges = rr_node[inode].num_edges;
	for(iconn = 0; iconn < num_edges; iconn++)
	{
		to_node = rr_node[inode].edges[iconn];

		/*	    
			    if(rr_node[to_node].xhigh < route_bb[inet].xmin || 
			    rr_node[to_node].xlow > route_bb[inet].xmax || 
			    rr_node[to_node].yhigh < route_bb[inet].ymin || 
			    rr_node[to_node].ylow > route_bb[inet].ymax) 
			    continue;*/	/* Node is outside (expanded) bounding box. */

		/* EH: Ignore nodes that are already fully occupied by
		 * the user circuit */
		if((rr_node[to_node].occ-rr_node[to_node].inc_occ) >= rr_node[to_node].capacity)
			continue;

		tot_cost = pcost + get_rr_cong_cost(to_node);

		if(bend_cost != 0.)
		{
			from_type = rr_node[inode].type;
			to_type = rr_node[to_node].type;
			if((from_type == CHANX && to_type == CHANY) ||
					(from_type == CHANY && to_type == CHANX))
				tot_cost += bend_cost;
		}

		node_to_heap(to_node, tot_cost, inode, iconn, OPEN, OPEN);
	}
}


/**
 * Adapted from breadth_first_add_source_to_heap() 
 */
static void inc_breadth_first_add_inode_to_heap(int inode)
{
	float cost;
	cost = get_rr_cong_cost(inode);
	node_to_heap(inode, cost, NO_PREVIOUS, NO_PREVIOUS, OPEN, OPEN);
}

/**
 * Adapted from breadth_first_route_net() 
 */
boolean inc_breadth_first_route_net(int inet, float bend_cost, struct s_trace **old_trace_tail, int old_num_nets, struct s_router_opts *router_opts)
{
	/* Uses a maze routing (Dijkstra's) algorithm to route a net.  The net       *
	 * begins at the net output, and expands outward until it hits a target      *
	 * pin.  The algorithm is then restarted with the entire first wire segment  *
	 * included as part of the source this time.  For an n-pin net, the maze     *
	 * router is invoked n-1 times to complete all the connections.  Inet is     *
	 * the index of the net to be routed.  Bends are penalized by bend_cost      *
	 * (which is typically zero for detailed routing and nonzero only for global *
	 * routing), since global routes with lots of bends are tougher to detailed  *
	 * route (using a detailed router like SEGA).                                *
	 * If this routine finds that a net *cannot* be connected (due to a complete *
	 * lack of potential paths, rather than congestion), it returns FALSE, as    *
	 * routing is impossible on this architecture.  Otherwise it returns TRUE.   */

	int inode, prev_node, remaining_connections_to_sink;
	int itarget, target_inode, target_iblk;
	int max_mdist;
	float pcost, new_pcost;
	struct s_heap *current;
	struct s_trace *tptr;

	/* Rip up incremental trace only */
	inc_free_traceback(inet, old_trace_tail);

	inode = net_rr_terminals[inet][0];

	/* EH: Extract the assigned TB target */
	itarget = clb_net[inet].num_sinks;
	target_inode = net_rr_terminals[inet][itarget+1];
	assert(target_inode != OPEN);
	target_iblk = clb_net[inet].node_block[itarget+1];

	/* If LE symmetry enabled, and inet was local, add all CLB_OPINs 
	 * from other local nets onto heap too */
	if (router_opts->inc_le_symmetry && inet >= old_num_nets)
	{
		int num_sources, *sources;
		int i;
		boolean found = FALSE;
		num_sources = inc_local_OPINs(inet, old_num_nets, &sources);
		for (i = 0; i < num_sources; i++)
		{
			inc_breadth_first_add_inode_to_heap(sources[i]);
			if (sources[i] == inode)
				found = TRUE;
		}
		assert(found == TRUE);
		free(sources);
	}
	else
	{
		breadth_first_add_source_to_heap(inet);
	}
	/*mark_ends(inet); */

	tptr = trace_head[inet];
	remaining_connections_to_sink = 0;
	max_mdist = 0;

	/*    for(i = 1; i <= clb_net[inet].num_sinks; i++) */
	{			/* Need n-1 wires to connect n pins */
		breadth_first_expand_trace_segment(tptr,
				remaining_connections_to_sink);
		current = get_heap_head();

		if(current == NULL)
		{		/* Infeasible routing.  No possible path for net. */
			reset_path_costs();	/* Clean up before leaving. */
			return (FALSE);
		}

		inode = current->index;

		/*while(rr_node_route_inf[inode].target_flag == 0)*/
		while(rr_node_route_inf[inode].target_flag != target_iblk)
		{
			/*max_mdist = max(max_mdist, inc_traceback_len(current));*/
			pcost = rr_node_route_inf[inode].path_cost;
			new_pcost = current->cost;
			if(pcost > new_pcost)
			{	/* New path is lowest cost. */
				rr_node_route_inf[inode].path_cost = new_pcost;
				prev_node = current->u.prev_node;
				rr_node_route_inf[inode].prev_node = prev_node;
				rr_node_route_inf[inode].prev_edge =
					current->prev_edge;

				if(pcost > 0.99 * HUGE_POSITIVE_FLOAT)	/* First time touched. */
					add_to_mod_list(&rr_node_route_inf[inode].
							path_cost);

				inc_breadth_first_expand_neighbours(inode, new_pcost,
						inet, bend_cost);
			}

			free_heap_data(current);
			current = get_heap_head();

			if(current == NULL)
			{	/* Impossible routing. No path for net. */
				fprintf(stdout, "\ninet %d (vnet %d) failed", inet, clb_to_vpack_net_mapping[inet]);
				empty_heap();
				reset_path_costs();
				return (FALSE);
			}

			inode = current->index;
		}

		/*rr_node_route_inf[inode].target_flag--;*/	/* Connected to this SINK. */
		remaining_connections_to_sink =
			rr_node_route_inf[inode].target_flag;

		tptr = update_traceback(current, inet);
		free_heap_data(current);
	}

	/*printf("max_mdist = %d\n", max_mdist);*/
	empty_heap();
	reset_path_costs();

	return (TRUE);
}


