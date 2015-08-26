#include <stdio.h>
#include "vpr_types.h"
#include "globals.h"
#include "mst.h"
#include "route_common.h"
#include "route_breadth_first.h"
#include "inc_trace.h"
#include "inc_route.h"
#include "inc_route_bfs.h"
#include "rr_graph2.h"


#include <signal.h>
#include <assert.h>
#include <string.h>
#include <limits.h>

/** 
 * Free incremental traceback of inet
 * Adapted from free_traceback()
 */
void
inc_free_traceback(int inet, struct s_trace **old_trace_tail)
{
	struct s_trace *cnet, *nnet;

	trace_tail[inet] = old_trace_tail[inet];
	if (trace_tail[inet] == NULL) return;

	cnet = trace_tail[inet]->next;

	while(cnet)
	{
		nnet = cnet->next;
		free_trace_data(cnet);
		cnet = nnet;
	}

	trace_tail[inet]->next = NULL;
}

/**
 * Return all local OPINs in the cluster
 */
int
inc_local_OPINs(int inet, int old_num_nets, int **sources)
{
	int iblk, ptc, vnet;
	int ipin;
    	int nbles, ible;
	int inode;
	int num_sources;

    	iblk = clb_net[inet].node_block[0];
	ipin = clb_net[inet].node_block_pin[0];
	if (inet < old_num_nets)
	{
		vnet = block[iblk].pb->rr_graph[ipin].net_num;
		assert(vpack_to_clb_net_mapping[vnet] == inet);
	}
	num_sources = 0;

    	assert(strcmp(type_descriptors[2].name, "clb") == 0);
    	if (block[iblk].type->index == 2)
    	{
		int ninpins, noutpins;
    		assert(block[iblk].pb->pb_graph_node->pb_type->num_pb == 1);
    		assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 1);
    		assert(block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children == 1);
    		nbles = block[iblk].pb->pb_graph_node->pb_type->modes[0].pb_type_children[0].num_pb;
    		assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->num_input_ports == 1);
		ninpins = block[iblk].pb->child_pbs[0][0].pb_graph_node->num_input_pins[0];
  		assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->num_output_ports == 1);
		noutpins = block[iblk].pb->child_pbs[0][0].pb_graph_node->num_output_pins[0];
		
		*sources = (int *) malloc(sizeof(int) * nbles * noutpins);

    		for (ible = 0; ible < nbles; ible++)
    		{
			int used_inputs;
			bool fractured;

    			assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_input_ports == 1);
			assert(ninpins == block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_input_pins[0]);

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
			fractured = used_inputs < ninpins;

    			assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_ports == 1);
			assert(noutpins == block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_pins[0]);

			/* Fracture{d,able} */
			if (fractured)
			{
				int new_sources;
				boolean source_ble;
				new_sources = 0;
				source_ble = FALSE;
				for (ipin = 0; ipin < noutpins; ipin++)
				{
    					/* BLE OPIN ptc */
    					ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
    						.pin_count_in_cluster;
    					vnet = block[iblk].pb->rr_graph[ptc].net_num;

					/* if this is our source ble */
					if (clb_to_vpack_net_mapping[inet] == vnet)
					{
						source_ble = TRUE;
					}
    					/* if this net is a local net (has no global net, or has a global net which we created for tracing) */
					if (vnet == OPEN || clb_to_vpack_net_mapping[inet] == vnet || vpack_to_clb_net_mapping[vnet] == OPEN || vpack_to_clb_net_mapping[vnet] >= old_num_nets) 
    					{
    						assert(block[iblk].pb->pb_graph_node->num_output_ports == 1);
    						/* CLB OPIN */
    						ptc = block[iblk].pb->pb_graph_node->output_pins[0][ible*noutpins+ipin].pin_count_in_cluster;

    						/* get rr_node index of CLB_OPIN SOURCE */
    						inode = get_rr_node_index(block[iblk].x, block[iblk].y, 
    							SOURCE, block[iblk].type->pin_class[ptc], rr_node_indices);

						assert((rr_node[inode].occ - rr_node[inode].inc_occ) < rr_node[inode].capacity);

						(*sources)[num_sources + new_sources] = inode;
						new_sources++;
    					}
				}

				/* Only add those sources if ALL pins are available (not global) */
				if (source_ble || new_sources == noutpins)
				{
					num_sources += new_sources;
				}
			}
			else /* Not fractured */
			{
    				/* BLE OPIN ptc */
    				ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][0]
    					.pin_count_in_cluster;
    				vnet = block[iblk].pb->rr_graph[ptc].net_num;

    				/* if this net is a local net (has no global net, or has a global net which we created for tracing) */
    				if (vnet == OPEN || clb_to_vpack_net_mapping[inet] == vnet || vpack_to_clb_net_mapping[vnet] == OPEN || vpack_to_clb_net_mapping[vnet] >= old_num_nets) 
    				{
    					assert(block[iblk].pb->pb_graph_node->num_output_ports == 1);
    					/* CLB OPIN */
    					ptc = block[iblk].pb->pb_graph_node->output_pins[0][ible*noutpins].pin_count_in_cluster;

    					/* get rr_node index of CLB_OPIN SOURCE */
    					inode = get_rr_node_index(block[iblk].x, block[iblk].y, 
    						SOURCE, block[iblk].type->pin_class[ptc], rr_node_indices);

					assert((rr_node[inode].occ - rr_node[inode].inc_occ) < rr_node[inode].capacity);

					(*sources)[num_sources] = inode;
					num_sources++;
    				}
			}
    		}
    	}
	return num_sources;
}

/** 
 * Modify the incremental occupancy (inc_occ) field of the rr_node 
 * Adapted from pathfinder_update_one_cost()
 */
void 
inc_update_one_cost(struct s_trace *route_segment_start,
					int add_or_sub,
					int **inode2fanouts,
					float pres_fac)
{

/* This routine updates the occupancy and pres_cost of the rr_nodes that are *
 * affected by the portion of the routing of one net that starts at          *
 * route_segment_start.  If route_segment_start is trace_head[inet], the     *
 * cost of all the nodes in the routing of net inet are updated.  If         *
 * add_or_sub is -1 the net (or net portion) is ripped up, if it is 1 the    *
 * net is added to the routing.  The size of pres_fac determines how severly *
 * oversubscribed rr_nodes are penalized.                                    */

    struct s_trace *tptr;
    int inode, occ;

    tptr = route_segment_start;
    if(tptr == NULL)		/* No routing yet. */
	return;

    while(tptr)
	{
	    inode = tptr->index;
		assert(inode != OPEN);

	    occ = rr_node[inode].inc_occ + add_or_sub;
	    assert(occ >= 0 && occ <= USHRT_MAX);

	    rr_node[inode].inc_occ = occ;

		if (rr_node[inode].type == CHANX || rr_node[inode].type == CHANY ||
			rr_node[inode].type == IPIN || rr_node[inode].type == OPIN || 
			rr_node[inode].type == SINK)
		{
			assert(rr_node[inode].occ == rr_node[inode].inc_occ);

			/* If this is the first incremental route, extend its capacity */
			if (rr_node[inode].type != OPIN)
			{
				if (add_or_sub > 0)
				{
					if (rr_node[inode].inc_occ == 1)
					{
						assert(rr_node[inode].capacity == 1);
						rr_node[inode].capacity = USHRT_MAX;
					}
				}
				/* But if the last incremental route was removed,
				 * reset capacity to 1 */
				else if (add_or_sub < 0)
				{
					if (rr_node[inode].inc_occ == 0)	
					{
						assert(rr_node[inode].capacity == USHRT_MAX);
						rr_node[inode].capacity = 1;
					}
				}
			}

			/* Update inode2fanouts structure */
			if (rr_node[inode].type == CHANX || rr_node[inode].type == CHANY/* ||
				rr_node[inode].type == OPIN*/)
			{
				int next_inode, iedge, edges_used;

				next_inode = tptr->next->index;
				for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
				{
					if (rr_node[inode].edges[iedge] == next_inode) break;
				}
				assert(iedge < rr_node[inode].num_edges);

				if (inode2fanouts[inode] == NULL)
				{
					inode2fanouts[inode] = (int *) calloc(rr_node[inode].num_edges, sizeof(int));
					assert(add_or_sub > 0);
				}
				inode2fanouts[inode][iedge] += add_or_sub;

				edges_used = 0;
				for (iedge = 0; iedge < rr_node[inode].num_edges; iedge++)
				{
					if (inode2fanouts[inode][iedge] > 0) edges_used++;
				}			

				/*assert(edges_used > 0);*/
				if (edges_used > 1)
				{
					rr_node_route_inf[inode].pres_cost +=
						1. + edges_used * pres_fac;
				}
			}
		}

	    if(rr_node[inode].type == SINK)
		{
		    tptr = tptr->next;	/* Skip next segment. */
		    if(tptr == NULL)
			break;
		}

	    tptr = tptr->next;

	}			/* End while loop -- did an entire traceback. */
}





/** 
 * Adapted from reset_rr_node_route_structs 
 */
void inc_reset_rr_node_route_structs()
{
    int inode;

    assert(rr_node_route_inf != NULL);

    for(inode = 0; inode < num_rr_nodes; inode++)
	{
	    rr_node_route_inf[inode].prev_node = NO_PREVIOUS;
	    rr_node_route_inf[inode].prev_edge = NO_PREVIOUS;
	    rr_node_route_inf[inode].pres_cost = 1.;
	    rr_node_route_inf[inode].acc_cost = 1.;
	    rr_node_route_inf[inode].path_cost = HUGE_POSITIVE_FLOAT;
	    /*rr_node_route_inf[inode].target_flag = 0; */
	    
	    rr_node[inode].inc_occ = 0;
	}
}

/* Macro used below to ensure that fractions are rounded up, but floating   *
 * point values very close to an integer are rounded to that integer.       */

#define ROUND_UP(x) (ceil (x - 0.001))

/**
 * Adapted from get_expected_segs_to_target()
 */
int
inc_get_expected_segs_to_target(int inode, int *num_segs_ortho_dir_ptr,
				int target_x, int target_y)
{

/* Returns the number of segments the same type as inode that will be needed *
 * to reach target_node (not including inode) in each direction (the same    *
 * direction (horizontal or vertical) as inode and the orthogonal direction).*/

    t_rr_type rr_type;
    int num_segs_same_dir, cost_index, ortho_cost_index;
    int no_need_to_pass_by_clb;
    float inv_length, ortho_inv_length, ylow, yhigh, xlow, xhigh;

    /*
    target_x = rr_node[target_node].xlow;
    target_y = rr_node[target_node].ylow;
    */
    cost_index = rr_node[inode].cost_index;
    inv_length = rr_indexed_data[cost_index].inv_length;
    ortho_cost_index = rr_indexed_data[cost_index].ortho_cost_index;
    ortho_inv_length = rr_indexed_data[ortho_cost_index].inv_length;
    rr_type = rr_node[inode].type;

    if(rr_type == CHANX)
	{
	    ylow = rr_node[inode].ylow;
	    xhigh = rr_node[inode].xhigh;
	    xlow = rr_node[inode].xlow;

	    /* Count vertical (orthogonal to inode) segs first. */

	    if(ylow > target_y)
		{		/* Coming from a row above target? */
		    *num_segs_ortho_dir_ptr =
			ROUND_UP((ylow - target_y + 1.) * ortho_inv_length);
		    no_need_to_pass_by_clb = 1;
		}
	    else if(ylow < target_y - 1)
		{		/* Below the CLB bottom? */
		    *num_segs_ortho_dir_ptr = ROUND_UP((target_y - ylow) *
						       ortho_inv_length);
		    no_need_to_pass_by_clb = 1;
		}
	    else
		{		/* In a row that passes by target CLB */
		    *num_segs_ortho_dir_ptr = 0;
		    no_need_to_pass_by_clb = 0;
		}

	    /* Now count horizontal (same dir. as inode) segs. */

	    if(xlow > target_x + no_need_to_pass_by_clb)
		{
		    num_segs_same_dir =
			ROUND_UP((xlow - no_need_to_pass_by_clb -
				  target_x) * inv_length);
		}
	    else if(xhigh < target_x - no_need_to_pass_by_clb)
		{
		    num_segs_same_dir =
			ROUND_UP((target_x - no_need_to_pass_by_clb -
				  xhigh) * inv_length);
		}
	    else
		{
		    num_segs_same_dir = 0;
		}
	}

    else
	{			/* inode is a CHANY */
	    ylow = rr_node[inode].ylow;
	    yhigh = rr_node[inode].yhigh;
	    xlow = rr_node[inode].xlow;

	    /* Count horizontal (orthogonal to inode) segs first. */

	    if(xlow > target_x)
		{		/* Coming from a column right of target? */
		    *num_segs_ortho_dir_ptr =
			ROUND_UP((xlow - target_x + 1.) * ortho_inv_length);
		    no_need_to_pass_by_clb = 1;
		}
	    else if(xlow < target_x - 1)
		{		/* Left of and not adjacent to the CLB? */
		    *num_segs_ortho_dir_ptr = ROUND_UP((target_x - xlow) *
						       ortho_inv_length);
		    no_need_to_pass_by_clb = 1;
		}
	    else
		{		/* In a column that passes by target CLB */
		    *num_segs_ortho_dir_ptr = 0;
		    no_need_to_pass_by_clb = 0;
		}

	    /* Now count vertical (same dir. as inode) segs. */

	    if(ylow > target_y + no_need_to_pass_by_clb)
		{
		    num_segs_same_dir =
			ROUND_UP((ylow - no_need_to_pass_by_clb -
				  target_y) * inv_length);
		}
	    else if(yhigh < target_y - no_need_to_pass_by_clb)
		{
		    num_segs_same_dir =
			ROUND_UP((target_y - no_need_to_pass_by_clb -
				  yhigh) * inv_length);
		}
	    else
		{
		    num_segs_same_dir = 0;
		}
	}

    return (num_segs_same_dir);
}

