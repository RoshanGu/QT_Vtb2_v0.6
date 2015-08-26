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
#include "inc_misc.h"
#include "inc_collapse.h"

/**
 * Build the traceback that the match specifies
 */
static boolean
inc_collapse_match( 
		const int *trace_sinks,
		int *trace_nets,
		const int num_trace_nets,
		const int old_num_nets,
		struct s_trace ** const old_trace_tail,
		const float pres_fac,
		int ** const inode2fanouts,
		t_ivec ** const clb_opins_used_locally)
{
	int itrace;

	/* Go through each trace net */
	for (itrace = 0; itrace < num_trace_nets; itrace++)
	{
		struct s_trace *tptr, *tptr_inc_head, *tptr_last_sink;
		int inet, target_inode;
		bool global, discard;

		inet = trace_nets[itrace];
		if (inet == OPEN)
			continue;

		global = inet < old_num_nets;
		if (global)
		{
			tptr_inc_head = old_trace_tail[inet]->next;
		}
		else
		{
			tptr_inc_head = trace_head[inet];
		}
		assert(tptr_inc_head);

		/* Remove rr_node.occ */
		pathfinder_update_one_cost(trace_head[inet], -1, pres_fac);
		/* Remove rr_node.inc_occ */
		if (global)
		{
			if (old_trace_tail[inet]->next)
				inc_update_one_cost(tptr_inc_head->next, -1, inode2fanouts, pres_fac);
		}
		else
		{
			inc_update_one_cost(tptr_inc_head, -1, inode2fanouts, pres_fac);
		}

		target_inode = trace_sinks[itrace];

		/* If this net doesn't connect to a trace-buffer input */
		if (target_inode == OPEN)
		{
			/* Then remove it */
			if (global)
			{
				inc_free_traceback(inet, old_trace_tail);

				assert(trace_tail[inet] == old_trace_tail[inet]);
				assert(trace_tail[inet]->next == NULL);
			}
			else
			{
				inc_remove_clb_net(inet);

				assert(trace_head[inet] == NULL);
				assert(trace_tail[inet] == NULL);
				assert(old_trace_tail[inet] == NULL);
			}
			trace_nets[itrace] = OPEN;
			
			pathfinder_update_one_cost(trace_head[inet], 1, pres_fac);

			continue;
		}
		assert(rr_node[target_inode].type == SINK);

		/* Remove everything after the sink */
		tptr = tptr_inc_head;
		tptr_last_sink = NULL;
		discard = FALSE;

		/* Follow the traceback to find the sink net is matched to */
		while (tptr)
		{
			int inode;
			struct s_trace *tn;

			inode = tptr->index;
			tn = tptr->next;
			
			if (discard)
			{
				free_trace_data(tptr);
			}
			else if (rr_node[inode].type == SINK)
			{
				if (inode == target_inode)
				{
					discard = TRUE;
					trace_tail[inet] = tptr;
					tptr->next = NULL;
				}
				else
				{
					tptr_last_sink = tptr;
				}
			}

			tptr = tn;
		}
		assert(discard);

		while (tptr_last_sink) 
		{
			int inode;
			struct s_trace *tptr_last_last_sink, *tptr_target;
			struct s_trace *tn;

			/* Starting from tptr_inc_head, 
			 * try and find the first instance it is used */
			tptr = tptr_inc_head;
			target_inode = tptr_last_sink->next->index;


			/* Find first occurrence of inode */
			inode = tptr->index;
			tptr_last_last_sink = NULL;
			while (inode != target_inode) {
				tptr = tptr->next;
				assert(tptr);
				inode = tptr->index;
				if (rr_node[inode].type == SINK)
					tptr_last_last_sink = tptr;
			}
			/* If the first instance is after the instance
			 * of the last sink, this means that this is the only 
			 * occurrence, so break */
			if (tptr == tptr_last_sink->next)
			{
				old_trace_tail[inet]->next = tptr;
				break;
			}

			/* Free everything after that point */
			tn = tptr->next;
			tptr_target = tptr_last_sink->next->next;
			tptr->next = tptr_target;
			tptr = tn;
			while (tptr != tptr_target)
			{
				tn = tptr->next;
				free_trace_data(tptr);
				tptr = tn;
				assert(tptr);
			}

			assert(tptr_last_last_sink != tptr_last_sink);
			tptr_last_sink = tptr_last_last_sink;
		}
		

		/* TODO: free from old_trace_tail onwards? */
		if (tptr)
		{
			if (global)
				tptr_inc_head = old_trace_tail[inet]->next;
			else
				tptr_inc_head = trace_head[inet];
		}

		/* Add rr_node.occ back again */
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

		/* Triple check that this is the only incremental sink */
		tptr = tptr_inc_head;
		while(tptr)
		{
			int inode;
			inode = tptr->index;
			if (rr_node[inode].type == SINK)
				break;
			tptr = tptr->next;
		}
		assert(tptr);
		assert(tptr->next == NULL);
		assert(tptr == trace_tail[inet]);
	}

	recompute_occupancy_from_scratch(clb_opins_used_locally);
	/* Double check that everything is feasible now */
	assert(feasible_routing());

	return TRUE;
}

/**
 * Read in a match file and then check it
 */
boolean
inc_collapse_match_from_file( 
		char *fn,
		int *trace_nets,
		int num_trace_nets,
		t_trace_buffer *tb,
		int num_tb,
		int old_num_nets,
		struct s_trace **old_trace_tail,
		float pres_fac,
		int **inode2fanouts,
		t_ivec **clb_opins_used_locally)
{
	FILE *fp;
	char *line;
	int itrace;
	int *trace_sinks;
	int matches;

	fp = fopen(fn, "r");
	assert(fp);

	line = (char *) malloc(1024*1024*sizeof(char));
	trace_sinks = (int *) malloc(num_trace_nets*sizeof(int));
	for (itrace = 0; itrace < num_trace_nets; ++itrace)
	{
		trace_sinks[itrace] = OPEN;
	}

	/* Get the first line */
	line = fgets(line, 1024*1024, fp);
	assert(line);

	matches = 0;
	while(line)
	{
		char *pch;
		pch = strtok(line, "-\n");	
		if (pch) 
		{
			int vnet, inet, itb, ipin;
			vnet = atoi(pch);
			assert(vnet < num_logical_nets);
			inet = vpack_to_clb_net_mapping[vnet];
			assert(inet != OPEN);
			pch = strtok(NULL, ".");	
			assert(pch);
			itb = atoi(pch);
			assert(itb < num_tb);
			pch = strtok(NULL, "\n");	
			assert(pch);
			ipin = atoi(pch);
			assert(ipin < tb[itb].capacity);

			/*printf("%d - %d.%d\n", vnet, itb, ipin);*/
			for (itrace = 0; itrace < num_trace_nets; ++itrace)
			{
				/* Find the itrace */
				if (trace_nets[itrace] == inet)
				{
					int inode;
					assert(trace_sinks[itrace] == OPEN);
					inode = get_rr_node_index(tb[itb].x, tb[itb].y, SINK, 
						ipin+tb[itb].data_pin, rr_node_indices);
					assert(rr_node[inode].xlow == tb[itb].x);
					assert(rr_node[inode].ylow == tb[itb].y);
					assert(rr_node[inode].type == SINK);
					/* Assign the itrace to the trace-buffer sink inode */
					trace_sinks[itrace] = inode;
					break;
				}
			}
			assert(itrace < num_trace_nets);
			matches++;
		}

		line = fgets(line, 1024*1024, fp);
	}
	free(line);
	fclose(fp);

	printf("\nRead %d net-to-trace match(es) from %s\n", matches, fn);
	inc_collapse_match(trace_sinks, trace_nets, num_trace_nets,
			old_num_nets, old_trace_tail,
			pres_fac, inode2fanouts, clb_opins_used_locally);
	printf("Match collapsed!\n\n");


	return TRUE;
}
