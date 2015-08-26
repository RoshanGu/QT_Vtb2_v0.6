#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <string.h>
#include <limits.h>
#include <signal.h>

#include "util.h"
#include "vpr_types.h"
#include "vpr_utils.h"
#include "globals.h"
#include "route_export.h"
#include "route_common.h"
#include "route_tree_timing.h"
#include "route_timing.h"
#include "route_breadth_first.h"
#include "place_and_route.h"
#include "rr_graph.h"
#include "read_xml_arch_file.h"
#include "ReadOptions.h"
#include "rr_graph_util.h"
#include "rr_graph2.h"

#include "check_route.h"
#include <set>

/***************** Variables shared only by route modules *******************/

t_rr_node_route_inf *rr_node_route_inf = NULL; /* [0..num_rr_nodes-1] */

struct s_bb *route_bb = NULL; /* [0..num_nets-1]. Limits area in which each  */

/* net must be routed.                         */

/**************** Static variables local to route_common.c ******************/

static struct s_heap **heap; /* Indexed from [1..heap_size] */
static int heap_size; /* Number of slots in the heap array */
static int heap_tail; /* Index of first unused slot in the heap array */

/* For managing my own list of currently free heap data structures.     */
static struct s_heap *heap_free_head = NULL;
/* For keeping track of the sudo malloc memory for the heap*/
static t_chunk heap_ch = {NULL, 0, NULL};

/* For managing my own list of currently free trace data structures.    */
static struct s_trace *trace_free_head = NULL;
/* For keeping track of the sudo malloc memory for the trace*/
static t_chunk trace_ch = {NULL, 0, NULL};

#ifdef DEBUG
static int num_trace_allocated = 0; /* To watch for memory leaks. */
static int num_heap_allocated = 0;
static int num_linked_f_pointer_allocated = 0;
#endif

static struct s_linked_f_pointer *rr_modified_head = NULL;
static struct s_linked_f_pointer *linked_f_pointer_free_head = NULL;

static t_chunk linked_f_pointer_ch = {NULL, 0, NULL};

/*  The numbering relation between the channels and clbs is:				*
 *																	        *
 *  |    IO     | chan_   |   CLB     | chan_   |   CLB     |               *
 *  |grid[0][2] | y[0][2] |grid[1][2] | y[1][2] | grid[2][2]|               *
 *  +-----------+         +-----------+         +-----------+               *
 *                                                            } capacity in *
 *   No channel           chan_x[1][1]          chan_x[2][1]  } chan_width  *
 *                                                            } _x[1]       *
 *  +-----------+         +-----------+         +-----------+               *
 *  |           |  chan_  |           |  chan_  |           |               *
 *  |    IO     | y[0][1] |   CLB     | y[1][1] |   CLB     |               *
 *  |grid[0][1] |         |grid[1][1] |         |grid[2][1] |               *
 *  |           |         |           |         |           |               *
 *  +-----------+         +-----------+         +-----------+               *
 *                                                            } capacity in *
 *                        chan_x[1][0]          chan_x[2][0]  } chan_width  * 
 *                                                            } _x[0]       *
 *                        +-----------+         +-----------+               *
 *                 No     |           |	   No   |           |               *
 *               Channel  |    IO     | Channel |    IO     |               *
 *                        |grid[1][0] |         |grid[2][0] |               *
 *                        |           |         |           |               *
 *                        +-----------+         +-----------+               *
 *                                                                          *
 *               {=======}              {=======}                           *
 *              Capacity in            Capacity in                          *
 *            chan_width_y[0]        chan_width_y[1]                        *
 *                                                                          */

/******************** Subroutines moved to route_common.h for QT ************/
void free_trace_data(struct s_trace *tptr);  // added Qt 1 - not local anymore
struct s_trace *alloc_trace_data(void); //added qt 2 - not local anymore


/******************** Subroutines local to route_common.c *******************/

static void load_route_bb(int bb_factor);
static void add_to_heap(struct s_heap *hptr);
static struct s_heap *alloc_heap_data(void);
static struct s_linked_f_pointer *alloc_linked_f_pointer(void);

t_ivec **alloc_and_load_clb_opins_used_locally(void);  //added qt 3
static void adjust_one_rr_occ_and_pcost(int inode, int add_or_sub,
		float pres_fac);

static boolean read_route(const char *route_file); //added QT 3

/* EH */
static void
block_routing_nodes(struct s_router_opts *router_opts);
static void
reserve_slice_ipins(void);
static void
reserve_net_ipins(int inet);
static void
reserve_byps(void);

/************************** Subroutine definitions ***************************/

void save_routing(struct s_trace **best_routing,
		t_ivec ** clb_opins_used_locally,
		t_ivec ** saved_clb_opins_used_locally) {

	/* This routing frees any routing currently held in best routing,       *
	 * then copies over the current routing (held in trace_head), and       *
	 * finally sets trace_head and trace_tail to all NULLs so that the      *
	 * connection to the saved routing is broken.  This is necessary so     *
	 * that the next iteration of the router does not free the saved        *
	 * routing elements.  Also saves any data about locally used clb_opins, *
	 * since this is also part of the routing.                              */

	int inet, iblk, iclass, ipin, num_local_opins;
	struct s_trace *tptr, *tempptr;
	t_type_ptr type;

	for (inet = 0; inet < num_nets; inet++) {

		/* Free any previously saved routing.  It is no longer best. */
		tptr = best_routing[inet];
		while (tptr != NULL) {
			tempptr = tptr->next;
			free_trace_data(tptr);
			tptr = tempptr;
		}

		/* Save a pointer to the current routing in best_routing. */
		best_routing[inet] = trace_head[inet];

		/* Set the current (working) routing to NULL so the current trace       *
		 * elements won't be reused by the memory allocator.                    */

		trace_head[inet] = NULL;
		trace_tail[inet] = NULL;
	}

	/* Save which OPINs are locally used.                           */

	for (iblk = 0; iblk < num_blocks; iblk++) {
		type = block[iblk].type;
		for (iclass = 0; iclass < type->num_class; iclass++) {
			num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;
			for (ipin = 0; ipin < num_local_opins; ipin++) {
				saved_clb_opins_used_locally[iblk][iclass].list[ipin] =
						clb_opins_used_locally[iblk][iclass].list[ipin];
			}
		}
	}
}

void restore_routing(struct s_trace **best_routing,
		t_ivec ** clb_opins_used_locally,
		t_ivec ** saved_clb_opins_used_locally) {

	/* Deallocates any current routing in trace_head, and replaces it with    *
	 * the routing in best_routing.  Best_routing is set to NULL to show that *
	 * it no longer points to a valid routing.  NOTE:  trace_tail is not      *
	 * restored -- it is set to all NULLs since it is only used in            *
	 * update_traceback.  If you need trace_tail restored, modify this        *
	 * routine.  Also restores the locally used opin data.                    */

	int inet, iblk, ipin, iclass, num_local_opins;
	t_type_ptr type;

	for (inet = 0; inet < num_nets; inet++) {

		/* Free any current routing. */
		free_traceback(inet);

		/* Set the current routing to the saved one. */
		trace_head[inet] = best_routing[inet];
		best_routing[inet] = NULL; /* No stored routing. */
	}

	/* Save which OPINs are locally used.                           */

	for (iblk = 0; iblk < num_blocks; iblk++) {
		type = block[iblk].type;
		for (iclass = 0; iclass < type->num_class; iclass++) {
			num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;
			for (ipin = 0; ipin < num_local_opins; ipin++) {
				clb_opins_used_locally[iblk][iclass].list[ipin] =
						saved_clb_opins_used_locally[iblk][iclass].list[ipin];
			}
		}

	}
}

void get_serial_num(void) {

	/* This routine finds a "magic cookie" for the routing and prints it.    *
	 * Use this number as a routing serial number to ensure that programming *
	 * changes do not break the router.                                      */

	int inet, serial_num, inode;
	struct s_trace *tptr;

	serial_num = 0;

	for (inet = 0; inet < num_nets; inet++) {

		/* Global nets will have null trace_heads (never routed) so they *
		 * are not included in the serial number calculation.            */

		tptr = trace_head[inet];
		while (tptr != NULL) {
			inode = tptr->index;
			serial_num += (inet + 1)
					* (rr_node[inode].xlow * (nx + 1) - rr_node[inode].yhigh);

			serial_num -= rr_node[inode].ptc_num * (inet + 1) * 10;

			serial_num -= rr_node[inode].type * (inet + 1) * 100;
			serial_num %= 2000000000; /* Prevent overflow */
			tptr = tptr->next;
		}
	}
	vpr_printf(TIO_MESSAGE_INFO, "Serial number (magic cookie) for the routing is: %d\n", serial_num);
}

boolean try_route(int width_fac, struct s_router_opts router_opts,
		struct s_det_routing_arch det_routing_arch, t_segment_inf * segment_inf,
		t_timing_inf timing_inf, float **net_delay, t_slack * slacks,
		t_chan_width_dist chan_width_dist, t_ivec ** clb_opins_used_locally,
		boolean * Fc_clipped, enum e_operation operation, t_direct_inf *directs, struct s_file_name_opts *FileNameOpts, int num_directs,
		const char *arch_file) {

	/* Attempts a routing via an iterated maze router algorithm.  Width_fac *
	 * specifies the relative width of the channels, while the members of   *
	 * router_opts determine the value of the costs assigned to routing     *
	 * resource node, etc.  det_routing_arch describes the detailed routing *
	 * architecture (connection and switch boxes) of the FPGA; it is used   *
	 * only if a DETAILED routing has been selected.                        */

	int tmp;
	clock_t begin, end;
	boolean success;
	t_graph_type graph_type;
	int inet, num_global_nets;

	if (router_opts.route_type == GLOBAL) {
		graph_type = GRAPH_GLOBAL;
	} else {
		graph_type = (
				det_routing_arch.directionality == BI_DIRECTIONAL ?
						GRAPH_BIDIR : GRAPH_UNIDIR);
	}

	/* Set the channel widths */

	init_chan(width_fac, chan_width_dist);

	/* Free any old routing graph, if one exists. */

	free_rr_graph();

	begin = clock();

	/* Set up the routing resource graph defined by this FPGA architecture. */

	build_rr_graph(graph_type, num_types, type_descriptors, nx, ny, grid,
			chan_width_x[0], NULL, det_routing_arch.switch_block_type,
			det_routing_arch.Fs, det_routing_arch.num_segment,
			det_routing_arch.num_switch, segment_inf,
			det_routing_arch.global_route_switch,
			det_routing_arch.delayless_switch, timing_inf,
			det_routing_arch.wire_to_ipin_switch, router_opts.base_cost_type,
			directs, num_directs, FALSE, arch_file,
			&tmp);

	end = clock();
#ifdef CLOCKS_PER_SEC
	vpr_printf(TIO_MESSAGE_INFO, "Build rr_graph took %g seconds.\n", (float)(end - begin) / CLOCKS_PER_SEC);
#else
	vpr_printf(TIO_MESSAGE_INFO, "Build rr_graph took %g seconds.\n", (float)(end - begin) / CLK_PER_SEC);
#endif

	/* Allocate and load some additional rr_graph information needed only by *
	 * the router.                                                           */

	alloc_and_load_rr_node_route_structs();

	/* EH: Block all routing nodes which have been used
	 * for local intra-CLB feedback */
	block_routing_nodes(&router_opts);

	init_route_structs(router_opts.bb_factor);

	num_global_nets = 0;
	for (inet = 0; inet < num_nets; ++inet) {
		if (clb_net[inet].is_global)
			++num_global_nets;
	}

	vpr_printf(TIO_MESSAGE_INFO, "Number of nets to be routed: %d (not including %d global nets).\n", num_nets-num_global_nets, num_global_nets);
	if (router_opts.router_algorithm == BREADTH_FIRST) {
		vpr_printf(TIO_MESSAGE_INFO, "Confirming Router Algorithm: BREADTH_FIRST.\n");
		success = try_breadth_first_route(router_opts, clb_opins_used_locally,
				width_fac);
				
	} else if (router_opts.router_algorithm == TIMING_DRIVEN) { /* TIMING_DRIVEN route */
		vpr_printf(TIO_MESSAGE_INFO, "Confirming Router Algorithm: TIMING_DRIVEN.\n");
		assert(router_opts.route_type != GLOBAL);
		success = try_timing_driven_route(router_opts, net_delay, slacks,
			clb_opins_used_locally,timing_inf.timing_analysis_enabled);
    //added QT 4			
	} else if (router_opts.router_algorithm == READ_ROUTE){
		int inet;
		vpr_printf(TIO_MESSAGE_INFO, "Confirming Router Algorithm: Reading from %s.\n", FileNameOpts->RouteFile);
        success = read_route(FileNameOpts->RouteFile);
		
		for(inet = 0; inet <num_nets; inet++){
			
			pathfinder_update_one_cost(trace_head[inet], 1, 0.0);
		}
		
		check_route(router_opts.route_type, det_routing_arch.num_switch, 
		            clb_opins_used_locally);
	}
	
	free_rr_node_route_structs();

	return (success);
}

boolean feasible_routing(void) {

	/* This routine checks to see if this is a resource-feasible routing.      *
	 * That is, are all rr_node capacity limitations respected?  It assumes    *
	 * that the occupancy arrays are up to date when it is called.             */

	int inode;

	for (inode = 0; inode < num_rr_nodes; inode++) {
		if (rr_node[inode].occ > rr_node[inode].capacity) {
			return (FALSE);
		}
	}

	return (TRUE);
}

int pathfinder_update_one_cost(struct s_trace *route_segment_start,
		int add_or_sub, float pres_fac) {

	/* This routine updates the occupancy and pres_cost of the rr_nodes that are *
	 * affected by the portion of the routing of one net that starts at          *
	 * route_segment_start.  If route_segment_start is trace_head[inet], the     *
	 * cost of all the nodes in the routing of net inet are updated.  If         *
	 * add_or_sub is -1 the net (or net portion) is ripped up, if it is 1 the    *
	 * net is added to the routing.  The size of pres_fac determines how severly *
	 * oversubscribed rr_nodes are penalized.                                    */

	struct s_trace *tptr;
	int inode, occ, capacity;
	int score;

	score = 0;

	tptr = route_segment_start;
	if (tptr == NULL) /* No routing yet. */
		return score;

	while (tptr) {
		inode = tptr->index;

		occ = rr_node[inode].occ + add_or_sub;
		capacity = rr_node[inode].capacity;

		rr_node[inode].occ = occ;

		/* pres_cost is Pn in the Pathfinder paper. I set my pres_cost according to *
		 * the overuse that would result from having ONE MORE net use this routing  *
		 * node.                                                                    */

		if (occ < capacity) {
			rr_node_route_inf[inode].pres_cost = 1.;
		} else {
			rr_node_route_inf[inode].pres_cost = 1.
				+ (occ + 1 - capacity) * pres_fac;
		}

		score += std::max(0, occ - capacity);

		if (rr_node[inode].type == SINK) {
			tptr = tptr->next; /* Skip next segment. */
			if (tptr == NULL)
				break;
		}

		tptr = tptr->next;

	} /* End while loop -- did an entire traceback. */

	return score;
}

void pathfinder_update_cost(float pres_fac, float acc_fac)  {

	/* This routine recomputes the pres_cost and acc_cost of each routing        *
	 * resource for the pathfinder algorithm after all nets have been routed.    *
	 * It updates the accumulated cost to by adding in the number of extra       *
	 * signals sharing a resource right now (i.e. after each complete iteration) *
	 * times acc_fac.  It also updates pres_cost, since pres_fac may have        *
	 * changed.  THIS ROUTINE ASSUMES THE OCCUPANCY VALUES IN RR_NODE ARE UP TO  *
	 * DATE.                                                                     */

	int inode, occ, capacity;
	for (inode = 0; inode < num_rr_nodes; inode++) {
		occ = rr_node[inode].occ;
		capacity = rr_node[inode].capacity;

		if (occ > capacity) {
			rr_node_route_inf[inode].acc_cost += (occ - capacity) * acc_fac;
			rr_node_route_inf[inode].pres_cost = 1.
					+ (occ + 1 - capacity) * pres_fac;
		}

		/* If occ == capacity, we don't need to increase acc_cost, but a change    *
		 * in pres_fac could have made it necessary to recompute the cost anyway.  */

		else if (occ == capacity) {
			rr_node_route_inf[inode].pres_cost = 1. + pres_fac;
		}
	}
}

void init_route_structs(int bb_factor) {

	/* Call this before you route any nets.  It frees any old traceback and   *
	 * sets the list of rr_nodes touched to empty.                            */

	int i;

	for (i = 0; i < num_nets; i++)
		free_traceback(i);

	load_route_bb(bb_factor);

	/* Check that things that should have been emptied after the last routing *
	 * really were.                                                           */

	if (rr_modified_head != NULL) {
		vpr_printf(TIO_MESSAGE_ERROR, "in init_route_structs. List of modified rr nodes is not empty.\n");
		exit(1);
	}

	if (heap_tail != 1) {
		vpr_printf(TIO_MESSAGE_ERROR, "in init_route_structs. Heap is not empty.\n");
		exit(1);
	}
}

struct s_trace *
update_traceback(struct s_heap *hptr, int inet) {

	/* This routine adds the most recently finished wire segment to the         *
	 * traceback linked list.  The first connection starts with the net SOURCE  *
	 * and begins at the structure pointed to by trace_head[inet]. Each         *
	 * connection ends with a SINK.  After each SINK, the next connection       *
	 * begins (if the net has more than 2 pins).  The first element after the   *
	 * SINK gives the routing node on a previous piece of the routing, which is *
	 * the link from the existing net to this new piece of the net.             *
	 * In each traceback I start at the end of a path and trace back through    *
	 * its predecessors to the beginning.  I have stored information on the     *
	 * predecesser of each node to make traceback easy -- this sacrificies some *
	 * memory for easier code maintenance.  This routine returns a pointer to   *
	 * the first "new" node in the traceback (node not previously in trace).    */

	struct s_trace *tptr, *prevptr, *temptail, *ret_ptr;
	int inode;
	short iedge;

#ifdef DEBUG
	t_rr_type rr_type;
#endif

	inode = hptr->index;

#ifdef DEBUG
	rr_type = rr_node[inode].type;
	if (rr_type != SINK) {
		vpr_printf(TIO_MESSAGE_ERROR, "in update_traceback. Expected type = SINK (%d).\n", SINK);
		vpr_printf(TIO_MESSAGE_ERROR, "\tGot type = %d while tracing back net %d.\n", rr_type, inet);
		exit(1);
	}
#endif

	tptr = alloc_trace_data(); /* SINK on the end of the connection */
	tptr->index = inode;
	tptr->iswitch = OPEN;
	tptr->next = NULL;
	temptail = tptr; /* This will become the new tail at the end */
	/* of the routine.                          */

	/* Now do it's predecessor. */

	inode = hptr->u.prev_node;
	iedge = hptr->prev_edge;

	while (inode != NO_PREVIOUS) {
		prevptr = alloc_trace_data();
		prevptr->index = inode;
		prevptr->iswitch = rr_node[inode].switches[iedge];
		prevptr->next = tptr;
		tptr = prevptr;

		iedge = rr_node_route_inf[inode].prev_edge;
		inode = rr_node_route_inf[inode].prev_node;
	}

	if (trace_tail[inet] != NULL) {
		trace_tail[inet]->next = tptr; /* Traceback ends with tptr */
		ret_ptr = tptr->next; /* First new segment.       */
	} else { /* This was the first "chunk" of the net's routing */
		trace_head[inet] = tptr;
		ret_ptr = tptr; /* Whole traceback is new. */
	}

	trace_tail[inet] = temptail;
	return (ret_ptr);
}

void reset_path_costs(void) {

	/* The routine sets the path_cost to HUGE_POSITIVE_FLOAT for all channel segments   *
	 * touched by previous routing phases.                                     */

	struct s_linked_f_pointer *mod_ptr;

#ifdef DEBUG
	int num_mod_ptrs;
#endif

	/* The traversal method below is slightly painful to make it faster. */

	if (rr_modified_head != NULL) {
		mod_ptr = rr_modified_head;

#ifdef DEBUG
		num_mod_ptrs = 1;
#endif

		while (mod_ptr->next != NULL) {
			*(mod_ptr->fptr) = HUGE_POSITIVE_FLOAT;
			mod_ptr = mod_ptr->next;
#ifdef DEBUG
			num_mod_ptrs++;
#endif
		}
		*(mod_ptr->fptr) = HUGE_POSITIVE_FLOAT; /* Do last one. */

		/* Reset the modified list and put all the elements back in the free   *
		 * list.                                                               */

		mod_ptr->next = linked_f_pointer_free_head;
		linked_f_pointer_free_head = rr_modified_head;
		rr_modified_head = NULL;

#ifdef DEBUG
		num_linked_f_pointer_allocated -= num_mod_ptrs;
#endif
	}
}

float get_rr_cong_cost(int inode) {

	/* Returns the *congestion* cost of using this rr_node. */

	short cost_index;
	float cost;

	cost_index = rr_node[inode].cost_index;
	cost = rr_indexed_data[cost_index].base_cost
			* rr_node_route_inf[inode].acc_cost
			* rr_node_route_inf[inode].pres_cost;
	return (cost);
}

void mark_ends(int inet) {

	/* Mark all the SINKs of this net as targets by setting their target flags  *
	 * to the number of times the net must connect to each SINK.  Note that     *
	 * this number can occassionally be greater than 1 -- think of connecting   *
	 * the same net to two inputs of an and-gate (and-gate inputs are logically *
	 * equivalent, so both will connect to the same SINK).                      */

	int ipin, inode;

	for (ipin = 1; ipin <= clb_net[inet].num_sinks; ipin++) {
		inode = net_rr_terminals[inet][ipin];
		rr_node_route_inf[inode].target_flag++;
	}
}

void node_to_heap(int inode, float cost, int prev_node, int prev_edge,
		float backward_path_cost, float R_upstream) {

	/* Puts an rr_node on the heap, if the new cost given is lower than the     *
	 * current path_cost to this channel segment.  The index of its predecessor *
	 * is stored to make traceback easy.  The index of the edge used to get     *
	 * from its predecessor to it is also stored to make timing analysis, etc.  *
	 * easy.  The backward_path_cost and R_upstream values are used only by the *
	 * timing-driven router -- the breadth-first router ignores them.           */

	struct s_heap *hptr;

	if (cost >= rr_node_route_inf[inode].path_cost)
		return;

	hptr = alloc_heap_data();
	hptr->index = inode;
	hptr->cost = cost;
	hptr->u.prev_node = prev_node;
	hptr->prev_edge = prev_edge;
	hptr->backward_path_cost = backward_path_cost;
	hptr->R_upstream = R_upstream;
	add_to_heap(hptr);
}

void free_traceback(int inet) {

	/* Puts the entire traceback (old routing) for this net on the free list *
	 * and sets the trace_head pointers etc. for the net to NULL.            */

	struct s_trace *tptr, *tempptr;

	if(trace_head == NULL) {
		return;
	}

	tptr = trace_head[inet];

	while (tptr != NULL) {
		tempptr = tptr->next;
		free_trace_data(tptr);
		tptr = tempptr;
	}

	trace_head[inet] = NULL;
	trace_tail[inet] = NULL;
}

t_ivec **
alloc_route_structs(void) {

	/* Allocates the data structures needed for routing.    */

	t_ivec **clb_opins_used_locally;

	alloc_route_static_structs();
	clb_opins_used_locally = alloc_and_load_clb_opins_used_locally();

	return (clb_opins_used_locally);
}

void alloc_route_static_structs(void) {
	trace_head = (struct s_trace **) my_calloc(num_nets,
			sizeof(struct s_trace *));
	trace_tail = (struct s_trace **) my_malloc(
			num_nets * sizeof(struct s_trace *));

	heap_size = nx * ny;
	heap = (struct s_heap **) my_malloc(heap_size * sizeof(struct s_heap *));
	heap--; /* heap stores from [1..heap_size] */
	heap_tail = 1;

	route_bb = (struct s_bb *) my_malloc(num_nets * sizeof(struct s_bb));
}

struct s_trace **
alloc_saved_routing(t_ivec ** clb_opins_used_locally,
		t_ivec *** saved_clb_opins_used_locally_ptr) {

	/* Allocates data structures into which the key routing data can be saved,   *
	 * allowing the routing to be recovered later (e.g. after a another routing  *
	 * is attempted).                                                            */

	struct s_trace **best_routing;
	t_ivec **saved_clb_opins_used_locally;
	int iblk, iclass, num_local_opins;
	t_type_ptr type;

	best_routing = (struct s_trace **) my_calloc(num_nets,
			sizeof(struct s_trace *));

	saved_clb_opins_used_locally = (t_ivec **) my_malloc(
			num_blocks * sizeof(t_ivec *));

	for (iblk = 0; iblk < num_blocks; iblk++) {
		type = block[iblk].type;
		saved_clb_opins_used_locally[iblk] = (t_ivec *) my_malloc(
				type->num_class * sizeof(t_ivec));
		for (iclass = 0; iclass < type->num_class; iclass++) {
			num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;
			saved_clb_opins_used_locally[iblk][iclass].nelem = num_local_opins;

			if (num_local_opins == 0) {
				saved_clb_opins_used_locally[iblk][iclass].list = NULL;
			} else {
				saved_clb_opins_used_locally[iblk][iclass].list =
						(int *) my_malloc(num_local_opins * sizeof(int));
			}
		}
	}

	*saved_clb_opins_used_locally_ptr = saved_clb_opins_used_locally;
	return (best_routing);
}

/* TODO: super hacky, jluu comment, I need to rethink this whole function, without it, logically equivalent output pins incorrectly use more pins than needed.  I force that CLB output pin uses at most one output pin  */
t_ivec **
alloc_and_load_clb_opins_used_locally(void) {

	/* Allocates and loads the data needed to make the router reserve some CLB  *
	 * output pins for connections made locally within a CLB (if the netlist    *
	 * specifies that this is necessary).                                       */

	t_ivec **clb_opins_used_locally;
	int iblk, clb_pin, iclass, num_local_opins;
	int class_low, class_high;
	t_type_ptr type;

	clb_opins_used_locally = (t_ivec **) my_malloc(
			num_blocks * sizeof(t_ivec *));

	for (iblk = 0; iblk < num_blocks; iblk++) {
		type = block[iblk].type;
		get_class_range_for_block(iblk, &class_low, &class_high);
		clb_opins_used_locally[iblk] = (t_ivec *) my_malloc(
				type->num_class * sizeof(t_ivec));
		for (iclass = 0; iclass < type->num_class; iclass++)
			clb_opins_used_locally[iblk][iclass].nelem = 0;

		for (clb_pin = 0; clb_pin < type->num_pins; clb_pin++) {
			// another hack to avoid I/Os, whole function needs a rethink
			if(type == IO_TYPE) {
				continue;
			}
		
			if ((block[iblk].nets[clb_pin] != OPEN
					&& clb_net[block[iblk].nets[clb_pin]].num_sinks == 0) || block[iblk].nets[clb_pin] == OPEN
				) {
				iclass = type->pin_class[clb_pin];
				if(type->class_inf[iclass].type == DRIVER) {
					/* Check to make sure class is in same range as that assigned to block */
					if (iclass >= class_low && iclass <= class_high)
					//assert(iclass >= class_low && iclass <= class_high);
					clb_opins_used_locally[iblk][iclass].nelem++;
				}
			}
		}

		for (iclass = 0; iclass < type->num_class; iclass++) {
			num_local_opins = clb_opins_used_locally[iblk][iclass].nelem;

			if (num_local_opins == 0)
				clb_opins_used_locally[iblk][iclass].list = NULL;
			else
				clb_opins_used_locally[iblk][iclass].list = (int *) my_malloc(
						num_local_opins * sizeof(int));
		}
	}

	return (clb_opins_used_locally);
}

void free_trace_structs(void) {
	/*the trace lists are only freed after use by the timing-driven placer */
	/*Do not  free them after use by the router, since stats, and draw  */
	/*routines use the trace values */
	int i;

	for (i = 0; i < num_nets; i++)
		free_traceback(i);

	if(trace_head) {
		free(trace_head);
		free(trace_tail);
	}
	trace_head = NULL;
	trace_tail = NULL;
}

void free_route_structs() {

	/* Frees the temporary storage needed only during the routing.  The  *
	 * final routing result is not freed.                                */
	if(heap != NULL) {
		free(heap + 1);
	}
	if(route_bb != NULL) {
		free(route_bb);
	}

	heap = NULL; /* Defensive coding:  crash hard if I use these. */
	route_bb = NULL;

	/*free the memory chunks that were used by heap and linked f pointer */
	free_chunk_memory(&heap_ch);
	free_chunk_memory(&linked_f_pointer_ch);
	heap_free_head = NULL;
	linked_f_pointer_free_head = NULL;
}

void free_saved_routing(struct s_trace **best_routing,
		t_ivec ** saved_clb_opins_used_locally) {

	/* Frees the data structures needed to save a routing.                     */
	int i;

	free(best_routing);
	for (i = 0; i < num_blocks; i++) {
		free_ivec_vector(saved_clb_opins_used_locally[i], 0,
				block[i].type->num_class - 1);
	}
	free(saved_clb_opins_used_locally);
}

void alloc_and_load_rr_node_route_structs(void) {

	/* Allocates some extra information about each rr_node that is used only   *
	 * during routing.                                                         */

	int inode;

	if (rr_node_route_inf != NULL) {
 		vpr_printf(TIO_MESSAGE_ERROR, "in alloc_and_load_rr_node_route_structs: old rr_node_route_inf array exists.\n");
		exit(1);
	}

	rr_node_route_inf = (t_rr_node_route_inf *) my_malloc(num_rr_nodes * sizeof(t_rr_node_route_inf));

	for (inode = 0; inode < num_rr_nodes; inode++) {
		rr_node_route_inf[inode].prev_node = NO_PREVIOUS;
		rr_node_route_inf[inode].prev_edge = NO_PREVIOUS;
		rr_node_route_inf[inode].pres_cost = 1.;
		rr_node_route_inf[inode].acc_cost = 1.;
		rr_node_route_inf[inode].path_cost = HUGE_POSITIVE_FLOAT;
		rr_node_route_inf[inode].target_flag = 0;
		rr_node_route_inf[inode].reserved_for = OPEN;
	}
}

void reset_rr_node_route_structs(void) {

	/* Allocates some extra information about each rr_node that is used only   *
	 * during routing.                                                         */

	int inode;

	assert(rr_node_route_inf != NULL);

	for (inode = 0; inode < num_rr_nodes; inode++) {
		rr_node_route_inf[inode].prev_node = NO_PREVIOUS;
		rr_node_route_inf[inode].prev_edge = NO_PREVIOUS;
		rr_node_route_inf[inode].pres_cost = 1.;
		rr_node_route_inf[inode].acc_cost = 1.;
		rr_node_route_inf[inode].path_cost = HUGE_POSITIVE_FLOAT;
		rr_node_route_inf[inode].target_flag = 0;
		rr_node_route_inf[inode].reserved_for = OPEN;
	}
}

void free_rr_node_route_structs(void) {

	/* Frees the extra information about each rr_node that is needed only      *
	 * during routing.                                                         */

	free(rr_node_route_inf);
	rr_node_route_inf = NULL; /* Mark as free */
}

/* RESEARCH TODO: Bounding box heuristic needs to be redone for heterogeneous blocks */
static void load_route_bb(int bb_factor) {

	/* This routine loads the bounding box arrays used to limit the space  *
	 * searched by the maze router when routing each net.  The search is   *
	 * limited to channels contained with the net bounding box expanded    *
	 * by bb_factor channels on each side.  For example, if bb_factor is   *
	 * 0, the maze router must route each net within its bounding box.     *
	 * If bb_factor = nx, the maze router will search every channel in     *
	 * the FPGA if necessary.  The bounding boxes returned by this routine *
	 * are different from the ones used by the placer in that they are     * 
	 * clipped to lie within (0,0) and (nx+1,ny+1) rather than (1,1) and   *
	 * (nx,ny).                                                            */

	int k, xmax, ymax, xmin, ymin, x, y, inet;

	for (inet = 0; inet < num_nets; inet++) {
		int iblk;
		/* EH: Ignore bb for global nets */
		if (clb_net[inet].is_global)
			continue;

		x = block[clb_net[inet].node_block[0]].x;
		y =
				block[clb_net[inet].node_block[0]].y
						/*+ block[clb_net[inet].node_block[0]].type->pin_height[clb_net[inet].node_block_pin[0]];*/
						+ block[clb_net[inet].node_block[0]].type->height;

		xmin = x;
		ymin = block[clb_net[inet].node_block[0]].y;
		xmax = x;
		ymax = y;

		for (k = 1; k <= clb_net[inet].num_sinks; k++) {
			x = block[clb_net[inet].node_block[k]].x;
			y =
					block[clb_net[inet].node_block[k]].y
							/*+ block[clb_net[inet].node_block[k]].type->pin_height[clb_net[inet].node_block_pin[k]]*/;

			if (x < xmin) {
				xmin = x;
			} else if (x > xmax) {
				xmax = x;
			}

			if (y < ymin) {
				ymin = y;
			} 
			
			y = block[clb_net[inet].node_block[k]].y + block[clb_net[inet].node_block[k]].type->height;
			if (y > ymax) {
				ymax = y;
			}
		}

		/* Want the channels on all 4 sides to be usuable, even if bb_factor = 0. */

		/* EH:
		xmin -= 1;
		ymin -= 1;*/

		/* Expand the net bounding box by bb_factor, then clip to the physical *
		 * chip area.                                                          */

		/* EH: For nets starting at BUFG (e.g. global opin)
		 * then consider clock regions above and below */
		iblk = clb_net[inet].node_block[0];
		assert(iblk != OPEN);
		if (strncmp(clb_net[inet].name, "GLOBAL_LOGIC", strlen("GLOBAL_LOGIC")) == 0) {
			route_bb[inet].xmin = xmin;
			route_bb[inet].xmax = xmax;
			route_bb[inet].ymin = ymin;
			route_bb[inet].ymax = ymax;
		}
		else if (strcmp(block[iblk].type->name, "BUFG") == 0) {
			route_bb[inet].xmin = std::max(xmin - bb_factor, 0);
			route_bb[inet].xmax = std::min(xmax + bb_factor, nx + 1);
			route_bb[inet].ymin = std::max(ymin - std::max(bb_factor, 25), 0);
			route_bb[inet].ymax = std::min(ymax + std::max(bb_factor, 25), ny + 1);
		}
		else {
			route_bb[inet].xmin = std::max(xmin - bb_factor, 0);
			route_bb[inet].xmax = std::min(xmax + bb_factor, nx + 1);
			route_bb[inet].ymin = std::max(ymin - bb_factor, 0);
			route_bb[inet].ymax = std::min(ymax + bb_factor, ny + 1);
		}
	}
}

void add_to_mod_list(float *fptr) {

	/* This routine adds the floating point pointer (fptr) into a  *
	 * linked list that indicates all the pathcosts that have been *
	 * modified thus far.                                          */

	struct s_linked_f_pointer *mod_ptr;

	mod_ptr = alloc_linked_f_pointer();

	/* Add this element to the start of the modified list. */

	mod_ptr->next = rr_modified_head;
	mod_ptr->fptr = fptr;
	rr_modified_head = mod_ptr;
}

static void add_to_heap(struct s_heap *hptr) {

	/* Adds an item to the heap, expanding the heap if necessary.             */

	int ito, ifrom;
	struct s_heap *temp_ptr;

	if (heap_tail > heap_size) { /* Heap is full */
		heap_size *= 2;
		heap = (struct s_heap **) my_realloc((void *) (heap + 1),
				heap_size * sizeof(struct s_heap *));
		heap--; /* heap goes from [1..heap_size] */
	}

	heap[heap_tail] = hptr;
	ifrom = heap_tail;
	ito = ifrom / 2;
	heap_tail++;

	while ((ito >= 1) && (heap[ifrom]->cost < heap[ito]->cost)) {
		temp_ptr = heap[ito];
		heap[ito] = heap[ifrom];
		heap[ifrom] = temp_ptr;
		ifrom = ito;
		ito = ifrom / 2;
	}
}

/*WMF: peeking accessor :) */
boolean is_empty_heap(void) {
	return (boolean)(heap_tail == 1);
}

struct s_heap *
get_heap_head(void) {

	/* Returns a pointer to the smallest element on the heap, or NULL if the     *
	 * heap is empty.  Invalid (index == OPEN) entries on the heap are never     *
	 * returned -- they are just skipped over.                                   */

	int ito, ifrom;
	struct s_heap *heap_head, *temp_ptr;

	do {
		if (heap_tail == 1) { /* Empty heap. */
			vpr_printf(TIO_MESSAGE_WARNING, "Empty heap occurred in get_heap_head.\n");
			vpr_printf(TIO_MESSAGE_WARNING, "Some blocks are impossible to connect in this architecture.\n");
			return (NULL);
		}

		heap_head = heap[1]; /* Smallest element. */

		/* Now fix up the heap */

		heap_tail--;
		heap[1] = heap[heap_tail];
		ifrom = 1;
		ito = 2 * ifrom;

		while (ito < heap_tail) {
			if (heap[ito + 1]->cost < heap[ito]->cost)
				ito++;
			if (heap[ito]->cost > heap[ifrom]->cost)
				break;
			temp_ptr = heap[ito];
			heap[ito] = heap[ifrom];
			heap[ifrom] = temp_ptr;
			ifrom = ito;
			ito = 2 * ifrom;
		}

	} while (heap_head->index == OPEN); /* Get another one if invalid entry. */

	return (heap_head);
}

void empty_heap(void) {

	int i;

	for (i = 1; i < heap_tail; i++)
		free_heap_data(heap[i]);

	heap_tail = 1;
}

static struct s_heap *
alloc_heap_data(void) {

	struct s_heap *temp_ptr;

	if (heap_free_head == NULL) { /* No elements on the free list */
		heap_free_head = (struct s_heap *) my_chunk_malloc(sizeof(struct s_heap),&heap_ch);
		heap_free_head->u.next = NULL;
	}

	temp_ptr = heap_free_head;
	heap_free_head = heap_free_head->u.next;
#ifdef DEBUG
	num_heap_allocated++;
#endif
	return (temp_ptr);
}

void free_heap_data(struct s_heap *hptr) {

	hptr->u.next = heap_free_head;
	heap_free_head = hptr;
#ifdef DEBUG
	num_heap_allocated--;
#endif
}

void invalidate_heap_entries(int sink_node, int ipin_node) {

	/* Marks all the heap entries consisting of sink_node, where it was reached *
	 * via ipin_node, as invalid (OPEN).  Used only by the breadth_first router *
	 * and even then only in rare circumstances.                                */

	int i;

	for (i = 1; i < heap_tail; i++) {
		if (heap[i]->index == sink_node && heap[i]->u.prev_node == ipin_node)
			heap[i]->index = OPEN; /* Invalid. */
	}
}

struct s_trace *
alloc_trace_data(void) {

	struct s_trace *temp_ptr;

	if (trace_free_head == NULL) { /* No elements on the free list */
		trace_free_head = (struct s_trace *) my_chunk_malloc(sizeof(struct s_trace),&trace_ch);
		trace_free_head->next = NULL;
	}
	temp_ptr = trace_free_head;
	trace_free_head = trace_free_head->next;
#ifdef DEBUG
	num_trace_allocated++;
#endif
	return (temp_ptr);
}

void free_trace_data(struct s_trace *tptr) {

	/* Puts the traceback structure pointed to by tptr on the free list. */

	tptr->next = trace_free_head;
	trace_free_head = tptr;
#ifdef DEBUG
	num_trace_allocated--;
#endif
}

static struct s_linked_f_pointer *
alloc_linked_f_pointer(void) {

	/* This routine returns a linked list element with a float pointer as *
	 * the node data.                                                     */

	/*int i;*/
	struct s_linked_f_pointer *temp_ptr;

	if (linked_f_pointer_free_head == NULL) {
		/* No elements on the free list */	
	linked_f_pointer_free_head = (struct s_linked_f_pointer *) my_chunk_malloc(sizeof(struct s_linked_f_pointer),&linked_f_pointer_ch);
	linked_f_pointer_free_head->next = NULL;
	}

	temp_ptr = linked_f_pointer_free_head;
	linked_f_pointer_free_head = linked_f_pointer_free_head->next;

#ifdef DEBUG
	num_linked_f_pointer_allocated++;
#endif

	return (temp_ptr);
}

//added QT 5 read_route
static boolean read_route(const char *route_file){
	
	FILE *fp;
	char line[1024], tmp[1024], *pline, name[1024];
	int i, inet, inode, ipin, ilow, jlow, ptc_num;
	int chanwidth;
	t_rr_type rr_type;
	char *name_type[] =
		{ "SOURCE", "SINK", "IPIN", "OPIN", "CHANX", "CHANY", "INTRA_CLUSTER_EDGE" };
	char type[7], c;
 
	fp = my_fopen(route_file, "r", 0);
	assert(fp);

	assert(fscanf(fp, "Array size: %d x %d logic blocks.\n", &nx, &ny) == 2);
	fscanf(fp, "Channel Width: %d.\n", &chanwidth);
    	fscanf(fp, "\nRouting:");
	c = fgetc(fp); assert(c == '\n');

	while(!feof(fp))
	{
		c = fgetc(fp); assert(c == '\n');
		assert(fgets(line, 1024, fp));
		assert(sscanf(line, "Net %d %n", &inet, &i) == 1);
		pline = line + i;
		assert(inet < num_nets);
		assert(sscanf(pline, "(%[^)])%n", name, &i) == 1);
		pline += i;
		assert(strcmp(clb_net[inet].name, name) == 0);
		if (*pline == '\n') 
		{
    		struct s_trace **ptptr;
			struct s_trace *tptr_prev;
			assert(clb_net[inet].is_global == FALSE);
			c = fgetc(fp); assert(c == '\n');
			ptptr = &(trace_head[inet]);
			tptr_prev = NULL;

			while(fgets(line, 1024, fp) && line[0] != '\n')
			{
				assert(sscanf(line, "Node:\t%d\t%6s (%d,%d) %n", &inode, type, &ilow, &jlow, &i) == 4);
				pline = line + i;
				for (i = 0; i < 7; i++)
				{
					if (strcmp(name_type[i], type) == 0)
					{
						rr_type = (t_rr_type)i;
						break;
					}
				}
				assert(i < 7);
				assert(sscanf(pline, "%[^:]: %n", tmp, &i) == 1);
				pline += i;
				assert(sscanf(pline, "%d  %n", &ptc_num, &i) == 1);
				pline += i;
				//inode = get_rr_node_index(ilow, jlow, rr_type, ptc_num, rr_node_indices);
				
				if (tptr_prev != NULL) {
					int iedge;
					int inode_prev;
					inode_prev = tptr_prev->index;

					for (iedge = 0; iedge < rr_node[inode_prev].num_edges; iedge++)
					{
						if (rr_node[inode_prev].edges[iedge] == inode) break;
					}
					assert(iedge < rr_node[inode_prev].num_edges);

					tptr_prev->iswitch = rr_node[inode_prev].switches[iedge];
				}

				assert(rr_node[inode].type == rr_type);
				assert(rr_node[inode].xlow == ilow);
				assert(rr_node[inode].ylow == jlow);
				assert(rr_node[inode].ptc_num == ptc_num);

				*ptptr = alloc_trace_data();
				(*ptptr)->index = inode;
				trace_tail[inet] = *ptptr;
				if (rr_node[inode].type == SINK)
				{
					(*ptptr)->iswitch = OPEN;
					tptr_prev = NULL;
				}
				else
					tptr_prev = *ptptr;
				ptptr = &((*ptptr)->next);
			}
			*ptptr = NULL;
		}
		else if (*pline == ':')
		{
			assert(strcmp(pline, ": global net connecting:\n") == 0);
			assert(clb_net[inet].is_global == TRUE);
			c = fgetc(fp); assert(c == '\n');
			ipin = 0;
			while(fgets(line, 1024, fp) && line[0] != '\n')
			{
				int node_block_pin, bnum, x, y, iclass;
				assert(ipin <= clb_net[inet].num_sinks);
				assert(sscanf(line,
				    "Block %s (#%d) at (%d, %d), Pin class %d.\n",
				    name, &bnum, &x, &y, &iclass) == 5);
				if (strcmp(clb_net[inet].name, "vcc") != 0 && strcmp(clb_net[inet].name, "gnd") != 0) {
					assert(clb_net[inet].node_block[ipin] == bnum);
					node_block_pin = clb_net[inet].node_block_pin[ipin];
					assert(iclass == block[bnum].type->pin_class[node_block_pin]);
				}
				assert(strcmp(name, block[bnum].name) == 0);
				assert(block[bnum].x == x);
				assert(block[bnum].y == y);
				ipin++;
			}
		}
		else
		{
			assert(*pline == '\n' || *pline == ':');
		}
	}
	return (TRUE);
	
}

void print_route(char *route_file) {

	/* Prints out the routing to file route_file.  */

	int inet, inode, ipin, bnum, ilow, jlow, node_block_pin, iclass;
	t_rr_type rr_type;
	struct s_trace *tptr;
	const char *name_type[] = { "SOURCE", "SINK", "IPIN", "OPIN", "CHANX", "CHANY",
			"INTRA_CLUSTER_EDGE" };
	FILE *fp;

	fp = fopen(route_file, "w");

	fprintf(fp, "Array size: %d x %d logic blocks.\n", nx, ny);
	fprintf(fp, "\nRouting:");
	for (inet = 0; inet < num_nets; inet++) {
		if (clb_net[inet].is_global == FALSE) {
			if (clb_net[inet].num_sinks == FALSE) {
				fprintf(fp, "\n\nNet %d (%s)\n\n", inet, clb_net[inet].name);
				fprintf(fp, "\n\nUsed in local cluster only, reserved one CLB pin\n\n");
			} else {
				fprintf(fp, "\n\nNet %d (%s)\n\n", inet, clb_net[inet].name);
				tptr = trace_head[inet];

				while (tptr != NULL) {
					inode = tptr->index;
					rr_type = rr_node[inode].type;
					ilow = rr_node[inode].xlow;
					jlow = rr_node[inode].ylow;

					fprintf(fp, "Node:\t%d\t%6s (%d,%d) ", inode, name_type[rr_type], ilow, jlow);

					if ((ilow != rr_node[inode].xhigh)
							|| (jlow != rr_node[inode].yhigh))
						fprintf(fp, "to (%d,%d) ", rr_node[inode].xhigh,
								rr_node[inode].yhigh);

					switch (rr_type) {

					case IPIN:
					case OPIN:
						if (grid[ilow][jlow].type == IO_TYPE) {
							fprintf(fp, " Pad: ");
						} else { /* IO Pad. */
							fprintf(fp, " Pin: ");
						}
						break;

					case CHANX:
					case CHANY:
						fprintf(fp, " Track: ");
						break;

					case SOURCE:
					case SINK:
						if (grid[ilow][jlow].type == IO_TYPE) {
							fprintf(fp, " Pad: ");
						} else { /* IO Pad. */
							fprintf(fp, " Class: ");
						}
						break;

					default:
						vpr_printf(TIO_MESSAGE_ERROR, "in print_route: Unexpected traceback element type: %d (%s).\n", 
								rr_type, name_type[rr_type]);
						exit(1);
						break;
					}

					fprintf(fp, "%d  ", rr_node[inode].ptc_num);
					/* Uncomment line below if you're debugging and want to see the switch types *
					 * used in the routing.                                                      */
					/*          fprintf (fp, "Switch: %d", tptr->iswitch);    */

					fprintf(fp, "\n");

					tptr = tptr->next;
				}
			}
		}

		else { /* Global net.  Never routed. */
			fprintf(fp, "\n\nNet %d (%s): global net connecting:\n\n", inet,
					clb_net[inet].name);

			for (ipin = 0; ipin <= clb_net[inet].num_sinks; ipin++) {
				bnum = clb_net[inet].node_block[ipin];
				if (bnum == OPEN) {
					assert(ipin == 0);
					continue;
				}

				node_block_pin = clb_net[inet].node_block_pin[ipin];
				iclass = block[bnum].type->pin_class[node_block_pin];

				fprintf(fp, "Block %s (#%d) at (%d, %d), Pin class %d.\n",
						block[bnum].name, bnum, block[bnum].x, block[bnum].y,
						iclass);
			}
		}
	}

	fclose(fp);

	if (getEchoEnabled() && isEchoFileEnabled(E_ECHO_MEM)) {
		fp = my_fopen(getEchoFileName(E_ECHO_MEM), "w", 0);
		fprintf(fp, "\nNum_heap_allocated: %d   Num_trace_allocated: %d\n",
				num_heap_allocated, num_trace_allocated);
		fprintf(fp, "Num_linked_f_pointer_allocated: %d\n",
				num_linked_f_pointer_allocated);
		fclose(fp);
	}

}

/* TODO: check if this is still necessary for speed */
void reserve_locally_used_opins(float pres_fac, boolean rip_up_local_opins,
		t_ivec ** clb_opins_used_locally) {

	/* In the past, this function implicitly allowed LUT duplication when there are free LUTs. 
	 This was especially important for logical equivalence; however, now that we have a very general
	 logic cluster, it does not make sense to allow LUT duplication implicitly. we'll need to look into how we want to handle this case

	 */

	int iblk, num_local_opin, inode, from_node, iconn, num_edges, to_node;
	int iclass, ipin;
	float cost;
	struct s_heap *heap_head_ptr;
	t_type_ptr type;

	if (rip_up_local_opins) {
		for (iblk = 0; iblk < num_blocks; iblk++) {
			type = block[iblk].type;
			for (iclass = 0; iclass < type->num_class; iclass++) {
				num_local_opin = clb_opins_used_locally[iblk][iclass].nelem;
				/* Always 0 for pads and for RECEIVER (IPIN) classes */
				for (ipin = 0; ipin < num_local_opin; ipin++) {
					inode = clb_opins_used_locally[iblk][iclass].list[ipin];
					adjust_one_rr_occ_and_pcost(inode, -1, pres_fac);
				}
			}
		}
	}

	for (iblk = 0; iblk < num_blocks; iblk++) {
		type = block[iblk].type;
		for (iclass = 0; iclass < type->num_class; iclass++) {
			num_local_opin = clb_opins_used_locally[iblk][iclass].nelem;
			/* Always 0 for pads and for RECEIVER (IPIN) classes */

			if (num_local_opin != 0) { /* Have to reserve (use) some OPINs */
				from_node = rr_blk_source[iblk][iclass];
				num_edges = rr_node[from_node].num_edges;
				for (iconn = 0; iconn < num_edges; iconn++) {
					to_node = rr_node[from_node].edges[iconn];
					cost = get_rr_cong_cost(to_node);
					node_to_heap(to_node, cost, OPEN, OPEN, 0., 0.);
				}

				for (ipin = 0; ipin < num_local_opin; ipin++) {
					heap_head_ptr = get_heap_head();
					inode = heap_head_ptr->index;
					adjust_one_rr_occ_and_pcost(inode, 1, pres_fac);
					clb_opins_used_locally[iblk][iclass].list[ipin] = inode;
					free_heap_data(heap_head_ptr);
				}

				empty_heap();
			}
		}
	}
}

static void adjust_one_rr_occ_and_pcost(int inode, int add_or_sub,
		float pres_fac) {

	/* Increments or decrements (depending on add_or_sub) the occupancy of    *
	 * one rr_node, and adjusts the present cost of that node appropriately.  */

	int occ, capacity;

	occ = rr_node[inode].occ + add_or_sub;
	capacity = rr_node[inode].capacity;
	rr_node[inode].occ = occ;

	if (occ < capacity) {
		rr_node_route_inf[inode].pres_cost = 1.;
	} else {
		rr_node_route_inf[inode].pres_cost = 1.
				+ (occ + 1 - capacity) * pres_fac;
	}
}


void free_chunk_memory_trace(void) {
	if (trace_ch.chunk_ptr_head != NULL) {
		free_chunk_memory(&trace_ch);
	}
}

/* EH */
#define NUM_MOST_CONGESTED 10
int feasible_routing_score(void) {

	/* This routine checks to see if this is a resource-feasible routing.      *
	 * That is, are all rr_node capacity limitations respected?  It assumes    *
	 * that the occupancy arrays are up to date when it is called.             */

	int inode;
	int score, total_score /*, count*/;
	std::set<std::pair<int,int> > most_congested;

	total_score = 0;
	//count = 0;
	for (inode = 0; inode < num_rr_nodes; inode++) {
		if (rr_node[inode].occ > rr_node[inode].capacity) {
			score = std::max(0, rr_node[inode].occ - rr_node[inode].capacity);
			total_score += score;

#ifdef NUM_MOST_CONGESTED
			if (score > 0) {
				if ((rr_node[inode].type == CHANX || rr_node[inode].type == CHANY)
						&& rr_node[inode].prev_node == OPEN) {
					if (most_congested.size() < NUM_MOST_CONGESTED)
						most_congested.insert(std::make_pair(score, inode));
					else if (most_congested.size() == NUM_MOST_CONGESTED) {
						std::pair<int,int> smallest = *most_congested.begin();
						if (smallest.first < score) {
							most_congested.erase(most_congested.begin());
							most_congested.insert(std::make_pair(score, inode));
						}
					}
				}
			}
#endif
		}
	}
	
#ifdef NUM_MOST_CONGESTED
	if (!most_congested.empty()) {
		std::set<std::pair<int,int> >::const_reverse_iterator it = most_congested.rbegin();
		std::set<std::pair<int,int> >::const_reverse_iterator ie = most_congested.rend();
		vpr_printf(TIO_MESSAGE_INFO, "Top %d most congested nodes:", NUM_MOST_CONGESTED);
		vpr_printf(TIO_MESSAGE_INFO, "\n");
		for (; it != ie; ++it) {
			score = it->first;
			inode = it->second;
			vpr_printf(TIO_MESSAGE_INFO, "\t");
			vpr_printf(TIO_MESSAGE_INFO, " %d: %d", score, inode);
			/*vpr_printf(TIO_MESSAGE_INFO, " (%d@%d)", rr_node[inode].ptc_num, rr_node[inode].net_num);*/
			vpr_printf(TIO_MESSAGE_INFO, "\n");
		}
	}

	int inet;
	struct s_trace *tptr;
	most_congested.clear();
	for (inet = 0; inet < num_nets; ++inet) {
		tptr = trace_head[inet];
		score = 0;
		while (tptr != NULL) {
			inode = tptr->index;
			if (rr_node[inode].occ > rr_node[inode].capacity)
				score += std::max(0, rr_node[inode].occ - rr_node[inode].capacity);
			tptr = tptr->next;
		}

		if (score > 0) {
			if (most_congested.size() < NUM_MOST_CONGESTED)
				most_congested.insert(std::make_pair(score, inet));
			else if (most_congested.size() == NUM_MOST_CONGESTED) {
				std::pair<int,int> smallest = *most_congested.begin();
				if (smallest.first < score) {
					most_congested.erase(most_congested.begin());
					most_congested.insert(std::make_pair(score, inet));
				}
			}
		}
	}

	if (!most_congested.empty()) {
		std::set<std::pair<int,int> >::const_reverse_iterator it = most_congested.rbegin();
		std::set<std::pair<int,int> >::const_reverse_iterator ie = most_congested.rend();
		vpr_printf(TIO_MESSAGE_INFO, "Top %d most congested nets:", NUM_MOST_CONGESTED);
		vpr_printf(TIO_MESSAGE_INFO, "\n");
		for (; it != ie; ++it) {
			score = it->first;
			inet = it->second;
			vpr_printf(TIO_MESSAGE_INFO, "\t");
			vpr_printf(TIO_MESSAGE_INFO, " %d: inet %d (%s, %d sinks, bb: (%d,%d)-(%d,%d))\n", 
					score, inet, clb_net[inet].name, clb_net[inet].num_sinks,
					route_bb[inet].xmin, route_bb[inet].ymin,
					route_bb[inet].xmax, route_bb[inet].ymax);
		}
	}
#endif

	return total_score;
}

static short*
find_switch(int src_inode, int sink_inode)
{
	int iedge;
	assert(rr_node[src_inode].num_edges > 0);
	for (iedge = 0; iedge < rr_node[src_inode].num_edges; ++iedge)
	{
		int inode;
		inode = rr_node[src_inode].edges[iedge];
		if (inode == sink_inode)
			break;
	}
	/*assert(iedge < rr_node[src_inode].num_edges);*/
	if (iedge == rr_node[src_inode].num_edges)
		return NULL;
	return &rr_node[src_inode].switches[iedge];
}

short*
find_rt(int x, int y, int src_ptc, int sink_ptc) 
{
	int src_inode, sink_inode;
	short *pswitch;

	src_inode = get_rr_node_index(x, y, IPIN, src_ptc, rr_node_indices);
	assert(rr_node[src_inode].xlow == x);
	assert(rr_node[src_inode].ylow == y);
	assert(rr_node[src_inode].type == IPIN);
	src_inode = rr_node[src_inode].prev_node;
	assert(src_inode != OPEN);
	assert(rr_node[src_inode].type == CHANX || rr_node[src_inode].type == CHANY);

	sink_inode = get_rr_node_index(x, y, OPIN, sink_ptc, rr_node_indices);
	assert(rr_node[sink_inode].xlow == x);
	assert(rr_node[sink_inode].ylow == y);
	assert(rr_node[sink_inode].type == OPIN);
	assert(rr_node[sink_inode].num_edges == 1);
	sink_inode = rr_node[sink_inode].edges[0];
	assert(sink_inode != OPEN);
	assert(rr_node[sink_inode].type == CHANX || rr_node[sink_inode].type == CHANY);

	pswitch = find_switch(src_inode, sink_inode);
	if (pswitch && *pswitch == OPEN)
		return NULL;
	return pswitch;
}

static void
delete_rt(int x, int y, int src_ptc, int sink_ptc)
{
	short *pswitch;

	pswitch = find_rt(x, y, src_ptc, sink_ptc);
	if (pswitch) {
		assert(*pswitch == LUT_SWITCH_INDEX);
		*pswitch = OPEN;
	}
}

static void
reserve_ipin_and_delete_other_edges(int x, int y, int ptc, int inet)
{
	int inode, prev_node;
	//short *pswitch;

	inode = get_rr_node_index(x, y, IPIN, ptc, rr_node_indices);
	assert(rr_node[inode].xlow == x);
	assert(rr_node[inode].ylow == y);
	assert(rr_node[inode].type == IPIN);

	prev_node = rr_node[inode].prev_node;
	assert(prev_node != OPEN);

	rr_node_route_inf[prev_node].reserved_for = inet;
	 
	// Erase all other edges apart from prev_node to
	// the IPIN node we want to reserve
	if (rr_node[prev_node].num_edges > 1) {
		int iedge;
		for (iedge = 0; iedge < rr_node[prev_node].num_edges; ++iedge) {
			if (rr_node[prev_node].edges[iedge] != inode) {
				rr_node[prev_node].switches[iedge] = OPEN;
			}
		}
	}

	//pswitch = find_switch(rr_node[inode].prev_node, inode);
	//assert(pswitch);
	//if (*pswitch == OPEN)
	//	return;
	//assert(*pswitch == DEFAULT_SWITCH_INDEX);
	//*pswitch = OPEN;
}

static void
block_routing_nodes(struct s_router_opts *router_opts) {
	int inet, inet_vcc, inet_gnd;

	reserve_slice_ipins();

	inet_vcc = inet_gnd = OPEN;
	for (inet = 0; inet < num_nets; ++inet) {
		if (strcmp(clb_net[inet].name, "gnd") == 0)
			inet_gnd = inet;
		else if (strcmp(clb_net[inet].name, "vcc") == 0)
			inet_vcc = inet;
		if (inet_vcc != OPEN && inet_gnd != OPEN)
			break;
	}
	assert(inet_vcc != OPEN && inet_gnd != OPEN);

	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);
	for (inet = 0; inet < num_nets; ++inet) {
		int isink, inet_ref;
		char c;
		if (strncmp(clb_net[inet].name, "GLOBAL_LOGIC", strlen("GLOBAL_LOGIC")) != 0)
			continue;

		c = clb_net[inet].name[strlen("GLOBAL_LOGIC")];
		if (c == '0')
			inet_ref = inet_gnd;
		else if (c == '1')
			inet_ref = inet_vcc;
		else assert(FALSE);

		for (isink = 1; isink <= clb_net[inet].num_sinks; ++isink) {
			int iblk, ptc, inode, prev_node;
			iblk = clb_net[inet].node_block[isink];
			if (block[iblk].type != FILL_TYPE)
				continue;
			ptc = clb_net[inet].node_block_pin[isink];
			inode = get_rr_node_index(block[iblk].x, block[iblk].y, IPIN, ptc, rr_node_indices);
			assert(inode != OPEN);
			prev_node = rr_node[inode].prev_node;
			assert(prev_node != OPEN);
			if (rr_node_route_inf[prev_node].reserved_for == inet_ref)
				rr_node_route_inf[prev_node].reserved_for = inet;
		}
	}

	reserve_byps();

	if (router_opts->noRoutethru)
		delete_all_rt();

	/* If global net routing not performed, reserve their IPINs? */
	if (router_opts->noGlobals) {
		for (inet = 0; inet < num_nets; ++inet) {
			if (strcmp(clb_net[inet].name, "gnd") == 0)
				reserve_net_ipins(inet);
			else if (strcmp(clb_net[inet].name, "vcc") == 0)
				reserve_net_ipins(inet);
		}
	}

}

/* Block route-throughs for LUT sites in user circuit
 * and remove A6 IPIN (from being pinswap-pable) for
 * fractured LUTs since A6 must be tied to vcc */
static void
reserve_slice_ipins(void) {
	int iblk;
	int inet_vcc;
	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);

	for (inet_vcc = 0; inet_vcc < num_nets; ++inet_vcc) {
		if (strcmp(clb_net[inet_vcc].name, "vcc") == 0)
			break;
	}
	assert(inet_vcc < num_nets);

	for (iblk = 0; iblk < num_blocks; ++iblk)
	{
		int slice_mode;
		int bx, by, bz;
		int imode;
		int num_bles, ible;

		if (block[iblk].type != FILL_TYPE)
			continue;

		//assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 3);

		bx = block[iblk].x;
		by = block[iblk].y;
		bz = block[iblk].z;

		assert(block[iblk].pb->child_pbs);
		assert(block[iblk].pb->child_pbs[0]);

		imode = block[iblk].pb->mode;
		//if (strcmp(block[iblk].pb->pb_graph_node->pb_type->modes[imode].pb_type_children->name, "ble6") != 0)
		//	continue;
		num_bles = block[iblk].pb->pb_graph_node->pb_type->modes[imode].pb_type_children->num_pb;
		for (ible = 0; ible < num_bles; ++ible)
		{
			const char *bleModeName;

			if (!block[iblk].pb->child_pbs[0][ible].child_pbs)
				continue;

			imode = block[iblk].pb->child_pbs[0][ible].mode;
			bleModeName = block[iblk].pb->child_pbs[0][ible].pb_graph_node->pb_type->modes[imode].name;
			if (strcmp(bleModeName, "O6LUT") == 0
					|| strcmp(bleModeName, "ble7") == 0
					|| strcmp(bleModeName, "ble8") == 0) {
				int lut_in, lut_out;

				/* Delete A6:A1 -> {A,AQ,AMUX} RTs */
				if (strcmp(bleModeName, "ble7") == 0)
					for (lut_in = 0; lut_in < 6; ++lut_in) {
						for (lut_out = 0; lut_out < 3; ++lut_out) {
							delete_rt(bx, by, bz*PINS_PER_SLICEL + (ible*2+0)*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + (ible*2+0)*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
							delete_rt(bx, by, bz*PINS_PER_SLICEL + (ible*2+1)*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + (ible*2+1)*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
						}
					}
				else if (strcmp(bleModeName, "ble8") == 0)
					for (lut_in = 0; lut_in < 6; ++lut_in) {
						for (lut_out = 0; lut_out < 3; ++lut_out) {
							delete_rt(bx, by, bz*PINS_PER_SLICEL + 0*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + 0*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
							delete_rt(bx, by, bz*PINS_PER_SLICEL + 1*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + 1*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
							delete_rt(bx, by, bz*PINS_PER_SLICEL + 2*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + 2*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
							delete_rt(bx, by, bz*PINS_PER_SLICEL + 3*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + 3*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
						}
					}
				else
					for (lut_in = 0; lut_in < 6; ++lut_in) {
						for (lut_out = 0; lut_out < 3; ++lut_out) {
							delete_rt(bx, by, bz*PINS_PER_SLICEL + ible*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + ible*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
						}
					}
			}
			else {
				int num_flut, iflut;
				int lut_in, lut_out;
				assert(strncmp(bleModeName, "O6O5LUT", strlen("O6O5LUT")) == 0);
				lut_in = 0;
				lut_out = 0;
				/* Delete A6->A route through */
				delete_rt(bx, by, bz*PINS_PER_SLICEL + ible*IPINS_PER_BLE + lut_in, 
						bz*PINS_PER_SLICEL + ible*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);

				num_flut = block[iblk].pb->child_pbs[0][ible].child_pbs[0][0].pb_graph_node->pb_type->num_pb;
				for (iflut = 0; iflut < num_flut; ++iflut) {
					if (!block[iblk].pb->child_pbs[0][ible].child_pbs[0][iflut].parent_pb)
						continue;

					if (iflut == 0) {
						/* Delete A5:A1 -> A RTs too */
						lut_out = 0;
						for (lut_in = 1; lut_in < 6; ++lut_in) {
							delete_rt(bx, by, bz*PINS_PER_SLICEL + ible*IPINS_PER_BLE + lut_in, 
									bz*PINS_PER_SLICEL + ible*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
						}
					}
					else {
						assert(iflut == 1);
						lut_in = PTC_SLICEL_A6_VCCONLY;
						// Reserve A6 IPIN if O6 is not used (to prevent routethru)
						reserve_ipin_and_delete_other_edges(bx, by, bz*PINS_PER_SLICEL + ible*IPINS_PER_BLE + lut_in, inet_vcc);
					}
				}	
			}
		}
	}
}

void delete_all_rt(void) {
	int x, y;
	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);
	for (y = 0; y <= ny+1; ++y) {
		for (x = 0; x <=  nx+1; ++x) {
			int z, ible;
			if (grid[x][y].type != FILL_TYPE)
				continue;
			for (z = 0; z < 2; ++z) {
				for (ible = 0; ible < 4; ++ible) {
					int lut_in, lut_out;
					for (lut_in = 0; lut_in < 6; ++lut_in) {
						for (lut_out = 0; lut_out < 3; ++lut_out) {
							delete_rt(x, y, z*PINS_PER_SLICEL + ible*IPINS_PER_BLE + lut_in, 
									z*PINS_PER_SLICEL + ible*OPINS_PER_BLE + PTC_SLICEL_A + lut_out);
						}
					}
				}
			}

		}
	}
}


static int
add_sink(int iblk, int inet, int ptc)
{
	int num_sinks;
	assert(inet != OPEN);
	num_sinks = clb_net[inet].num_sinks;
	clb_net[inet].node_block = (int*)realloc(clb_net[inet].node_block, (num_sinks+2)*sizeof(int));
	clb_net[inet].node_block[num_sinks+1] = iblk;
	clb_net[inet].node_block_pin = (int*)realloc(clb_net[inet].node_block_pin, (num_sinks+2)*sizeof(int));
	clb_net[inet].node_block_pin[num_sinks+1] = ptc;
	++clb_net[inet].num_sinks;
	//assert(block[iblk].pb->rr_graph[ptc].net_num == OPEN);
	//block[iblk].pb->rr_graph[ptc].net_num = clb_to_vpack_net_mapping[inet];
	ptc %= block[iblk].type->num_pins / block[iblk].type->capacity;
	return block[iblk].pb->rr_graph[ptc].net_num;
}

static void
connect_vcc_to_A6(int inet_vcc, int iblk) {
	int bz;
	int imode;
	int num_bles, ible;

	assert(strcmp(block[iblk].pb->pb_graph_node->pb_type->name, "SLICEL") == 0);
	//assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 3);

	assert(block[iblk].pb->child_pbs);
	assert(block[iblk].pb->child_pbs[0]);

	bz = block[iblk].z;

	imode = block[iblk].pb->mode;
	if (strcmp(block[iblk].pb->pb_graph_node->pb_type->modes[imode].pb_type_children->name, "ble6") != 0)
		return;

	num_bles = block[iblk].pb->pb_graph_node->pb_type->modes[imode].pb_type_children->num_pb;
	for (ible = 0; ible < num_bles; ++ible)
	{
		const char *bleModeName;

		if (!block[iblk].pb->child_pbs[0][ible].child_pbs)
			continue;

		imode = block[iblk].pb->child_pbs[0][ible].mode;
		bleModeName = block[iblk].pb->child_pbs[0][ible].pb_graph_node->pb_type->modes[imode].name;
		if (strncmp(bleModeName, "O6O5LUT", strlen("O6O5LUT")) == 0) {
			boolean O6used = FALSE;
			int num_flut, iflut;
			num_flut = block[iblk].pb->child_pbs[0][ible].child_pbs[0][0].pb_graph_node->pb_type->num_pb;
			for (iflut = 0; iflut < num_flut; ++iflut)
			{
				if (!block[iblk].pb->child_pbs[0][ible].child_pbs[0][iflut].parent_pb)
					continue;

				if (iflut == 0) {
					O6used = TRUE;
				}
				/* If O5 is used, connect A6 to VCC */
				else {
					int lut_in;
					int net_num;
					assert(iflut == 1);
					if (O6used) {
						lut_in = PTC_SLICEL_A6_VCCONLY;
						net_num = add_sink(iblk, inet_vcc, bz*PINS_PER_SLICEL + ible*IPINS_PER_BLE + lut_in);
						assert(net_num == OPEN);
					}
				}
			}
		}
		else {
			assert(strcmp(bleModeName, "O6LUT") == 0);
		}
	}
}

static void
connect_gnd_vcc_to_BRAM(int inet_gnd, int inet_vcc, int iblk) {
	int num_slices;
	int imode;
	char *modeName;

	assert(!strcmp(block[iblk].pb->pb_graph_node->pb_type->name, "RAMB36E1"));
	//assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 1);

	num_slices = block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children;

	imode = block[iblk].pb->mode;
	modeName = block[iblk].pb->pb_graph_node->pb_type->modes[imode].name;
	if (strncmp(modeName, "RAMB36E1", 8) == 0) {
		int iport, num_ports;
		int ptc;
		size_t modeNameLen;
		boolean singlePort = FALSE;

		assert(num_slices == 1);

		modeNameLen = strlen(modeName);
		if (strncmp(modeName+modeNameLen-3, "_sp", 3) == 0)
			singlePort = TRUE;

		num_ports = block[iblk].type->pb_type->num_ports;
		ptc = 0;
		for (iport = 0; iport < num_ports; ++iport) {
			int net_num = OPEN;
			char *portName = block[iblk].type->pb_type->ports[iport].name;
			int num_pins = block[iblk].type->pb_type->ports[iport].num_pins;

			/* Tie off EN{ARD,BWR}{L,U} to vcc */
			if (strcmp(portName, "ENARDENL") == 0
				|| strcmp(portName, "ENARDENU") == 0
				|| strcmp(portName, "ENBWRENL") == 0
				|| strcmp(portName, "ENBWRENU") == 0) {
				assert(num_pins == 1);
				net_num = add_sink(iblk, inet_vcc, ptc);
			}
			/* Tie off all ADDR unused LSBs to vcc */
			else if (strcmp(portName, "ADDRARDADDRL") == 0
					|| strcmp(portName, "ADDRBWRADDRL") == 0) {
				int ipin;
				int not_open_from = -1;
				assert(num_pins == 16);
				for (ipin = 0; ipin < num_pins-1; ++ipin) {
					if (not_open_from < 0) {
						if (block[iblk].nets[ptc+ipin] != OPEN)
							not_open_from = ipin;
					}
					else
						assert(block[iblk].nets[ptc+ipin] != OPEN);
				}
				assert(block[iblk].nets[ptc+num_pins-1] == OPEN);
				net_num = add_sink(iblk, inet_vcc, ptc+num_pins-1);
				assert(net_num == OPEN);
				for (ipin = 0; ipin < not_open_from; ++ipin) {
					net_num = add_sink(iblk, inet_vcc, ptc+ipin);
					assert(net_num == OPEN);
				}

				if (strcmp(portName, "ADDRARDADDRL") == 0)
					assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "ADDRARDADDRU") == 0);
				else if (strcmp(portName, "ADDRBWRADDRL") == 0)
					assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "ADDRBWRADDRU") == 0);
				else assert(FALSE);

				assert(block[iblk].type->pb_type->ports[iport+1].num_pins == 15);

				/* Then copy ADDRARDADDRL into ADDRARDADDRU */
				for (ipin = 0; ipin < not_open_from; ++ipin) {
					assert(block[iblk].nets[ptc+num_pins+ipin] == OPEN);
					net_num = add_sink(iblk, inet_vcc, ptc+num_pins+ipin);
					assert(net_num == OPEN);
				}
			}
			/* Tie off various REG/RST to gnd */
			else if (strcmp(portName, "REGCLKARDRCLKL") == 0
				|| strcmp(portName, "REGCLKARDRCLKU") == 0 
				|| strcmp(portName, "REGCLKBL") == 0
				|| strcmp(portName, "REGCLKBU") == 0
				|| strcmp(portName, "REGCEAREGCEL") == 0
				|| strcmp(portName, "REGCEAREGCEU") == 0
				|| strcmp(portName, "REGCEBL") == 0
				|| strcmp(portName, "REGCEBU") == 0
				|| strcmp(portName, "RSTRAMARSTRAMLRST") == 0
				|| strcmp(portName, "RSTRAMARSTRAMU") == 0
				|| strcmp(portName, "RSTRAMBL") == 0
				|| strcmp(portName, "RSTRAMBU") == 0
				|| strcmp(portName, "RSTREGARSTREGL") == 0
				|| strcmp(portName, "RSTREGARSTREGU") == 0
				|| strcmp(portName, "RSTREGBL") == 0
				|| strcmp(portName, "RSTREGBU") == 0
				) {
				assert(num_pins == 1);
				net_num = add_sink(iblk, inet_gnd, ptc);
			}
			/* If single port, tie off WEAL to gnd 
			 * then fix_bram_connections() will copy into WEAU */
			else if (singlePort && strcmp(portName, "WEAL") == 0) {
				int ipin;
				assert(num_pins == 4);
				for (ipin = 0; ipin < num_pins; ++ipin) {
					net_num = add_sink(iblk, inet_gnd, ptc+ipin);
					assert(net_num == OPEN);
				}
			}
			else 
			assert(net_num == OPEN);
			ptc += block[iblk].type->pb_type->ports[iport].num_pins;
		}
	}
	else if (strcmp(modeName, "RAMB18E1x2") == 0) {
		int iport, num_ports;
		int islice;
		int ptc;

		/* EH: Only TBs have a blank name, ignore */
		if (block[iblk].name == NULL)
			return;

		assert(num_slices == 1);

		num_slices = block[iblk].pb->child_pbs[0][0].pb_graph_node->pb_type->num_pb;
		for (islice = 0; islice < num_slices; ++islice) {
			if (!block[iblk].pb->child_pbs[0][islice].child_pbs)
				continue;

			num_ports = block[iblk].type->pb_type->num_ports;
			ptc = 0;
			for (iport = 0; iport < num_ports; ++iport) {
				int net_num = OPEN;
				char *portName = block[iblk].type->pb_type->ports[iport].name;
				int num_pins = block[iblk].type->pb_type->ports[iport].num_pins;

				/* Tie off EN{ARD,BWR}EN to vcc */
				if ((islice == 0 && 
					(strcmp(portName, "s0_ENARDEN") == 0
					|| strcmp(portName, "s0_ENBWREN") == 0)) ||
					(islice == 1 && 
					(strcmp(portName, "s1_ENARDEN") == 0
					|| strcmp(portName, "s1_ENBWREN") == 0))) {
					assert(num_pins == 1);
					net_num = add_sink(iblk, inet_vcc, ptc);
				}
				/* Tie off unused LSBs in ADDR{ARD,BWR}ADDR to vcc */
				else if ((islice == 0 && 
					(strcmp(portName, "s0_ADDRARDADDR") == 0
					|| strcmp(portName, "s0_ADDRBWRADDR") == 0)) ||
					(islice == 1 && 
					(strcmp(portName, "s1_ADDRARDADDR") == 0
					|| strcmp(portName, "s1_ADDRBWRADDR") == 0))) {
					// Tie off all unused LSBs to vcc
					int ipin;
					int not_open_from = -1;
					assert(num_pins == 14);
					for (ipin = 0; ipin < num_pins; ++ipin) {
						if (not_open_from < 0) {
							if (block[iblk].nets[ptc+ipin] != OPEN)
								not_open_from = ipin;
						}
						/*else
							assert(block[iblk].nets[ptc+ipin] != OPEN);*/
					}
					for (ipin = 0; ipin < not_open_from; ++ipin) {
						net_num = add_sink(iblk, inet_vcc, ptc+ipin);
						assert(net_num == OPEN);
					}
				}
				/* Tie off ADDR{A,B}TIEHIGH to vcc */
				else if ((islice == 0 && 
					(strcmp(portName, "s0_ADDRATIEHIGH") == 0
					|| strcmp(portName, "s0_ADDRBTIEHIGH") == 0)) ||
					(islice == 1 && 
					(strcmp(portName, "s1_ADDRATIEHIGH") == 0
					|| strcmp(portName, "s1_ADDRBTIEHIGH") == 0))) {
					int ipin;
					assert(num_pins == 2);
					for (ipin = 0; ipin < num_pins; ++ipin) {
						net_num = add_sink(iblk, inet_vcc, ptc+ipin);
						assert(net_num == OPEN);
					}
				}
				/* Tie off various REG/RST to gnd */
				else if ((islice == 0 && 
					(strcmp(portName, "s0_REGCLKARDRCLK") == 0
					|| strcmp(portName, "s0_REGCLKB") == 0
					|| strcmp(portName, "s0_REGCEAREGCE") == 0
					|| strcmp(portName, "s0_REGCEB") == 0
					|| strcmp(portName, "s0_RSTRAMARSTRAM") == 0
					|| strcmp(portName, "s0_RSTRAMB") == 0
					|| strcmp(portName, "s0_RSTREGARSTREG") == 0
					|| strcmp(portName, "s0_RSTREGB") == 0
					)) ||
					(islice == 1 &&
					(strcmp(portName, "s1_REGCLKARDRCLK") == 0
					|| strcmp(portName, "s1_REGCLKB") == 0
					|| strcmp(portName, "s1_REGCEAREGCE") == 0
					|| strcmp(portName, "s1_REGCEB") == 0
					|| strcmp(portName, "s1_RSTRAMARSTRAM") == 0
					|| strcmp(portName, "s1_RSTRAMB") == 0
					|| strcmp(portName, "s1_RSTREGARSTREG") == 0
					|| strcmp(portName, "s1_RSTREGB") == 0
					))) {
					assert(num_pins == 1);
					net_num = add_sink(iblk, inet_gnd, ptc);
				}
				/* In RAMB18 mode, WEBWE[7:4] are always tied off to gnd */
				else if ((islice == 0 && strcmp(portName, "s0_WEBWE") == 0) ||
						(islice == 1 && strcmp(portName, "s1_WEBWE") == 0)) {
					int ipin;
					assert(num_pins == 8);
					for (ipin = 4; ipin < num_pins; ++ipin) {
						net_num = add_sink(iblk, inet_gnd, ptc+ipin);
						assert(net_num == OPEN);
					}
				}
				assert(net_num == OPEN);
				ptc += block[iblk].type->pb_type->ports[iport].num_pins;
			}
		}
	}
	else {
		vpr_printf(TIO_MESSAGE_ERROR, "RAMB36 mode %s not recognised!\n", modeName);
		exit(1);
	}
}

static void
connect_gnd_vcc_to_DSP(int inet_gnd, int inet_vcc, int iblk) {
	int num_slices;
	int iport, num_ports;
	int imode;
	int ptc;

	assert(!strcmp(block[iblk].pb->pb_graph_node->pb_type->name, "DSP48E1"));
	assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 1);

	imode = block[iblk].pb->mode;
	assert(imode == 0);

	num_slices = block[iblk].pb->pb_graph_node->pb_type->modes[imode].num_pb_type_children;
	assert(num_slices == 4);

	assert(strcmp(block[iblk].pb->child_pbs[0][0].pb_graph_node->pb_type->name, "mult_25x18") == 0);
	//assert(block[iblk].pb->child_pbs[0][0].pb_graph_node->pb_type->num_modes == 1);

	num_slices = block[iblk].pb->child_pbs[0][0].pb_graph_node->pb_type->num_pb;
	assert(num_slices == 1);
	assert(block[iblk].pb->child_pbs[0]);

	num_ports = block[iblk].type->pb_type->num_ports;
	ptc = block[iblk].z * block[iblk].type->num_pins / block[iblk].type->capacity;
	for (iport = 0; iport < num_ports; ++iport) {
		int net_num = OPEN;
		char *portName = block[iblk].type->pb_type->ports[iport].name;
		if (strcmp(portName, "CEA1") == 0
			|| strcmp(portName, "CEA2") == 0 
			|| strcmp(portName, "CEB1") == 0
			|| strcmp(portName, "CEB2") == 0
			|| strcmp(portName, "CEM") == 0
			|| strcmp(portName, "CEP") == 0) {
			net_num = add_sink(iblk, inet_gnd, ptc);
		}
		else if (strcmp(portName, "INMODE") == 0) {
			assert(block[iblk].type->pb_type->ports[iport].num_pins == 5);
			net_num = add_sink(iblk, inet_vcc, ptc + 2);

			net_num = add_sink(iblk, inet_gnd, ptc + 0);
			net_num = add_sink(iblk, inet_gnd, ptc + 1);
			net_num = add_sink(iblk, inet_gnd, ptc + 3);
			net_num = add_sink(iblk, inet_gnd, ptc + 4);
		}
		else if (strcmp(portName, "OPMODE") == 0) {
			assert(block[iblk].type->pb_type->ports[iport].num_pins == 7);
			net_num = add_sink(iblk, inet_vcc, ptc + 0);
			net_num = add_sink(iblk, inet_vcc, ptc + 2);

			net_num = add_sink(iblk, inet_gnd, ptc + 1);
			net_num = add_sink(iblk, inet_gnd, ptc + 3);
			net_num = add_sink(iblk, inet_gnd, ptc + 4);
			net_num = add_sink(iblk, inet_gnd, ptc + 5);
			net_num = add_sink(iblk, inet_gnd, ptc + 6);
		}
		assert(net_num == OPEN);
		ptc += block[iblk].type->pb_type->ports[iport].num_pins;
	}
}

static int 
add_net(const char *name) {
	clb_net = (struct s_net*)realloc(clb_net, sizeof(struct s_net)*(num_nets+1));
	clb_net[num_nets].name = my_strdup(name);
	clb_net[num_nets].is_global = FALSE;

	clb_net[num_nets].num_sinks = 0;
	clb_net[num_nets].node_block = (int*)malloc(sizeof(int));
	clb_net[num_nets].node_block[0] = OPEN;
	clb_net[num_nets].node_block_port = NULL;
	clb_net[num_nets].node_block_pin = (int*)malloc(sizeof(int));
	clb_net[num_nets].node_block_pin[0] = OPEN;
	return num_nets++;
}

static void
connect_gnd_vcc(int inet_vcc, int inet_gnd) {
	int iblk;
	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);
	for (iblk = 0; iblk < num_blocks; ++iblk)
	{
		if (block[iblk].type == FILL_TYPE) {
			connect_vcc_to_A6(inet_vcc, iblk);
		}
		else if (strcmp(block[iblk].type->name, "RAMB36E1") == 0) {
			connect_gnd_vcc_to_BRAM(inet_gnd, inet_vcc, iblk);
		}
		else if (strcmp(block[iblk].type->name, "DSP48E1") == 0) {
			connect_gnd_vcc_to_DSP(inet_gnd, inet_vcc, iblk);
		}
	}
}

static void
reserve_net_ipins(int inet) {
	int isink, num_sinks;
	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);
	num_sinks = clb_net[inet].num_sinks;
	for (isink = 1; isink <= num_sinks; ++isink) {
		int iblk, ptc;
		iblk = clb_net[inet].node_block[isink];
		if (block[iblk].type != FILL_TYPE)
			continue;
		ptc = clb_net[inet].node_block_pin[isink];
		ptc %= FILL_TYPE->num_pins / FILL_TYPE->capacity;
		/* Only if [ABCD] LUT */
		if (ptc / IPINS_PER_BLE > 3)
			continue;

		/* Only if [ABCD][1-6] */
		if (ptc % IPINS_PER_BLE < PTC_SLICEL_A6 
				|| ptc % IPINS_PER_BLE > PTC_SLICEL_A1)
			continue;

		ptc = clb_net[inet].node_block_pin[isink];
		reserve_ipin_and_delete_other_edges(block[iblk].x, block[iblk].y, ptc, inet);
	}
}

static void
reserve_byps(void) {
	int inet;
	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);
	for (inet = 0; inet < num_nets; ++inet) {
		int isink, num_sinks;

		/* Ignore vcc/gnd nets, because their sinks have
		 * been split into GLOBAL_LOGIC[01] */
		if (strcmp(clb_net[inet].name, "vcc") == 0
				|| strcmp(clb_net[inet].name, "gnd") == 0)
			continue;

		num_sinks = clb_net[inet].num_sinks;
		for (isink = 1; isink <= num_sinks; ++isink) {
			int iblk, ptc;
			int ipin_node, clb_x_node, byp_b_node, byp_node;

			iblk = clb_net[inet].node_block[isink];
			if (block[iblk].type != FILL_TYPE)
				continue;
			ptc = clb_net[inet].node_block_pin[isink];
			ptc %= FILL_TYPE->num_pins / FILL_TYPE->capacity;
			/* Only if [ABCD] LUT */
			if (ptc / IPINS_PER_BLE > 3)
				continue;

			/* Only if [ABCD]X */
			if (ptc % IPINS_PER_BLE != PTC_SLICEL_AX)
				continue;

			ptc = clb_net[inet].node_block_pin[isink];
			ipin_node = get_rr_node_index(block[iblk].x, block[iblk].y, IPIN, ptc, rr_node_indices);
			assert(ipin_node != OPEN);

			clb_x_node = rr_node[ipin_node].prev_node;
			assert(clb_x_node != OPEN);

			byp_b_node = rr_node[clb_x_node].prev_node;
			assert(byp_b_node != OPEN);

			byp_node = rr_node[byp_b_node].prev_node;
			assert(byp_node != OPEN);

			assert(rr_node[byp_node].num_edges == 2);

			//if (rr_node[byp_node].edges[0] != byp_b_node) {
			//	assert(rr_node[byp_node].edges[1] == byp_b_node);
			//	rr_node[byp_node].edges[0] = byp_b_node;
			//}
			//rr_node[byp_node].num_edges = 1;

			/* Reserve this BYP node exclusively for this
			 * net only */
			rr_node_route_inf[byp_node].reserved_for = inet;
		}
	}
}

void
split_gnd_vcc_nets(struct s_router_opts *router_opts) {
	int inet;
	int inet_vcc;
	int inet_gnd;

	inet_vcc = inet_gnd = OPEN;
	for (inet = 0; inet < num_nets; ++inet) {
		if (strcmp(clb_net[inet].name, "vcc") == 0) inet_vcc = inet;
		else if (strcmp(clb_net[inet].name, "gnd") == 0) inet_gnd = inet;
		if (inet_vcc != OPEN && inet_gnd != OPEN) break;
	}
	if (inet == num_nets) {
		if (inet_vcc == OPEN) {
			inet_vcc = add_net("vcc");
			clb_net[inet_vcc].is_global = TRUE;
		}
		if (inet_gnd == OPEN) {
			inet_gnd = add_net("gnd");
			clb_net[inet_gnd].is_global = TRUE;
		}
	}
	assert(inet_vcc != OPEN && inet_gnd != OPEN);
	assert(clb_net[inet_vcc].is_global);
	assert(clb_net[inet_gnd].is_global);

	/* First, add all fractured LUTs' A6 pin
	 * to VCC before splitting */
	connect_gnd_vcc(inet_vcc, inet_gnd);

	assert(strcmp(FILL_TYPE->name, "SLICEL") == 0);

	if (!router_opts->noGlobals) {
		std::map<std::pair<char,std::pair<int,int> >,int> xy2net;
		typedef std::map<std::pair<char,std::pair<int,int> >,int>::const_iterator t_it;

		for (inet = 0; inet < num_nets; ++inet) {
			char *port_name, *net_name;
			char vcc_not_gnd;
			if (!clb_net[inet].is_global)
				continue;
			port_name = NULL;
			vcc_not_gnd = -1;
			if (inet == inet_gnd) {
				port_name = "GND_WIRE";
				net_name = "GLOBAL_LOGIC0";
				vcc_not_gnd = 0;
			}
			else if (inet == inet_vcc) {
				port_name = "VCC_WIRE";
				net_name = "GLOBAL_LOGIC1";
				vcc_not_gnd = 1;
			}
			if (port_name) {
				int isink;
				int num_sinks = clb_net[inet].num_sinks;
				int iblk = clb_net[inet].node_block[0];
				int ipin;
				if (iblk != OPEN) {
					ipin = clb_net[inet].node_block_pin[0] 
						% (block[iblk].type->num_pins / block[iblk].type->capacity);
					assert(block[iblk].pb->rr_graph[ipin].net_num == clb_to_vpack_net_mapping[inet]);
				}
				//block[clb_net[inet].node_block[0]].pb->rr_graph[clb_net[inet].node_block_pin[0]].net_num = OPEN;

				for (isink = 1; isink <= num_sinks; ++isink) {
					int jnet, ptc;
					iblk = clb_net[inet].node_block[isink];
					assert(iblk != OPEN);

					int x = block[iblk].x;
					int y = block[iblk].y;
					int z = block[iblk].z;
					t_it it = xy2net.find(std::make_pair(vcc_not_gnd,std::make_pair(x,y)));
					if (it == xy2net.end()) {
						int iport, num_ports;
						char buf[32];

						sprintf(buf, "%s_X%dY%d", net_name, x, y);
						jnet = add_net(buf);

						/* Find the lowest z block at this grid to host GND/VCC wire */
						for (; z >= 0; --z) {
							if (grid[x][y].blocks[z] != OPEN)
								clb_net[jnet].node_block[0] = grid[x][y].blocks[z];
						}
						++z;

						/* Find the ptc for the appropriate wire */
						num_ports = block[iblk].type->pb_type->num_ports;
						ptc = 0;
						for (iport = 0; iport < num_ports; ++iport) {
							if (!strcmp(block[iblk].type->pb_type->ports[iport].name, port_name))
								break;
							ptc += block[iblk].type->pb_type->ports[iport].num_pins;
						}
						assert(iport != num_ports);
						assert(block[iblk].type->class_inf[iport].num_pins == 1);

						assert(z == 0);
						ptc += z * (block[iblk].type->num_pins / block[iblk].type->capacity);
						clb_net[jnet].node_block_pin[0] = ptc;
						//printf("%s at (%d,%d,%d)\n", buf, block[iblk].x, block[iblk].y, ptc);

						// Necessary to prevent reserve_locally_used_opins() from
						// occupying the const outpin
						assert(block[clb_net[jnet].node_block[0]].nets[ptc] == inet
							|| block[clb_net[jnet].node_block[0]].nets[ptc] == OPEN);
						block[clb_net[jnet].node_block[0]].nets[ptc] = jnet;

						xy2net.insert(std::make_pair(std::make_pair(vcc_not_gnd,std::make_pair(x,y)),jnet));
					}
					else {
						jnet = it->second;
					}

					ptc = clb_net[inet].node_block_pin[isink];
					if (block[iblk].type == FILL_TYPE
							&& ptc % (FILL_TYPE->num_pins / FILL_TYPE->capacity) == PTC_SLICEL_CIN) {
#if defined(__x86_64__)
						asm("int3");
#endif
						assert(FALSE);
					}
					add_sink(iblk, jnet, ptc);
				}
			}
		}
	}
}

void
fix_bram_connections(void) {
	int iblk;
	for (iblk = 0; iblk < num_blocks; ++iblk) {
		int num_slices;
		int imode;
		char *modeName;
		if (strcmp(block[iblk].type->name, "RAMB36E1") == 0) {
			int num_children;
			assert(!strcmp(block[iblk].pb->pb_graph_node->pb_type->name, "RAMB36E1"));

			num_children = block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children;

			imode = block[iblk].pb->mode;
			modeName = block[iblk].pb->pb_graph_node->pb_type->modes[imode].name;

			if (strncmp(modeName, "RAMB36E1", 6) == 0) {
				int iport, num_ports;
				int ptc;
				size_t modeNameLen;
				boolean singlePort = FALSE;

				assert(num_children == 1);
				assert(block[iblk].pb->child_pbs[0][0].child_pbs);

				modeNameLen = strlen(modeName);
				if (strncmp(modeName+modeNameLen-3, "_sp", 3) == 0)
					singlePort = TRUE;

				num_ports = block[iblk].type->pb_type->num_ports;
				ptc = 0;
				for (iport = 0; iport < num_ports; ++iport) {
					char *portName = block[iblk].type->pb_type->ports[iport].name;
					if (strcmp(portName, "DIADI") == 0 || strcmp(portName, "DIBDI") == 0) {
						/* For the thinnest RAMB36E1 mode, 
						 * connect the 1 bit input to DI[AB]DI1 as well as 0 
						 * (which translates to DI[AB]DIU0 */
						if (strncmp(modeName, "RAMB36E1_32768x1_", strlen("RAMB36E1_32768x1_")) == 0) {
							int inet, net_num;
							inet = block[iblk].nets[ptc];
							assert(inet != OPEN);
							assert(block[iblk].nets[ptc+16] == OPEN);
							net_num = add_sink(iblk, inet, ptc+16);
							assert(net_num == OPEN);
						}
					}
					/* Copy ADDRARDADDRL to U */
					else if (strcmp(portName, "ADDRARDADDRL") == 0) {
						int ipin, num_pins1, num_pins2;
						int inet;
						int net_num;
						num_pins1 = block[iblk].type->pb_type->ports[iport].num_pins;
						//inet = block[iblk].nets[ptc];
						//assert(inet != OPEN);
						assert(num_pins1 == 16);

						assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "ADDRARDADDRU") == 0);
						num_pins2 = block[iblk].type->pb_type->ports[iport+1].num_pins;
						assert(num_pins2 == 15);

						/* For the widest mode, copy ADDRARDADDRL from ADDRBWRADDRL */
						if (strncmp(modeName, "RAMB36E1_512x72_sp", strlen("RAMB36E1_512x72_sp")) == 0) {
							assert(strcmp(block[iblk].type->pb_type->ports[iport+2].name, "ADDRBWRADDRL") == 0);
							assert(block[iblk].type->pb_type->ports[iport+2].num_pins == num_pins1);
							for (ipin = 0; ipin < num_pins1; ++ipin) {
								assert(block[iblk].nets[ptc+ipin] == OPEN);
								inet = block[iblk].nets[ptc+num_pins1+num_pins2+ipin];
								if (inet != OPEN) {
									net_num = add_sink(iblk, inet, ptc+ipin);
									assert(net_num == OPEN);
									block[iblk].nets[ptc+ipin] = inet;
								}
							}
						}

						/* Then copy ADDRARDADDRL into ADDRARDADDRU */
						for (ipin = 0; ipin < num_pins2; ++ipin) {
							inet = block[iblk].nets[ptc+ipin];
							assert(block[iblk].nets[ptc+num_pins1+ipin] == OPEN);
							if (inet != OPEN) {
								net_num = add_sink(iblk, inet, ptc+num_pins1+ipin);
								assert(net_num == OPEN);
							}
						}
						//vpr_printf(TIO_MESSAGE_INFO, "%s\n", portName);
						ptc += num_pins1;
						++iport;
					}
					/* If dual port, extend ADDRBWRADDRL to U */
					else if (!singlePort && strcmp(portName, "ADDRBWRADDRL") == 0) {
						int ipin, num_pins1, num_pins2;
						int inet;
						int net_num;
						num_pins1 = block[iblk].type->pb_type->ports[iport].num_pins;
						//inet = block[iblk].nets[ptc];
						//assert(inet != OPEN);
						assert(num_pins1 == 16);

						assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "ADDRBWRADDRU") == 0);
						num_pins2 = block[iblk].type->pb_type->ports[iport+1].num_pins;
						assert(num_pins2 == 15);
						for (ipin = 0; ipin < num_pins2; ++ipin) {
							inet = block[iblk].nets[ptc+ipin];
							assert(block[iblk].nets[ptc+num_pins1+ipin] == OPEN);
							if (inet != OPEN) {
								net_num = add_sink(iblk, inet, ptc+num_pins1+ipin);
								assert(net_num == OPEN);
							}
						}
						//vpr_printf(TIO_MESSAGE_INFO, "%s\n", portName);
						ptc += num_pins1;
						++iport;
					}
					/* If dual port, extend WEAL and copy to WEAU */
					else if (!singlePort && strcmp(portName, "WEAL") == 0) {
						int ipin, num_pins;
						int inet;
						int net_num;
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						inet = block[iblk].nets[ptc];
						assert(inet != OPEN);
						assert(num_pins == 4);
						for (ipin = 1; ipin < num_pins; ++ipin) {
							assert(block[iblk].nets[ptc+ipin] == OPEN);
							net_num = add_sink(iblk, inet, ptc+ipin);
							assert(net_num == OPEN);
						}
						assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "WEAU") == 0);
						ptc += num_pins;
						++iport;
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						assert(num_pins == 4);
						for (ipin = 0; ipin < num_pins; ++ipin) {
							assert(block[iblk].nets[ptc+ipin] == OPEN);
							net_num = add_sink(iblk, inet, ptc+ipin);
							assert(net_num == OPEN);
						}
						//vpr_printf(TIO_MESSAGE_INFO, "%d:%s %s <-- %s\n", iblk, block[iblk].name, portName, clb_net[inet].name);
					}
					/* Extend WEBWEL, and copy into WEBWEU */
					else if (strcmp(portName, "WEBWEL") == 0) {
						int ipin, num_pins;
						int inet;
						int net_num;
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						inet = block[iblk].nets[ptc];
						assert(inet != OPEN);
						assert(num_pins == 8);
						for (ipin = 1; ipin < num_pins; ++ipin) {
							assert(block[iblk].nets[ptc+ipin] == OPEN);
							net_num = add_sink(iblk, inet, ptc+ipin);
							assert(net_num == OPEN);
						}

						assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "WEBWEU") == 0);
						ptc += num_pins;
						++iport;
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						assert(num_pins == 8);
						for (ipin = 0; ipin < num_pins; ++ipin) {
							assert(block[iblk].nets[ptc+ipin] == OPEN);
							net_num = add_sink(iblk, inet, ptc+ipin);
							assert(net_num == OPEN);
						}
						//vpr_printf(TIO_MESSAGE_INFO, "%s\n", portName);
					}
					/* If dual port, extend ARDCLKL to U and copy into BWR */
					else if (!singlePort && strcmp(portName, "CLKARDCLKL") == 0) {
						int num_pins;
						int inet;
						int net_num;
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						assert(num_pins == 1);
						inet = block[iblk].nets[ptc];
						assert(inet != OPEN);
						++iport;
						ptc += num_pins;

						assert(strcmp(block[iblk].type->pb_type->ports[iport].name, "CLKARDCLKU") == 0);
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						assert(num_pins == 1);
						assert(block[iblk].nets[ptc] == OPEN);
						net_num = add_sink(iblk, inet, ptc);
						assert(net_num == OPEN);
						ptc += num_pins;
						++iport;

						assert(strcmp(block[iblk].type->pb_type->ports[iport].name, "CLKBWRCLKL") == 0);
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						assert(num_pins == 1);
						assert(block[iblk].nets[ptc] == OPEN);
						net_num = add_sink(iblk, inet, ptc);
						assert(net_num == OPEN);
						ptc += num_pins;
						++iport;

						assert(strcmp(block[iblk].type->pb_type->ports[iport].name, "CLKBWRCLKU") == 0);
						num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
						assert(num_pins == 1);
						assert(block[iblk].nets[ptc] == OPEN);
						net_num = add_sink(iblk, inet, ptc);
						assert(net_num == OPEN);
					}

					ptc += block[iblk].type->pb_type->ports[iport].num_pins;
				}
			}
			else if (strncmp(modeName, "RAMB18E1", 8) == 0) {
				assert(num_children == 1);

				/* EH: Only TBs have a blank name, ignore */
				if (block[iblk].name == NULL)
					continue;

				assert(block[iblk].pb->child_pbs[0]);
				int islice;
				num_slices = block[iblk].pb->child_pbs[0][0].pb_graph_node->pb_type->num_pb;
				for (islice = 0; islice < num_slices; ++islice) {
					int iport, num_ports;
					int ptc;
					size_t modeNameLen;
					boolean singlePort = FALSE;

					if (!block[iblk].pb->child_pbs[0][islice].child_pbs)
						continue;

					imode = block[iblk].pb->child_pbs[0][islice].mode;
					modeName = block[iblk].pb->child_pbs[0][islice].pb_graph_node->pb_type->modes[imode].name;

					modeNameLen = strlen(modeName);
					if (strncmp(modeName+modeNameLen-3, "_sp", 3) == 0)
						singlePort = TRUE;


					num_ports = block[iblk].type->pb_type->num_ports;
					ptc = 0;
					for (iport = 0; iport < num_ports; ++iport) {
						char *portName = block[iblk].type->pb_type->ports[iport].name;
						if (strcmp(portName, "ADDRARDADDR") == 0) {
							int ipin, num_pins;
							int inet;
							int net_num;
							num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
							assert(num_pins == 14);

							/* For the widest mode, copy ADDRARDADDR from ADDRBWRADDR */
							if (strncmp(modeName, "RAMB18E1_512x36_sp", strlen("RAMB18E1_512x36_sp")) == 0) {
								assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "ADDRBWRADDR") == 0);
								assert(block[iblk].type->pb_type->ports[iport+1].num_pins == num_pins);
								for (ipin = 0; ipin < num_pins; ++ipin) {
									assert(block[iblk].nets[ptc+ipin] == OPEN);
									inet = block[iblk].nets[ptc+num_pins+ipin];
									if (inet != OPEN) {
										net_num = add_sink(iblk, inet, ptc+ipin);
										assert(net_num == OPEN);
										block[iblk].nets[ptc+ipin] = inet;
									}
								}
							}
						}
						/* If dual port, extend WEA */
						else if (!singlePort && (
							(islice == 0 && strcmp(portName, "s0_WEA") == 0) ||
							(islice == 1 && strcmp(portName, "s1_WEA") == 0))) {
							int ipin, num_pins;
							int inet;
							int net_num;
							num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
							assert(num_pins == 4);
							inet = block[iblk].nets[ptc];
							assert(inet != OPEN);
							for (ipin = 1; ipin < num_pins; ++ipin) {
								assert(block[iblk].nets[ptc+ipin] == OPEN);
								net_num = add_sink(iblk, inet, ptc+ipin);
								assert(net_num == OPEN);
							}
							//vpr_printf(TIO_MESSAGE_INFO, "%s\n", portName);
						}
						/* Extend WEBWE */
						else if ((islice == 0 && strcmp(portName, "s0_WEBWE") == 0) ||
							(islice == 1 && strcmp(portName, "s1_WEBWE") == 0)) {
							int ipin, num_pins;
							int inet;
							int net_num;
							num_pins = block[iblk].type->pb_type->ports[iport].num_pins;
							assert(num_pins == 8);
							inet = block[iblk].nets[ptc];
							if (inet != OPEN) {
								for (ipin = 1; ipin < 4; ++ipin) {
									assert(block[iblk].nets[ptc+ipin] == OPEN);
									net_num = add_sink(iblk, inet, ptc+ipin);
									assert(net_num == OPEN);
								}
							}
							//vpr_printf(TIO_MESSAGE_INFO, "%s\n", portName);
						}
						/* If dual port, copy ARDCLK into BWR */
						else if (!singlePort && 
							((islice == 0 && strcmp(portName, "s0_CLKARDCLK") == 0) ||
							(islice == 1 && strcmp(portName, "s1_CLKARDCLK") == 0))) {
							int ipin, num_pins1, num_pins2;
							int inet;
							int net_num;
							num_pins1 = block[iblk].type->pb_type->ports[iport].num_pins;
							assert(num_pins1 == 1);
							inet = block[iblk].nets[ptc];
							assert(inet != OPEN);
							if (islice == 0)
								assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "s0_CLKBWRCLK") == 0);
							else if (islice == 1)
								assert(strcmp(block[iblk].type->pb_type->ports[iport+1].name, "s1_CLKBWRCLK") == 0);
							num_pins2 = block[iblk].type->pb_type->ports[iport+1].num_pins;
							assert(num_pins2 == 1);
							for (ipin = 0; ipin < num_pins2; ++ipin) {
								assert(block[iblk].nets[ptc+num_pins1+ipin] == OPEN);
								net_num = add_sink(iblk, inet, ptc+num_pins1+ipin);
								assert(net_num == OPEN);
							}
							ptc += num_pins1;
							++iport;
						}

						ptc += block[iblk].type->pb_type->ports[iport].num_pins;
					}
				}
				num_slices = block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children;
			}
		}
	}
}

void
transform_clocks(void) {
	int inet;
	for (inet = 0; inet < num_nets; ++inet) {
		if (!clb_net[inet].is_global)
			continue;
		if (strcmp(clb_net[inet].name, "gnd") != 0 &&
			strcmp(clb_net[inet].name, "vcc") != 0) {
			clb_net[inet].is_global = FALSE;
		}
	}
}


