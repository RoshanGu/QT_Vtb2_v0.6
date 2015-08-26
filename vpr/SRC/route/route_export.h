/******** Function prototypes for functions in route_common.c that ***********
 ******** are used outside the router modules.                     ***********/

boolean try_route(int width_fac, struct s_router_opts router_opts,
		struct s_det_routing_arch det_routing_arch, t_segment_inf * segment_inf,
		t_timing_inf timing_inf, float **net_delay, t_slack * slacks,
		t_chan_width_dist chan_width_dist, t_ivec ** clb_opins_used_locally,
		boolean * Fc_clipped, enum e_operation operation,
		t_direct_inf *directs, struct s_file_name_opts *FileNameOpts, int num_directs,
		const char *arch_file);

boolean feasible_routing(void);
int feasible_routing_score(void);

t_ivec **alloc_route_structs(void);

void free_route_structs();

struct s_trace **alloc_saved_routing(t_ivec ** clb_opins_used_locally,
		t_ivec *** saved_clb_opins_used_locally_ptr);

void free_saved_routing(struct s_trace **best_routing,
		t_ivec ** saved_clb_opins_used_locally);

void save_routing(struct s_trace **best_routing,
		t_ivec ** clb_opins_used_locally,
		t_ivec ** saved_clb_opins_used_locally);

void restore_routing(struct s_trace **best_routing,
		t_ivec ** clb_opins_used_locally,
		t_ivec ** saved_clb_opins_used_locally);

void get_serial_num(void);

void print_route(char *name);

t_ivec **
alloc_and_load_clb_opins_used_locally();

