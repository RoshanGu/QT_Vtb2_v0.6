typedef struct s_trace_buffer
{
	int x, y;
	int iblk;
	int data_pin;
	int usage;
	int capacity;
	int *sink_inodes;
} t_trace_buffer;

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
		float T_crit);

int 
inc_reclaim_tbs(t_trace_buffer **tb, const t_arch *arch);

int 
inc_setup_trace(int **trace_nets, int *new_num_nets);

void 
inc_remove_clb_net(int inet);

