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
		t_ivec **clb_opins_used_locally);

