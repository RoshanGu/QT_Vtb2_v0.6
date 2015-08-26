boolean
inc_directed_search_route_net(int inet,
		float pres_fac,
		float astar_fac,
		float bend_cost,
		struct s_trace **old_trace_tail,
		int old_num_nets,
		struct s_router_opts *router_opts,
		int num_tb,
		t_trace_buffer *tb,
		float criticality,
		int **inode2fanouts);
