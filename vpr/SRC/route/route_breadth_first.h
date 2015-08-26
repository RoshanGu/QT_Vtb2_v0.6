boolean try_breadth_first_route(struct s_router_opts router_opts,
		t_ivec ** clb_opins_used_locally, int width_fac);
		
void breadth_first_expand_trace_segment(struct s_trace *start_ptr,
					       int remaining_connections_to_sink);

void breadth_first_add_source_to_heap(int inet);