void inc_reset_rr_node_route_structs(void);
void inc_update_one_cost(struct s_trace *route_segment_start, int add_or_sub, int **inode2fanout, float pres_fac);
void inc_free_traceback(int inet, struct s_trace **old_trace_tail);
int inc_local_OPINs(int inet, int old_num_nets, int **sources);
int inc_get_expected_segs_to_target(int inode, int *num_segs_ortho_dir_ptr, int target_x, int target_y);
