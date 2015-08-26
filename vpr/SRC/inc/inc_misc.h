void inc_dump_nets(struct s_file_name_opts *FileNameOpts);
void inc_infer_vpack_blocks_and_pins(void);

boolean inc_read_route(const char *route_file);
void inc_print_route(char *route_file, struct s_trace **old_trace_tail);
