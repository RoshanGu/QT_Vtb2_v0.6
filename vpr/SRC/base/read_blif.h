#ifndef	READ_BLIF_H
#define READ_BLIF_H

void read_and_process_blif(char *blif_file,
		boolean sweep_hanging_nets_and_inputs, t_model *user_models,
		t_model *library_models, boolean read_activity_file,
		char * activity_file);
void echo_input(char *blif_file, char *echo_file, t_model *library_models);
int add_vpack_net(char *ptr, int type, int bnum, int bport, int bpin,
		boolean is_global, int doall);
void dum_parse(char *buf);

#endif /*READ_BLIF_H*/
