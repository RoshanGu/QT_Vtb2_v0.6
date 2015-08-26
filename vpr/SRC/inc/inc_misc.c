#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "vpr_types.h"
#include "globals.h"
#include "route_common.h"
#include "rr_graph_util.h"
#include "rr_graph2.h"

#include "inc_misc.h"

/**
 * Write out the net name -> vnet mapping into <circuitname>.map
 */
void 
inc_dump_nets(struct s_file_name_opts *FileNameOpts)
{
	FILE *fp;
	int vnet, inet, iblk;
	char *map_file;

	inc_infer_vpack_blocks_and_pins();

	map_file = (char *) malloc(sizeof(char)*strlen(FileNameOpts->CircuitName)+strlen(".map")+1);
	strcpy(map_file, FileNameOpts->CircuitName);
	strcat(map_file, ".map");
	fp = fopen(map_file, "w");
	for (vnet = 0; vnet < num_logical_nets; vnet++)
	{
		inet = vpack_to_clb_net_mapping[vnet];
		if (inet == OPEN)
			iblk = vpack_net[vnet].node_block[0];
		else
			iblk = clb_net[inet].node_block[0];
		if ((inet == OPEN || !clb_net[inet].is_global) && iblk != OPEN)
			fprintf(fp, "%d,%s,%d,%d\n", vnet, vpack_net[vnet].name, block[iblk].x, block[iblk].y);
	}
	fclose(fp);
	free(map_file);
}

/**
 * Infer each vpack_net with node_block[] and node_block_pins[] 
 * from its block's pb structure
 */
void 
inc_infer_vpack_blocks_and_pins(void)
{
	int iblk;
	//printf("name before 1st chk for name : %s  and num of blocks-> %d \n", type_descriptors[2].name, num_blocks);
	assert(strcmp(type_descriptors[2].name, "SLICEL") == 0); //change
	
	for (iblk = 0; iblk < num_blocks; iblk++)
	{	
		//printf(" name of this type : %s  and index is : %d \n", block[iblk].type->name, block[iblk].type->index );
		if (block[iblk].type->index == 2) /* clb */
		{
			int ible, nbles;
			assert(block[iblk].pb->pb_graph_node->pb_type->num_pb == 1);
			//printf(" num_modes_actually : %d \n", block[iblk].pb->pb_graph_node->pb_type->num_modes );
			assert(block[iblk].pb->pb_graph_node->pb_type->num_modes == 3); //change 1 -> 3
			//printf(" num_pb_type_children mode 0 actually : %d \n", block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children );
			//printf(" num_pb_type_children mode 1 actually : %d \n", block[iblk].pb->pb_graph_node->pb_type->modes[1].num_pb_type_children );
			//printf(" num_pb_type_children mode 2 actually : %d \n", block[iblk].pb->pb_graph_node->pb_type->modes[2].num_pb_type_children );
			assert(block[iblk].pb->pb_graph_node->pb_type->modes[0].num_pb_type_children == 2); //change 1- 2
			nbles = block[iblk].pb->pb_graph_node->pb_type->modes[0].pb_type_children[0].num_pb;
			for (ible = 0; ible < nbles; ible++)
			{
				int ipin, npins;
				//printf(" num_output_ports : %d \n", block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_ports );
				//printf(" name of child_pbs : %s \n", block[iblk].pb->child_pbs[0][ible].name );
				//printf(" num_output_ports here should be: %d and name: %s \n", block[iblk].pb->pb_graph_node->num_output_ports, block[iblk].pb->name );
				assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_ports == 4); //change 1 -> 4
				npins = block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_pins[0];
				for (ipin = 0; ipin < npins; ipin++)
				{
					int ptc, inet;
					/*assert(block[iblk].pb->child_pbs[0][ible].pb_graph_node->num_output_pins[0] == 1);*/
					/* BLE OPIN ptc */
					ptc = block[iblk].pb->child_pbs[0][ible].pb_graph_node->output_pins[0][ipin]
						.pin_count_in_cluster;
					inet = block[iblk].pb->rr_graph[ptc].net_num;
					if (inet == OPEN) continue;

					//printf(" num_output_ports here should be: %d and name: %s \n", block[iblk].pb->pb_graph_node->num_output_ports, block[iblk].pb->name );
					assert(block[iblk].pb->pb_graph_node->num_output_ports == 15); //change from 1 - 15
					/*assert(block[iblk].pb->pb_graph_node->num_output_pins[0] == 10);*/
					/* CLB OPIN */
					ptc = block[iblk].pb->pb_graph_node->output_pins[0][ible*npins+ipin].pin_count_in_cluster;
					
					//if(strcmp(vpack_net[inet].name, "top^MULTI_PORT_MUX~1660^MUX_2~2527") == 0)
					//{
					 //  printf(" yehi number hai hai %d \n", vpack_net[inet].num_sinks);	
					//}
					//printf(" yehi number hai %d \n", vpack_net[inet].num_sinks);
					
					/*if (vpack_net[inet].node_block[0] != iblk)
					{
						//printf(" whats here: %d \n", vpack_net[inet].node_block[4]);
						//printf("and here; %d \n", vpack_net[inet].node_block_pin[4]);
						
						assert(vpack_net[inet].node_block[0] == OPEN);
						vpack_net[inet].node_block[0] = iblk;
					}*/
					assert(vpack_net[inet].node_block[0] != OPEN);

					/*if (vpack_net[inet].node_block_pin[0] != ptc)
					{
						assert(vpack_net[inet].node_block_pin[0] == OPEN);
						vpack_net[inet].node_block_pin[0] = ptc;
					}*/
					assert(vpack_net[inet].node_block_pin[0] != OPEN);
				}
			}
		}
	}
}

/** 
 * Read in just the overlay routing from file route_file.  
 */
boolean
inc_read_route(const char *route_file)
{
	FILE *fp;
	char line[1024], tmp[1024], *pline, name[1024];
	int i, inet, inode, ilow, jlow, ptc_num;
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
			if (trace_tail[inet])
			{
				ptptr = &(trace_tail[inet]->next);
			}
			else
			{
				ptptr = &(trace_head[inet]);
			}
			tptr_prev = NULL;

			while(fgets(line, 1024, fp) && line[0] != '\n')
			{
				assert(sscanf(line, "%6s (%d,%d) %n", type, &ilow, &jlow, &i) == 3);
				pline = line + i;
				for (i = 0; i < 7; i++)
				{
					if (strcmp(name_type[i], type) == 0)
					{
						rr_type = (t_rr_type)i;    //needs casting to enum type to avoid error - good practice
						break;
					}
				}
				assert(i < 7);
				assert(sscanf(pline, "%[^:]: %n", tmp, &i) == 1);
				pline += i;
				assert(sscanf(pline, "%d  %n", &ptc_num, &i) == 1);
				pline += i;
				inode = get_rr_node_index(ilow, jlow, rr_type, ptc_num, rr_node_indices);
				
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
			assert(FALSE);
		}
		else
		{
			assert(*pline == '\n' || *pline == ':');
		}
	}
	return (TRUE);
}

/** 
 * Prints out just the overlay routing to file route_file.  
 */
void 
inc_print_route(char *route_file, struct s_trace **old_trace_tail)
{

    int inet, inode, ilow, jlow;
    t_rr_type rr_type;
    struct s_trace *tptr;
    char *name_type[] =
	{ "SOURCE", "SINK", "IPIN", "OPIN", "CHANX", "CHANY", "INTRA_CLUSTER_EDGE" };
    FILE *fp;

    fp = fopen(route_file, "w");

    fprintf(fp, "Array size: %d x %d logic blocks.\n", nx, ny);
    fprintf(fp, "\nRouting:");
    for(inet = 0; inet < num_nets; inet++)
	{
		if (clb_net[inet].is_global)
			continue;

		if (old_trace_tail[inet])
			tptr = old_trace_tail[inet]->next;
		else
			tptr = trace_head[inet];

		if (!tptr || !tptr->next)
			continue;

		fprintf(fp, "\n\nNet %d (%s)\n\n", inet, clb_net[inet].name);

		while(tptr != NULL)
		{
			inode = tptr->index;
			rr_type = rr_node[inode].type;
			ilow = rr_node[inode].xlow;
			jlow = rr_node[inode].ylow;

			fprintf(fp, "%6s (%d,%d) ", name_type[rr_type],
				ilow, jlow);

			if((ilow != rr_node[inode].xhigh) || (jlow !=
							  rr_node
							  [inode].
							  yhigh))
			fprintf(fp, "to (%d,%d) ",
				rr_node[inode].xhigh,
				rr_node[inode].yhigh);

			switch (rr_type)
			{

			case IPIN:
			case OPIN:
				if(grid[ilow][jlow].type == IO_TYPE)
				{
					fprintf(fp, " Pad: ");
				}
				else
				{	/* IO Pad. */
					fprintf(fp, " Pin: ");
				}
				break;

			case CHANX:
			case CHANY:
				fprintf(fp, " Track: ");
				break;

			case SOURCE:
			case SINK:
				if(grid[ilow][jlow].type == IO_TYPE)
				{
					fprintf(fp, " Pad: ");
				}
				else
				{	/* IO Pad. */
					fprintf(fp, " Class: ");
				}
				break;

			default:
				printf
				("Error in print_route:  Unexpected traceback element "
				 "type: %d (%s).\n", rr_type,
				 name_type[rr_type]);
				exit(1);
				break;
			}

			fprintf(fp, "%d  ", rr_node[inode].ptc_num);

			/* Uncomment line below if you're debugging and want to see the switch types *
			 * used in the routing.                                                      */
			/*fprintf (fp, "Switch: %d", tptr->iswitch);*/

			fprintf(fp, "\n");

			tptr = tptr->next;
		}
	}

    fclose(fp);
}


