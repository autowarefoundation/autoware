#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include "../common/shm_common.h"

 

#define VERBOSE 1

int main(int argc, char ** argv)
{
    char    c;
    int     j, shmid;
    struct state_data_t *state_data;
    struct state_data_t data;
    unsigned char buffer[8192];
    long long int update_tstamp;
    key_t shm_key =SHM_STATE_KEY;

    int count = 0;
    
    fprintf(stderr, "shmserver: calling shmget for size = %d\n", (int)SHM_STATE_SZ);
    /* Create the shared memory segment. */
    if ((shmid = shmget(shm_key, SHM_STATE_SZ, IPC_CREAT|0666)) < 0) {
      perror("shmget");
      fprintf(stderr,"error no = %d\n", errno);
      if (errno == EEXIST) {
	fprintf(stderr,"shm already exists\n");
	
	
      
	
      }
    }
    else {
      fprintf(stderr,"got shmid = %d, key = 0x%x, size = %d\n",shmid, shm_key, (int)SHM_STATE_SZ); 
      if ((state_data = (struct state_data_t *)shmat(shmid, NULL, 0)) == (void *)-1) {
	perror("shmat");
	exit(1);
      }
      
      while(1) {



	if (VERBOSE == 1) {
	  memcpy(&data, state_data, SHM_STATE_SZ);
	  
	  fprintf(stderr, "shmserver -> %d\n\t odom (%.2f %.2f %.2f %.2f %.2f %.2f)\n\t estp (%.2f %.2f %.2f %.2f %.2f %.2f)\n\t vcmd (%.2f %.2f)\n \t vel_cur (%.2f %.2f)\n", 
		  count, 
		  data.base.odom_pose.x, data.base.odom_pose.y, data.base.odom_pose.z, 
		  data.base.odom_pose.R, data.base.odom_pose.P, data.base.odom_pose.Y,
		  data.est_pose.x, data.est_pose.y, data.est_pose.z, 
		  data.est_pose.R, data.est_pose.P, data.est_pose.Y,
		  data.vel_cmd.tv, data.vel_cmd.sv,
		  data.base.vel_cur.tv, data.base.vel_cur.sv);
	  fprintf(stderr,"\t batt = %.2f\n", (double)data.base.batt_voltage);
	  fprintf(stderr,"\t motor state = %d\n", (int) data.base.motor_state);
	}
	count++;
	usleep(100000);
      }
    }
    /* dettach the segment to data space */
    /*
    if (shm_unlink((const char *)shm_key) == -1){
    perror("shm_unlink");
    exit(1);
    }
    */
    return 0;
}

