
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <strings.h>
#include <string.h>
#include <errno.h>
#include "shm_client.h"
#include "state_comm_shm.h"


#define LIVE_ROBOT 0

int has_to_stop;

#define CYCLE_TIME (13333)

void sigquit_handler(int q __attribute__((unused))){
  fprintf(stderr,"sigquit handler\n");
	has_to_stop=1;
}

#define rdtscll(val) \
  __asm__ __volatile__("rdtsc" : "=A" (val))


double cputick_per_us = 0; // cputick for 1us

/*
int odomWriteStrHandler(int socket, SHMClient * shm)
{
  fprintf(stderr,"in odomHandler\n");
  
  int buffer_size = 256;
  char buffer[buffer_size];
  bzero(buffer, buffer_size);

  state_data_t state = shm->readState();
  sprintf(buffer, "%.5f %.5f %.5f %.5f %.5f %.5f\n", 
	  state.base.odom_pose.x, state.base.odom_pose.y, state.base.odom_pose.z, 
	  state.base.odom_pose.R, state.base.odom_pose.P, state.base.odom_pose.Y);

  fprintf(stderr,"odomWriteStrHandler(): write>%s\n",buffer);

  int bs = write(socket, buffer, strlen(buffer));
  if (bs == strlen(buffer)){
    fprintf(stderr,"write ok\n");
    return 1;
  }
  fprintf(stderr,"bs = %d returning 0\n", bs);
  return 0;
}
*/

int cmdReadHandler(int socket_fd, SHMClient * shm)
{
  fprintf(stderr,"in cmdReadHandler\n");  
  
  vel_cmd_data_t vel_cmd;
  bzero(&vel_cmd, sizeof(vel_cmd));
  
  int br = read(socket_fd, &(vel_cmd), sizeof(vel_cmd_data_t));
  if (br < 0) {
    fprintf(stderr,"cmdReadHandler(): bytes read < 0 \n");
    return 0;
  }
  
  if (br == sizeof(vel_cmd)){
    shm->writeControl(vel_cmd.mode, vel_cmd.vel);
    return 1;
  }
  fprintf(stderr,"br = %d returning 0, errno = %d\n", br, errno);
  return 0;
}

int fn = 0;

int main(int argc, char** argv){
 
	has_to_stop=0;
	signal(SIGINT, sigquit_handler);
      
	
	SHMClient shm;
	
	StateCommShm server(&shm, STATE_VEL_CMD, SERVER_MODE, 
			    STATE_COMM_DEFAULT_VEL_CMD_PORT, 
			    cmdReadHandler);
	
	server.start();
	while(! has_to_stop){
	  // rdtscll(t1);
	  
	  usleep(50000);
	  fn++;
	}
	server.stop();
	return 0;
}
