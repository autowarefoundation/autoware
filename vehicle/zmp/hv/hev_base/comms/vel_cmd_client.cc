
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



int cmdWriteHandler(int socket_fd, SHMClient * shm)
{
  fprintf(stderr,"in cmdWriteHandler\n");
  
  vel_cmd_data_t vel_cmd = shm->readVelCmd();
  
  int bs = write(socket_fd, &(vel_cmd), sizeof(vel_cmd));
  if (bs == sizeof(pose_data_t)){
    return 1;
  }
  fprintf(stderr,"bs = %d returning 0, errno = %d\n", bs, errno);
  return 0;
}

int fn = 0;

int main(int argc, char** argv){
 
	has_to_stop=0;
	signal(SIGINT, sigquit_handler);
      
	
	SHMClient shm;
	
	StateCommShm client(&shm, STATE_VEL_CMD, CLIENT_MODE, STATE_COMM_DEFAULT_VEL_CMD_PORT, cmdWriteHandler, "192.168.1.100");
	client.start();
	while(! has_to_stop){
	  // rdtscll(t1);
	  
	  usleep(100000);
	  fn++;
	}
	client.stop();
	return 0;
}
