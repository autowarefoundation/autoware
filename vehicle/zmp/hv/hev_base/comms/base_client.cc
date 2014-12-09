
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <strings.h>
#include <string.h>
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



int baseReadHandler(int socket_fd, SHMClient * shm)
{
  

  base_data_t base;
  bzero(&base, sizeof(base));

  //fprintf(stderr,"client calling read\n");
  int br = read(socket_fd, &base, sizeof(base));
  if (br < 0){
    fprintf(stderr,"bytes read <=0\n");
    return 0;
  }
  if (br == sizeof(base)) {
    fprintf(stderr,"read base, odom: %.2f %.2f %.2f\n", base.odom_pose.x, base.odom_pose.y, base.odom_pose.Y);
    shm->writeBaseState(base);
    
  }
  return 1;
}

static int fn = 0;
//---------------------------------------------------------------
//
//
//
//---------------------------------------------------------------

int main(int argc, char** argv){
 
	has_to_stop=0;
	signal(SIGINT, sigquit_handler);
      
	
	SHMClient shm;
	
	StateCommShm client(&shm, STATE_BASE, CLIENT_MODE, 
			    STATE_COMM_DEFAULT_BASE_PORT, 
			    baseReadHandler, 
			    "192.168.1.100");
	
	client.start();
	while(! has_to_stop){
	  // rdtscll(t1);
	  
	  usleep(50000);
	  fn++;
	}
	client.stop();
	return 0;
}
