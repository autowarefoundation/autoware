#include <stdio.h>
#include <string.h>
#include "shm_client.h"
#include <iostream>
#include <sys/time.h>
using namespace std;

int main(int argc, char **argv) {

  SHMClient _shared_memory;
  struct timeval start,end;
  gettimeofday(&start,NULL);
  printf("clock start : %d\n",start.tv_sec);
  vel_data_t test_cmd;
  memset(&test_cmd, 0 ,sizeof(test_cmd));
  
  int i=0;
  int j=0;
  printf("accelerate!\n");
  for(i = 0;i <6;i++){
  test_cmd.tv = i;
  test_cmd.sv = 0;
  test_cmd.tstamp =0;

  printf("set accelerate control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
  _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
  usleep(100000); 
  }


 
  while(1){
    test_cmd.tv = 5;
    test_cmd.sv = 0;
    test_cmd.tstamp =0;

    printf("set accelerate control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
    _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);

  gettimeofday(&end,NULL);
  //printf("clock : %d\n",end.tv_sec);
  printf("time progress : %d\n",end.tv_sec-start.tv_sec);
  if(end.tv_sec - start.tv_sec == 5)break;
    usleep(100000);
  }
    
  printf("decelerate!\n");
  for(j = 5;j >=0;j--){
    test_cmd.tv = j;
    test_cmd.sv = 0;
    test_cmd.tstamp =0;
    
    printf("set decelerate control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
    _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
    usleep(100000);
  }
  return 0;
}
