#include <stdio.h>
#include <string.h>
#include "shm_client.h"
#include <iostream>
#include <sys/time.h>
#include <stdlib.h>
#include <math.h>
using namespace std;

int main(int argc, char **argv) {

  SHMClient _shared_memory;
  struct timeval start,end;

  int velocity = 5;
  int time = 15;
  vel_data_t test_cmd;
  memset(&test_cmd, 0 ,sizeof(test_cmd));
  _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
  printf("side brake release ok?(y) : ");
  char side[1]; 
  cin >> side;

  if(strcmp(side,"y") == 0){

    //速度コマンドテスト
    
   
    /*
    int i=0;
    int j=0;
    printf("accelerate!\n");
    /*for(i = 0;i <=velocity;i++){
      test_cmd.tv = i;
      test_cmd.sv = 0;
      test_cmd.tstamp =0;

      printf("set accelerate control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
      _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
      usleep(100000); 
      }*/
    /* gettimeofday(&start,NULL);
    printf("clock start : %d\n",start.tv_sec);

 
    while(1){
      test_cmd.tv = velocity;
      test_cmd.sv = 0;
      test_cmd.tstamp =0;

      printf("set accelerate control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
      _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);

      gettimeofday(&end,NULL);
      //printf("clock : %d\n",end.tv_sec);
      printf("time progress : %d\n",end.tv_sec-start.tv_sec);
      if(end.tv_sec - start.tv_sec == time)break;
      usleep(100000);
    }
    
    printf("decelerate!\n");
    for(j = velocity;j >=0;j--){
      test_cmd.tv = j;
      test_cmd.sv = 0;
      test_cmd.tstamp =0;
    
      printf("set decelerate control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
      _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
      usleep(100000);
      }
  
    */

    //ステアコマンドテスト
    double degree = 45;
    for (int i = 0;i<=degree;i++){
    test_cmd.tv = 0;
    test_cmd.sv = i * M_PI/180;
    test_cmd.tstamp = 0;
    printf("set steering control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
    _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
    usleep(100000); 
    }
    for (int j = degree;j>=0;j--){
      test_cmd.tv = 0;
      test_cmd.sv = j * M_PI/180;
    test_cmd.tstamp = 0;
    printf("set steering control cmd tv: %.3lf sv: %.3lf\n",test_cmd.tv,test_cmd.sv);
    _shared_memory.writeControl(SHM_CONTROL_MODE_AUTO,test_cmd);
    usleep(100000); 
    }
  }else{
    printf("exit\n");
    exit(-1);
  }
  return 0;
}
