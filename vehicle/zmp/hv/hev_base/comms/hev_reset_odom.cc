#include <stdio.h>
#include <string.h>
#include "shm_client.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv) {

  SHMClient _shared_memory;

  pose_data_t zero_odom;
  memset(&zero_odom, 0, sizeof(zero_odom));
  _shared_memory.writeOdomPose(zero_odom);
  fprintf(stderr,"shm reset!\n");
  
}
