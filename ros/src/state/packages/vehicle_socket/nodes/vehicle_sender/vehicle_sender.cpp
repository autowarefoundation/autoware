
/*#include "ros/ros.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <string>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#include <vehicle_socket/CanInfo.h>

#include "SendData.h"
using namespace std;



void CMDCallback(const string a)
{




}

void GearCallback(const ui_socket::gear_cmdConstPtr &gear)
{




}


int main(int argc, char **argv){

  ros::init(argc ,argv, "vehicle_sender") ;
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;
  ros::Subscriber sub[5];
  sub[0] = nh.subscribe("auto_cmd", 100,CMDCallback);
  sub[0] = nh.subscribe("gear_cmd", 100,GearCallback);

  pthread_t th;
  if(pthread_create(&th, NULL, senderCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}*/
