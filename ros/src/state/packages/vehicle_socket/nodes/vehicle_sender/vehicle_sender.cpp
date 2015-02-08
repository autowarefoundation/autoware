
#include "ros/ros.h"

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



void CMDCallback(const geometry_msgs::TwistConstPtr &msg)
{
 double linear_x = msg->linear.x;
 double angular_z = msg->angular.z;


}

//void ModeCallback(){}

/*void GearCallback(const ui_socket::gear_cmdConstPtr &gear)
{
}
*/

int main(int argc, char **argv){

  ros::init(argc ,argv, "vehicle_sender") ;
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;
  ros::Subscriber sub[1];
  sub[0] = nh.subscribe("cmd_vel", 100,CMDCallback);
//sub[1] = nh.subscribe("",100,ModeCallback);
  //sub[1] = nh.subscribe("gear_cmd", 100,GearCallback);

 /* pthread_t th;
  if(pthread_create(&th, NULL, senderCaller, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);
*/
  ros::spin();

  return 0;
}
