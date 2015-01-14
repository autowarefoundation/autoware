#include "ros/ros.h"

#include <iostream>

int main(int argc, char **argv){
  
  ros::init(argc ,argv, "vehicle_receiver") ;
  ros::NodeHandle nh;
  
  std::cout << "vehicle receiver" << std::endl;
  return 0;
}
