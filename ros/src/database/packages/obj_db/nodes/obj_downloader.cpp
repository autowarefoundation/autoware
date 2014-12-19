 //obj_donwloaderのサンプルソースファイルです

#include "ros/ros.h"

#include <iostream>

/*void Callback_Name(topic_type topic_name){
  
}
*/

int main(int argc, char **argv){
  
  ros::init(argc ,argv, "obj_downloader") ;
  ros::NodeHandle nh;
  
  std::cout << "obj_downloader" << std::endl;
 
  
  //ros::Subscriber subscriber = nh,subscribe("topic_name",1000,Callback_Name)
  

  ros::spin();
}
