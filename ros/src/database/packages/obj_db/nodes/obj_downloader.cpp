 //obj_donwloaderのサンプルソースファイルです

#include "std_msgs/String.h"
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <vector>
#include <boost/array.hpp>
#include <iostream>
#include <string>
#include <sstream>
/*
#include "opencv/cv.h" 
#include "opencv/highgui.h" 
#include "opencv/cxcore.h" 
#include "std_msgs/Float64.h"
#include "scan_to_image/ScanImage.h"
*/
#include "SendData.h"

ros::Publisher pub;

char serverName[100] = "db1.ertl.jp";
std::string dbres;
SendData sd;

std::vector<std::string> split(const std::string& input, char delimiter)
{
    std::istringstream stream(input);

    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

//wrap SendData class
void* wrapSender(void *tsd){

  //I assume that values has 4 value ex: "0 0 0 0"   "1 2 3 4"
  //And if setting the other number of value , sendData will be failed.
  sd.setServerName(serverName);
  dbres = sd.Sender();

  printf("%lu\n",dbres.size());
  printf("%s\n",dbres.c_str());

  std_msgs::String msg;
  std::stringstream ss;
  ss << dbres;
  msg.data = ss.str();
  ROS_INFO("%s",msg.data.c_str());

  pub.publish(msg);

  /*
  std::vector<std::string>::iterator itr = dbres.begin();
  while(itr != dbres.end()){
    std::string temp = *itr;
    separateData = split(temp,'\t');
    std::cout << separateData[1] << std::endl;
    std::cout << *itr << std::endl;
    itr++;
  }
  */

}

void* intervalCall(void *a){

  pthread_t th;

  while(1){
    //create new thread for socket communication.      
    if(pthread_create(&th, NULL, wrapSender, NULL)){
      printf("thread create error\n");
    }
    sleep(1);
    if(pthread_join(th,NULL)){
      printf("thread join error.\n");
    }
  }
}


int main(int argc, char **argv){
  
  std::string argFlag = argv[1];
  if(argc == 1){
    sd = SendData(0);
  }else if(argc == 2){
    if(argFlag.compare("--test") == 0){
      printf("test access\n");
      sd = SendData(1);
    }else{
      printf("invalid argment\n");
      sd = SendData(0);
    }
  }


  ros::init(argc ,argv, "obj_downloader") ;
  ros::NodeHandle nh;
  
  std::cout << "obj_downloader" << std::endl;

  pub = nh.advertise<std_msgs::String>("mo",1000); 
  //ros::Subscriber subscriber = nh,subscribe("topic_name",1000,Callback_Name)

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }

  ros::spin();

}
