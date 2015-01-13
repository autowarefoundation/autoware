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

/*
long startTime[10] = {
    1255698868,
    1255723190,
    1266425473,
    1266765883,
    1266851714,
    1266938129,
    1267471638,
    1267542268,
    1267715826,
    1268755256
};
*/


enum TYPE{
  NORMAL,
  RANGE,
  TEST
};


ros::Publisher pub;

char serverName[100] = "db1.ertl.jp";
double positionRange[4];

std::string dbres;
SendData sd;
TYPE SendDataType;
int counter;

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

bool isNumeric(const std::string str){
  if(str.find_first_not_of("-0123456789. Ee\t") != std::string::npos) return false;
  return true;
}

//wrap SendData class
void* wrapSender(void *tsd){

  //I assume that values has 4 value ex: "0 0 0 0"   "1 2 3 4"
  //And if setting the other number of value , sendData will be failed.

  std::string data;
  std::ostringstream oss;
  switch (SendDataType){
  case TEST:
    {
      oss << "select order\t" << counter << "<E>";
      data = oss.str();
      printf("test\n");
      counter++;
    }
  case NORMAL:
    data = "select order<E>";
  case RANGE:
    data = "select order<E>";
  }

  sd.setValue(data);
  dbres = sd.Sender();

  printf("%lu\n",dbres.size());

  std_msgs::String msg;
  std::stringstream ss;
  ss << dbres;
  msg.data = ss.str();

  if(msg.data.compare("") != 0){
    ROS_INFO("test\t%s",msg.data.c_str());
    pub.publish(msg);
  }

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
  
  ros::init(argc ,argv, "obj_downloader") ;
  ros::NodeHandle nh;
  
  std::cout << "obj_downloader" << std::endl;

  pub = nh.advertise<std_msgs::String>("mo",1000); 
  //ros::Subscriber subscriber = nh,subscribe("topic_name",1000,Callback_Name)

  if(argc == 1){
    printf("normal execution\n");
    SendDataType = NORMAL;
  }else if(argc == 2){
    std::string argFlag = argv[1];
    if(argFlag.compare("--test") == 0){
      printf("test access\n");
      SendDataType = TEST;
    }else{
      printf("invalid argment\n");
      SendDataType = NORMAL;
    }
  }else if(argc == 5){
    std::string arg;
    for(int i=1; i<5 ;i++){
      arg = argv[i];
      if(!isNumeric(arg)){
	fprintf(stderr,"argment is not numeric.%s\n",arg.c_str());
	return 0;
      }
      positionRange[i-1] = atof(arg.c_str());
    }
    SendDataType = RANGE;

  }else{
    fprintf(stderr,"The number of argment is invalid.\n");
    return 0;
  }
  sd = SendData(serverName,5678);
  counter = 0;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }

  ros::spin();

}
