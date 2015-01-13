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
#include "opencv/cv.h" 
#include "opencv/highgui.h" 
#include "opencv/cxcore.h" 
#include "std_msgs/Float64.h"
#include "scan_to_image/ScanImage.h"
#include "SendData.h"

using namespace std;

enum TYPE{
  NORMAL,
  RANGE,
  TEST
};


ros::Publisher pub;

double positionRange[4];
SendData sd;
TYPE SendDataType;
int counter;

std::vector<std::string> split(const string& input, char delimiter)
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
  if(str.find_first_not_of("-0123456789. Ee\t") != string::npos) return false;
  return true;
}

//wrap SendData class
void* wrapSender(void *tsd){

  //I assume that values has 4 value ex: "0 0 0 0"   "1 2 3 4"
  //And if setting the other number of value , sendData will be failed.

  string dbres;
  string data;
  stringstream oss;

  switch (SendDataType){
  case TEST:
    {
      oss << "select order\t" << counter << "<E>";
      data = oss.str();
      printf("test\n");
      counter++;
      break;
    }
  case RANGE:
    {
      oss <<  "select order";
      for(int i=0; i<4; i++){
	oss << "\t" << fixed << setprecision(7) <<positionRange[i];
      }
      oss << "<E>";
      data = oss.str();
      break;
    }
  case NORMAL:
  default:
    data = "select order<E>";
  }

  sd.setValue(data);
  dbres = sd.Sender();

  printf("%lu\n",dbres.size());

  std_msgs::String msg;
  msg.data = dbres.c_str();

  /*
  ostringstream ss;
  ss << dbres;
  msg.data = ss.str();
  */

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
  char serverName[100] = "db1.ertl.jp";
  
  cout << "obj_downloader" << endl;

  pub = nh.advertise<std_msgs::String>("mo",1000); 
  //ros::Subscriber subscriber = nh,subscribe("topic_name",1000,Callback_Name)

  if(argc == 1){
    printf("normal execution\n");
    SendDataType = NORMAL;
  }else if(argc == 2){
    string argFlag = argv[1];
    if(argFlag.compare("--test") == 0){
      printf("test access\n");
      SendDataType = TEST;
    }else{
      printf("invalid argment\n");
      SendDataType = NORMAL;
    }
  }else if(argc == 5){
    if(static_cast<std::string>(argv[1]).compare("10000")==0){
      printf("normal access\n");
      SendDataType = NORMAL;
    }else if(static_cast<string>(argv[1]).compare("10001")==0){
      printf("test access\n");
      SendDataType = TEST;
    }else if(static_cast<string>(argv[1]).compare("10002")==0){
      printf("fixed range access\n");
      positionRange[0] = 35.2038955;
      positionRange[1] = 35.2711311;
      positionRange[2] = 136.9813925;
      positionRange[3] = 137.055852;

      SendDataType = RANGE;
    }else{
      printf("range access\n");
      string arg;
      for(int i=1; i<5 ;i++){
	arg = argv[i];
	if(!isNumeric(arg)){
	  fprintf(stderr,"argment is not numeric.%s\n",arg.c_str());
	  exit(1);
	}
	positionRange[i-1] = atof(arg.c_str());

	if(!(positionRange[i-1]>=-360 && positionRange[i-1]<=360)){
	  fprintf(stderr,"error.\ninvalid range.\n");
	  exit(1);
	}
      }
      SendDataType = RANGE;
    }
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
