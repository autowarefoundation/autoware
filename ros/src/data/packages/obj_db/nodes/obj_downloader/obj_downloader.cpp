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
//#include "scan_to_image/ScanImage.h"
#include "../SendData.h"

using namespace std;

string serverName = "db1.ertl.jp";
int PORT = 5678;

enum TYPE{
  NORMAL,
  RANGE,
  TEST,
  DTN
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

  //create header
  char magic[5] = "MPWC";
  u_int16_t major = htons(1);
  u_int16_t minor = htons(0);
  u_int32_t sqlinst = htonl(1);
  u_int32_t sqlnum = htonl(1);
  char header[16];
  memcpy(header,magic,4);
  memcpy(&header[4],&major,2);
  memcpy(&header[6],&minor,2);
  memcpy(&header[8],&sqlinst,4);
  memcpy(&header[12],&sqlnum,4);
  data.append(header,16);

  switch (SendDataType){
  case RANGE:
    {
      oss << "select id,lat,lon,ele,timestamp from select_test where timestamp = (select max(timestamp) from select_test) and lat >= " << fixed << setprecision(7) << positionRange[0] << " and lat < "  << fixed << setprecision(7) << positionRange[1] << " and lon >= " << fixed << setprecision(7) << positionRange[2] << " and lon < " << fixed << setprecision(7) << positionRange[3] << ";";
      /*
      for(int i=0; i<4; i++){
	oss << "\t" << fixed << setprecision(7) <<positionRange[i];
      }
      */
      data += oss.str();
      break;
    }
  case DTN:
    {
      oss << "select terminal,latitude,longitude,azimuth,timestamp from test_map where timestamp = (select max(timestamp) from test_map) and latitude >= " << fixed << setprecision(7) << positionRange[0] << " and latitude < "  << fixed << setprecision(7) << positionRange[1] << " and longitude >= " << fixed << setprecision(7) << positionRange[2] << " and longitude < " << fixed << setprecision(7) << positionRange[3] << ";";
      //oss << "select tm,id,x,y,type,self,area from pos_nounique where tm = (select max(tm) from pos_nounique);";
      //oss << "select * from pos_nounique limit 1;";
      data += oss.str();
      break;
    }
  case NORMAL:
  default:
    data += "select id,lat,lon,ele,timestamp from select_test where timestamp = (select max(timestamp) from select_test) and lat >= 35.2038955 and lat < 35.2711311 and lon >= 136.9813925 and lon < 137.055852;";
  }

  data += "\r\n";

  cout << "sql : " << data << endl;
  //printf("sql : %s\n",data.c_str());

  dbres = sd.Sender(data);

  printf("%lu\n",dbres.size());

  std_msgs::String msg;
  msg.data = dbres.c_str();

  if(msg.data.compare("") != 0){
    ROS_INFO("test\t%s",msg.data.c_str());
    pub.publish(msg);
  }

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
  
  cout << "obj_downloader" << endl;

  pub = nh.advertise<std_msgs::String>("mo",10); 
  //ros::Subscriber subscriber = nh,subscribe("topic_name",1000,Callback_Name)

  if(argc == 1){
    printf("normal execution\n");
    SendDataType = NORMAL;
  }else if(argc == 5){
    if(static_cast<std::string>(argv[1]).compare("10000")==0){
      printf("normal access\n");
      SendDataType = NORMAL;
    }else if(static_cast<string>(argv[1]).compare("10002")==0){
      printf("fixed range access\n");
      positionRange[0] = 35.2038955;
      positionRange[1] = 35.2711311;
      positionRange[2] = 136.9813925;
      positionRange[3] = 137.055852;

      SendDataType = RANGE;
    }else if(static_cast<string>(argv[1]).compare("10003") == 0){
      printf("fixed range access\n");
      positionRange[0] = 30;
      positionRange[1] = 40;
      positionRange[2] = 130;
      positionRange[3] = 140;
      PORT = 5678;
      serverName = "db3.ertl.jp";
      SendDataType = DTN;
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

  sd = SendData(serverName,PORT);
  counter = 0;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }

  pthread_detach(th);

  ros::spin();

}
