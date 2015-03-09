/*
obj_downloader
This node get location data from db server and 
publish data as ractangular plane
*/

#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <geo_pos_conv.hh>
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

string serverName = "db3.ertl.jp";
int PORT = 5678;

enum TYPE{
  NORMAL,
  RANGE,
  TEST,
  DB1
};

typedef struct{
  int gps_id;
  double lat;
  double lon;
  double ele;
  std::string timestamp;
  double x;
  double y;
  double z;
}car_info;

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

void marker_publisher(const std_msgs::String msg)
{
  std::vector<car_info> cars;
  std::vector<std::string> db_data, tmp;
  visualization_msgs::Marker sphere_list;
  car_info a;
  sphere_list.header.frame_id = "/mobility";
  sphere_list.header.stamp = ros::Time::now();
  sphere_list.ns = "mo_marker";
  sphere_list.id = 0;
  sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_list.action = visualization_msgs::Marker::ADD;
  sphere_list.lifetime = ros::Duration();
  sphere_list.color.r = 1.0;
  sphere_list.color.g = 0.0;
  sphere_list.color.b = 0.0;
  sphere_list.color.a = 1.0;
  sphere_list.scale.x = 10.0;
  sphere_list.scale.y = 10.0;
  sphere_list.scale.z = 10.0;
  ros::Rate rate(50);
  geo_pos_conv geo;
  geo.set_plane(7);
  // Loading data.
  db_data = split(msg.data.c_str(),'\n');
  for(uint i = 0; i < (int)db_data.size(); i++){
    geometry_msgs::Point p;
    if(db_data[i].compare("")==0) continue;
    tmp = split(db_data[i], '\t');
    if(tmp.size()!=5) continue;
    a.gps_id = std::stoi(tmp[0]);
    a.lat = std::stod(tmp[1].c_str());
    a.lon = std::stod(tmp[2].c_str());
    a.ele = std::stod(tmp[3].c_str());
    a.timestamp = tmp[4];
    // Convert from lat,lon to x,y
    // geo.set_llh_nmea_degrees(a.lat, a.lon, a.ele);
    geo.llh_to_xyz(a.lat, a.lon, a.ele);
    a.x = geo.x();
    a.y = geo.y();
    a.z = geo.z();
    // swap x and y
    p.x = a.y;
    p.y = a.x;
    p.z = a.z;
    cars.push_back(a);
    sphere_list.points.push_back(p);
    std::cout << "gps_id: " << cars[i].gps_id << std::endl;
    std::cout << "lat: " << cars[i].lat << std::endl;
    std::cout << "lon: " << cars[i].lon << std::endl;
    std::cout << "ele: " << cars[i].ele << std::endl;
    std::cout << "timestamp: " << cars[i].timestamp << std::endl;
    std::cout << "x: " << cars[i].x << std::endl;
    std::cout << "y: " << cars[i].y << std::endl;
    std::cout << "z: " << cars[i].z << std::endl;
  }

  pub.publish(sphere_list);
  // Free vector
  std::vector<car_info>().swap(cars);
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
  case RANGE://first argument is 10003
    {
      oss << "select 0,latitude,longitude,azimuth,timestamp from test_map where latitude >= " << fixed << setprecision(7) << positionRange[0] << " and latitude < "  << fixed << setprecision(7) << positionRange[1] << " and longitude >= " << fixed << setprecision(7) << positionRange[2] << " and longitude < " << fixed << setprecision(7) << positionRange[3] << " and timestamp > TO_TIMESTAMP(Second,SINCE_EPOCH(Second,current_timestamp)-1) and timestamp <= current_timestamp;";
      data += oss.str();
      break;
    }
  case TEST://first argument is 10002
    {
      oss << "select 0,latitude,longitude,azimuth,timestamp from test_map where latitude >= " << fixed << setprecision(7) << positionRange[0] << " and latitude < "  << fixed << setprecision(7) << positionRange[1] << " and longitude >= " << fixed << setprecision(7) << positionRange[2] << " and longitude < " << fixed << setprecision(7) << positionRange[3] << " order by timestamp desc limit 1";
      data += oss.str();
      break;
    }
  case DB1://first argument is 10001
    {
      oss << "select id,lat,lon,ele,timestamp from select_test where timestamp = (select max(timestamp) from select_test) and lat >= " << fixed << setprecision(7) << positionRange[0] << " and lat < "  << fixed << setprecision(7) << positionRange[1] << " and lon >= " << fixed << setprecision(7) << positionRange[2] << " and lon < " << fixed << setprecision(7) << positionRange[3] << ";<E>";
      data = oss.str();
      break;
    }
  case NORMAL:
  default:
    oss << "select 0,latitude,longitude,azimuth,timestamp from test_map where latitude >= 30 and latitude < 40 and longitude >= 130 and longitude < 140 and timestamp > TO_TIMESTAMP(Second,SINCE_EPOCH(Second,current_timestamp)-1) and timestamp <= current_timestamp;";
  }

  data += "\n";

  cout << "sql : " << data << endl;
  //printf("sql : %s\n",data.c_str());

  dbres = sd.Sender(data);

  printf("%lu\n",dbres.size());

  std_msgs::String msg;
  msg.data = dbres.c_str();
  
  marker_publisher(msg);

  return nullptr;

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

  return nullptr;
}


int main(int argc, char **argv){
  
  ros::init(argc ,argv, "obj_downloader") ;
  ros::NodeHandle nh;
  
  cout << "obj_downloader" << endl;

  pub = nh.advertise<visualization_msgs::Marker>("mo_marker",1); 
  //ros::Subscriber subscriber = nh,subscribe("topic_name",1000,Callback_Name)

  if(argc == 1){
    printf("normal execution\n");
    SendDataType = NORMAL;
  }else if(argc == 5){
    if(static_cast<std::string>(argv[1]).compare("10000")==0){
      printf("normal access\n");
      SendDataType = NORMAL;
    }else if(static_cast<string>(argv[1]).compare("10001")==0){
      printf("fixed range access\n");
      positionRange[0] = 35.2038955;
      positionRange[1] = 35.2711311;
      positionRange[2] = 136.9813925;
      positionRange[3] = 137.055852;
      serverName = "db1.ertl.jp";
      PORT = 5700;
      SendDataType = DB1;

    }else if(static_cast<string>(argv[1]).compare("10002") == 0){
      printf("test access\n");
      positionRange[0] = 30;
      positionRange[1] = 40;
      positionRange[2] = 130;
      positionRange[3] = 140;
      SendDataType = TEST;
    }else if(static_cast<string>(argv[1]).compare("10003") == 0){
      printf("current data get test access\n");
      positionRange[0] = 30;
      positionRange[1] = 40;
      positionRange[2] = 130;
      positionRange[3] = 140;
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

  sd = SendData(serverName,PORT);
  counter = 0;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }

  pthread_detach(th);

  ros::spin();

}
