/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
自分から認識した物体の種類と位置情報をデータベースに送信する

簡単な仕様：
１、pedestrian_pos_xyzとcar_pos_xyzから画面上のxy座標とdistanceを取得する
２、取得したxyとdistanceから上から見たxy座標を求める
３、このxy座標はカメラから見た座標なのでvelodyneからみ見た座標に変換する
４、これでvelodyneから座標が得られるのでこれを東西南北を軸とする直交座標に変換する
５、直交座標を緯度・経度に変換する
６、データベースサーバに対して1秒ごとに送信する

送信データのフォーマットは
緯度、経度、物体の種類、自動車のid

データは認識した物体ごとに送る

*/

#include "std_msgs/String.h"
#include "ros/ros.h"

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
//#include "car_detector/FusedObjects.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <sys/time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <SendData.h>

/*
#include "structure.h"
#include "calcoordinates.h"
#include "axialMove.h"
#include "geo_pos_conv.hh"
*/

using namespace std;

//store subscribed value
static geometry_msgs::PoseArray car_position_array;
static geometry_msgs::PoseArray pedestrian_position_array;

//default server name and port to send data
static const string defaultServerName = "db3.ertl.jp";
static const int PORT = 5678;
//magic that I am C++
static const char MAGIC[5] = "MPWC";
static const int area = 7;

//flag for comfirming whether updating position or not
static bool positionGetFlag;

//send to server class
static SendData sd;

//store own position and direction now.updated by position_getter
static geometry_msgs::PoseStamped my_loc;

static string getTimeStamp(long sec,long nsec){
  struct tm *tmp;
  struct timeval tv;
  char temp[30];
  string res;

  tv.tv_sec = sec;
  tv.tv_usec = nsec/1000;

  tmp=localtime(&tv.tv_sec);
  sprintf(temp,"%04d-%02d-%02d %02d:%02d:%02d.%d",
	  tmp->tm_year + 1900, tmp->tm_mon + 1,
	  tmp->tm_mday, tmp->tm_hour,
	  tmp->tm_min, tmp->tm_sec,
	  static_cast<int>(tv.tv_usec/1000));
  res = temp;
  return res;
}

static string makeSendDataDetectedObj(geometry_msgs::PoseArray cp_array){
  ostringstream oss;
  vector<geometry_msgs::Pose>::iterator cp_iterator;
  cp_iterator = cp_array.poses.begin();

  for(uint i=0; i<cp_array.poses.size() ; i++, cp_iterator++){
    //create sql
    oss << "INSERT INTO POS(id,x,y,z,area,type,tm) ";
    oss << "values('0'," << fixed << setprecision(6) << cp_iterator->position.y << ","
	<< fixed << setprecision(6) << cp_iterator->position.x << ","
	<< fixed << setprecision(6) << cp_iterator->position.z << ","
	<< area << ",0,'" << getTimeStamp(cp_array.header.stamp.sec,cp_array.header.stamp.nsec) << "');\n";
  }

  return oss.str();
}

//wrap SendData class
static void* wrapSender(void *tsd){
  ostringstream oss;
  string value;

  //create header
  char magic[5] = "MPWC";
  u_int16_t major = htons(1);
  u_int16_t minor = htons(0);
  u_int32_t sqlinst = htonl(2);
  u_int32_t sqlnum = htonl(car_position_array.poses.size()+pedestrian_position_array.poses.size()+1);
  char header[16];
  memcpy(header,magic,4);
  memcpy(&header[4],&major,2);
  memcpy(&header[6],&minor,2);
  memcpy(&header[8],&sqlinst,4);
  memcpy(&header[12],&sqlnum,4);
  value.append(header,16);

  cout << "sqlnum : " << car_position_array.poses.size() + pedestrian_position_array.poses.size() + 1 << endl;

  //get data of car and pedestrian recognizing
  if(car_position_array.poses.size() > 0 ){
    value += makeSendDataDetectedObj(car_position_array);
  }

  if(pedestrian_position_array.poses.size() > 0){
    value += makeSendDataDetectedObj(pedestrian_position_array);
  }

  oss << "INSERT INTO POS(id,x,y,z,area,type,tm) ";
  oss << "values('0'," <<  fixed << setprecision(6) << my_loc.pose.position.y << ","
      << fixed << setprecision(6) << my_loc.pose.position.x << ","
      << fixed << setprecision(6) << my_loc.pose.position.z << ","
      << area << ",0,'" << getTimeStamp(my_loc.header.stamp.sec,my_loc.header.stamp.nsec) << "');\n";
 
  value += oss.str();
  cout << value << endl;

  string res = sd.Sender(value);
  cout << "retrun message from DBserver : " << res << endl;

  return nullptr;
}

static void* intervalCall(void *a){
  pthread_t th;

  while(1){
    //If angle and position data is not updated from prevous data send,
    //data is not sent
    if(!positionGetFlag) {
      sleep(1);
      continue;
    }
    positionGetFlag = false;

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

static void car_locateCallback(const geometry_msgs::PoseArray car_locate)
{
  car_position_array = car_locate;
}

static void pedestrian_locateCallback(const geometry_msgs::PoseArray pedestrian_locate)
{
  pedestrian_position_array = pedestrian_locate;
}

static void position_getter_ndt(const geometry_msgs::PoseStamped &pose){
  my_loc = pose;
  positionGetFlag = true;
}

int main(int argc, char **argv){
  ros::init(argc ,argv, "obj_uploader") ;  
  cout << "obj_uploader" << endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber car_locate = n.subscribe("/car_pose", 1, car_locateCallback);
  ros::Subscriber pedestrian_locate = n.subscribe("/pedestrian_pose", 1, pedestrian_locateCallback);
  ros::Subscriber gnss_pose = n.subscribe("/ndt_pose", 1, position_getter_ndt);

  //set server name and port
  string serverName = defaultServerName;
  int portNum = PORT;
  if(argc == 3){
    serverName = argv[1];
    portNum = atoi(argv[2]);
  }

  sd = SendData(serverName,portNum);

  //set angle and position flag : false at first
  positionGetFlag = false;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();
  return 0;
}
