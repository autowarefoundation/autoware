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
#include <sys/time.h>
#include <bitset>

#include "../SendData.h"
#include "vehicle_socket/CanInfo.h"


#define XSTR(x) #x
#define STR(x) XSTR(x)

using namespace std;

//for timestamp
struct my_tm {
  time_t tim; // yyyymmddhhmmss
  long msec;  // milli sec
};

pthread_mutex_t mutex;

//default server name and port to send data
const string defaultServerName = "db1.ertl.jp";
const int PORT = 5678;
//magic that I am C++
const char MAGIC[5] = "MPWC";

//flag for comfirming whether updating position or not
bool canGetFlag;

//send to server class
SendData sd;

//send data
string CanSql;

void printDiff(struct timeval begin, struct timeval end){
  long diff;
  diff = (end.tv_sec - begin.tv_sec)*1000*1000 + (end.tv_usec - begin.tv_usec);
  printf("Diff: %ld us (%ld ms)\n",diff,diff/1000);
}

string getTimeStamp(long sec,long nsec){
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


//wrap SendData class
void* wrapSender(void *tsd){

  string value;

  //create header
  char magic[5] = "MPWC";
  u_int16_t major = htons(1);
  u_int16_t minor = htons(0);
  u_int32_t sqlinst = htonl(2);
  u_int32_t sqlnum = htonl(1);
  char header[16];
  memcpy(header,magic,4);
  memcpy(&header[4],&major,2);
  memcpy(&header[6],&minor,2);
  memcpy(&header[8],&sqlinst,4);
  memcpy(&header[12],&sqlnum,4);
  value.append(header,16);

  value += CanSql;
  //cout << value;

  string res = sd.Sender(value);
  cout << "retrun message from DBserver : " << res << endl;
  
  return nullptr;

}


void* intervalCall(void *a){

  pthread_t th;

  while(1){
    //If angle and position data is not updated from prevous data send,
    //data is not sent
    //if(1){
    if(!canGetFlag) {
      sleep(1);
      continue;
    }
    canGetFlag = false;

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


void can_infoCallback(const vehicle_socket::CanInfo& can)
{

  ostringstream oss;

  oss << "INSERT INTO CAN(";


  oss << "tm,";
  oss << "devmode,";
  oss << "drvcontmode,";
  oss << "drvoverridemode,";
  oss << "drvservo,";
  oss << "drivepedal,";
  oss << "targetpedalstr,";
  oss << "inputpedalstr,";
  oss << "targetveloc,";
  oss << "speed,";
  oss << "driveshift,";
  oss << "targetshift,";
  oss << "inputshift,";
  oss << "strmode,";
  oss << "strcontmode,";
  oss << "stroverridemode,";
  oss << "strservo,";  
  oss << "targettorque,";
  oss << "torque,";
  oss << "angle,";
  oss << "targetangle,";
  oss << "bbrakepress,";
  oss << "brakepedal,";
  oss << "brtargetpedalstr,";
  oss << "brinputpedalstr,";
  oss << "battery,";
  oss << "voltage,";
  oss << "anp,";
  oss << "battmaxtemparature,";
  oss << "battmintemparature,";
  oss << "maxchgcurrent,";
  oss << "maxdischgcurrent,";
  oss << "sideacc,";
  oss << "accellfromp,";
  oss << "anglefromp,";
  oss << "brakepedalfromp,";
  oss << "speedfr,";
  oss << "speedfl,";
  oss << "speedrr,";
  oss << "speedrl,";
  oss << "velocfromp2,";
  oss << "drvmode,";
  oss << "devpedalstrfromp,";
  oss << "rpm,";
  oss << "velocflfromp,";
  oss << "ev_mode,";
  oss << "temp,";
  oss << "shiftfrmprius,";
  oss << "light,";
  oss << "gaslevel,";
  oss << "door,";
  oss << "cluise";

  oss << ") VALUES(";

  oss << "'" << can.tm << "',";
  oss << can.devmode << ",";
  oss << can.drvcontmode << ",";
  oss << can.drvoverridemode << ",";
  oss << can.drvservo << ",";
  oss << can.drivepedal << ",";
  oss << can.targetpedalstr << ",";
  oss << can.inputpedalstr << ",";
  oss << fixed << setprecision(6) << can.targetveloc << ",";
  oss << fixed << setprecision(6) << can.speed << ",";
  oss << can.driveshift << ",";
  oss << can.targetshift << ",";
  oss << can.inputshift << ",";
  oss << can.strmode << ",";
  oss << can.strcontmode << ",";
  oss << can.stroverridemode << ",";
  oss << can.strservo << ",";
  oss << can.targettorque << ",";
  oss << can.torque << ",";
  oss << fixed << setprecision(6) << can.angle << ",";
  oss << fixed << setprecision(6) << can.targetangle << ",";
  oss << can.bbrakepress << ",";
  oss << can.brakepedal << ",";
  oss << can.brtargetpedalstr << ",";
  oss << can.brinputpedalstr << ",";
  oss << fixed << setprecision(6) << can.battery << ",";
  oss << can.voltage << ",";
  oss << fixed << setprecision(6) << can.anp << ",";
  oss << can.battmaxtemparature << ",";
  oss << can.battmintemparature << ",";
  oss << fixed << setprecision(6) << can.maxchgcurrent << ",";
  oss << fixed << setprecision(6) << can.maxdischgcurrent << ",";
  oss << fixed << setprecision(6) << can.sideacc << ",";
  oss << fixed << setprecision(6) << can.accellfromp << ",";
  oss << fixed << setprecision(6) << can.anglefromp << ",";
  oss << fixed << setprecision(6) << can.brakepedalfromp << ",";
  oss << fixed << setprecision(6) << can.speedfr << ",";
  oss << fixed << setprecision(6) << can.speedfl << ",";
  oss << fixed << setprecision(6) << can.speedrr << ",";
  oss << fixed << setprecision(6) << can.speedrl << ",";
  oss << fixed << setprecision(6) << can.velocfromp2 << ",";
  oss << can.drvmode << ",";
  oss << can.devpedalstrfromp << ",";
  oss << can.rpm << ",";
  oss << fixed << setprecision(6) << can.velocflfromp << ",";
  oss << can.ev_mode << ",";
  oss << can.temp << ",";
  oss << can.shiftfrmprius << ",";
  oss << can.light << ",";
  oss << can.gaslevel << ",";
  oss << can.door << ",";
  oss << can.cluise;

  oss << ");\n";

  CanSql = oss.str();
  
  canGetFlag = true;

}


int main(int argc, char **argv){
  
  ros::init(argc ,argv, "can_uploader") ;  
  cout << "can_uploader" << endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber can = n.subscribe("/can_info", 1, can_infoCallback);

  //set server name and port
  string serverName = defaultServerName;
  int portNum = PORT;
  if(argc == 3){
    serverName = argv[1];
    portNum = atoi(argv[2]);
  }

  sd = SendData(serverName,portNum);

  //set angle and position flag : false at first
  canGetFlag = false;

  pthread_t th;
  if(pthread_create(&th, NULL, intervalCall, NULL)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

}
