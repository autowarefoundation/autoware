/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "autoware_can_msgs/CANInfo.h"

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

#include <obj_db.h>

using namespace std;

//default server name and port to send data
static const string default_db_host = "db1.ertl.jp";
static const int default_db_port = 5678;

//flag for comfirming whether updating position or not
static bool canGetFlag;

//send to server class
static SendData sd;

//send data
static string CanSql;

//wrap SendData class
static void send_sql()
{
  std::string value = make_header(2, 1);

  value += CanSql;

  string res;
  int ret = sd.Sender(value, res);
  if (ret == -1) {
    std::cerr << "Failed: sd.Sender" << std::endl;
    return;
  }

  std::cout << "retrun message from DBserver : " << res << std::endl;
}

static void* intervalCall(void *unused)
{
  while(1){
    //If angle and position data is not updated from prevous data send,
    //data is not sent
    if(!canGetFlag) {
      sleep(1);
      continue;
    }
    canGetFlag = false;

    send_sql();
    sleep(1);
  }

  return nullptr;
}

static void can_infoCallback(const autoware_can_msgs::CANInfo& can)
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

int main(int argc, char **argv)
{
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
  string host_name = default_db_host;
  int port = default_db_port;
  if(argc >= 3){
    host_name = argv[1];
    port = std::atoi(argv[2]);
  }

  sd = SendData(host_name, port);

  //set angle and position flag : false at first
  canGetFlag = false;

  pthread_t th;
  if(pthread_create(&th, nullptr, intervalCall, nullptr)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();

  return 0;
}
