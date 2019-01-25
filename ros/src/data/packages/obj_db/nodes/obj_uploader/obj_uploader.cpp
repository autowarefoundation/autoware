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

#include <cstdio>
#include <time.h>
#include <pthread.h>
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <sys/time.h>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <obj_db.h>

//store subscribed value
static geometry_msgs::PoseArray car_position_array;
static geometry_msgs::PoseArray pedestrian_position_array;

//default server name and port to send data
static const std::string default_host_name = "db3.ertl.jp";
static constexpr int db_port = 5678;

//flag for comfirming whether updating position or not
static bool is_subscribed_ndt_pose;

//send to server class
static SendData sd;

//store own position and direction now.updated by position_getter
static geometry_msgs::PoseStamped my_location;

static std::string getTimeStamp(time_t sec, time_t nsec)
{
  char buf[30];
  int msec = static_cast<int>(nsec / (1000 * 1000));

  tm *t = localtime(&sec);
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d.%d",
          t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
          t->tm_hour, t->tm_min, t->tm_sec, msec);

  return std::string(static_cast<const char*>(buf));
}

static std::string pose_to_insert_statement(const geometry_msgs::Pose& pose, const std::string& timestamp)
{
  std::ostringstream oss;
  constexpr int AREA = 7;

  oss << "INSERT INTO POS(id,x,y,z,area,type,tm) "
      << "VALUES("
      << "'0',"
      << std::fixed << std::setprecision(6) << pose.position.y << ","
      << std::fixed << std::setprecision(6) << pose.position.x << ","
      << std::fixed << std::setprecision(6) << pose.position.z << ","
      << AREA << ","
      << "0,"
      << "'" << timestamp << "'"
      << ");";

  return oss.str();
}

static std::string makeSendDataDetectedObj(const geometry_msgs::PoseArray& cp_array)
{
  std::string timestamp = getTimeStamp(cp_array.header.stamp.sec, cp_array.header.stamp.nsec);

  std::string ret;
  for(const auto& pose : cp_array.poses){
    //create sql
    ret += pose_to_insert_statement(pose, timestamp);
    ret += "\n";
  }

  return ret;
}

//wrap SendData class
static void send_sql()
{
  size_t car_num = car_position_array.poses.size();
  size_t pedestrian_num = pedestrian_position_array.poses.size();
  std::cout << "sqlnum : " << (car_num + pedestrian_num) << std::endl;

  //create header
  std::string value = make_header(2, car_num + pedestrian_num);

  //get data of car and pedestrian recognizing
  if(car_num > 0){
    value += makeSendDataDetectedObj(car_position_array);
  }

  if(pedestrian_num > 0){
    value += makeSendDataDetectedObj(pedestrian_position_array);
  }

  std::string timestamp = getTimeStamp(my_location.header.stamp.sec,my_location.header.stamp.nsec);
  value += pose_to_insert_statement(my_location.pose, timestamp);
  value += "\n";

  std::cout << value << std::endl;

  std::string res;
  int ret = sd.Sender(value, res);
  if (ret == -1) {
    std::cerr << "Failed: sd.Sender" << std::endl;
    return;
  }

  std::cout << "retrun message from DBserver : " << res << std::endl;
  return;
}

static void* intervalCall(void *unused)
{
  while(1){
    //If angle and position data is not updated from previous data send,
    //data is not sent
    if(!is_subscribed_ndt_pose) {
      sleep(1);
      continue;
    }

    is_subscribed_ndt_pose = false;

    send_sql();
    sleep(1);
  }

  return nullptr;
}

static void car_locate_cb(const geometry_msgs::PoseArray& car_locate)
{
  car_position_array = car_locate;
}

static void pedestrian_locate_cb(const geometry_msgs::PoseArray& pedestrian_locate)
{
  pedestrian_position_array = pedestrian_locate;
}

static void ndt_pose_cb(const geometry_msgs::PoseStamped &pose)
{
  my_location = pose;
  is_subscribed_ndt_pose = true;
}

int main(int argc, char **argv)
{
  ros::init(argc ,argv, "obj_uploader");
  std::cout << "obj_uploader" << std::endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Subscriber car_locate = n.subscribe("/car_pose", 1, car_locate_cb);
  ros::Subscriber pedestrian_locate = n.subscribe("/pedestrian_pose", 1, pedestrian_locate_cb);
  ros::Subscriber gnss_pose = n.subscribe("/current_pose", 1, ndt_pose_cb);

  //set server name and port
  std::string host_name = default_host_name;
  int port = db_port;
  if(argc >= 3){
    host_name = argv[1];
    port = std::atoi(argv[2]);
  }

  sd = SendData(host_name, port);

  //set angle and position flag : false at first
  is_subscribed_ndt_pose = false;

  pthread_t th;
  if(pthread_create(&th, nullptr, intervalCall, nullptr)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();
  return 0;
}
