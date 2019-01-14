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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


#include <pos_db.h>
#include "autoware_msgs/ObjLabel.h"

#define MYNAME		"pos_uploader"
#define OWN_TOPIC_NAME	"current_pose"
#define CAR_TOPIC_NAME	"obj_car/obj_pose"
#define PERSON_TOPIC_NAME	"obj_person/obj_pose"

using namespace std;

//store subscribed value
static std::vector <autoware_msgs::ObjLabel> car_positions_array;
static std::vector <autoware_msgs::ObjLabel> person_positions_array;
//flag for comfirming whether updating position or not
static size_t car_num = 0;
static size_t person_num = 0;

static int sleep_msec = 250;		// period
static int use_current_time = 0;

static string db_host_name;
static int db_port;
static string sshpubkey;
static string sshprivatekey;
static int ssh_port;
static string sshtunnelhost;

//send to server class
static SendData sd;

//store own position and direction now.updated by position_getter
static std::vector <geometry_msgs::PoseStamped> current_pose_position;
pthread_mutex_t pose_lock_;

static char mac_addr[MAC_ADDRBUFSIZ];

static std::string getTimeStamp(time_t sec, time_t nsec)
{
  char buf[30];
  int msec = static_cast<int>(nsec / (1000 * 1000));

  tm *t = gmtime(&sec);
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d.%03d",
          t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
          t->tm_hour, t->tm_min, t->tm_sec, msec);

  return std::string(static_cast<const char*>(buf));
}

static int get_type_from_name(const char *name)
{
  if (strcmp(name, OWN_TOPIC_NAME) == 0) {
    return 1;
  } else if (strcmp(name, CAR_TOPIC_NAME) == 0) {
    return 2;
  } else if (strcmp(name, PERSON_TOPIC_NAME) == 0) {
    return 3;
  } else {
    std::cerr << "Cannot convert name \"" << name << "\" to type" << std::endl;
    return 0;
  }
}

static std::string pose_to_insert_statement(const geometry_msgs::Pose& pose, const std::string& timestamp, const char *name)
{
  std::ostringstream oss;
  constexpr int AREA = 7;

  oss << "INSERT INTO POS(id,x,y,z,area,or_x,or_y,or_z,or_w,type,tm) "
      << "VALUES("
      << "'" << mac_addr << "',"
      << std::fixed << std::setprecision(6) << pose.position.y << ","
      << std::fixed << std::setprecision(6) << pose.position.x << ","
      << std::fixed << std::setprecision(6) << pose.position.z << ","
      << AREA << ","
      << std::fixed << std::setprecision(6) << pose.orientation.y << ","
      << std::fixed << std::setprecision(6) << pose.orientation.x << ","
      << std::fixed << std::setprecision(6) << pose.orientation.z << ","
      << std::fixed << std::setprecision(6) << pose.orientation.w << ","
      << get_type_from_name(name) << ","
      << "'" << timestamp << "'"
      << ");";

  return oss.str();
}

static std::string point_to_insert_statement(const geometry_msgs::Point& point, const std::string& timestamp, const char *name)
{
  std::ostringstream oss;
  constexpr int AREA = 7;

  oss << "INSERT INTO POS(id,x,y,z,area,or_x,or_y,or_z,or_w,type,tm) "
      << "VALUES("
      << "'" << mac_addr << "',"
      << std::fixed << std::setprecision(6) << point.y << ","
      << std::fixed << std::setprecision(6) << point.x << ","
      << std::fixed << std::setprecision(6) << point.z << ","
      << AREA << ","
      << std::fixed << std::setprecision(6) << 0 << ","
      << std::fixed << std::setprecision(6) << 0 << ","
      << std::fixed << std::setprecision(6) << 0 << ","
      << std::fixed << std::setprecision(6) << 0 << ","
      << get_type_from_name(name) << ","
      << "'" << timestamp << "'"
      << ");";

  return oss.str();
}

static std::string makeSendDataDetectedObj(const autoware_msgs::ObjLabel& cp_array, const char *name)
{
  std::string timestamp;
  if(use_current_time  || cp_array.header.stamp.sec == 0) {
    ros::Time t = ros::Time::now();
    timestamp = getTimeStamp(t.sec, t.nsec);
  } else {
    timestamp = getTimeStamp(cp_array.header.stamp.sec, cp_array.header.stamp.nsec);
  }

  std::string ret;
  for (const auto& point : cp_array.reprojected_pos) {
    //create sql
    ret += point_to_insert_statement(point, timestamp, name);
    ret += "\n";
  }

  return ret;
}

//wrap SendData class
static void send_sql()
{
  int sql_num = car_num + person_num + current_pose_position.size();
  std::cout << "sqlnum : " << sql_num << std::endl;

  //create header
  std::string value = make_header(2, sql_num);

  std::cout << "current_num=" << current_pose_position.size()
    << ", car_num=" << car_num << "(" << car_positions_array.size() << ")"
    << ",person_num=" << person_num << "(" << person_positions_array.size() << ")"
    << std::endl;

  //get data of car and person recognizing
  pthread_mutex_lock(&pose_lock_);
  if(car_positions_array.size() > 0){
    for(size_t i = 0; i < car_positions_array.size(); i++) {
      value += makeSendDataDetectedObj(car_positions_array[i], CAR_TOPIC_NAME);
    }
  }
  car_positions_array.clear();
  car_num = 0;

  if(person_positions_array.size() > 0){
    for(size_t i = 0; i < person_positions_array.size(); i++) {
      value += makeSendDataDetectedObj(person_positions_array[i], PERSON_TOPIC_NAME);
    }
  }
  person_positions_array.clear();
  person_num = 0;


  // my_location
  for(size_t i = 0; i < current_pose_position.size(); i++) {
    std::string timestamp;
    timestamp = getTimeStamp(current_pose_position[i].header.stamp.sec,current_pose_position[i].header.stamp.nsec);
    value += pose_to_insert_statement(current_pose_position[i].pose, timestamp, OWN_TOPIC_NAME);
    value += "\n";
  }
  current_pose_position.clear();
  pthread_mutex_unlock(&pose_lock_);

#ifdef POS_DB_VERBOSE
  std::cout << "val=" << value.substr(POS_DB_HEAD_LEN) << std::endl;
#endif /* POS_DB_VERBOSE */

  std::string res;
  int ret = sd.Sender(value, res, sql_num);
  if (ret < 0) {
    std::cerr << "Failed: sd.Sender" << std::endl;
    return;
  }

#ifdef POS_DB_VERBOSE
  std::cout << "retrun message from DBserver : " << res << std::endl;
#endif /* POS_DB_VERBOSE */

  return;
}

static void* intervalCall(void *unused)
{
  while(1){
    //If angle and position data is not updated from previous data send,
    //data is not sent
    if((car_num + person_num + current_pose_position.size()) <= 0) {
      usleep(sleep_msec*1000);
      continue;
    }

    send_sql();
    usleep(sleep_msec*1000);
  }

  return nullptr;
}


static void car_locate_cb(const jsk_recognition_msgs::BoundingBoxArray& obj_pose_msg)
{
	if (obj_pose_msg.boxes.size() > 0) {
		geometry_msgs::Point tmpPoint;
		autoware_msgs::ObjLabel tmpLabel;

		pthread_mutex_lock(&pose_lock_);

		for (jsk_recognition_msgs::BoundingBox tmpBox : obj_pose_msg.boxes) {
			tmpPoint.x = tmpBox.pose.position.x;
			tmpPoint.y = tmpBox.pose.position.y;
			tmpPoint.z = tmpBox.pose.position.z;

			tmpLabel.reprojected_pos.push_back(tmpPoint);
		}

		car_positions_array.push_back(tmpLabel);
		car_num += obj_pose_msg.boxes.size();

		pthread_mutex_unlock(&pose_lock_);
	}
}

static void person_locate_cb(const jsk_recognition_msgs::BoundingBoxArray &obj_pose_msg)
{
	if (obj_pose_msg.boxes.size() > 0) {
		geometry_msgs::Point tmpPoint;
		autoware_msgs::ObjLabel tmpLabel;

		pthread_mutex_lock(&pose_lock_);

		for (jsk_recognition_msgs::BoundingBox tmpBox : obj_pose_msg.boxes) {
			tmpPoint.x = tmpBox.pose.position.x;
			tmpPoint.y = tmpBox.pose.position.y;
			tmpPoint.z = tmpBox.pose.position.z;

			tmpLabel.reprojected_pos.push_back(tmpPoint);
		}

		person_positions_array.push_back(tmpLabel);
		person_num += obj_pose_msg.boxes.size();

		pthread_mutex_unlock(&pose_lock_);
	}
}

static void current_pose_cb(const geometry_msgs::PoseStamped &pose)
{
  pthread_mutex_lock(&pose_lock_);
  if(use_current_time) {
    geometry_msgs::PoseStamped n = pose;
    ros::Time t = ros::Time::now();
    n.header.stamp = t;
    current_pose_position.push_back(n);
  } else {
    current_pose_position.push_back(pose);
  }
  pthread_mutex_unlock(&pose_lock_);
}

int main(int argc, char **argv)
{
  ros::init(argc ,argv, MYNAME);
  std::cout << MYNAME << std::endl;

  if(argc < 2) {
    std::cerr << "usage : \n\trosrun " << MYNAME << " <user name> [now]" << std::endl;
    return -1;
  }
  if(argc > 2) {
    if(strncmp(argv[2], "now", 3) == 0) use_current_time = 1;
  }
  std::cerr << "use_current_time=" << use_current_time << std::endl;

  pose_lock_ = PTHREAD_MUTEX_INITIALIZER;

  probe_mac_addr(mac_addr);
  std::cerr <<  "mac_addr=" << mac_addr << std::endl;

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  ros::Subscriber car_locate = nh.subscribe("/" CAR_TOPIC_NAME, 1, car_locate_cb);
  ros::Subscriber person_locate = nh.subscribe("/" PERSON_TOPIC_NAME, 1, person_locate_cb);
  ros::Subscriber gnss_pose = nh.subscribe("/" OWN_TOPIC_NAME, 1, current_pose_cb);

  string home_dir = getenv("HOME");

  nh.param<string>("pos_db/db_host_name", db_host_name, DB_HOSTNAME);
  cout << "db_host_name=" << db_host_name << endl;
  nh.param<int>("pos_db/db_port", db_port, DB_PORT);
  cout << "db_port=" << db_port << endl;
  nh.param<string>("pos_db/sshpubkey", sshpubkey, home_dir+SSHPUBKEY);
  cout << "sshpubkey=" << sshpubkey << endl;
  nh.param<string>("pos_db/sshprivatekey", sshprivatekey, home_dir+SSHPRIVATEKEY);
  cout << "sshprivatekey=" << sshprivatekey << endl;
  nh.param<int>("pos_db/ssh_port", ssh_port, SSHPORT);
  cout << "ssh_port=" << ssh_port << endl;
  nh.param<string>("pos_db/sshtunnelhost", sshtunnelhost, SSHTUNNELHOST);
  cout << "sshtunnelhost=" << sshtunnelhost << endl;

  //set server name and port
  sd = SendData(db_host_name, db_port, argv[1], sshpubkey, sshprivatekey, ssh_port, sshtunnelhost);

  pthread_t th;
  if(pthread_create(&th, nullptr, intervalCall, nullptr)){
    printf("thread create error\n");
  }
  pthread_detach(th);

  ros::spin();
  return 0;
}
