/*
 *  Copyright (c) 2017, Tier IV, Inc.
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

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <mosquitto.h>
#include <yaml-cpp/yaml.h>
using namespace std;
#include "mqtt_socket/mqtt_setting.hpp"
#include "autoware_msgs/CanInfo.h"
#include <tablet_socket_msgs/mode_info.h>

class MqttSender
{
public:
  MqttSender();
  ~MqttSender();
  void canInfoCallback(const autoware_msgs::CanInfoConstPtr &msg);
  static void on_connect(struct mosquitto *mosq, void *obj, int result);
  static void on_disconnect(struct mosquitto *mosq, void *obj, int rc);
  static void on_publish(struct mosquitto *mosq, void *userdata, int mid);
  static void load_config();

private:
  void targetVelocityArrayCallback(const std_msgs::Float64MultiArray &msg);
  void twistCmdCallback(const geometry_msgs::TwistStamped &msg);
  void stateCallback(const std_msgs::String &msg);
  void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  void modeInfoCallback(const tablet_socket_msgs::mode_info& msg);
  unordered_map<string, ros::Subscriber> Subs;
  ros::NodeHandle node_handle_;

  // MQTT TOPIC
  string mqtt_topic_can_info_;
  string mqtt_topic_state_;
  string mqtt_topic_target_velocity_;
  string mqtt_topic_current_target_velocity_;
  string mqtt_topic_current_pose_;
  string mqtt_topic_drive_mode_;

  // current behavior/status
  std_msgs::Float64MultiArray current_target_velocity_array_; //kmph
  geometry_msgs::TwistStamped current_twist_cmd_; //mps
  double current_target_velocity_; // mps2kmph(current_twist_cmd_.twist.twist.linear.x);
  std_msgs::String current_state_;

  int can_info_callback_counter_ = 0;
  int mode_info_callback_counter_ = 0;
};

inline double mps2kmph(double _mpsval)
{
	return (_mpsval * 60 * 60) / 1000; // mps * 60sec * 60 minute / 1000m
}

MqttSender::MqttSender() :
    node_handle_("~")
{
  // ROS Subscriber
  Subs["can_info"] = node_handle_.subscribe("/can_info", 100, &MqttSender::canInfoCallback, this);
  Subs["target_velocity_array"] = node_handle_.subscribe("/target_velocity_array", 1, &MqttSender::targetVelocityArrayCallback, this);
  Subs["twist_cmd"] = node_handle_.subscribe("/twist_cmd", 1, &MqttSender::twistCmdCallback, this);
  Subs["state"] = node_handle_.subscribe("/state", 1, &MqttSender::stateCallback, this);
  Subs["drive_mode"] = node_handle_.subscribe("/mode_info", 1, &MqttSender::modeInfoCallback, this);
  Subs["current_pose"] = node_handle_.subscribe("/current_pose", 1, &MqttSender::currentPoseCallback, this);

  // MQTT PARAMS
  mosquitto_lib_init();

  // Load Config
  MqttSender::load_config();

  mqtt_topic_can_info_ = "vehicle/" + to_string(vehicle_id) + "/can_info";
  mqtt_topic_state_ = "vehicle/" + to_string(vehicle_id) + "/state";
  mqtt_topic_target_velocity_ = "vehicle/" + to_string(vehicle_id) + "/target_velocity";
  mqtt_topic_current_target_velocity_ = "vehicle/" + to_string(vehicle_id) + "/current_velocity";
  mqtt_topic_current_pose_ = "vehicle/" + to_string(vehicle_id) + "/current_pose";
  mqtt_topic_drive_mode_ = "vehicle/" + to_string(vehicle_id) + "/drive_mode";

  mqtt_client = mosquitto_new(mqtt_client_id.c_str(), true, NULL);
  mosquitto_connect_callback_set(mqtt_client, &MqttSender::on_connect);
  mosquitto_disconnect_callback_set(mqtt_client, &MqttSender::on_disconnect);

  if(mosquitto_connect_bind(mqtt_client, mqtt_address.c_str(), mqtt_port, mqtt_timeout, NULL)){
    ROS_INFO("Failed to connect broker.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }
}

MqttSender::~MqttSender()
{
  mosquitto_destroy(mqtt_client);
  mosquitto_lib_cleanup();
}

void MqttSender::on_connect(struct mosquitto *mosq, void *obj, int result)
{
    ROS_INFO("on_connect: %s(%d)\n", __FUNCTION__, __LINE__);
}

void MqttSender::on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
    ROS_INFO("on_disconnect: %s(%d)\n", __FUNCTION__, __LINE__);
}

static void MqttSender::load_config()
{
  string config_file_path = ros::package::getPath(MQTT_NODE_NAME) + MQTT_CONFIG_FILE_NAME;
  ROS_INFO("Load Config File: %s %s(%d)\n", config_file_path.c_str(), __FUNCTION__, __LINE__);

  YAML::Node config = YAML::LoadFile(config_file_path);
  vehicle_id = config["mqtt"]["VEHICLEID"].as<int>();
  mqtt_client_id = "vehicle_" + to_string(vehicle_id) + "_snd";
  mqtt_address = config["mqtt"]["ADDRESS"].as<string>();
  mqtt_port = config["mqtt"]["PORT"].as<int>();
  mqtt_qos = config["mqtt"]["QOS"].as<int>();
  mqtt_timeout = config["mqtt"]["TIMEOUT"].as<int>();

  caninfo_downsample = config["mqtt"]["CANINFO_DOWNSAMPLE"].as<float>();
  gear_d = config["mqtt"]["GEAR_D"].as<int>();
  gear_n = config["mqtt"]["GEAR_N"].as<int>();
  unordered_map<string, ros::Subscriber> Subs;
  gear_r = config["mqtt"]["GEAR_R"].as<int>();
  gear_p = config["mqtt"]["GEAR_P"].as<int>();

  ROS_INFO("MQTT Sender ADDR: %s:%d, ID: %s\n", mqtt_address.c_str(), mqtt_port,  mqtt_client_id.c_str());

  ROS_INFO(
    "[MQTT Receiver Settings] vehicle_id: %d, \
    caninfo_downsample: %f, gear_d: %d, gear_n: %d, gear_r: %d, \
    gear_p: %d", vehicle_id, caninfo_downsample, gear_d, gear_n, gear_r, gear_p);
}

void MqttSender::targetVelocityArrayCallback(const std_msgs::Float64MultiArray &msg)
{
  ostringstream publish_msg;
  current_target_velocity_array_ = msg;

  for(int i = 0; i < sizeof(msg.data); i++) {
    publish_msg << to_string(msg.data[i]) << ",";
  }
  string publish_msg_str = publish_msg.str();

  int ret = mosquitto_publish(
    mqtt_client,
    NULL,
    mqtt_topic_target_velocity_.c_str(),
    strlen(publish_msg_str.c_str()),
    publish_msg_str.c_str(),
    mqtt_qos,
    false
  );
}

void MqttSender::twistCmdCallback(const geometry_msgs::TwistStamped &msg)
{
  ostringstream publish_msg;
  current_twist_cmd_ = msg;
  current_target_velocity_ = mps2kmph(msg.twist.linear.x);

  publish_msg << to_string(current_target_velocity_);
  string publish_msg_str = publish_msg.str();

  int ret = mosquitto_publish(
    mqtt_client,
    NULL,
    mqtt_topic_current_target_velocity_.c_str(),
    strlen(publish_msg_str.c_str()),
    publish_msg_str.c_str(),
    mqtt_qos,
    false
  );
}

void MqttSender::stateCallback(const std_msgs::String &msg)
{
  ROS_INFO("State: %s\n", msg.data.c_str());

  int ret = mosquitto_publish(
    mqtt_client,
    NULL,
    mqtt_topic_state_.c_str(),
    strlen(msg.data.c_str()),
    msg.data.c_str(),
    mqtt_qos,
    false
  );
}

void MqttSender::currentPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  ostringstream publish_msg;

  publish_msg << to_string(msg.pose.position.x) << ",";
  publish_msg << to_string(msg.pose.position.y) << ",";
  publish_msg << to_string(msg.pose.position.z) << ",";
  publish_msg << to_string(msg.pose.orientation.x) << ",";
  publish_msg << to_string(msg.pose.orientation.y) << ",";
  publish_msg << to_string(msg.pose.orientation.z) << ",";
  publish_msg << to_string(msg.pose.orientation.w) << ",";

  string publish_msg_str = publish_msg.str();
  int ret = mosquitto_publish(
    mqtt_client,
    NULL,
    mqtt_topic_current_pose_.c_str(),
    strlen(publish_msg_str.c_str()),
    publish_msg_str.c_str(),
    mqtt_qos,
    false
  );
}

void MqttSender::modeInfoCallback(const tablet_socket_msgs::mode_info& msg)
{
  if(mode_info_callback_counter_ > caninfo_downsample * 100) {
    ostringstream publish_msg;
    publish_msg << to_string(msg.mode);
    string publish_msg_str = publish_msg.str();
    ROS_INFO("Drive Mode: %s\n", publish_msg_str.c_str());

    int ret = mosquitto_publish(
      mqtt_client,
      NULL,
      mqtt_topic_drive_mode_.c_str(),
      strlen(publish_msg_str.c_str()),
      publish_msg_str.c_str(),
      mqtt_qos,
      false
    );
    mode_info_callback_counter_ = 0;
  }
  else {
    mode_info_callback_counter_++;
  }
}

void MqttSender::canInfoCallback(const autoware_msgs::CanInfoConstPtr &msg)
{

  if(can_info_callback_counter_ > caninfo_downsample * 100) {
    ostringstream publish_msg;

    publish_msg << msg->tm << ",";
    publish_msg << to_string(msg->devmode) << ",";
    publish_msg << to_string(msg->drvcontmode) << ",";
    publish_msg << to_string(msg->drvoverridemode) << ",";
    publish_msg << to_string(msg->drvservo) << ",";
    publish_msg << to_string(msg->drivepedal) << ",";
    publish_msg << to_string(msg->targetpedalstr) << ",";
    publish_msg << to_string(msg->inputpedalstr) << ",";
    publish_msg << to_string(msg->targetveloc) << ",";
    publish_msg << to_string(msg->speed) << ",";
    if (msg->driveshift == gear_d)
      publish_msg << "D,";
    else if (msg->driveshift == gear_n)
      publish_msg << "N,";
    else if (msg->driveshift == gear_r)
      publish_msg << "R,";
    else if (msg->driveshift == gear_p)
      publish_msg << "P,";
    else
      publish_msg << "Unkwown,";
    publish_msg << to_string(msg->targetshift) << ",";
    publish_msg << to_string(msg->inputshift) << ",";
    publish_msg << to_string(msg->strmode) << ",";
    publish_msg << to_string(msg->strcontmode) << ",";
    publish_msg << to_string(msg->stroverridemode) << ",";
    publish_msg << to_string(msg->strservo) << ",";
    publish_msg << to_string(msg->targettorque) << ",";
    publish_msg << to_string(msg->torque) << ",";
    publish_msg << to_string(msg->angle) << ",";
    publish_msg << to_string(msg->targetangle) << ",";
    publish_msg << to_string(msg->bbrakepress) << ",";
    publish_msg << to_string(msg->brakepedal) << ",";
    publish_msg << to_string(msg->brtargetpedalstr) << ",";
    publish_msg << to_string(msg->brinputpedalstr) << ",";
    publish_msg << to_string(msg->battery) << ",";
    publish_msg << to_string(msg->voltage) << ",";
    publish_msg << to_string(msg->anp) << ",";
    publish_msg << to_string(msg->battmaxtemparature) << ",";
    publish_msg << to_string(msg->battmintemparature) << ",";
    publish_msg << to_string(msg->maxchgcurrent) << ",";
    publish_msg << to_string(msg->maxdischgcurrent) << ",";
    publish_msg << to_string(msg->sideacc) << ",";
    publish_msg << to_string(msg->accellfromp) << ",";
    publish_msg << to_string(msg->anglefromp) << ",";
    publish_msg << to_string(msg->brakepedalfromp) << ",";
    publish_msg << to_string(msg->speedfr) << ",";
    publish_msg << to_string(msg->speedfl) << ",";
    publish_msg << to_string(msg->speedrr) << ",";
    publish_msg << to_string(msg->speedrl) << ",";
    publish_msg << to_string(msg->velocfromp2) << ",";
    publish_msg << to_string(msg->drvmode) << ",";
    publish_msg << to_string(msg->devpedalstrfromp) << ",";
    publish_msg << to_string(msg->rpm) << ",";
    publish_msg << to_string(msg->velocflfromp) << ",";
    publish_msg << to_string(msg->ev_mode) << ",";
    publish_msg << to_string(msg->temp) << ",";
    publish_msg << to_string(msg->shiftfrmprius) << ",";
    publish_msg << to_string(msg->light) << ",";
    publish_msg << to_string(msg->gaslevel) << ",";
    publish_msg << to_string(msg->door) << ",";
    publish_msg << to_string(msg->cluise);

    // ostringstream publish_msg = create_message(msg);
    string publish_msg_str = publish_msg.str();
    int ret = mosquitto_publish(
      mqtt_client,
      NULL,
      mqtt_topic_can_info_.c_str(),
      strlen(publish_msg_str.c_str()),
      publish_msg_str.c_str(),
      mqtt_qos,
      false
    );

    can_info_callback_counter_ = 0;
  }
  else {
    can_info_callback_counter_++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_sender");
  MqttSender node;
  ros::spin();

  return 0;
}
