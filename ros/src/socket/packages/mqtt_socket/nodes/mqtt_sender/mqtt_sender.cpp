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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdlib.h"
#include "string.h"
#include "mqtt_socket/mqtt_setting.hpp"
#include "autoware_msgs/CanInfo.h"
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <unordered_map>
#include <mosquitto.h>

class MqttSender
{
public:
  MqttSender();
  ~MqttSender();
  void canInfoCallback(const autoware_msgs::CanInfoConstPtr &msg);
  static void on_connect(struct mosquitto *mosq, void *obj, int result);
  static void on_disconnect(struct mosquitto *mosq, void *obj, int rc);
  static void on_publish(struct mosquitto *mosq, void *userdata, int mid);

private:
  std::unordered_map<std::string, ros::Subscriber> Subs;
  ros::NodeHandle node_handle_;

  // MQTT
  struct mosquitto *mqtt_client_ = NULL;
  std::string mqtt_address_;
  int mqtt_port_;
  std::string mqtt_topic_can_info_;
  std::string mqtt_topic_state_;
  std::string mqtt_topic_target_velocity_;
  std::string mqtt_topic_current_target_velocity_;

  std::string mqtt_client_id_;
  int mqtt_qos_;
  int mqtt_timeout_;
  double mqtt_downsample_;
  int callback_counter_;

  // current behavior/status
  std_msgs::Float64MultiArray current_target_velocity_array_; //kmph
  geometry_msgs::TwistStamped current_twist_cmd_; //mps
  double current_target_velocity_; // mps2kmph(current_twist_cmd_.twist.twist.linear.x);
  std_msgs::String current_state_;

  void targetVelocityArrayCallback(const std_msgs::Float64MultiArray &msg);
  void twistCmdCallback(const geometry_msgs::TwistStamped &msg);
  void stateCallback(const std_msgs::String &msg);

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

  // MQTT PARAMS
  mosquitto_lib_init();

  mqtt_address_ = MQTT_ADDRESS;
  mqtt_port_ = MQTT_PORT;
  mqtt_topic_can_info_ = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/can_info";
  mqtt_topic_state_ = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/state";
  mqtt_topic_target_velocity_ = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/target_velocity";
  mqtt_topic_current_target_velocity_ = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/current_velocity";

  mqtt_client_id_ = std::string(CLIENTID) + "_" + std::string(VEHICLEID) + "_snd";
  mqtt_qos_ = QOS;
  mqtt_timeout_ = TIMEOUT;
  mqtt_downsample_ = DOWNSAMPLE;
  callback_counter_ = 0;

  node_handle_.param("/confing/mqtt/address", mqtt_address_, mqtt_address_);
  node_handle_.param("/confing/mqtt/port", mqtt_port_, mqtt_port_);
  node_handle_.param("/confing/mqtt/client_id", mqtt_client_id_, mqtt_client_id_);
  node_handle_.param("/confing/mqtt/qos", mqtt_qos_, mqtt_qos_);
  node_handle_.param("/confing/mqtt/timeout", mqtt_timeout_, mqtt_timeout_);
  node_handle_.param("/confing/mqtt/downsample", mqtt_downsample_, mqtt_downsample_);
  ROS_INFO("MQTT Sender ADDR: %s:%d, TOPIC: %s, ID: %s, DOWNSAMPLE: %f\n", mqtt_address_.c_str(), mqtt_port_, mqtt_topic_can_info_.c_str(), mqtt_client_id_.c_str(), mqtt_downsample_);

  mqtt_client_ = mosquitto_new(mqtt_client_id_.c_str(), true, NULL);
  mosquitto_connect_callback_set(mqtt_client_, &MqttSender::on_connect);
  mosquitto_disconnect_callback_set(mqtt_client_, &MqttSender::on_disconnect);

  if(mosquitto_connect_bind(mqtt_client_, mqtt_address_.c_str(), mqtt_port_, mqtt_timeout_, NULL)){
    ROS_INFO("Failed to connect broker.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }
}

MqttSender::~MqttSender()
{
  mosquitto_destroy(mqtt_client_);
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

void MqttSender::targetVelocityArrayCallback(const std_msgs::Float64MultiArray &msg)
{
  std::ostringstream publish_msg;
  current_target_velocity_array_ = msg;

  for(int i = 0; i < sizeof(msg.data); i++) {
    publish_msg << std::to_string(msg.data[i]) << ",";
  }
  std::string publish_msg_str = publish_msg.str();

  int ret = mosquitto_publish(
    mqtt_client_,
    NULL,
    mqtt_topic_target_velocity_.c_str(),
    strlen(publish_msg_str.c_str()),
    publish_msg_str.c_str(),
    mqtt_qos_,
    false
  );
}

void MqttSender::twistCmdCallback(const geometry_msgs::TwistStamped &msg)
{
  std::ostringstream publish_msg;
  current_twist_cmd_ = msg;
  current_target_velocity_ = mps2kmph(msg.twist.linear.x);

  publish_msg << std::to_string(current_target_velocity_);
  std::string publish_msg_str = publish_msg.str();

  int ret = mosquitto_publish(
    mqtt_client_,
    NULL,
    mqtt_topic_current_target_velocity_.c_str(),
    strlen(publish_msg_str.c_str()),
    publish_msg_str.c_str(),
    mqtt_qos_,
    false
  );
}

void MqttSender::stateCallback(const std_msgs::String &msg)
{
  ROS_INFO("State: %s\n", msg.data.c_str());

  int ret = mosquitto_publish(
    mqtt_client_,
    NULL,
    mqtt_topic_state_.c_str(),
    strlen(msg.data.c_str()),
    msg.data.c_str(),
    mqtt_qos_,
    false
  );
}

void MqttSender::canInfoCallback(const autoware_msgs::CanInfoConstPtr &msg)
{

  if(callback_counter_ > mqtt_downsample_ * 100) {
    std::ostringstream publish_msg;

    publish_msg << msg->tm << ",";
    publish_msg << std::to_string(msg->devmode) << ",";
    publish_msg << std::to_string(msg->drvcontmode) << ",";
    publish_msg << std::to_string(msg->drvoverridemode) << ",";
    publish_msg << std::to_string(msg->drvservo) << ",";
    publish_msg << std::to_string(msg->drivepedal) << ",";
    publish_msg << std::to_string(msg->targetpedalstr) << ",";
    publish_msg << std::to_string(msg->inputpedalstr) << ",";
    publish_msg << std::to_string(msg->targetveloc) << ",";
    publish_msg << std::to_string(msg->speed) << ",";
    publish_msg << std::to_string(msg->driveshift) << ",";
    publish_msg << std::to_string(msg->targetshift) << ",";
    publish_msg << std::to_string(msg->inputshift) << ",";
    publish_msg << std::to_string(msg->strmode) << ",";
    publish_msg << std::to_string(msg->strcontmode) << ",";
    publish_msg << std::to_string(msg->stroverridemode) << ",";
    publish_msg << std::to_string(msg->strservo) << ",";
    publish_msg << std::to_string(msg->targettorque) << ",";
    publish_msg << std::to_string(msg->torque) << ",";
    publish_msg << std::to_string(msg->angle) << ",";
    publish_msg << std::to_string(msg->targetangle) << ",";
    publish_msg << std::to_string(msg->bbrakepress) << ",";
    publish_msg << std::to_string(msg->brakepedal) << ",";
    publish_msg << std::to_string(msg->brtargetpedalstr) << ",";
    publish_msg << std::to_string(msg->brinputpedalstr) << ",";
    publish_msg << std::to_string(msg->battery) << ",";
    publish_msg << std::to_string(msg->voltage) << ",";
    publish_msg << std::to_string(msg->anp) << ",";
    publish_msg << std::to_string(msg->battmaxtemparature) << ",";
    publish_msg << std::to_string(msg->battmintemparature) << ",";
    publish_msg << std::to_string(msg->maxchgcurrent) << ",";
    publish_msg << std::to_string(msg->maxdischgcurrent) << ",";
    publish_msg << std::to_string(msg->sideacc) << ",";
    publish_msg << std::to_string(msg->accellfromp) << ",";
    publish_msg << std::to_string(msg->anglefromp) << ",";
    publish_msg << std::to_string(msg->brakepedalfromp) << ",";
    publish_msg << std::to_string(msg->speedfr) << ",";
    publish_msg << std::to_string(msg->speedfl) << ",";
    publish_msg << std::to_string(msg->speedrr) << ",";
    publish_msg << std::to_string(msg->speedrl) << ",";
    publish_msg << std::to_string(msg->velocfromp2) << ",";
    publish_msg << std::to_string(msg->drvmode) << ",";
    publish_msg << std::to_string(msg->devpedalstrfromp) << ",";
    publish_msg << std::to_string(msg->rpm) << ",";
    publish_msg << std::to_string(msg->velocflfromp) << ",";
    publish_msg << std::to_string(msg->ev_mode) << ",";
    publish_msg << std::to_string(msg->temp) << ",";
    publish_msg << std::to_string(msg->shiftfrmprius) << ",";
    publish_msg << std::to_string(msg->light) << ",";
    publish_msg << std::to_string(msg->gaslevel) << ",";
    publish_msg << std::to_string(msg->door) << ",";
    publish_msg << std::to_string(msg->cluise);

    // std::ostringstream publish_msg = create_message(msg);
    std::string publish_msg_str = publish_msg.str();
    int ret = mosquitto_publish(
      mqtt_client_,
      NULL,
      mqtt_topic_can_info_.c_str(),
      strlen(publish_msg_str.c_str()),
      publish_msg_str.c_str(),
      mqtt_qos_,
      false
    );

    callback_counter_ = 0;
  }
  else {
    callback_counter_++;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_sender");
  MqttSender node;
  ros::spin();

  return 0;
}
