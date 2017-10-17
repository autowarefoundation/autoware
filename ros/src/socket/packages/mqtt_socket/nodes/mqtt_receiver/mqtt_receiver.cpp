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
#include "autoware_msgs/RemoteCmd.h"
#include <mosquitto.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <string>
#include <signal.h>

static struct mosquitto *mqtt_client_ = NULL;
static std::string mqtt_topic_;
static int mqtt_qos_;
static ros::Publisher remote_cmd_pub_;

class MqttReceiver
{
public:
  MqttReceiver();
  ~MqttReceiver();
  static void on_connect(struct mosquitto *mosq, void *obj, int result);
  static void on_disconnect(struct mosquitto *mosq, void *obj, int rc);
  static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);

private:
  ros::NodeHandle node_handle_;

  // MQTT
  std::string mqtt_address_;
  int mqtt_port_;
  std::string mqtt_client_id_;
  int mqtt_timeout_;
};

MqttReceiver::MqttReceiver() :
    node_handle_("~")
{
  // ROS Publisher
  remote_cmd_pub_ = node_handle_.advertise<autoware_msgs::RemoteCmd>("/remote_cmd", 1);

  // MQTT PARAMS
  mosquitto_lib_init();

  mqtt_address_ = MQTT_ADDRESS;
  mqtt_port_ = MQTT_PORT;
  mqtt_topic_ = std::string(RECIEVER_TOPIC) + std::string(VEHICLEID) + "/remote_cmd";
  mqtt_client_id_ = std::string(CLIENTID) + "_" + std::string(VEHICLEID) + "_rcv";
  mqtt_qos_ = QOS;

  node_handle_.param("/confing/mqtt/address", mqtt_address_, mqtt_address_);
  node_handle_.param("/confing/mqtt/port", mqtt_port_, mqtt_port_);
  node_handle_.param("/confing/mqtt/topic", mqtt_topic_, mqtt_topic_);
  node_handle_.param("/confing/mqtt/client_id", mqtt_client_id_, mqtt_client_id_);
  node_handle_.param("/confing/mqtt/qos", mqtt_qos_, mqtt_qos_);
  node_handle_.param("/confing/mqtt/timeout", mqtt_timeout_, mqtt_timeout_);
  ROS_INFO("MQTT Receiver ADDR: %s:%d, TOPIC: %s, ID: %s\n", mqtt_address_.c_str(), mqtt_port_, mqtt_topic_.c_str(), mqtt_client_id_.c_str());

  mqtt_client_ = mosquitto_new(mqtt_client_id_.c_str(), true, NULL);
  mosquitto_connect_callback_set(mqtt_client_, &MqttReceiver::on_connect);
  mosquitto_disconnect_callback_set(mqtt_client_, &MqttReceiver::on_disconnect);
  mosquitto_message_callback_set(mqtt_client_, &MqttReceiver::on_message);

  if(mosquitto_connect_bind(mqtt_client_, mqtt_address_.c_str(), mqtt_port_, mqtt_timeout_, NULL)){
    ROS_INFO("Failed to connect broker.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }

  // Start Subscribe
  int ret = mosquitto_loop(mqtt_client_, -1, 1);
  ret = mosquitto_loop_start(mqtt_client_);
}

MqttReceiver::~MqttReceiver()
{
  int ret = mosquitto_loop_stop(mqtt_client_, 1);
  mosquitto_destroy(mqtt_client_);
  mosquitto_lib_cleanup();
}

static void MqttReceiver::on_connect(struct mosquitto *mosq, void *obj, int result)
{
  ROS_INFO("on_connect: %s(%d)\n", __FUNCTION__, __LINE__);
  mosquitto_subscribe(mqtt_client_, NULL, mqtt_topic_.c_str(), mqtt_qos_);
}

static void MqttReceiver::on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
  ROS_INFO("on_disconnect: %s(%d)\n", __FUNCTION__, __LINE__);
}

static void MqttReceiver::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
  if(message->payloadlen) {
    std::string delim (",");
    std::string msg_str((char *)message->payload, message->payloadlen);
    std::vector<std::string> cmds;
    boost::algorithm::split(cmds, msg_str, boost::is_any_of(","));

    if(cmds.size() == 8) {
      autoware_msgs::RemoteCmd msg;
      msg.steer = std::stof(cmds[0]) * STEER_MAX_VAL;
      msg.accel = std::stof(cmds[1]) * ACCEL_MAX_VAL;
      msg.brake = std::stof(cmds[2]) * BRAKE_MAX_VAL;
      msg.gear = std::stoi(cmds[3]);
      msg.blinker = std::stoi(cmds[4]);
      msg.mode = std::stoi(cmds[5]);
      msg.control_mode = std::stoi(cmds[6]);
      msg.emergency = std::stoi(cmds[7]);
      remote_cmd_pub_.publish(msg);
    }
    else {
      ROS_INFO("Failed to parse remote command.\n");
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_receiver");
  MqttReceiver node;
  ros::spin();

  return 0;
}
