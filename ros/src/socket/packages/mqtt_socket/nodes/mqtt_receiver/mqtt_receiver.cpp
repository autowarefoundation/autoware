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
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <mosquitto.h>
#include <yaml-cpp/yaml.h>
using namespace std;
#include "mqtt_socket/mqtt_setting.hpp"
#include "autoware_msgs/RemoteCmd.h"

static ros::Publisher remote_cmd_pub;

class MqttReceiver
{
public:
  MqttReceiver();
  ~MqttReceiver();
  static void on_connect(struct mosquitto *mosq, void *obj, int result);
  static void on_disconnect(struct mosquitto *mosq, void *obj, int rc);
  static void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);
  static void load_config();

private:
  ros::NodeHandle node_handle_;
};

MqttReceiver::MqttReceiver() :
    node_handle_("~")
{
  // ROS Publisher
  remote_cmd_pub = node_handle_.advertise<autoware_msgs::RemoteCmd>("/remote_cmd", 1);

  // Load Config
  MqttReceiver::load_config();

  // MQTT PARAMS
  mosquitto_lib_init();

  mqtt_client = mosquitto_new(mqtt_client_id.c_str(), true, NULL);
  mosquitto_connect_callback_set(mqtt_client, &MqttReceiver::on_connect);
  mosquitto_disconnect_callback_set(mqtt_client, &MqttReceiver::on_disconnect);
  mosquitto_message_callback_set(mqtt_client, &MqttReceiver::on_message);

  if(mosquitto_connect_bind(mqtt_client, mqtt_address.c_str(), mqtt_port, mqtt_timeout, NULL)){
    ROS_INFO("Failed to connect broker.\n");
    mosquitto_lib_cleanup();
    exit(EXIT_FAILURE);
  }

  // Start Subscribe
  int ret = mosquitto_loop(mqtt_client, -1, 1);
  ret = mosquitto_loop_start(mqtt_client);
}

MqttReceiver::~MqttReceiver()
{
  int ret = mosquitto_loop_stop(mqtt_client, 1);
  mosquitto_destroy(mqtt_client);
  mosquitto_lib_cleanup();
}

static void MqttReceiver::on_connect(struct mosquitto *mosq, void *obj, int result)
{
  ROS_INFO("on_connect: %s(%d)\n", __FUNCTION__, __LINE__);
  mosquitto_subscribe(mqtt_client, NULL, mqtt_topic.c_str(), mqtt_qos);
}

static void MqttReceiver::on_disconnect(struct mosquitto *mosq, void *obj, int rc)
{
  ROS_INFO("on_disconnect: %s(%d)\n", __FUNCTION__, __LINE__);
}

static void MqttReceiver::load_config()
{
  string config_file_path = ros::package::getPath(MQTT_NODE_NAME) + MQTT_CONFIG_FILE_NAME;
  ROS_INFO("Load Config File: %s %s(%d)\n", config_file_path.c_str(), __FUNCTION__, __LINE__);

  YAML::Node config = YAML::LoadFile(config_file_path);
  vehicle_id = config["mqtt"]["VEHICLEID"].as<int>();
  mqtt_client_id = "vehicle_" + to_string(vehicle_id) + "_rcv";
  mqtt_address = config["mqtt"]["ADDRESS"].as<string>();
  mqtt_port = config["mqtt"]["PORT"].as<int>();
  mqtt_qos = config["mqtt"]["QOS"].as<int>();
  mqtt_topic = "vehicle/" + to_string(vehicle_id) + "/remote_cmd";
  mqtt_timeout = config["mqtt"]["TIMEOUT"].as<int>();

  accel_max_val = config["mqtt"]["ACCEL_MAX_VAL"].as<int>();
  brake_max_val = config["mqtt"]["BRAKE_MAX_VAL"].as<int>();
  steer_max_val = config["mqtt"]["STEER_MAX_VAL"].as<float>();
  linear_x_max_val = config["mqtt"]["LINEAR_X_MAX_VAL"].as<float>();

  ROS_INFO("MQTT Receiver ADDR: %s:%d, TOPIC: %s, ID: %s\n", mqtt_address.c_str(), mqtt_port, mqtt_topic.c_str(), mqtt_client_id.c_str());

  ROS_INFO(
    "[MQTT Receiver Settings] vehicle_id: %d, accel_max_val: %d, \
    brake_max_val: %d, steer_max_val: %f, linear_x_max_val: %f,",
    vehicle_id, accel_max_val, brake_max_val, steer_max_val, linear_x_max_val);
}

static void MqttReceiver::on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message)
{
  if(message->payloadlen) {
    string delim (",");
    string msg_str((char *)message->payload, message->payloadlen);
    vector<string> cmds;
    boost::algorithm::split(cmds, msg_str, boost::is_any_of(","));

    if(cmds.size() == 8) {
      autoware_msgs::RemoteCmd msg;
      msg.vehicle_cmd.steer_cmd.steer = stof(cmds[0]) * steer_max_val;
      msg.vehicle_cmd.accel_cmd.accel = stof(cmds[1]) * accel_max_val;
      msg.vehicle_cmd.brake_cmd.brake = stof(cmds[2]) * brake_max_val;
      msg.vehicle_cmd.gear = stoi(cmds[3]);
      // lamp
      switch(stoi(cmds[4])) {
        msg.vehicle_cmd.lamp_cmd.l = 0;
        msg.vehicle_cmd.lamp_cmd.r = 0;
        case 0:
          break;
        case 1:
          msg.vehicle_cmd.lamp_cmd.l = 1;
          break;
        case 2:
          msg.vehicle_cmd.lamp_cmd.r = 1;
          break;
        case 3:
          msg.vehicle_cmd.lamp_cmd.l = 1;
          msg.vehicle_cmd.lamp_cmd.r = 1;
          break;
        default:
          ROS_WARN("Invalid lamp_cmd");
          break;
      }
      msg.vehicle_cmd.twist_cmd.twist.linear.x = stof(cmds[1]) * linear_x_max_val;
      msg.vehicle_cmd.twist_cmd.twist.angular.z = stof(cmds[0]);
      msg.vehicle_cmd.ctrl_cmd.linear_velocity = stof(cmds[1]) * linear_x_max_val;
      msg.vehicle_cmd.ctrl_cmd.steering_angle = stof(cmds[0]) * steer_max_val;
      msg.vehicle_cmd.mode = stoi(cmds[5]);
      msg.vehicle_cmd.emergency = stoi(cmds[7]);
      msg.control_mode = stoi(cmds[6]);
      remote_cmd_pub.publish(msg);
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
