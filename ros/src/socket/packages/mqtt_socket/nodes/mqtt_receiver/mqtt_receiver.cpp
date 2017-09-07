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
#include <vector>
#include <boost/algorithm/string.hpp>
#include <string>
#include <list>
#include "stdlib.h"
#include "string.h"
#include "MQTTClient.h"
#include "mqtt_socket/mqtt_setting.hpp"

class MqttReciever
{
public:
  MqttReciever();
  ~MqttReciever();
  static void delivered(void *context, MQTTClient_deliveryToken dt);
  static int msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message);
  static void connlost(void *context, char *cause);

private:
  static ros::Publisher remote_cmd_pub_;
  static MQTTClient_deliveryToken deliveredtoken_;
	ros::NodeHandle node_handle_;
  MQTTClient mqtt_client_;
};

ros::Publisher MqttReciever::remote_cmd_pub_;
MQTTClient_deliveryToken MqttReciever::deliveredtoken_;

MqttReciever::MqttReciever() :
    node_handle_("~")
{
  // ROS Publisher
  remote_cmd_pub_ = node_handle_.advertise<mqtt_socket_msgs::RemoteCmd>("/remote_cmd", 1000);

  // MQTT PARAMS
  std::string mqtt_address = ADDRESS;
  std::string mqtt_topic = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/remote_cmd";
  std::string mqtt_client_id = std::string(CLIENTID) + "_" + std::string(VEHICLEID) + "_rcv";
  int mqtt_qos = QOS;
  int mqtt_timeout = TIMEOUT;

  node_handle_.param("/confing/mqtt/address", mqtt_address, mqtt_address);
  node_handle_.param("/confing/mqtt/topic", mqtt_topic, mqtt_topic);
  node_handle_.param("/confing/mqtt/client_id", mqtt_client_id, mqtt_client_id);
  node_handle_.param("/confing/mqtt/qos", mqtt_qos, mqtt_qos);
  node_handle_.param("/confing/mqtt/timeout", mqtt_timeout, mqtt_timeout);
  ROS_INFO("%s, %s, %s\n", mqtt_address.c_str(), mqtt_topic.c_str(), mqtt_client_id.c_str());
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  int rc;
  int ch;

  MQTTClient_create(&mqtt_client_, mqtt_address.c_str(), mqtt_client_id.c_str(),
      MQTTCLIENT_PERSISTENCE_NONE, NULL);
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;

  MQTTClient_setCallbacks(mqtt_client_, NULL, connlost, msgarrvd, delivered);
  if ((rc = MQTTClient_connect(mqtt_client_, &conn_opts)) != MQTTCLIENT_SUCCESS)
  {
      ROS_INFO("Failed to connect, return code %d\n", rc);
      exit(EXIT_FAILURE);
  }

  MQTTClient_subscribe(mqtt_client_, mqtt_topic.c_str(), mqtt_qos);
}

MqttReciever::~MqttReciever()
{
  MQTTClient_disconnect(mqtt_client_, 10000);
  MQTTClient_destroy(&mqtt_client_);
}

static void MqttReciever::delivered(void *context, MQTTClient_deliveryToken dt)
{
    ROS_INFO("Message with token value %d delivery confirmed\n", dt);
    MqttReciever::deliveredtoken_ = dt;
}

static int MqttReciever::msgarrvd(void *context, char *topicName, int topicLen, MQTTClient_message *message)
{
    std::string delim (",");
    std::string msg_str((char *)message->payload, message->payloadlen);
    std::vector<std::string> cmds;
    boost::algorithm::split(cmds, msg_str, boost::is_any_of(","));

    mqtt_socket_msgs::RemoteCmd msg;
    msg.accel = std::stof(cmds[0]) * ACCEL_MAX_VAL;
    msg.brake = std::stof(cmds[1]) * BRAKE_MAX_VAL;
    msg.steer = std::stof(cmds[2]) * STEER_MAX_VAL;
    msg.gear = std::stoi(cmds[3]);
    msg.mode = std::stoi(cmds[4]);
    msg.emergency = std::stoi(cmds[5]);
    remote_cmd_pub_.publish(msg);

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

static void MqttReciever::connlost(void *context, char *cause)
{
    ROS_INFO("\nConnection lost\n");
    ROS_INFO("     cause: %s\n", cause);
    mqtt_socket_msgs::RemoteCmd msg;
    msg.accel = 0;
    msg.brake = 0;
    msg.steer = 0;
    msg.gear = 0;
    msg.mode = AUTO_MODE;
    msg.emergency = EMERGENCY_MODE;
    remote_cmd_pub_.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqtt_receiver");
  MqttReciever node;
  ros::spin();

  return 0;
}
