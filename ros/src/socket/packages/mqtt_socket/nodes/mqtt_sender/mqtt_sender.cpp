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
#include "MQTTClient.h"
#include "mqtt_socket/mqtt_setting.hpp"
#include "autoware_msgs/CanInfo.h"

class MqttSender
{
public:
  MqttSender();
  ~MqttSender();
  void canInfoCallback(const autoware_msgs::CanInfoConstPtr &msg);

private:
  ros::Subscriber vehicle_info_sub_;
	ros::NodeHandle node_handle_;
  MQTTClient_message pubmsg_;
  MQTTClient_deliveryToken deliveredtoken_;

  MQTTClient mqtt_client_;
  std::string mqtt_address_;
  std::string mqtt_topic_;
  std::string mqtt_client_id_;
  int mqtt_qos_;
  int mqtt_timeout_;
  double mqtt_downsample_;
  int callback_counter_;
};

MqttSender::MqttSender() :
    node_handle_("~")
{
  // ROS Publisher
  vehicle_info_sub_ = node_handle_.subscribe("/can_info", 1000, &MqttSender::canInfoCallback, this);

  // MQTT PARAMS
  pubmsg_ = MQTTClient_message_initializer;
  mqtt_address_ = ADDRESS;
  mqtt_topic_ = std::string(SENDER_TOPIC) + std::string(VEHICLEID) + "/can_info";
  mqtt_client_id_ = std::string(CLIENTID) + "_" + std::string(VEHICLEID) + "_snd";
  mqtt_qos_ = QOS;
  mqtt_timeout_ = TIMEOUT;
  mqtt_downsample_ = DOWNSAMPLE;
  callback_counter_ = 0;
  node_handle_.param("/confing/mqtt/address", mqtt_address_, mqtt_address_);
  node_handle_.param("/confing/mqtt/topic", mqtt_topic_, mqtt_topic_);
  node_handle_.param("/confing/mqtt/client_id", mqtt_client_id_, mqtt_client_id_);
  node_handle_.param("/confing/mqtt/qos", mqtt_qos_, mqtt_qos_);
  node_handle_.param("/confing/mqtt/timeout", mqtt_timeout_, mqtt_timeout_);
  node_handle_.param("/confing/mqtt/downsample", mqtt_downsample_, mqtt_downsample_);
  ROS_INFO("MQTT Sender ADDR: %s, TOPIC: %s, ID: %s, DOWNSAMPLE: %f\n", mqtt_address_.c_str(), mqtt_topic_.c_str(), mqtt_client_id_.c_str(), mqtt_downsample_);
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  int rc;

  MQTTClient_create(&mqtt_client_, mqtt_address_.c_str(), mqtt_client_id_.c_str(),
      MQTTCLIENT_PERSISTENCE_NONE, NULL);
  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;

  if ((rc = MQTTClient_connect(mqtt_client_, &conn_opts)) != MQTTCLIENT_SUCCESS)
  {
      ROS_INFO("Failed to connect, return code %d\n", rc);
      exit(EXIT_FAILURE);
  }
}

MqttSender::~MqttSender()
{
  MQTTClient_disconnect(mqtt_client_, 10000);
  MQTTClient_destroy(&mqtt_client_);
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

    pubmsg_.payload = publish_msg_str.c_str();
    pubmsg_.payloadlen = strlen(publish_msg_str.c_str());
    pubmsg_.qos = mqtt_qos_;
    pubmsg_.retained = 0;
    MQTTClient_publishMessage(mqtt_client_, mqtt_topic_.c_str(), &pubmsg_, &deliveredtoken_);
    int rc = MQTTClient_waitForCompletion(mqtt_client_, deliveredtoken_, mqtt_timeout_);
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
