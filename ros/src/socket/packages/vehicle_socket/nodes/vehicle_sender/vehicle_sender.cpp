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

#include <ros/ros.h>
#include "autoware_msgs/VehicleCmd.h"

#include <iostream>
#include <string>
#include <cstdio>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

struct CommandData {
  double linear_x;
  double angular_z;
  int modeValue;
  int gearValue;
  int lampValue;
  int accellValue;
  int brakeValue;
  int steerValue;
  double linear_velocity;
  double steering_angle;

  void reset();
};

void CommandData::reset()
{
  linear_x      = 0;
  angular_z     = 0;
  modeValue     = 0;
  gearValue     = 0;
  lampValue     = 0;
  accellValue   = 0;
  brakeValue    = 0;
  steerValue    = 0;
  linear_velocity = -1;
  steering_angle = 0;
}

static CommandData command_data;

static void vehicleCmdCallback(const autoware_msgs::VehicleCmd& msg)
{
  command_data.linear_x = msg.twist_cmd.twist.linear.x;
  command_data.angular_z = msg.twist_cmd.twist.angular.z;
  command_data.modeValue = msg.mode;
  command_data.gearValue = msg.gear;
  if(msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 0) {
    command_data.lampValue = 0;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 0) {
    command_data.lampValue = 1;
  }
  else if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 1) {
    command_data.lampValue = 2;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 1) {
    command_data.lampValue = 3;
  }
  command_data.accellValue = msg.accel_cmd.accel;
  command_data.steerValue = msg.steer_cmd.steer;
  command_data.brakeValue = msg.brake_cmd.brake;
  command_data.linear_velocity = msg.ctrl_cmd.linear_velocity;
  command_data.steering_angle = msg.ctrl_cmd.steering_angle;
}

static void *sendCommand(void *arg)
{
  int *client_sockp = static_cast<int*>(arg);
  int client_sock = *client_sockp;
  delete client_sockp;

  std::ostringstream oss;
  oss << command_data.linear_x << ",";
  oss << command_data.angular_z << ",";
  oss << command_data.modeValue << ",";
  oss << command_data.gearValue << ",";
  oss << command_data.accellValue << ",";
  oss << command_data.brakeValue << ",";
  oss << command_data.steerValue << ",";
  oss << command_data.linear_velocity << ",";
  oss << command_data.steering_angle << ",";
  oss << command_data.lampValue;

  std::string cmd(oss.str());
  ssize_t n = write(client_sock, cmd.c_str(), cmd.size());
  if(n < 0){
    std::perror("write");
    return nullptr;
  }

  if(close(client_sock) == -1){
    std::perror("close");
    return nullptr;
  }

  std::cout << "cmd: " << cmd << ", size: " << cmd.size() << std::endl;
  return nullptr;
}

static void* receiverCaller(void *unused)
{
  constexpr int listen_port = 10001;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock == -1){
    std::perror("socket");
    return nullptr;
  }

  sockaddr_in addr;
  sockaddr_in client;
  socklen_t len = sizeof(client);

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = PF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = INADDR_ANY;

  int ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
  if(ret == -1){
    std::perror("bind");
    goto error;
  }

  ret = listen(sock, 20);
  if(ret == -1){
    std::perror("listen");
    goto error;
  }

  while(true){
    //get connect to android
    std::cout << "Waiting access..." << std::endl;

    int *client_sock = new int();
    *client_sock = accept(sock, reinterpret_cast<sockaddr*>(&client), &len);
    if(*client_sock == -1){
      std::perror("accept");
      break;
    }

    std::cout << "get connect." << std::endl;

    pthread_t th;
    if(pthread_create(&th, nullptr, sendCommand, static_cast<void*>(client_sock)) != 0){
      std::perror("pthread_create");
      break;
    }

    if(pthread_detach(th) != 0){
      std::perror("pthread_detach");
      break;
    }
  }

error:
  close(sock);
  return nullptr;
}

int main(int argc, char **argv)
{
  ros::init(argc ,argv, "vehicle_sender") ;
  ros::NodeHandle nh;

  std::cout << "vehicle sender" << std::endl;
  ros::Subscriber sub = nh.subscribe("/vehicle_cmd", 1, vehicleCmdCallback);

  command_data.reset();

  pthread_t th;
  if(pthread_create(&th, nullptr, receiverCaller, nullptr) != 0){
    std::perror("pthread_create");
    std::exit(1);
  }

  if (pthread_detach(th) != 0){
    std::perror("pthread_detach");
    std::exit(1);
  }

  ros::spin();
  return 0;
}
