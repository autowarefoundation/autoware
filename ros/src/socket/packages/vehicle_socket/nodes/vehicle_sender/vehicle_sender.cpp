/*
 *  Copyright (c) 2015, Nagoya University
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
#include <geometry_msgs/TwistStamped.h>
#include <tablet_socket_msgs/mode_cmd.h>
#include <tablet_socket_msgs/gear_cmd.h>
#include "autoware_msgs/accel_cmd.h"
#include "autoware_msgs/brake_cmd.h"
#include "autoware_msgs/steer_cmd.h"
#include "autoware_msgs/ControlCommandStamped.h"

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
  int accellValue;
  int brakeValue;
  int steerValue;
  double linear_velocity;
  double steering_angle;

  void reset();
};

void CommandData::reset()
{
  linear_x    = 0;
  angular_z   = 0;
  modeValue   = 0;
  gearValue   = 0;
  accellValue = 0;
  brakeValue  = 0;
  steerValue  = 0;
  linear_velocity = -1;
  steering_angle = 0;
}

static CommandData command_data;

static void twistCMDCallback(const geometry_msgs::TwistStamped& msg)
{
  command_data.linear_x = msg.twist.linear.x;
  command_data.angular_z = msg.twist.angular.z;
}

static void modeCMDCallback(const tablet_socket_msgs::mode_cmd& mode)
{
  if(mode.mode == -1 || mode.mode == 0){
    command_data.reset();
  }

  command_data.modeValue = mode.mode;
}

static void gearCMDCallback(const tablet_socket_msgs::gear_cmd& gear)
{
  command_data.gearValue = gear.gear;
}

static void accellCMDCallback(const autoware_msgs::accel_cmd& accell)
{
  command_data.accellValue = accell.accel;
}

static void steerCMDCallback(const autoware_msgs::steer_cmd& steer)
{
  command_data.steerValue = steer.steer;
}

static void brakeCMDCallback(const autoware_msgs::brake_cmd &brake)
{
  command_data.brakeValue = brake.brake;
}

static void ctrlCMDCallback(const autoware_msgs::ControlCommandStamped& msg)
{
  command_data.linear_velocity = msg.cmd.linear_velocity;
  command_data.steering_angle = msg.cmd.steering_angle;
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
  oss << command_data.steering_angle;

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
  ros::Subscriber sub[7];
  sub[0] = nh.subscribe("/twist_cmd", 1, twistCMDCallback);
  sub[1] = nh.subscribe("/mode_cmd",  1, modeCMDCallback);
  sub[2] = nh.subscribe("/gear_cmd",  1, gearCMDCallback);
  sub[3] = nh.subscribe("/accel_cmd", 1, accellCMDCallback);
  sub[4] = nh.subscribe("/steer_cmd", 1, steerCMDCallback);
  sub[5] = nh.subscribe("/brake_cmd", 1, brakeCMDCallback);
  sub[6] = nh.subscribe("/ctrl_cmd", 1, ctrlCMDCallback);

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
