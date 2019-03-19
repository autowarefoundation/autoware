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
#include <tablet_socket_msgs/mode_info.h>
#include "autoware_can_msgs/CANInfo.h"

#include <netinet/in.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#define CAN_KEY_MODE (0)
#define CAN_KEY_TIME (1)
#define CAN_KEY_VELOC (2)
#define CAN_KEY_ANGLE (3)
#define CAN_KEY_TORQUE (4)
#define CAN_KEY_ACCEL (5)
#define CAN_KEY_BRAKE (6)
#define CAN_KEY_SHIFT (7)

static ros::Publisher can_pub;
static ros::Publisher mode_pub;
static int mode;

static bool parseCanValue(const std::string &can_data, autoware_can_msgs::CANInfo &msg)
{
  std::istringstream ss(can_data);
  std::vector<std::string> columns;

  std::string column;
  while (std::getline(ss, column, ','))
  {
    columns.push_back(column);
  }

  for (std::size_t i = 0; i < columns.size(); i += 2)
  {
    int key = std::stoi(columns[i]);
    switch (key)
    {
      case CAN_KEY_MODE:
        mode = std::stoi(columns[i + 1]);
        if (mode & 0x1)
        {
          msg.devmode = 1;
        }
        else
        {
          msg.devmode = 0;
        }

        if (mode & 0x2)
        {
          msg.strmode = 1;
        }
        else
        {
          msg.strmode = 0;
        }
        break;
      case CAN_KEY_TIME:
        msg.tm = columns[i + 1].substr(1, columns[i + 1].length() - 2);  // skip '
        break;
      case CAN_KEY_VELOC:
        msg.speed = std::stod(columns[i + 1]);
        break;
      case CAN_KEY_ANGLE:
        msg.angle = std::stod(columns[i + 1]);
        break;
      case CAN_KEY_TORQUE:
        msg.torque = std::stoi(columns[i + 1]);
        break;
      case CAN_KEY_ACCEL:
        msg.drivepedal = std::stoi(columns[i + 1]);
        break;
      case CAN_KEY_BRAKE:
        msg.brakepedal = std::stoi(columns[i + 1]);
        break;
      case CAN_KEY_SHIFT:
        msg.driveshift = std::stoi(columns[i + 1]);
        break;
      default:
        std::cout << "Warning: unknown key : " << key << std::endl;
    }
  }

  return true;
}

static void *getCanValue(void *arg)
{
  int *client_sockp = static_cast<int *>(arg);
  int sock = *client_sockp;
  delete client_sockp;

  char recvdata[1024];
  std::string can_data("");
  constexpr int LIMIT = 1024 * 1024;

  while (true)
  {
    ssize_t n = recv(sock, recvdata, sizeof(recvdata), 0);

    if (n < 0)
    {
      std::perror("recv");
      can_data = "";
      break;
    }
    else if (n == 0)
    {
      break;
    }
    can_data.append(recvdata, n);

    // recv data is bigger than 1M,return error
    if (can_data.size() > LIMIT)
    {
      std::cerr << "recv data is too big." << std::endl;
      can_data = "";
      break;
    }
  }

  if (close(sock) < 0)
  {
    std::perror("close");
    return nullptr;
  }

  if (can_data.empty())
    return nullptr;

  autoware_can_msgs::CANInfo can_msg;
  bool ret = parseCanValue(can_data, can_msg);
  if (!ret)
    return nullptr;

  can_msg.header.frame_id = "/can";
  can_msg.header.stamp = ros::Time::now();
  can_pub.publish(can_msg);

  tablet_socket_msgs::mode_info mode_msg;
  mode_msg.header.frame_id = "/mode";
  mode_msg.header.stamp = ros::Time::now();
  mode_msg.mode = mode;
  mode_pub.publish(mode_msg);

  return nullptr;
}

static void *receiverCaller(void *unused)
{
  constexpr int listen_port = 10000;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock == -1)
  {
    std::perror("socket");
    return nullptr;
  }

  sockaddr_in client;
  socklen_t len = sizeof(client);
  sockaddr_in addr;

  std::memset(&addr, 0, sizeof(sockaddr_in));
  addr.sin_family = PF_INET;
  addr.sin_port = htons(listen_port);
  addr.sin_addr.s_addr = INADDR_ANY;
  // make it available immediately to connect
  // setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
  int ret = bind(sock, (sockaddr *)&addr, sizeof(addr));
  if (ret == -1)
  {
    std::perror("bind");
    goto error;
  }

  ret = listen(sock, 5);
  if (ret == -1)
  {
    std::perror("listen");
    goto error;
  }

  while (true)
  {
    // get connect to android
    std::cout << "Waiting access..." << std::endl;
    int *client_sock = new int();
    *client_sock = accept(sock, reinterpret_cast<sockaddr *>(&client), &len);
    if (*client_sock == -1)
    {
      std::perror("accept");
      break;
    }

    std::cout << "Get connect" << std::endl;

    pthread_t th;
    if (pthread_create(&th, nullptr, getCanValue, static_cast<void *>(client_sock)))
    {
      std::perror("pthread_create");
      break;
    }

    ret = pthread_detach(th);
    if (ret != 0)
    {
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
  ros::init(argc, argv, "vehicle_receiver");
  ros::NodeHandle nh;

  std::cout << "vehicle receiver" << std::endl;

  can_pub = nh.advertise<autoware_can_msgs::CANInfo>("can_info", 100);
  mode_pub = nh.advertise<tablet_socket_msgs::mode_info>("mode_info", 100);

  pthread_t th;
  int ret = pthread_create(&th, nullptr, receiverCaller, nullptr);
  if (ret != 0)
  {
    std::perror("pthread_create");
    std::exit(1);
  }

  ret = pthread_detach(th);
  if (ret != 0)
  {
    std::perror("pthread_detach");
    std::exit(1);
  }

  ros::spin();

  return 0;
}
