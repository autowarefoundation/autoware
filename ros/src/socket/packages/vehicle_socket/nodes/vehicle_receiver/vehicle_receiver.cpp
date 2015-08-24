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
#include <vehicle_socket/CanInfo.h>

#include <iostream>
#include <string>
#include <vector>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

static constexpr int CAN_DATA_NUM = 52;

static ros::Publisher pub;
static int mode;

static bool parseCanValue(const std::string& can_data, vehicle_socket::CanInfo& msg)
{
  std::istringstream ss(can_data);
  std::vector<std::string> columns;

  std::string column;
  while(std::getline(ss, column, ',')){
    columns.push_back(column);
  }

  if(columns.size() != (CAN_DATA_NUM + 1))
    return false;

  msg.tm = columns[0].substr(1, columns[0].length() - 2);
  msg.devmode = std::stoi(columns[1]);
  msg.drvcontmode = std::stoi(columns[2]);
  msg.drvoverridemode = std::stoi(columns[3]);
  msg.drvservo = std::stoi(columns[4]);
  msg.drivepedal = std::stoi(columns[5]);
  msg.targetpedalstr = std::stoi(columns[6]);
  msg.inputpedalstr = std::stoi(columns[7]);
  msg.targetveloc = std::stod(columns[8]);
  msg.speed = std::stod(columns[9]);
  msg.driveshift = std::stoi(columns[10]);
  msg.targetshift = std::stoi(columns[11]);
  msg.inputshift = std::stoi(columns[12]);
  msg.strmode = std::stoi(columns[13]);
  msg.strcontmode = std::stoi(columns[14]);
  msg.stroverridemode = std::stoi(columns[15]);
  msg.strservo = std::stoi(columns[16]);
  msg.targettorque = std::stoi(columns[17]);
  msg.torque = std::stoi(columns[18]);
  msg.angle = std::stod(columns[19]);
  msg.targetangle = std::stod(columns[20]);
  msg.bbrakepress = std::stoi(columns[21]);
  msg.brakepedal = std::stoi(columns[22]);
  msg.brtargetpedalstr = std::stoi(columns[23]);
  msg.brinputpedalstr = std::stoi(columns[24]);
  msg.battery = std::stod(columns[25]);
  msg.voltage = std::stoi(columns[26]);
  msg.anp = std::stod(columns[27]);
  msg.battmaxtemparature = std::stoi(columns[28]);
  msg.battmintemparature = std::stoi(columns[29]);
  msg.maxchgcurrent = std::stod(columns[30]);
  msg.maxdischgcurrent = std::stod(columns[31]);
  msg.sideacc = std::stod(columns[32]);
  msg.accellfromp = std::stod(columns[33]);
  msg.anglefromp = std::stod(columns[34]);
  msg.brakepedalfromp = std::stod(columns[35]);
  msg.speedfr = std::stod(columns[36]);
  msg.speedfl = std::stod(columns[37]);
  msg.speedrr = std::stod(columns[38]);
  msg.speedrl = std::stod(columns[39]);
  msg.velocfromp2 = std::stod(columns[40]);
  msg.drvmode = std::stoi(columns[41]);
  msg.devpedalstrfromp = std::stoi(columns[42]);
  msg.rpm = std::stoi(columns[43]);
  msg.velocflfromp = std::stod(columns[44]);
  msg.ev_mode = std::stoi(columns[45]);
  msg.temp = std::stoi(columns[46]);
  msg.shiftfrmprius = std::stoi(columns[47]);
  msg.light = std::stoi(columns[48]);
  msg.gaslevel = std::stoi(columns[49]);
  msg.door = std::stoi(columns[50]);
  msg.cluise = std::stoi(columns[51]);
  mode = std::stoi(columns[52]);

  return true;
}

static void* getCanValue(void *arg)
{
  int *client_sockp = static_cast<int*>(arg);
  int sock = *client_sockp;
  delete client_sockp;

  char recvdata[1024];
  std::string can_data("");
  constexpr int LIMIT = 1024 * 1024;

  while(true){
    ssize_t n = recv(sock, recvdata, sizeof(recvdata), 0);

    if(n<0){
      std::perror("recv");
      can_data = "";
      break;
    }else if(n == 0){
      break;
    }
    can_data.append(recvdata,n);

    //recv data is bigger than 1M,return error
    if(can_data.size() > LIMIT){
      std::cerr << "recv data is too big." << std::endl;
      can_data = "";
      break;
    }
  }

  if(close(sock)<0){
    std::perror("close");
    return nullptr;
  }

  if(can_data.empty())
    return nullptr;

  vehicle_socket::CanInfo msg;
  bool ret = parseCanValue(can_data, msg);
  if(!ret)
    return nullptr;

  msg.header.frame_id = "/can";
  msg.header.stamp = ros::Time::now();
  pub.publish(msg);

  return nullptr;
}

static void* receiverCaller(void *unused)
{
  constexpr int listen_port = 10000;

  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if(sock == -1){
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
  //make it available immediately to connect
  //setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
  int ret = bind(sock, (sockaddr*)&addr, sizeof(addr));
  if(ret == -1){
    std::perror("bind");
    goto error;
  }

  ret = listen(sock, 5);
  if(ret == -1) {
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

    std::cout << "Get connect" << std::endl;

    pthread_t th;
    if(pthread_create(&th, nullptr, getCanValue, static_cast<void*>(client_sock))){
      std::perror("pthread_create");
      break;
    }

    ret = pthread_detach(th);
    if(ret != 0){
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
  ros::init(argc ,argv, "vehicle_receiver");
  ros::NodeHandle nh;

  std::cout << "vehicle receiver" << std::endl;

  pub = nh.advertise<vehicle_socket::CanInfo>("can_info", 100);

  pthread_t th;
  int ret = pthread_create(&th, nullptr, receiverCaller, nullptr);
  if (ret != 0) {
    std::perror("pthread_create");
    std::exit(1);
  }

  ret = pthread_detach(th);
  if(ret != 0){
    std::perror("pthread_detach");
    std::exit(1);
  }

  ros::spin();

  return 0;
}
