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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "mainwindow.h"
#include "autoware_socket.h"

double cycle_time = 0.0;

pthread_t _modesetter;
pthread_t _gearsetter;

// hmm, dirty hacks...
int current_mode = -1;
int current_gear = -1;
int mode_is_setting = false;
int gear_is_setting = false;

std::vector<std::string> split(const std::string& input, char delimiter)
{
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void Getter(CMDDATA &cmddata)
{
  std::string request;
  std::string cmdRes;
  char recvdata[32];

  struct sockaddr_in server;
  int sock;
  // char deststr[80] = serverIP.c_str();
  unsigned int **addrptr;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    perror("socket");
    return;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(10001);

  server.sin_addr.s_addr = inet_addr(ros_ip_address.c_str());
  if (server.sin_addr.s_addr == 0xffffffff) {
    struct hostent *host;

    host = gethostbyname(ros_ip_address.c_str());
    if (host == NULL) {
      if (h_errno == HOST_NOT_FOUND) {
        fprintf(stdout,"cmd : ROS PC not found : %s\n", ros_ip_address.c_str());
      } else {
        fprintf(stdout,"cmd : %s : %s\n", hstrerror(h_errno), ros_ip_address.c_str());
      }
      return;
    }

    addrptr = (unsigned int **)host->h_addr_list;

    while (*addrptr != NULL) {
      server.sin_addr.s_addr = *(*addrptr);

      /* break the loop when connected. */
      if (connect(sock, (struct sockaddr *)&server, sizeof(server)) == 0) {
        break;
      }

      addrptr++;
      // let's try another IP address if not successfully connected.
    }
   
    // if all connections failed...
    if (*addrptr == NULL) {
      perror("cmd : connect");
      return;
    }
  } else {
    if (connect(sock,
		(struct sockaddr *)&server, sizeof(server)) != 0) {
      perror("cmd : connect");
      return;
    }
  }

  int n;

  while (true) {
    memset(recvdata, 0, sizeof(recvdata));
    n = recv(sock, recvdata, sizeof(recvdata),0);
    if (n < 0) {
      perror("cmd : read erro");
      return;
    } else if (n == 0) {
      break;
    }
    cmdRes.append(recvdata,n);
  }

  // string version
  std::vector<std::string> cmdVector;
  cmdVector = split(cmdRes,',');
  if (cmdVector.size() == 7) {
    cmddata.vel.tv = atof(cmdVector[0].c_str());
    cmddata.vel.sv = atof(cmdVector[1].c_str());
    
    cout << endl << endl;
    cout << "cmddata.vel.tv = " << cmddata.vel.tv << endl;
    cout << "cmddata.vel.sv = " << cmddata.vel.sv << endl;
    
#if 0 /* log */
      ofstream ofs("/tmp/cmd.log", ios::app);
      ofs << cmddata.vel.tv << " " 
      << cmddata.vel.sv << " " 
      << endl;
#endif

    cmddata.mode = atoi(cmdVector[2].c_str());
    cmddata.gear = atoi(cmdVector[3].c_str());
    cmddata.accel = atoi(cmdVector[4].c_str());
    cmddata.steer = atoi(cmdVector[5].c_str());
    cmddata.brake = atoi(cmdVector[6].c_str());
  } else {
    fprintf(stderr,"cmd : Recv data is invalid\n");
  }
  cout << "cmd : return data : " << cmdRes.c_str() << endl;

  close(sock);
}

void Update(void *p)
{
  MainWindow* main = (MainWindow*)p;

  // update robocar state.
  main->UpdateState();
}

void SetState(int mode, int gear, void* p) 
{
  if (mode != current_mode) {
    current_mode = mode;
    pthread_create(&_modesetter, NULL, MainWindow::ModeSetterEntry, p);
  }

  if (gear != current_gear) {
    double current_velocity = vstate.velocity; // km/h
    // never change the gear when driving!
    if (current_velocity == 0) {
      current_gear = gear;
      pthread_create(&_gearsetter, NULL, MainWindow::GearSetterEntry, p);
    }
  }
}


void Control(vel_data_t vel, void* p) 
{
  MainWindow* main = (MainWindow*)p;
  static long long int old_tstamp = 0;

  cycle_time = (vstate.tstamp - old_tstamp) / 1000.0; /* seconds */

  double current_velocity = vstate.velocity; // km/h
  double current_steering_angle = vstate.steering_angle; // degree
 
  int cmd_velocity = vel.tv * 3.6;
  int cmd_steering_angle;

  // We assume that the slope against the entire arc toward the 
  // next waypoint is almost equal to that against 
  // $l = 2 \pi r \times \frac{\theta}{360} = r \times \theta$
  // \theta = cmd_wheel_angle
  // vel.sv/vel.tv = Radius
  // l \simeq VEHICLE_LENGTH
  if (vel.tv < 0.1) { // just avoid divided by zero.
    cmd_steering_angle = current_steering_angle;
  }
  else {
    double wheel_angle_pi = (vel.sv / vel.tv) * WHEEL_BASE;
    double wheel_angle = (wheel_angle_pi / M_PI) * 180.0;
    cmd_steering_angle = wheel_angle * WHEEL_TO_STEERING;
  }

#if 0 /* just for a debug */
  std::ifstream ifs("/tmp/velocity");
  std::string s;
  getline(ifs, s);
  int vel_debug = atoi(s.c_str());
  cout << "vel_debug = " << vel_debug << " km/h" << endl;
  cmd_velocity = vel_debug;
  if (cmd_velocity > 50)
    cmd_velocity = 50;
  if (cmd_velocity < 0)
    cmd_velocity = 0;
#endif
#if 0 /* just for a debug */
  std::ifstream ifs("/tmp/steering");
  std::string s;
  getline(ifs, s);
  int str_debug = atoi(s.c_str());
  cout << "str_debug = " << str_debug << " degree" << endl;
  cmd_steering_angle = str_debug;
  if (cmd_steering_angle > 360)
    cmd_steering_angle = 360;
  if (cmd_steering_angle < -360)
    cmd_steering_angle = -360;

#endif


  cout << "Current: " << "vel = " << current_velocity 
       << ", str = " << current_steering_angle << endl; 
  cout << "Command: " << "vel = " << cmd_velocity 
       << ", str = " << cmd_steering_angle << endl; 

  for (int i = 0; i < cmd_rx_interval/STEERING_INTERNAL_PERIOD - 1; i++) {
    //////////////////////////////////////////////////////
    // Accel and Brake
    //////////////////////////////////////////////////////
    
    main->StrokeControl(current_velocity, cmd_velocity);
    
    //////////////////////////////////////////////////////
    // Steering
    //////////////////////////////////////////////////////
    
    main->SteeringControl(current_steering_angle, cmd_steering_angle);

    usleep(STEERING_INTERNAL_PERIOD * 1000);  
    Update(main);
    current_velocity = vstate.velocity; // km/h
    current_steering_angle = vstate.steering_angle; // degree
  }

  //////////////////////////////////////////////////////
  // remaining period.
  //////////////////////////////////////////////////////

  main->StrokeControl(current_velocity, cmd_velocity);

  main->SteeringControl(current_steering_angle, cmd_steering_angle);

  // save the time stamp.
  old_tstamp = vstate.tstamp;
}

void *MainWindow::ModeSetterEntry(void *a)
{
  MainWindow* main = (MainWindow*)a;

  mode_is_setting = true; // loose critical section

  main->ClearCntDiag();
  sleep(1);
  main->SetStrMode(current_mode); // steering
  sleep(1);
  main->SetDrvMode(current_mode); // accel/brake
  sleep(1);

  mode_is_setting = false; // loose critical section

  return NULL;
}

void *MainWindow::GearSetterEntry(void *a)
{
  MainWindow* main = (MainWindow*)a;

  gear_is_setting = true; // loose critical section

  main->SetGear(current_gear);
  sleep(1);

  gear_is_setting = false; // loose critical section

  return NULL;
}

void *MainWindow::CMDGetterEntry(void *a)
{
  MainWindow* main = (MainWindow*)a;
  CMDDATA cmddata;
  long long int tstamp;
  long long int interval;

  while(1){

    // get commands from ROS.
    Getter(cmddata);

    // get time in milliseconds.
    tstamp = (long long int) getTime();

    // update robocar state.
    Update(main);

    // set mode and gear.
    SetState(cmddata.mode, cmddata.gear, main);

    if (!mode_is_setting && !gear_is_setting) {
#ifdef DIRECT_CONTROL
      // directly set accel, brake, and steer.
      Direct(cmddata.accel, cmddata.brake, cmddata.steer, main);
#else
      // control accel, brake, and steer.
      Control(cmddata.vel, main);
#endif
    }

    // get interval in milliseconds.
    interval = cmd_rx_interval - (getTime() - tstamp);

    if (interval > 0) {
      cout << "sleeping for " << interval << "ms" << endl;
      usleep(interval * 1000); // not really real-time...
    }
  }
  return NULL;
}
