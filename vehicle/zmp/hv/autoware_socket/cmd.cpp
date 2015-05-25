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
#include <queue>

#include "mainwindow.h"
#include "autoware_socket.h"

double cycle_time = 0.0;
double estimate_accel = 0.0;

#define OUTPUT_LOG

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
    
    /*
      ofstream ofs("/tmp/cmd.log", ios::app);
      ofs << cmddata.vel.tv << " " 
      << cmddata.vel.sv << " " 
      << endl;
    */

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

  // update Hev's drvinf/strinf/brkinf.
  main->UpdateState();
}

void Prepare(int mode, int gear, void* p) 
{
  static int old_mode = -1;
  static int old_gear = -1;
  MainWindow* main = (MainWindow*)p;

  if (mode != old_mode) {
    main->SetMode(mode);
    old_mode = mode;
  }

  if (gear != old_gear) {
    double current_velocity = _hev_state.drvInf.veloc; // km/h
    // never change the gear when driving!
    if (current_velocity == 0) {
      main->SetGear(gear);
      old_gear = gear;
    }
  }
}


void Control(vel_data_t vel, void* p) 
{
  MainWindow* main = (MainWindow*)p;
  static long long int old_tstamp = 0;

  // calculate current time
  cycle_time = (_hev_state.tstamp - old_tstamp) / 1000.0;

  queue<double> vel_buffer;
  static uint vel_buffer_size = 10; 

  double old_velocity = 0.0;

  double current_velocity = _hev_state.drvInf.veloc; // km/h
  double current_steering_angle = _hev_state.strInf.angle; // degree
 
  int cmd_velocity = vel.tv * 3.6;
  int cmd_steering_angle;

  //<tku debug  force velocity
#if 0
  cmd_velocity = 10;
  static int inc_flag = 1;
  if (current_velocity > 5) {
    cmd_velocity = 0;
    inc_flag = 0;
  }
  if (!inc_flag)
    cmd_velocity = 0;
#endif
  // />

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

  // estimate current acceleration.
  vel_buffer.push(fabs(current_velocity));
  
  if (vel_buffer.size() > vel_buffer_size) {
    old_velocity = vel_buffer.front();
#if 0 // debug
    cout << "old_velocity = " << old_velocity << endl;
    cout << "current_velocity = " << current_velocity << endl;
#endif
    vel_buffer.pop(); // remove old_velocity from the queue.
    estimate_accel = (fabs(current_velocity)-old_velocity)/(cycle_time*vel_buffer_size);
  }

  cout << "Current: " << "vel = " << current_velocity 
       << ", str = " << current_steering_angle << endl; 
  cout << "Command: " << "vel = " << cmd_velocity 
       << ", str = " << cmd_steering_angle << endl; 
  cout << "Estimate Accel: " << estimate_accel << endl; 

  // TRY TO INCREASE STEERING
  //  sv +=0.1*sv;

  // if tv non zero then check if in drive gear first
  //--------------------------------------------------------------
  // if in neutral and get +'ve cmd vel
  //    - put brake on
  //    - change shift to drive
  // if in neutral and vel cmd 0
  //    - release brake if pressed
  
  //------------------------------------------------------
  // if shift in drive
  //     - if cmd vel > 0 && brake pressed
  //              - release brake
  //     - if cmd vel == 0
  //              - if cur_vel < 0.1, nearly stopped
  //                     - full brake too stop, wait, and shift to neutral
  //              - else - brake to HEV_MED_BRAKE
  //                       - decellerate until nearly stopped
  // now motion if in drift mode and shift is in drive
  
 
  //////////////////////////////////////////////////////
  // Accel and Brake
  //////////////////////////////////////////////////////

  if (cmd_velocity < STROKE_CTRL_LIMIT) {
    if (fabs(cmd_velocity) >= fabs(current_velocity) 
        && fabs(cmd_velocity) > 0.0 
        && fabs(current_velocity) <= KmhToMs(SPEED_LIMIT) ) {
      //accelerate !!!!!!!!!!!!!!!!
      cout << "AccelerateControl(current_velocity=" << current_velocity 
           << ", cmd_velocity=" << cmd_velocity << ")" << endl;
      main->AccelerateControl(current_velocity, cmd_velocity);
    } 
    else if (fabs(cmd_velocity) < fabs(current_velocity) 
             && fabs(cmd_velocity) > 0.0) {
      //decelerate!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      cout << "DecelerateControl(current_velocity=" << current_velocity 
           << ", cmd_velocity=" << cmd_velocity << ")" << endl;
      main->DecelerateControl(current_velocity, cmd_velocity); 
    }
    else if (cmd_velocity == 0.0 && fabs(current_velocity) != 0) {
      //Stopping!!!!!!!!!!!
      cout << "StoppingControl(current_velocity=" << current_velocity 
           << ", cmd_velocity=" << cmd_velocity << ")" << endl;
      main->StoppingControl(current_velocity, cmd_velocity);
    }
    else {
      cout << "NothingAccelBrake(current_velocity=" << current_velocity 
           << ", cmd_velocity=" << cmd_velocity << ")" << endl;
    }
  } else { /* HEV velocity control */
    double vel_diff_inc = 1.0;
    double vel_diff_dec = 2.0;
    double vel_offset_inc = 2;
    double vel_offset_dec = 4;
    if (cmd_velocity > current_velocity){
      double increase_velocity = current_velocity + vel_diff_inc;
      main->VelocityControl(increase_velocity + vel_offset_inc);
      cout << "increase: " << "vel = " << increase_velocity  << endl; 
    } else {
      double decrease_velocity = current_velocity - vel_diff_dec;
      if (decrease_velocity > vel_offset_dec) {
        main->VelocityControl(decrease_velocity - vel_offset_dec);
      }
      else if (current_velocity > 0) {
        decrease_velocity = 0;
        main->VelocityControl(0);
      }
      cout << "decrease: " << "vel = " << decrease_velocity  << endl; 
    }
  }
    /*
    */    
  //////////////////////////////////////////////////////
  // Steering
  //////////////////////////////////////////////////////
  int steering_internal_period = STEERING_INTERNAL_PERIOD;
  for (int i = 0; i < cmd_rx_interval/steering_internal_period - 1; i++) {
    main->SteeringControl(current_steering_angle, cmd_steering_angle);
    usleep(steering_internal_period * 1000);  
    Update(main);
    current_steering_angle = _hev_state.strInf.angle; // degree
  }
  main->SteeringControl(current_steering_angle, cmd_steering_angle);

  // save the time stamp.
  old_tstamp = _hev_state.tstamp;
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

    // update HEV state.
    Update(main);

    // set mode and gear.
    Prepare(cmddata.mode, cmddata.gear, main);

    

#ifdef DIRECT_CONTROL
    // directly set accel, brake, and steer.
    Direct(cmddata.accel, cmddata.brake, cmddata.steer, main);
#else
      // control accel, brake, and steer.
      Control(cmddata.vel, main);
#endif

    // get interval in milliseconds.
    interval = cmd_rx_interval - (getTime() - tstamp);

    if (interval > 0) {
      cout << "sleeping for " << interval << "ms" << endl;
      usleep(interval * 1000); // not really real-time...
    }
  }
  return NULL;
}
