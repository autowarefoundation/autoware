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

#ifndef __AUTOWARE_SOCKET_H__
#define __AUTOWARE_SOCKET_H__

#include "mainwindow.h"
#include "autoware_config.h"

#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <queue>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <netdb.h>

using namespace zmp::hev;

#define NO_TSTAMP 0

#define CMD_MODE_MANUAL 0 
#define CMD_MODE_PROGRAM 1
#define CMD_GEAR_D 1
#define CMD_GEAR_R 2
#define CMD_GEAR_B 3
#define CMD_GEAR_N 4

#define MILLISECOND 1000  

#define SPEED_LIMIT 80 // km/h
#define STROKE_SPEED_LIMIT 80 // use stroke until 80 km/h

typedef struct pose_data {
  double x;
  double y;
  double z;
  double R;
  double P;
  double Y;
  long long int tstamp;
} pose_data_t;

typedef struct vel_data {
  double tv;
  double sv;
  long long int tstamp;
} vel_data_t;

typedef struct vel_cmd_data {
  vel_data_t vel;
  int mode;
} vel_cmd_data_t;

typedef struct base_data {
  pose_data_t odom_pose;
  vel_data_t vel_cur;
  unsigned char batt_voltage;
  bool motor_state;
  long long int tstamp;
} base_data_t;

typedef struct state_data {
  long long int update_tstamp;
  long long int control_tstamp;

  // state data reading from base
  base_data_t base;

  // control info
  pose_data_t est_pose;
  vel_data_t vel_cmd;
  int control_mode;

} state_data_t;

typedef struct lrf_config {
  int N;
  double fov; // field of view radians
  double res;
  double start_a;
  double stop_a;
} lrf_config_t;

typedef int lrf_data_t; // range data type

typedef struct _CMDDATA {
    vel_data_t vel;
    int mode;
    int gear;
    int accel;
    int steer;
    int brake;
} CMDDATA;

typedef struct vehicle_state {
	float accel_stroke;
	float brake_stroke;
	float steering_torque;
	float steering_angle;
	float velocity;
	long long int tstamp;
} vehicle_state_t;

extern vehicle_state_t vstate;
extern int can_tx_interval; // ms
extern int cmd_rx_interval; // ms
extern std::string ros_ip_address;
extern double estimate_accel;
extern double cycle_time;

// convert km/h to m/s
static inline double KmhToMs(double v)
{
  return (v*1000.0/(60.0*60.0));
}

// get current time
static inline long long int getTime(void)
{
  //returns time in milliseconds 
  struct timeval current_time;
  struct timezone ttz;
  double t;
  // lock();
  gettimeofday(&current_time, &ttz);
  t = ((current_time.tv_sec * 1000.0) +
       (current_time.tv_usec) / 1000.0);
  // unlock();
  return static_cast<long long int>(t);
}

#endif //__AUTOWARE_SOCKET_H__

