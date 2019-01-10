/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
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

#include "mainwindow.h"
#include "autoware_socket.h"

static double steering_diff_sum = 0;
queue<double> steering_diff_buffer;

static void clear_diff()
{
  int i;

  steering_diff_sum = 0;

  for (i = 0; i < (int) steering_diff_buffer.size(); i++) {
    steering_diff_buffer.pop();
  }
}

void MainWindow::SetStrMode(int mode)
{
  switch (mode) {
  case CMD_MODE_MANUAL:
    cout << "Switching to MANUAL (Steering)" << endl;
    ZMP_SET_STR_MANUAL();
    break;
  case CMD_MODE_PROGRAM:
    cout << "Switching to PROGRAM (Steering)" << endl;
    ZMP_SET_STR_PROGRAM();
    clear_diff();
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }
}

// positive torque: left/right
// nagative torque: left/right
double _str_torque_pid_control(double current_steering_angle, double cmd_steering_angle)
{
  double e;
  static double e_prev = 0xffffffff;
  double e_i;
  double e_d;
  double ret;

  // adjust the steering angle error (default offset).
  current_steering_angle -= _STEERING_ANGLE_ERROR;

  double steering_diff = cmd_steering_angle - current_steering_angle; 
  steering_diff_sum += steering_diff;

  if (steering_diff_sum > _STEERING_MAX_SUM) {
    steering_diff_sum = _STEERING_MAX_SUM;
  }
  if (steering_diff_sum < -_STEERING_MAX_SUM) {
    steering_diff_sum = -_STEERING_MAX_SUM;
  }

  // e: proportional error
  // e_i: integral error
  // e_d: differential error
  e = steering_diff;
  e_i = steering_diff_sum;

  if (e_prev == 0xffffffff) {
    e_prev = e;
  }

  e_d = (e - e_prev) / (STEERING_INTERNAL_PERIOD/1000.0);

  double k_p = _K_STEERING_P;
  double k_i = _K_STEERING_I;
  double k_d = _K_STEERING_D;

  double steering_max_torque = _STEERING_MAX_TORQUE;

  // torque control
  static double target_steering_torque = 0;

#if 0 // DEBUG configuration
  printf("accel_p = %lf, accel_i = %lf, accel_d = %lf\n",
         shm_ptr->steer.P, shm_ptr->steer.I, shm_ptr->steer.D);
  k_p = shm_ptr->steer.P;
  k_i = shm_ptr->steer.I;
  k_d = shm_ptr->steer.D;
#endif

  // if e is small, we have to move faster (more sensitive).
  if (fabs(e) < _STEERING_DIFF_SMALL) {
    k_p = _K_STEERING_P_STRICT;
    k_i = _K_STEERING_I_STRICT;
    k_d = _K_STEERING_D_STRICT;
  }

  // if we are slow on the straight line, do lazy control.
  if (fabs(e) < 0.3 && fabs(cmd_steering_angle) < 3 && vstate.velocity < 10) {
    // don't accumulate error.
    steering_diff_sum = 0;
    // skip PID control...
    if (e > 0) {
      target_steering_torque = 10;
    }
    else {
      target_steering_torque = -10;
    }
  }
  else {
    target_steering_torque = e * k_p + e_i * k_i + e_d * k_d;
  }

  // clip
  if (target_steering_torque > steering_max_torque) {
    target_steering_torque = steering_max_torque;
  }
  if (target_steering_torque < -steering_max_torque) {
    target_steering_torque = -steering_max_torque;
  }

  ret = target_steering_torque;

#if 1 /* log */ 
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << cmd_steering_angle << " " 
      << current_steering_angle << " " 
      << e << " "  
      << e_i << " "  
      << steering_diff_sum << " "//shm_ptr->steer.D * e_d << " "
      << (e - e_prev) / (STEERING_INTERNAL_PERIOD/1000.0) << " "
      << target_steering_torque << endl;
#endif

  e_prev = e;

  return ret;
}

void MainWindow::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  double torque;

  // positive {angle,torque} == left
  // negative {angle,torque} == right

  // don't control if not in program mode.
  if (!ZMP_STR_CONTROLLED()) {
    clear_diff();
    return;
  }

#if 1
  //------------ ANGLE CONTROL --------------
  cmd_steering_angle -= _STEERING_ANGLE_ERROR;
  ZMP_SET_STR_ANGLE(cmd_steering_angle*10);
#if 1 /* log */
  ofstream ofs("/tmp/angle.log", ios::app);
  ofs << cmd_steering_angle << " "
      << current_steering_angle << endl;
#endif
  return;
  //-----------------------------------------
#endif

  //----------- TORQUE CONTROL -------------
  if (vstate.velocity < 3) { // if nearly at stop, don't control steering.
    torque = 0;
  }
  else {
    torque = _str_torque_pid_control(current_steering_angle, cmd_steering_angle);
  }
  cout << "ZMP_SET_STR_TORQUE(" << torque << ")" << endl;
  ZMP_SET_STR_TORQUE(torque);
  //--------------------------------------
}
