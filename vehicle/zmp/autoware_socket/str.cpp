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

  e = steering_diff;
  e_i = steering_diff_sum;

  if (e_prev == 0xffffffff) {
    e_prev = e;
  }

  e_d = (e - e_prev) / (STEERING_INTERNAL_PERIOD/1000.0);

  double k_p = _K_STEERING_P_SLOW;
  double k_i = _K_STEERING_I_SLOW;
  double k_d = _K_STEERING_D_SLOW;

  double steering_max_torque = _STEERING_MAX_TORQUE;

  // change PID params depending on the driving speed.
  if (vstate.velocity < 30) {
    k_p = _K_STEERING_P_SLOW;
    k_i = _K_STEERING_I_SLOW;
    k_d = _K_STEERING_D_SLOW;

    // if angular velocity is slow, ignore D to smoothen the steer.
    if (fabs(e_d) < _STEERING_ANGVEL_BOUNDARY) {
      e_d = 0;
    }
    // if the error is small, ignore D and I to stabilize the steer.
    else if (fabs(e) < _STEERING_IGNORE_ERROR) {
      e_d = 0;
      steering_diff_sum = 0;
    }

    // slow down the steer speed.
    steering_max_torque /= 2.0;
  }

  // torque control
  double target_steering_torque = e * k_p + e_i * k_i + e_d * k_d;

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
      << e_d << " "  
      << (e - e_prev) / (STEERING_INTERNAL_PERIOD/1000.0) << " "
      << target_steering_torque << endl;
#endif

  e_prev = e;

  return ret;
}

void MainWindow::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  double torque;

  // don't control if not in program mode.
  if (!ZMP_STR_CONTROLLED()) {
    clear_diff();
    return;
  }

  if (vstate.velocity < 1) { // if nearly at stop, don't control steering.
    torque = 0;
  }
  else {
    torque = _str_torque_pid_control(current_steering_angle, cmd_steering_angle);
  }
  cout << "ZMP_SET_STR_TORQUE(" << torque << ")" << endl;
  ZMP_SET_STR_TORQUE(torque);
}
