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
  double ret;

  // the read value of current steering position may have some error.
  current_steering_angle -= _STEERING_ANGLE_ERROR;

  // angvel, not really used for steering control...
  static double prev_steering_angle = -100000;
  if (prev_steering_angle == -100000) {
    prev_steering_angle = current_steering_angle;
  }

  double current_steering_angvel = (current_steering_angle - prev_steering_angle) / (STEERING_INTERNAL_PERIOD/1000.0);

  /////////////////////////////////
  // angle PID control
  double steering_diff = cmd_steering_angle - current_steering_angle; 
  steering_diff_sum += steering_diff;

  if (steering_diff_sum > _STEERING_MAX_SUM) {
    steering_diff_sum = _STEERING_MAX_SUM;
  }
  if (steering_diff_sum < -_STEERING_MAX_SUM) {
    steering_diff_sum = -_STEERING_MAX_SUM;
  }

  static double angvel_diff = 0;
  angvel_diff = angvel_diff * 0.0 - current_steering_angvel * 1; 

  double k_p = _K_STEERING_P;
  double k_i = _K_STEERING_I;
  double k_d = _K_STEERING_D;

  // change PID params depending on the driving speed.
  if (vstate.velocity < 40) {
    k_p = _K_STEERING_P_40;
    k_i = _K_STEERING_I_40;
    k_d = _K_STEERING_D_40;
  }
  else if (vstate.velocity < 30) {
    k_p = _K_STEERING_P_30;
    k_i = _K_STEERING_I_30;
    k_d = _K_STEERING_D_30;
  }
  else if (vstate.velocity < 20) {
    k_p = _K_STEERING_P_20;
    k_i = _K_STEERING_I_20;
    k_d = _K_STEERING_D_20;
  }
  else if (vstate.velocity < 10) {
    k_p = _K_STEERING_P_10;
    k_i = _K_STEERING_I_10;
    k_d = _K_STEERING_D_10;
  }

  // torque control.
  double target_steering_torque = 
    steering_diff * k_p + steering_diff_sum * k_i + angvel_diff * k_d;

  // clip
  if (target_steering_torque > _STEERING_MAX_TORQUE) {
    target_steering_torque = _STEERING_MAX_TORQUE;
  }
  if (target_steering_torque < -_STEERING_MAX_TORQUE) {
    target_steering_torque = -_STEERING_MAX_TORQUE;
  }

  ret = target_steering_torque;

  prev_steering_angle  = current_steering_angle;

#if 1 /* log */ 
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << cmd_steering_angle << " " 
      << current_steering_angle << " " 
      << steering_diff << " "  
      << steering_diff_sum << " "  
      << current_steering_angvel << " "  
      << target_steering_torque << endl;
#endif

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
