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

#define IS_STR_MODE_PROGRAM() (_hev_state.strInf.mode == MODE_PROGRAM && _hev_state.strInf.servo == SERVO_TRUE)
#define IS_STR_MODE_MANUAL() (_hev_state.strInf.mode == MODE_MANUAL || _hev_state.strInf.servo == SERVO_FALSE)

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
    if (IS_STR_MODE_PROGRAM()) {
      cout << "Switching to MANUAL (Steering)" << endl;
      hev->SetStrMode(MODE_MANUAL);
      usleep(200000);
      hev->SetStrServo(SERVO_FALSE);
      //usleep(200000);
    }
    break;
  case CMD_MODE_PROGRAM:
    if (IS_STR_MODE_MANUAL()) {
      cout << "Switching to PROGRAM (Steering)" << endl;
      hev->SetStrMode(MODE_PROGRAM);
      usleep(200000);
      hev->SetStrCMode(CONT_MODE_TORQUE);
      //hev->SetStrCMode(CONT_MODE_ANGLE);
      usleep(200000);
      hev->SetStrServo(SERVO_TRUE);
      //usleep(200000);

      clear_diff();
    }
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }

}

// for torque control
#define _STEERING_MAX_ANGVELSUM 1000
#define _K_STEERING_TORQUE 10
#define _K_STEERING_TORQUE_I 0.5
#define _STEERING_MAX_TORQUE 2000
#define _STEERING_MAX_SUM 100 //deg*0.1s for I control

// PID params
#define _K_STEERING_P 60//130  
#define _K_STEERING_I 13//13
#define _K_STEERING_D 10//12

void _str_torque_pid_control(double current_steering_angle, double cmd_steering_angle, HevCnt* hev)
{
  static double prev_steering_angle = -100000;

  //calc angular velocity
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

  // magic params...
  double k_d = _K_STEERING_D;
  if (fabs(steering_diff) < 10) {
    k_d = _K_STEERING_D / 2;
  }
  
  // use k_d instead of _K_STEERING_D.
  double target_steering_torque = steering_diff * _K_STEERING_P + steering_diff_sum * _K_STEERING_I + angvel_diff * k_d;

  // clip
  if (target_steering_torque > _STEERING_MAX_TORQUE) {
    target_steering_torque = _STEERING_MAX_TORQUE;
  }
  if (target_steering_torque < -_STEERING_MAX_TORQUE) {
    target_steering_torque = -_STEERING_MAX_TORQUE;
  }

  //cout << "steering_angle = " << current_steering_angle << endl;
  //cout << "steering_angvel = " << current_steering_angvel << endl;

  hev->SetStrTorque(target_steering_torque);

  prev_steering_angle  = current_steering_angle;

#if 0 /* log */ 
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << cmd_steering_angle << " " 
      << current_steering_angle << " " 
      << current_steering_angvel << " "  
      << steering_diff_sum << " "  
      << target_steering_torque << endl;
#endif
}

void MainWindow::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  // do not call a control funtion in manual mode.
  if (IS_STR_MODE_MANUAL()) {
    clear_diff();
    return;
  }

  _str_torque_pid_control(current_steering_angle, cmd_steering_angle, hev);
}


