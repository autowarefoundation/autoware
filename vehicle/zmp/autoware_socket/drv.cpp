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
#include <queue>

double estimate_accel = 0.0;
int target_accel_level = 0;
double accel_diff_sum = 0;
double brake_diff_sum = 0;
queue<double> accel_diff_buffer;
queue<double> brake_diff_buffer;

static void clear_diff()
{
  int i;

  accel_diff_sum = 0;
  brake_diff_sum = 0;

  for (i = 0; i < (int) accel_diff_buffer.size(); i++) {
    accel_diff_buffer.pop();
  }
  for (i = 0; i < (int) brake_diff_buffer.size(); i++) {
    brake_diff_buffer.pop();
  }
}

void MainWindow::SetDrvMode(int mode)
{
  switch (mode) {
  case CMD_MODE_MANUAL:
    cout << "Switching to MANUAL (Accel/Brake)" << endl;
    ZMP_SET_DRV_MANUAL();
    break;
  case CMD_MODE_PROGRAM:
    cout << "Switching to PROGRAM (Accel/Brake)" << endl;
    ZMP_SET_DRV_PROGRAM();
    clear_diff(); // initialize I control.
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }
}

void MainWindow::SetGear(int gear)
{
  double current_velocity = vstate.velocity; // km/h

  // double check if the velocity is zero,
  // SetGear() should not be called when driving.
  if (current_velocity != 0.0) {
    return;
  }

  // make sure to stop the vehicle.
  ZMP_STOP();

  switch (gear) {
  case CMD_GEAR_D:
    cout << "Shifting to Gear D" << endl;
    ZMP_SET_SHIFT_POS_D();
    break;
  case CMD_GEAR_R:
    cout << "Shifting to Gear R" << endl;
    ZMP_SET_SHIFT_POS_R();
    break;
  case CMD_GEAR_B:
    cout << "Shifting to Gear B" << endl;
    ZMP_SET_SHIFT_POS_B();
    break;
  case CMD_GEAR_N:
    cout << "Shifting to Gear N" << endl;
    ZMP_SET_SHIFT_POS_N();
    break;
  default:
    cout << "Unknown gear: " << gear << endl;    
  }

  sleep(1); // wait for a while to change the gear
}

double _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;
  double ret;

  // acclerate by releasing the brake pedal if pressed.
  if (vstate.brake_stroke > _BRAKE_PEDAL_OFFSET) {
    /*
    double target_brake_stroke = vstate.brake_stroke - _BRAKE_RELEASE_STEP;
    if (target_brake_stroke < 0)
      target_brake_stroke = 0;
    ret = -target_brake_stroke; // if ret is negative, brake will be applied.
    */

    // vstate has some delay until applying the current state.
    // perhaps we can just return 0 (release brake pedal) here to avoid acceleration delay.
    ret = 0;

    /* reset PID variables. */
    e_prev = 0;
    clear_diff();
  }
  else { // PID control
    double target_accel_stroke;

    e = cmd_velocity - current_velocity;

    e_d = e - e_prev;
    
    accel_diff_sum += e;

#if 0 // shouldn't we limit the cycles for I control?
    accel_diff_buffer.push(e);
    if (accel_diff_buffer.size() > _K_ACCEL_I_CYCLES) {
      double e_old = accel_diff_buffer.front();
      accel_diff_sum -= e_old;
      if (accel_diff_sum < 0) {
        accel_diff_sum = 0;
      }
      accel_diff_buffer.pop();
    }
#endif

    if (accel_diff_sum > _ACCEL_MAX_I) {
      e_i = _ACCEL_MAX_I;
    }
    else {
      e_i = accel_diff_sum;
    }

#if 1
    target_accel_stroke = _K_ACCEL_P * e + _K_ACCEL_I * e_i + _K_ACCEL_D * e_d;
#else
    printf("accel_p = %lf, accel_i = %lf, accel_d = %lf\n", shm_ptr->accel.P, shm_ptr->accel.I, shm_ptr->accel.D);
    target_accel_stroke = shm_ptr->accel.P * e + shm_ptr->accel.I * e_i + shm_ptr->accel.D * e_d;
#endif
    
 if (target_accel_stroke > _ACCEL_PEDAL_MAX) {
      target_accel_stroke = _ACCEL_PEDAL_MAX;
    }
    else if (target_accel_stroke < 0) {
      target_accel_stroke = 0;
    }

    //cout << "e = " << e << endl;
    //cout << "e_i = " << e_i << endl;
    //cout << "e_d = " << e_d << endl;

    ret = target_accel_stroke;

    e_prev = e;

#if 1 /* log */
      ofstream ofs("/tmp/drv_accel.log", ios::app);
      ofs << cmd_velocity << " " 
      << current_velocity << " " 
      << e << " " 
      << e_i << " " 
      << e_d << " " 
      << target_accel_stroke << " " 
      << endl;
#endif
  }

  return ret;
}

double _brake_stroke_pid_control(double current_velocity, double cmd_velocity)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;
  double ret;

  // decelerate by releasing the accel pedal if pressed.
  if (vstate.accel_stroke > _ACCEL_PEDAL_OFFSET) {
    /*
    double target_accel_stroke = vstate.accel_stroke - _ACCEL_RELEASE_STEP;
    if (target_accel_stroke < 0)
      target_accel_stroke = 0;
    ret = -target_accel_stroke; // if ret is negative, accel will be applied.
    */

    // vstate has some delay until applying the current state.
    // perhaps we can just return 0 (release accel pedal) here to avoid deceleration delay.
    ret = 0;

    /* reset PID variables. */
    e_prev = 0;
    clear_diff();
  }
  else { // PID control
    double target_brake_stroke;

    // since this is braking, multiply -1.
    e = -1 * (cmd_velocity - current_velocity);

    e_d = e - e_prev;
    
    brake_diff_sum += e;
    brake_diff_buffer.push(e);
    if (brake_diff_buffer.size() > _K_BRAKE_I_CYCLES) {
      double e_old = brake_diff_buffer.front();
      brake_diff_sum -= e_old;
      if (brake_diff_sum < 0) {
        brake_diff_sum = 0;
      }
      brake_diff_buffer.pop();
    }

    if (brake_diff_sum > _BRAKE_MAX_I) {
      e_i = _BRAKE_MAX_I;
    }
    else {
      e_i = brake_diff_sum;
    }

    target_brake_stroke = _K_BRAKE_P * e + _K_BRAKE_I * e_i + _K_BRAKE_D * e_d;
    if (target_brake_stroke > _BRAKE_PEDAL_MAX) {
      target_brake_stroke = _BRAKE_PEDAL_MAX;
    }
    else if (target_brake_stroke < 0) {
      target_brake_stroke = 0;
    }

    if (target_brake_stroke - vstate.brake_stroke > _BRAKE_STROKE_DELTA_MAX) {
      target_brake_stroke = vstate.brake_stroke + _BRAKE_STROKE_DELTA_MAX;
    }

    cout << "e = " << e << endl;
    cout << "e_i = " << e_i << endl;
    cout << "e_d = " << e_d << endl;

    ret = target_brake_stroke;

    e_prev = e;

#if 1 /* log */
      ofstream ofs("/tmp/drv_brake.log", ios::app);
      ofs << cmd_velocity << " " 
      << current_velocity << " " 
      << e << " " 
      << e_i << " " 
      << e_d << " " 
      << target_brake_stroke << " " 
      << endl;
#endif
  }

  return ret;
}

double _stopping_control(double current_velocity)
{
  double ret;
  static double old_brake_stroke = _BRAKE_PEDAL_MED;

  // decelerate by using brake	
  if (current_velocity < 0.1) {
    // nearly at stop -> apply full brake. brake_stroke should reach BRAKE_PEDAL_MAX in one second.
    int gain = (int)(((double)_BRAKE_PEDAL_MAX)*cycle_time);
    ret = old_brake_stroke + gain;
    if ((int)ret > _BRAKE_PEDAL_MAX)
      ret = _BRAKE_PEDAL_MAX;
    old_brake_stroke = ret;
  }
  else {
    /*
    // one second is approximately how fast full brakes applied in sharp stop
    int gain = (int)(((double)_BRAKE_PEDAL_MED)*cycle_time);
    ret = vstate.brake_stroke + gain;
    if ((int)ret > _BRAKE_PEDAL_MED)
      ret = _BRAKE_PEDAL_MED;
    */

    // vstate has some delay until applying the current state.
    // perhaps we can just set BRAKE_PEDAL_MED to avoid deceleration delay.
    ret = _BRAKE_PEDAL_MED;
    old_brake_stroke = _BRAKE_PEDAL_MED;
  }

  return ret;
}

void MainWindow::StrokeControl(double current_velocity, double cmd_velocity)
{
  static queue<double> vel_buffer;
  static uint vel_buffer_size = 10; 
  double old_velocity = 0.0;

  // don't control if not in program mode.
  if (!ZMP_DRV_CONTROLLED()) {
    clear_diff();
    return;
  }

  // estimate current acceleration.
  vel_buffer.push(current_velocity);
  if (vel_buffer.size() > vel_buffer_size) {
    old_velocity = vel_buffer.front();
    vel_buffer.pop(); // remove old_velocity from the queue.
    estimate_accel = 
      (current_velocity-old_velocity)/(cycle_time*vel_buffer_size);
  }

  cout << "estimate_accel: " << estimate_accel << endl; 

  if (fabs(cmd_velocity) > current_velocity
      && fabs(cmd_velocity) > 0.0 
      && current_velocity < SPEED_LIMIT) {
    double accel_stroke;
    cout << "accelerate: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
    accel_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);
    if (accel_stroke > 0) {
      cout << "ZMP_SET_DRV_STROKE(" << accel_stroke << ")" << endl;
      ZMP_SET_DRV_STROKE(accel_stroke);
    }
    else {
      cout << "ZMP_SET_DRV_STROKE(0)" << endl;
      ZMP_SET_DRV_STROKE(0);
      cout << "ZMP_SET_BRAKE_STROKE(" << -accel_stroke << ")" << endl;
      ZMP_SET_BRAKE_STROKE(-accel_stroke);
    }
  } 
  else if (fabs(cmd_velocity) < current_velocity
           && fabs(cmd_velocity) > 0.0) {
    double brake_stroke;
    cout << "decelerate: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
    brake_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
    if (brake_stroke > 0) {
      cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
      ZMP_SET_BRAKE_STROKE(brake_stroke);
    }
    else {
      cout << "ZMP_SET_BRAKE_STROKE(0)" << endl;
      ZMP_SET_BRAKE_STROKE(0);
      cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
      ZMP_SET_DRV_STROKE(-brake_stroke);
    }
  }
  else if (cmd_velocity == 0.0 && current_velocity != 0.0) {
    double brake_stroke;
    cout << "stopping: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
    if (current_velocity < 3.0) { // nearly stopping
      ZMP_SET_DRV_STROKE(0);
      brake_stroke = _stopping_control(current_velocity);
      cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
      ZMP_SET_BRAKE_STROKE(brake_stroke);
    }
    else {
      brake_stroke = _brake_stroke_pid_control(current_velocity, 0);
      if (brake_stroke > 0) {
        cout << "ZMP_SET_BRAKE_STROKE(" << brake_stroke << ")" << endl;
        ZMP_SET_BRAKE_STROKE(brake_stroke);
      }
      else {
        cout << "ZMP_SET_DRV_STROKE(0)" << endl;
        ZMP_SET_DRV_STROKE(0);
        cout << "ZMP_SET_DRV_STROKE(" << -brake_stroke << ")" << endl;
        ZMP_SET_DRV_STROKE(-brake_stroke);
      }
    }
  }
  else {
    cout << "unknown: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
  }
}

void MainWindow::VelocityControl(double current_velocity, double cmd_velocity)
{
  double vel_diff_inc = 1.0;
  double vel_diff_dec = 2.0;
  double vel_offset_inc = 2;
  double vel_offset_dec = 4;
  if (cmd_velocity > current_velocity){
    double increase_velocity = current_velocity + vel_diff_inc;
    ZMP_SET_DRV_VELOC((increase_velocity + vel_offset_inc) * 100);
    cout << "increase: " << "vel = " << increase_velocity  << endl; 
  }
  else {
    double decrease_velocity = current_velocity - vel_diff_dec;
    if (decrease_velocity > vel_offset_dec) {
      ZMP_SET_DRV_VELOC((decrease_velocity - vel_offset_dec) * 100);
    }
    else if (current_velocity > 0) {
      decrease_velocity = 0;
      ZMP_SET_DRV_VELOC(0);
    }
    cout << "decrease: " << "vel = " << decrease_velocity  << endl; 
  }
}


