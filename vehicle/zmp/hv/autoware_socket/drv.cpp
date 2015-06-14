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

#define IS_DRV_MODE_PROGRAM() (_hev_state.drvInf.mode == MODE_PROGRAM && _hev_state.drvInf.servo == SERVO_TRUE)
#define IS_DRV_MODE_MANUAL() (_hev_state.drvInf.mode == MODE_MANUAL || _hev_state.drvInf.servo == SERVO_FALSE)

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
    if (IS_DRV_MODE_PROGRAM()) {
      cout << "Switching to MANUAL (Accel/Brake)" << endl;
      hev->SetDrvMode(MODE_MANUAL);
      usleep(200000);
      hev->SetDrvServo(SERVO_FALSE);
      usleep(200000);
    }
    break;
  case CMD_MODE_PROGRAM:
    if (IS_DRV_MODE_MANUAL()) {
      cout << "Switching to PROGRAM (Accel/Brake)" << endl;
      hev->SetDrvMode(MODE_PROGRAM);
      usleep(200000);
      hev->SetDrvCMode(CONT_MODE_STROKE);
      //hev->SetDrvCMode(CONT_MODE_VELOCITY);
      usleep(200000);
      hev->SetDrvServo(SERVO_TRUE);
      usleep(200000);

      // initialize I control.
      clear_diff();

      break;
    }
  default:
    cout << "Unknown mode: " << mode << endl;
  }
}

void MainWindow::SetGear(int gear)
{
  double current_velocity = _hev_state.drvInf.veloc; // km/h

  // double check if the velocity is zero,
  // SetGear() should not be called when driving.
  if (current_velocity != 0) {
    return;
  }

  hev->SetDrvStroke(0);
  sleep(1);
  hev->SetBrakeStroke(HEV_MAX_BRAKE);
  sleep(1);

  switch (gear) {
  case CMD_GEAR_D:
    cout << "Shifting to Gear D" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_D);
    break;
  case CMD_GEAR_R:
    cout << "Shifting to Gear R" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_R);
    break;
  case CMD_GEAR_B:
    cout << "Shifting to Gear B" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_B);
    break;
  case CMD_GEAR_N:
    cout << "Shifting to Gear N" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_N);
    break;
  default:
    cout << "Unknown gear: " << gear << endl;    
  }
  sleep(1);
}

bool _accel(int target_accel, int gain, int current_accel, HevCnt *hev)
{
  static int old_accel = 0;

  // for small increments of change in accel pedal,
  // prius sometimes does not apply 
  // assume  cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state

  int cmd_accel = current_accel;

  cout << "current_accel = " << current_accel << endl;
  cout << "target_accel = " << target_accel << endl;
  cout << "gain = " << gain << endl;
  cout << "old_accel = " << old_accel << endl;
  cout << "cmd_accel = " << cmd_accel << endl;

  if (current_accel < target_accel) {
    cmd_accel += gain;
    if (cmd_accel > target_accel) { 
      cmd_accel = target_accel;
    }
  } 

  if (current_accel > target_accel) {
    cmd_accel -= gain;
    if (cmd_accel < target_accel) {
      cmd_accel = target_accel;
    }
  }

  cout << "setting accel pedal to " << cmd_accel << endl;

  if (cmd_accel >= 0 && cmd_accel < ACCEL_PEDAL_MAX) {
    cout << "SetDrvStroke(" << cmd_accel << ")" << endl;
    hev->SetDrvStroke(cmd_accel);
  }

  old_accel = cmd_accel;

  if (target_accel == current_accel){ 
    old_accel = 0;
    return true;
  }

  return false;
}

bool _brake(int target_brake, int gain, int current_brake, HevCnt *hev) 
{
  static int old_brake = 0;

  // for small increments of change in brake pedal,
  // prius sometimes does not apply brake
  // assume brake cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state

  int cmd_brake = current_brake;
  if (old_brake != 0) 
    cmd_brake = old_brake;

  cout << "current_brake = " << current_brake << endl;
  cout << "target_brake = " << target_brake << endl;
  cout << "gain = " << gain << endl;
  cout << "old_brake = " << old_brake << endl;
  cout << "cmd_brake = " << cmd_brake << endl;

  if (current_brake < target_brake) {
    cmd_brake += gain;
    if (cmd_brake > target_brake) { 
      cmd_brake = target_brake;
    }
  } 

  if (current_brake > target_brake) {
    cmd_brake -= gain;
    if (cmd_brake < target_brake) {
      cmd_brake = target_brake;
    }
  }
  
  cout << "SetBrakeStroke(" << cmd_brake << ")" << endl;
  hev->SetBrakeStroke(cmd_brake);

  old_brake = cmd_brake;

  if (target_brake == current_brake){ 
    old_brake = -1;
    return true;
  }

  return false;
}

void _accelerate_control(double current_velocity,double cmd_velocity, HevCnt *hev)
{
  // acclerate by releasing the brake pedal if pressed.
  if (_hev_state.brkInf.pressed == true && CURRENT_BRAKE_STROKE() == 0) {
    cout << "brake pressed, release" << endl;
    int gain = (int)(((double)HEV_MAX_BRAKE)/0.5*cycle_time); 
    _brake(0, gain, CURRENT_BRAKE_STROKE(), hev);
  }
  
  // accelrate using accelrator pedal.
  double acc_vel_threshold = KmhToMs(8.0);
  int acc_start_target = ACCEL_PEDAL_SET_START;
        
  // if desired vel is high, up first acceleration.
  if (fabs(cmd_velocity) > KmhToMs(16.0)){ 
    acc_start_target = ACCEL_PEDAL_SET_START_FAST;
  }

  // if at low speed, initial acceleration to base level
  if (fabs(current_velocity) < acc_vel_threshold && 
      fabs(cmd_velocity) > acc_vel_threshold) {
    int gain = (int)(((double)acc_start_target)/1.0*cycle_time);
    _accel(acc_start_target, gain, CURRENT_ACCEL_STROKE(), hev); 
    target_accel_level = acc_start_target;
  }
  else { // else slowly accelrate as needed
    // test if accelerating
    if (estimate_accel < 0.05) {
      // else adjust control slowly 
      if (target_accel_level < ACCEL_PEDAL_MAX - ACCEL_PEDAL_STEP_BIG) {
        if ((fabs(cmd_velocity) - fabs(current_velocity)) > KmhToMs(5.0))
          target_accel_level += ACCEL_PEDAL_STEP_BIG;
        else
          target_accel_level+= ACCEL_PEDAL_STEP;
      }
    }

    int gain = (int)(((double) target_accel_level)/0.5*cycle_time);
    _accel(target_accel_level, gain, CURRENT_ACCEL_STROKE(), hev); 
          
  }
}

#define _K_ACCEL_P 30.0 //3.0
#define _K_ACCEL_I 2.0
#define _K_ACCEL_D 2.0
#define _K_ACCEL_I_CYCLES 100
#define _ACCEL_MAX_I 600

void _accel_stroke_pid_control(double current_velocity, double cmd_velocity, HevCnt* hev)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;

  // acclerate by releasing the brake pedal if pressed.
  if (_hev_state.brkInf.pressed == true) {
    cout << "brake pressed, release" << endl;
    int gain = 500;
    int cmd_brake = CURRENT_BRAKE_STROKE() - gain;
    if (cmd_brake < 0)
      cmd_brake = 0;
    cout << "SetBrakeStroke(" << cmd_brake << ")" << endl;
    hev->SetBrakeStroke(cmd_brake);    

    /* reset PID variables. */
    e_prev = 0;
    clear_diff();
  }
  else { // PID control
    int cmd_accel;

    // double check if cmd_velocity > current_velocity
    if (cmd_velocity <= current_velocity) {
      cout << "cmd_velocity is smaller than current_velocity!" << endl;
      return;
    }

    e = cmd_velocity - current_velocity;

    e_d = e - e_prev;
    
    accel_diff_sum += e;
    accel_diff_buffer.push(e);
    if (accel_diff_buffer.size() > _K_ACCEL_I_CYCLES) {
      double e_old = accel_diff_buffer.front();
      accel_diff_sum -= e_old;
      if (accel_diff_sum < 0) {
        accel_diff_sum = 0;
      }
      accel_diff_buffer.pop();
    }

    if (accel_diff_sum > _ACCEL_MAX_I) {
      e_i = _ACCEL_MAX_I;
    }
    else {
      e_i = accel_diff_sum;
    }

    cmd_accel = _K_ACCEL_P * e + _K_ACCEL_I * e_i + _K_ACCEL_D * e_d;
    if (cmd_accel > ACCEL_PEDAL_MAX) {
      cmd_accel = ACCEL_PEDAL_MAX;
    }
    else if (cmd_accel < 0) {
      cmd_accel = 0;
    }

    cout << "e = " << e << endl;
    cout << "e_i = " << e_i << endl;
    cout << "e_d = " << e_d << endl;
    cout << "SetDrvStroke(" << cmd_accel << ")" << endl;
    hev->SetDrvStroke(cmd_accel);

#if 1 /* log */
      ofstream ofs("/tmp/drv_accel.log", ios::app);
      ofs << cmd_velocity << " " 
      << current_velocity << " " 
      << e << " " 
      << e_i << " " 
      << e_d << " " 
      << cmd_accel << " " 
      << endl;
#endif

    e_prev = e;
  }
}

void _decelerate_control(double current_velocity,double cmd_velocity, HevCnt *hev)
{
  // decelerate but not stop
  // first try releasing accelerator
  // then try light braking
  double vel_diff = fabs(current_velocity) - fabs(cmd_velocity);
  cout << "vel diff = " << vel_diff << endl;

  double max_vel_diff = KmhToMs(10.0);
  double vel_diff_limit = KmhToMs(3.0);
  if (vel_diff > max_vel_diff) 
    vel_diff = max_vel_diff;
      
  if (estimate_accel > -0.05) {
    // not decellerating so release pedal to decel
    target_accel_level-=ACCEL_PEDAL_RELEASE_STEP;
    if (target_accel_level < 0) 
      target_accel_level = 0;
    int gain = (int)(((double)target_accel_level)/0.2*cycle_time);
    _accel(target_accel_level, gain, CURRENT_ACCEL_STROKE(), hev); 
    cout << "release pedal target = " << target_accel_level << endl;
  }
  cout << "actual pedal = " << _hev_state.drvInf.actualPedalStr << endl;

  // check if pedal nearly released, if so need to apply brakes to slow
  if (_hev_state.drvInf.actualPedalStr < 200  || vel_diff > vel_diff_limit) {
    // try to brake proportionally to difference in vel
    double w = vel_diff/max_vel_diff;
    int br = HEV_LIGHT_BRAKE + (int)(w*(((double)(HEV_MED2_BRAKE-HEV_LIGHT_BRAKE))));
		    
    // ensure br value is between light and med brake 
    if (br < HEV_LIGHT_BRAKE) 
      br = HEV_LIGHT_BRAKE;
    if (br > HEV_MED2_BRAKE) 
      br = HEV_MED2_BRAKE;

    // call brake
    cout << "calling proportional brake......... br = " << endl;
    int gain = (int)(((double)HEV_MED2_BRAKE)/0.5*cycle_time);
    _brake(br, gain, CURRENT_BRAKE_STROKE(), hev);
  }
}

#define _K_BRAKE_P 40.0
#define _K_BRAKE_I 10.0 //5.0
#define _K_BRAKE_D 10.0
#define _K_BRAKE_I_CYCLES 100
#define _BRAKE_MAX_I 200
#define _BRAKE_STROKE_DELTA_MAX 1000

void _brake_stroke_pid_control(double current_velocity, double cmd_velocity, HevCnt* hev)
{
  double e;
  static double e_prev = 0;
  double e_i;
  double e_d;

  // decelerate by releasing the accel pedal if pressed.
  if (_hev_state.drvInf.actualPedalStr > 0) {
    cout << "accel pressed, release" << endl;
    int gain = 400;
    int cmd_accel = CURRENT_ACCEL_STROKE() - gain;
    if (cmd_accel < 0)
      cmd_accel = 0;
    cout << "SetDrvStroke(" << cmd_accel << ")" << endl;
    hev->SetDrvStroke(cmd_accel);

    /* reset PID variables. */
    e_prev = 0;
    clear_diff();
  }
  else { // PID control
    int cmd_brake;

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

    cmd_brake = _K_BRAKE_P * e + _K_BRAKE_I * e_i + _K_BRAKE_D * e_d;
    if (cmd_brake > HEV_MAX_BRAKE) {
      cmd_brake = HEV_MAX_BRAKE;
    }
    else if (cmd_brake < 0) {
      cmd_brake = 0;
    }

    if (cmd_brake - CURRENT_BRAKE_STROKE() > _BRAKE_STROKE_DELTA_MAX) {
      cmd_brake = CURRENT_BRAKE_STROKE() + _BRAKE_STROKE_DELTA_MAX;
    }

    cout << "e = " << e << endl;
    cout << "e_i = " << e_i << endl;
    cout << "e_d = " << e_d << endl;
    cout << "SetBrakeStroke(" << cmd_brake << ")" << endl;
    hev->SetBrakeStroke(cmd_brake);

    e_prev = e;
  }
}

void _stopping_control(double current_velocity,double cmd_velocity, HevCnt *hev)
{
  // if using acc pedal release
  int gain = (int)(((double)1000)/0.2*cycle_time); 
  _accel(0, gain, CURRENT_ACCEL_STROKE(), hev); 

  // accel goes from pedal pos 1000 to 0 in 0.2s
  // should be faster than break application below
  target_accel_level = 0;
      
  // decelerate by using brake	
  if (cmd_velocity == 0.0 && current_velocity < 0.1) {
    // nearly at stop/at stop to stop -> apply full brake
    int gain = (int)(((double)HEV_MAX_BRAKE)/1.0*cycle_time);
    _brake(HEV_MAX_BRAKE, gain, CURRENT_BRAKE_STROKE(), hev);
  }
  else {
    // one second is approximately how fast full brakes applied in sharp stop
    int high_brake = HEV_MAX_BRAKE-500;
    int brake_target = HEV_MED_BRAKE;
    if (current_velocity > 16.0) {
      brake_target = HEV_MED_BRAKE + (int)((current_velocity - KmhToMs(16.0)/50.0)*((double)(high_brake-HEV_MED_BRAKE)));
      if (brake_target > high_brake)
        brake_target = high_brake;
    }
    int gain = (int)(((double)brake_target)/0.5*cycle_time);
    _brake(brake_target, gain, CURRENT_BRAKE_STROKE(), hev);
  }
}

void MainWindow::StrokeControl(double current_velocity, double cmd_velocity)
{
  static queue<double> vel_buffer;
  static uint vel_buffer_size = 10; 
  double old_velocity = 0.0;

  // do not call a control funtion in manual mode.
  if (IS_DRV_MODE_MANUAL()) {
    clear_diff();
    return;
  }

  // estimate current acceleration.
  vel_buffer.push(current_velocity);
  
  if (vel_buffer.size() > vel_buffer_size) {
    old_velocity = vel_buffer.front();
    vel_buffer.pop(); // remove old_velocity from the queue.
    estimate_accel = (current_velocity-old_velocity)/(cycle_time*vel_buffer_size);
  }

  cout << "estimate_accel: " << estimate_accel << endl; 

  if (fabs(cmd_velocity) >= current_velocity
      && fabs(cmd_velocity) > 0.0 
      && current_velocity <= SPEED_LIMIT) {
    cout << "accelerate: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
    //_accelerate_control(current_velocity, cmd_velocity, hev);
    _accel_stroke_pid_control(current_velocity, cmd_velocity, hev);
  } 
  else if (fabs(cmd_velocity) < current_velocity
           && fabs(cmd_velocity) > 0.0) {
    cout << "decelerate: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
    //_decelerate_control(current_velocity, cmd_velocity, hev); 
    _brake_stroke_pid_control(current_velocity, cmd_velocity, hev);
  }
  else if (cmd_velocity == 0.0 && current_velocity != 0.0) {
    cout << "stopping: current_velocity=" << current_velocity 
         << ", cmd_velocity=" << cmd_velocity << endl;
    if (current_velocity < 3.0) { // nearly stopping
      _stopping_control(current_velocity, 0, hev);
    }
    else {
      _brake_stroke_pid_control(current_velocity, 0, hev);
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
    hev->SetDrvVeloc((increase_velocity + vel_offset_inc) * 100);
    cout << "increase: " << "vel = " << increase_velocity  << endl; 
  }
  else {
    double decrease_velocity = current_velocity - vel_diff_dec;
    if (decrease_velocity > vel_offset_dec) {
      hev->SetDrvVeloc((decrease_velocity - vel_offset_dec) * 100);
    }
    else if (current_velocity > 0) {
      decrease_velocity = 0;
      hev->SetDrvVeloc(0);
    }
    cout << "decrease: " << "vel = " << decrease_velocity  << endl; 
  }
}

