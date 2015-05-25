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

static int target_accel_level = 0;
static double accel_diff_sum = 0;
static double brake_diff_sum = 0;

void MainWindow::SetDrvMode(int mode)
{
  switch (mode) {
  case CMD_MODE_MANUAL:
    cout << "Switching to MANUAL (Accel/Brake)" << endl;
    hev->SetDrvMode(MODE_MANUAL);
    usleep(200000);
    hev->SetDrvServo(SERVO_FALSE);
    usleep(200000);
    break;
  case CMD_MODE_PROGRAM:
    cout << "Switching to PROGRAM (Accel/Brake)" << endl;
    hev->SetDrvMode(MODE_PROGRAM);
    usleep(200000);
    hev->SetDrvCMode(CONT_MODE_STROKE);
    //hev->SetDrvCMode(CONT_MODE_VELOCITY);
    usleep(200000);
    hev->SetDrvServo(SERVO_TRUE);
    usleep(200000);
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }

  accel_diff_sum = 0;
  brake_diff_sum = 0;
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
  usleep(200000);
  hev->SetBrakeStroke(HEV_MAX_BRAKE);
  usleep(200000);

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
  usleep(200000);
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

void MainWindow::VelocityControl(double cmd_velocity)
{
  hev->SetDrvVeloc(cmd_velocity*100);
}

void MainWindow::AccelerateControl(double current_velocity,double cmd_velocity)
{
  // acclerate by releasing the brake pedal if pressed.
  if (_hev_state.brkInf.pressed == true) {
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

void MainWindow::DecelerateControl(double current_velocity,double cmd_velocity)
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

void MainWindow::StoppingControl(double current_velocity,double cmd_velocity)
{
  // if using acc pedal release
  int gain = (int)(((double)1000)/0.2*cycle_time); 
  _accel(0, gain, CURRENT_ACCEL_STROKE(), hev); 

  // accel goes from pedal pos 1000 to 0 in 0.2s
  // should be faster than break application below
  target_accel_level = 0;
      
  // decelerate by using brake	
  if (cmd_velocity == 0.0 && fabs(current_velocity) < 0.1) {
    // nearly at stop/at stop to stop -> apply full brake
    int gain = (int)(((double)HEV_MAX_BRAKE)/1.0*cycle_time);
    _brake(HEV_MAX_BRAKE, gain, CURRENT_BRAKE_STROKE(), hev);
  }
  else {
    // one second is approximately how fast full brakes applied in sharp stop
    int high_brake = HEV_MAX_BRAKE-500;
    int brake_target = HEV_MED_BRAKE;
    if (fabs(current_velocity) > KmhToMs(16.0)) {
      brake_target = HEV_MED_BRAKE + (int)((fabs(current_velocity) - KmhToMs(16.0)/50.0)*((double)(high_brake-HEV_MED_BRAKE)));
      if (brake_target > high_brake)
        brake_target = high_brake;
    }
    int gain = (int)(((double)brake_target)/0.5*cycle_time);
    _brake(brake_target, gain, CURRENT_BRAKE_STROKE(), hev);
  }
}
