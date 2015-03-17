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

void MainWindow::SetMode(int mode)
{
  switch (mode) {
  case CMD_MODE_MANUAL:
    cout << "switching to MANUAL" << endl;
    hev->SetDrvMode(MODE_MANUAL);
    usleep(200000);
    hev->SetStrMode(MODE_MANUAL);
    usleep(200000);
    hev->SetDrvServo(SERVO_FALSE);
    usleep(200000);
    hev->SetStrServo(SERVO_FALSE);
    usleep(200000);
    break;
  case CMD_MODE_PROGRAM:
    cout << "switching to PROGRAM" << endl;
    hev->SetDrvMode(MODE_PROGRAM);
    usleep(100000);
    //hev->SetDrvCMode(CONT_MODE_STROKE); // stroke mode not velocity
    //usleep(200000);
    hev->SetDrvServo(SERVO_TRUE);
    usleep(200000);
    hev->SetStrMode(MODE_PROGRAM);
    usleep(200000);
    hev->SetStrCMode(CONT_MODE_ANGLE); // angle mode not torque
    usleep(200000);
    hev->SetStrServo(SERVO_TRUE);
    usleep(200000);
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }
}

void MainWindow::SetGear(int gear)
{
  double current_velocity = _hev_state.drvInf.veloc; // km/h

  // double check if the velocity is zero or not,
  // though SetGear() should not be called when driving.
  if (current_velocity != 0) {
    return;
  }

  hev->SetDrvStroke(0);
  hev->SetBrakeStroke(HEV_MAX_BRAKE);

  switch (gear) {
  case CMD_GEAR_D:
    cout << "shifting to Gear D" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_D);
    break;
  case CMD_GEAR_R:
    cout << "shifting to Gear R" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_R);
    break;
  case CMD_GEAR_B:
    cout << "shifting to Gear B" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_B);
    break;
  case CMD_GEAR_N:
    cout << "shifting to Gear N" << endl;
    hev->SetDrvShiftMode(SHIFT_POS_N);
    break;
  default:
    cout << "Unknown gear: " << gear << endl;    
  }
}

bool _accel(int target_accel, int gain, int current_accel, HevCnt *hev)
{
  static int old_accel = -1;

  // for small increments of change in accel pedal,
  // prius sometimes does not apply 
  // assume  cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state

  int cmd_accel = current_accel;

  if (old_accel != -1) 
    cmd_accel = old_accel;

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
    old_accel = -1;
    return true;
  }

  return false;
}

bool _brake(int target_brake, int gain, int current_brake, HevCnt *hev) 
{
  static int old_brake = -1;

  // for small increments of change in brake pedal,
  // prius sometimes does not apply brake
  // assume brake cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state

  int cmd_brake = current_brake;

  if (old_brake != -1) 
    cmd_brake = old_brake;

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

//#define _VERY_NAIVE
#define _DELTA_LIMIT 90
#define _DELTA_LIMIT_STRICT 10
#define _STEERING_ANGLE_INC_INC 5
#define _FAST_STEERING_THRESHOLD 270
void _str_angle_naive(double current_steering_angle, double cmd_steering_angle, HevCnt* hev)
{
  double delta = cmd_steering_angle - current_steering_angle;
  double target_steering_angle = cmd_steering_angle;

  if (delta > _DELTA_LIMIT) {
    target_steering_angle = current_steering_angle + _DELTA_LIMIT;
    cout << "steering angle rounded to: " <<  target_steering_angle << endl;
  }
  else if (delta < -_DELTA_LIMIT) {
    target_steering_angle = current_steering_angle - _DELTA_LIMIT;
    cout << "steering angle rounded to: " <<  target_steering_angle << endl;
  }

  if (target_steering_angle > STEERING_ANGLE_LIMIT) {
    target_steering_angle = STEERING_ANGLE_LIMIT;
    cout << "steering angle limited to: " <<  target_steering_angle << endl;
  }
  else if (target_steering_angle < -STEERING_ANGLE_LIMIT) {
    target_steering_angle = -STEERING_ANGLE_LIMIT;  
    cout << "steering angle limited to: " <<  target_steering_angle << endl;
  }

#ifdef _VERY_NAIVE
  if (delta > _DELTA_LIMIT_STRICT) {
    target_steering_angle = current_steering_angle + _DELTA_LIMIT_STRICT;
    cout << "steering angle forced to: " <<  target_steering_angle << endl;
  }
  else if (delta < -_DELTA_LIMIT_STRICT) {
    str = current_steering_angle - _DELTA_LIMIT_STRICT;
    cout << "steering angle forced to: " <<  str << endl;
  }

  // set the steering angle to HEV.
  // note that we need *10 to input the steering.
  // a unit of steering for "set" is 0.1 degree 
  // while that for "get" is 1 degree.
  hev->SetStrAngle(target_steering_angle*10);
#else
  // quick hack to smoothen steering (naive P control).
  // if the target steering increase is 10, it is going to be:
  // 1(1+1), 3(1+2), 6(3+3), 8(6+2), 9(8+1)...

  int delta_tmp = 0; // temporary delta
  int target_tmp = current_steering_angle; // temporary steering
  int inc = 0; // increment value
  int inc_inc = 0;
  int second_half = 0; // just a flag

  if (fabs(delta) < _FAST_STEERING_THRESHOLD) {
    inc_inc = _STEERING_ANGLE_INC_INC; // slow rotation
  } 
  else {
    inc_inc = _STEERING_ANGLE_INC_INC * 2; // fast rotation
  }

  if ((delta > 0 && delta < inc_inc) || (delta < 0 && delta > -inc_inc)) {
    hev->SetStrAngle(target_steering_angle*10);
  }
  else if (delta > 0) {
    for (int i = 0; i < cmd_rx_interval/STEERING_INTERNAL_PERIOD - 1; i++) {
      if (delta_tmp < delta / 2) {
        inc += inc_inc;
      }
      else {
        inc -= inc_inc;
        if (second_half == 0) {
          inc += inc_inc;
        }
        second_half = 1;
      }

      if (inc < 0) {
        break;
      }

      delta_tmp += inc;
      target_tmp = current_steering_angle + inc;

      if (target_tmp < target_steering_angle) {
        hev->SetStrAngle(target_tmp*10);
      }
      else {
        hev->SetStrAngle(target_steering_angle*10);
        break;
      }
      usleep(STEERING_INTERNAL_PERIOD*1000);
    }
  }
  else {
    for (int i = 0; i < cmd_rx_interval/STEERING_INTERNAL_PERIOD - 1; i++) {
      if (delta_tmp > delta / 2) {
        inc -= inc_inc;
      }
      else {
        inc += inc_inc;
        if (second_half == 0) {
          inc -= inc_inc;
        }
        second_half = 1;
      }

      if (inc > 0) {
        break;
      }

      delta_tmp += inc;
      target_tmp = current_steering_angle + inc;

      if (target_tmp > target_steering_angle) {
        hev->SetStrAngle(target_tmp*10);
      }
      else {
        hev->SetStrAngle(target_steering_angle*10);
        break;
      }
      usleep(STEERING_INTERNAL_PERIOD*1000);
    }
  }
#endif

  cout << "target_steering_angle = " << target_steering_angle << endl;

  // output log.
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << target_steering_angle << " " 
      << current_steering_angle << " " 
      << cmd_steering_angle << endl;
}

// P control
#define _K_STEERING_SPEED 8 // danger (deg/s)/deg
// PI control
#define _K_STEERING_SPEED_I 0.5 // danger!!!! (deg/s)/(deg*0.1s)  
#define _STEERING_MAX_SUM 100 //deg*0.1s for I control
///////////////////////////////////////////////////////////
// _K_STEERING_SPEED_I*_STEERING_MAX_SUM is max offset angle
// small _K_STEERING_SPEED_I and large _STEERING_MAX_SUM 
//   -> slow fit and stable
// large _K_STEERING_SPEED_I and small _STEERING_MAX_SUM 
//   -> fast fit and unstable
///////////////////////////////////////////////////////////

// steering speed limitation
#define _STEERING_MAXANGVEL 300 // deg/s, limit for steering angular velocity
#define _STEERING_MAXACC 500 // deg/s^2, limit for steering angular acceleration
#define _STEERING_MAX_OFFSET 30 // deg, limit for steering angle (between current angle and target angle )
#define STEERING_PERIOD 0.1 // s, control period
void _str_angle_p_control(double current_steering_angle, double cmd_steering_angle, HevCnt* hev)
{
  static double prev_steering_angle = 0;
  static double prev_steering_angvel = 0;

  // prev_steering_angle = current_steering_angle; // override

  // P_Control
  double steering_diff = cmd_steering_angle - prev_steering_angle; 
  double target_steering_angvel = steering_diff * _K_STEERING_SPEED;
  
  // clip
  if (target_steering_angvel > _STEERING_MAXANGVEL) {
    target_steering_angvel = _STEERING_MAXANGVEL;
  }
  if(target_steering_angvel <-_STEERING_MAXANGVEL) {
    target_steering_angvel = -_STEERING_MAXANGVEL;
  }

  // acceleration limit
  double steering_angvel;
  steering_angvel = prev_steering_angvel;
  if (steering_angvel < target_steering_angvel) { //accel
    if (steering_angvel + _STEERING_MAXACC*STEERING_PERIOD < target_steering_angvel) {
      steering_angvel += _STEERING_MAXACC*STEERING_PERIOD;
    }
    else {
      steering_angvel = target_steering_angvel;
    }
  }
  else {
    if (steering_angvel - _STEERING_MAXACC*STEERING_PERIOD > target_steering_angvel) {
      steering_angvel -= _STEERING_MAXACC*STEERING_PERIOD;
    }
    else {
      steering_angvel = target_steering_angvel;
    }
  }

  // set target angle 
  double steering_angle = prev_steering_angle + steering_angvel*STEERING_PERIOD*1;

  // clip
  if (steering_angle > current_steering_angle + _STEERING_MAX_OFFSET) {
    steering_angle = current_steering_angle + _STEERING_MAX_OFFSET;
  }
  if (steering_angle < current_steering_angle - _STEERING_MAX_OFFSET) {
    steering_angle = current_steering_angle - _STEERING_MAX_OFFSET;
  }

  cout << "steering_angle = " << steering_angle << endl;
  cout << "steering_angvel = " << steering_angvel << endl;
  hev->SetStrAngle(steering_angle*10);

  prev_steering_angle = steering_angle;
  prev_steering_angvel = steering_angvel;

  // output log.
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << steering_angle << " " 
      << steering_angvel << " " 
      << current_steering_angle << " " 
      << cmd_steering_angle << endl;
}

void _str_angle_pi_control(double current_steering_angle, double cmd_steering_angle, HevCnt* hev)
{
  static double prev_steering_angle = 0;
  static double prev_steering_angvel = 0;
  static double steering_diff_sum = 0;

  prev_steering_angle = current_steering_angle; // override
  
  // PI_Control
  double steering_diff = cmd_steering_angle - prev_steering_angle; 
  steering_diff_sum += steering_diff;

  if (steering_diff_sum > _STEERING_MAX_SUM) {
    steering_diff_sum = _STEERING_MAX_SUM;
  }
  if(steering_diff_sum <-_STEERING_MAX_SUM) {
    steering_diff_sum = _STEERING_MAX_SUM;
  }

  double target_steering_angvel = steering_diff*_K_STEERING_SPEED + steering_diff_sum*_K_STEERING_SPEED_I;
  
  // clip
  if (target_steering_angvel > _STEERING_MAXANGVEL) {
    target_steering_angvel = _STEERING_MAXANGVEL;
  }
  if (target_steering_angvel < -_STEERING_MAXANGVEL) {
    target_steering_angvel = -_STEERING_MAXANGVEL;
  }

  // acceleration limit
  double steering_angvel;
  steering_angvel = prev_steering_angvel;
  if (steering_angvel < target_steering_angvel) { //accel
    if (steering_angvel + _STEERING_MAXACC*STEERING_PERIOD < target_steering_angvel) {
      steering_angvel += _STEERING_MAXACC*STEERING_PERIOD;
    }
    else {
      steering_angvel = target_steering_angvel;
    }
  }
  else{
    if (steering_angvel - _STEERING_MAXACC*STEERING_PERIOD > target_steering_angvel) {
      steering_angvel -= _STEERING_MAXACC*STEERING_PERIOD;
    }
    else {
      steering_angvel = target_steering_angvel;
    }
  }

  // set target angle 
  double steering_angle = prev_steering_angle + steering_angvel*STEERING_PERIOD;

  // clip
  if (steering_angle > current_steering_angle + _STEERING_MAX_OFFSET) {
    steering_angle = current_steering_angle + _STEERING_MAX_OFFSET;
  }
  if (steering_angle < current_steering_angle - _STEERING_MAX_OFFSET) {
    steering_angle = current_steering_angle - _STEERING_MAX_OFFSET;
  }

  cout << "steering_angle = " << steering_angle << endl;
  cout << "steering_angvel = " << steering_angvel << endl;
  hev->SetStrAngle(steering_angle*10);

  prev_steering_angle = steering_angle;
  prev_steering_angvel = steering_angvel;

  // output log.
  ofstream ofs("/tmp/steering.log", ios::app);
  ofs << steering_angle << " " 
      << steering_angvel << " " 
      << current_steering_angle << " " 
      << cmd_steering_angle << endl;
}

void MainWindow::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  //_str_angle_naive(current_steering_angle, cmd_steering_angle, hev);
  //_str_angle_p_control(current_steering_angle, cmd_steering_angle, hev);
  _str_angle_pi_control(current_steering_angle, cmd_steering_angle, hev);
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
      if (brake_target > high_brake) brake_target = high_brake;
    }
    int gain = (int)(((double)brake_target)/0.5*cycle_time);
    _brake(brake_target, gain, CURRENT_BRAKE_STROKE(), hev);
  }
}


