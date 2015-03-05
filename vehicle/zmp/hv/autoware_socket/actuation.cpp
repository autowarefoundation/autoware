#include "mainwindow.h"
#include "autoware_socket.h"

static int target_accel_level = 0;

void MainWindow::SetMode(int mode)
{
  switch (mode) {
  case CMD_MODE_MANUAL:
    cout << "switching to MANUAL" << endl;
    hev->SetDrvMode(MODE_MANUAL);
    usleep(100000);
    hev->SetStrMode(MODE_MANUAL);
    usleep(100000);
    hev->SetDrvServo(SERVO_FALSE);
    usleep(100000);
    hev->SetStrServo(SERVO_FALSE);
    break;
  case CMD_MODE_PROGRAM:
    cout << "switching to PROGRAM" << endl;
    hev->SetDrvMode(MODE_PROGRAM);
    usleep(100000);
    hev->SetStrMode(MODE_PROGRAM);
    usleep(100000);
    hev->SetDrvServo(SERVO_TRUE);
    usleep(100000);
    hev->SetStrServo(SERVO_TRUE);
    break;
  default:
    cout << "Unknown mode: " << mode << endl;
  }
}

void MainWindow::SetGear(int gear)
{
  switch (gear) {
  case CMD_GEAR_D:
    cout << "shifting to Gear D" << endl;
    hev->SetDrvStroke(0);
    hev->SetBrakeStroke(HEV_MAX_BRAKE);
    hev->SetDrvShiftMode(SHIFT_POS_D);
    break;
  case CMD_GEAR_R:
    cout << "shifting to Gear R" << endl;
    hev->SetDrvStroke(0);
    hev->SetBrakeStroke(HEV_MAX_BRAKE);
    hev->SetDrvShiftMode(SHIFT_POS_R);
    break;
  case CMD_GEAR_B:
    cout << "shifting to Gear R" << endl;
    hev->SetDrvStroke(0);
    hev->SetBrakeStroke(HEV_MAX_BRAKE);
    hev->SetDrvShiftMode(SHIFT_POS_B);
    break;
  case CMD_GEAR_N:
    cout << "shifting to Gear N" << endl;
    break;
  default:
    cout << "Unknown gear: " << gear << endl;    
  }
}

void MainWindow::UpdateState()
{
  _hev_state.tstamp = (long long int) (getTime());
  hev->GetDrvInf(&_hev_state.drvInf);
  hev->GetStrInf(&_hev_state.strInf);
  hev->GetBrakeInf(&_hev_state.brkInf);
}

bool MainWindow::Accel(int target_accel, int gain)
{
  static int old_accel = -1;
  int current_accel = _hev_state.drvInf.actualPedalStr;

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
    cout << "calling SetPedalStroke()" << endl;

    hev->SetDrvStroke(cmd_accel);
#ifdef DEBUG
    ndrv = cmd_accel;
#endif
  }

  old_accel = cmd_accel;

  if (target_accel == current_accel){ 
    old_accel = -1;
    return true;
  }

  return false;
}

bool MainWindow::Brake(int target_brake, int gain) 
{
  static int old_brake = -1;
  int current_brake = _hev_state.brkInf.actualPedalStr;

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
  
  cout << "setting brake pedal to " << cmd_brake << endl;

  cout << "calling SetBrakeStroke()" << endl;

  hev->SetBrakeStroke(cmd_brake);
#ifdef DEBUG
  nbrk = cmd_brake;
#endif
 
  old_brake = cmd_brake;

  if (target_brake == current_brake){ 
    old_brake = -1;
    return true;
  }

  return false;
}

void MainWindow::SteeringControl(double current_steering_angle, double cmd_steering_angle)
{
  double delta = cmd_steering_angle - current_steering_angle;
  double str = cmd_steering_angle;

  cout << "cmd_steering_angle = " << cmd_steering_angle << endl;

  if (delta > NORMAL_STEERING_THRETHOLD) {
    str = current_steering_angle + NORMAL_STEERING_THRETHOLD;
    cout << "steering angle rounded to: " <<  str << endl;
  } else if (delta < -NORMAL_STEERING_THRETHOLD) {
    str = current_steering_angle - NORMAL_STEERING_THRETHOLD;
    cout << "steering angle rounded to: " <<  str << endl;
  }

  if (str > STEERING_ANGLE_LIMIT) {
    cout << "steering angle too large: " <<  str << endl;
    str = STEERING_ANGLE_LIMIT;
  }
  else if (str < -STEERING_ANGLE_LIMIT) {
    cout << "steering angle too large: " <<  str << endl;
    str = -STEERING_ANGLE_LIMIT;  
  }

#if 0
  // set the steering angle to HEV.
  // note that we need *10 to input the steering.
  // a unit of steering for "set" is 0.1 degree 
  // while that for "get" is 1 degree.
  hev->SetStrAngle(str*10);
#else
  // quick hack to smoothen steering.
  // if the target steering increase is 10, it is going to be:
  // 1(1+1), 3(1+2), 6(3+3), 8(6+2), 9(8+1)...

  int delta_tmp = 0; // temporary delta
  int str_tmp = current_steering_angle; // temporary steering
  int inc = 0; // increment value
  int second_half = 0; // just a flag
  if (delta > 0) {
    for (int i = 0; i < cmd_rx_interval / STEERING_INTERNAL_PERIOD; i++) {
      if (delta_tmp < delta / 2) {
        inc += STEERING_ANGLE_INC_INC;
      } else {
        inc += -STEERING_ANGLE_INC_INC;
        if (second_half == 0) {
          inc += STEERING_ANGLE_INC_INC;
        }
        second_half = 1;
      }

      if (inc < 0) {
        hev->SetStrAngle(str*10); // set the target steering in the end
        break;
      }

      delta_tmp += inc;
      str_tmp = current_steering_angle + inc;
      //str_tmp = str_tmp + inc; // for debug at lab.
      if (str_tmp < str) {
        hev->SetStrAngle(str_tmp*10);
      } else {
        hev->SetStrAngle(str*10);
        break;
      }
      usleep(STEERING_INTERNAL_PERIOD*1000);
    }
  } else {
    for (int i = 0; i < cmd_rx_interval / STEERING_INTERNAL_PERIOD; i++) {
      if (delta_tmp > delta / 2) {
        inc += -STEERING_ANGLE_INC_INC;
      } else {
        inc += STEERING_ANGLE_INC_INC;
        if (second_half == 0) {
          inc += -STEERING_ANGLE_INC_INC;
        }
        second_half = 1;
      }

      if (inc > 0) {
        hev->SetStrAngle(str*10); // set the target steering in the end
        cout << "str = " << str_tmp << endl;
        break;
      }

      delta_tmp += inc;
      str_tmp = current_steering_angle + inc;
      //str_tmp = str_tmp + inc; // for debug at lab.
      if (str_tmp > str) {
        hev->SetStrAngle(str_tmp*10);
      } else {
        hev->SetStrAngle(str*10);
        break;
      }
      usleep(STEERING_INTERNAL_PERIOD*1000);
    }
  }
#endif

#ifdef DEBUG
  nstr = str;
#endif
}

void MainWindow::AccelerateControl(double current_velocity,double cmd_velocity)
{
  // acclerate by releasing the brake pedal if pressed.
  if (_hev_state.brkInf.pressed == true) {
    cout << "brake pressed, release" << endl;
    Brake(0, (int)(((double)HEV_MAX_BRAKE)/0.5*cycle_time));
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
    Accel(acc_start_target, (int)(((double)acc_start_target)/1.0*cycle_time)); 
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

    Accel(target_accel_level, (int)(((double) target_accel_level)/0.5*cycle_time)); 
          
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
    Accel(target_accel_level, (int)(((double)target_accel_level)/0.2*cycle_time)); 
    cout << "release pedal target = " << target_accel_level << endl;
  }
  cout << "actual pedal = " << _hev_state.drvInf.actualPedalStr << endl;

  // check if pedal nearly released, if so need to apply brakes to slow
  if (_drv_state.drvInf.actualPedalStr < 200  || vel_diff > vel_diff_limit) {
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
    Brake(br, (int)(((double)HEV_MED2_BRAKE)/0.5*cycle_time));
  }
}

void MainWindow::StoppingControl(double current_velocity,double cmd_velocity)
{
  // if using acc pedal release
  Accel(0, (int)(((double)1000)/0.2*cycle_time)); 

  // accel goes from pedal pos 1000 to 0 in 0.2s
  // should be faster than break application below
  target_accel_level = 0;
      
  // decelerate by using brake	
  if (cmd_velocity == 0.0 && fabs(current_velocity) < 0.1) {
    // nearly at stop/at stop to stop -> apply full brake
    Brake(HEV_MAX_BRAKE, (int)(((double)HEV_MAX_BRAKE)/1.0*cycle_time));
  }
  else {
    // one second is approximately how fast full brakes applied in sharp stop
    int high_brake = HEV_MAX_BRAKE-500;
    int brake_target = HEV_MED_BRAKE;
    if (fabs(current_velocity) > KmhToMs(16.0)) {
      brake_target = HEV_MED_BRAKE + (int)((fabs(current_velocity) - KmhToMs(16.0)/50.0)*((double)(high_brake-HEV_MED_BRAKE)));
      if (brake_target > high_brake) brake_target = high_brake;
    }
    Brake(brake_target, (int)(((double)brake_target)/0.5*cycle_time));
  }
}

void MainWindow::ChangeShiftMode(double cmd_velocity)
{
  if (_hev_state.drvInf.actualShift == SHIFT_POS_N) {
    printf("In Neutral\n");
    
    if (fabs(cmd_velocity) > 0.0) {
      
      printf("In Neutral -> put brake on\n"); 
      if (Brake(HEV_MAX_BRAKE, (int)(((double)HEV_MAX_BRAKE)/0.5*cycle_time))) {
        // brake fully on
        printf("Brake fully on, set shift to drive\n");
        if (cmd_velocity > 0.0){ 
          // hev->SetDrvShiftMode(SHIFT_POS_D);
          sndDrvShiftD(); //HEVの関数
        }
        else if (cmd_velocity < 0.0) {
          printf("-- WARNING: changing shift to REVERSE --\n");
          //hev->SetDrvShiftMode(SHIFT_POS_R);
          sndDrvShiftR(); //HEVの関数
        } 
      }
      
    }else { 
      // cmd vel == 0.0
      if (_hev_state.brkInf.pressed == true) {
        printf("in neutral, 0 vel cmd, brake pressed, release\n");
        Brake(0, (int)(((double)HEV_MAX_BRAKE)/0.5*cycle_time));
        
      } // if brake pressed
      
    }
    target_accel_level = 0;
  }
  printf("SHIFT = %d\n",_hev_state.drvInf.actualShift);
}
