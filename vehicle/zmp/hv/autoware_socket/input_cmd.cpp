#include "mainwindow.h"
#include "data.h"

#define HEV_WHEEL_BASE 2.7 // tire size of Prius.
#define STEERING_ANGLE_MAX 666 // max angle of steering
#define STEERING_ANGLE_LIMIT 600 // could be STEERING_ANGLE_MAX but...
#define WHEEL_ANGLE_MAX 31.28067 // max angle of front tires.
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define STEERING_ADDITIONAL_WEIGHT 1.0 // for tuning
#define TYPICAL_STEERING_THRETHOLD 10 // for tuning
#define ANGLE_ERROR 0.025 // for tuning when LIDAR is not aligned
#define MILLISECOND 1000  

#define MODE_MANUAL 0x00 //HEV Manual Mode = 0x00
#define MODE_PROGRAM 0x10 //HEV Program Mode = 0x10

#define SERVO_TRUE 0x10 //HEV Servo ON = 0x10
#define SERVO_FALSE 0x00 //HEV Servo OFF = 0x00

#define ACCEL_PEDAL_SET_START 100 //initial press value for accel pedal
#define ACCEL_PEDAL_SET_START_FAST 1000 //initial press value for accel pedal if cmd_vel is higer 
#define ACCEL_PEDAL_STEP 5 //increase press value for accel pedal 
#define ACCEL_PEDAL_STEP_BIG 5
#define ACCEL_PEDAL_RELEASE_STEP 100 //release value for accel pedal
#define ACCEL_PEDAL_MAX 300 

#define HEV_LIGHT_BRAKE 400 //brake pedal
#define HEV_MED_BRAKE 1000
#define HEV_MED2_BRAKE 2000
#define HEV_MAX_BRAKE 4096 //max brake pedal value 

static int target_accel_level = 0;
static int old_accel_cmd = -1;

void MainWindow::SetDrvMode(int mode){

  if(mode == DRVMODE_PROGRAM){
    hev->SetDrvMode(MODE_PROGRAM);
    hev->SetStrMode(MODE_PROGRAM);
    hev->SetDrvServo(SERVO_TRUE);
    hev->SetStrServo(SERVO_TRUE);
  }else{
    hev->SetDrvMode(MODE_MANUAL);
    hev->SetStrMode(MODE_MANUAL);
    hev->SetDrvServo(SERVO_FALSE);
    hev->SetStrServo(SERVO_FALSE);
  }

}


bool MainWindow::Accel(int target_accel, int gain) {

  fprintf(stdout,"calling _accel(%d %d)\n", target_accel, gain);
  //fprintf(stderr,"calling _accel(%d %d)\n", target_accel, gain);
  int current_accel = _drv_state.drvInf.actualPedalStr;

  // for small increments of change in accel pedal,
  // prius sometimes does not apply 
  // assume  cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state


  int cmd_accel = current_accel;

  if (old_accel_cmd != -1) 
    cmd_accel = old_accel_cmd;

  fprintf(stdout,"old accel command = %d\n", old_accel_cmd);
  //fprintf(stderr,"old accel command = %d\n", old_accel_cmd);
  fprintf(stdout,"cmd_accel now %d\n", cmd_accel);
  //fprintf(stderr,"cmd_accel now %d\n", cmd_accel);
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
    fprintf(stdout,"accel: cur: %d cmd: %d target: %d\n", current_accel, cmd_accel, target_accel); 
    //fprintf(stderr,"accel: cur: %d cmd: %d target: %d\n", current_accel, cmd_accel, target_accel); 
  }

  fprintf(stdout,"setting accel pedal to %d\n", cmd_accel);
  //fprintf(stderr,"setting accel pedal to %d\n", cmd_accel);

  if (cmd_accel >= 0 && cmd_accel < ACCEL_PEDAL_MAX) {
    fprintf(stdout,"calling SetPedalStroke\n");
    //fprintf(stderr,"calling SetPedalStroke\n");

    hev->SetDrvStroke(cmd_accel);
#ifdef DEBUG
    ndrv = cmd_accel;
#endif
  }

  old_accel_cmd = cmd_accel;

  if (target_accel == current_accel){ 
    old_accel_cmd = -1;
    return true;

  }
  return false;
}


static int old_brake_cmd = -1;
bool MainWindow::Brake(int target_brake, int gain) {


  int current_brake = _drv_state.brkInf.actualPedalStr;

  // for small increments of change in brake pedal,
  // prius sometimes does not apply brake
  // assume brake cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state


  int cmd_brake = current_brake;

  if (old_brake_cmd != -1) 
    cmd_brake = old_brake_cmd;

  fprintf(stdout,"old brake command = %d\n", old_brake_cmd);
  //fprintf(stderr,"old brake command = %d\n", old_brake_cmd);
  fprintf(stdout,"cmd_brake now %d\n", cmd_brake);
  //fprintf(stderr,"cmd_brake now %d\n", cmd_brake);
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
    fprintf(stdout,"cur: %d cmd: %d target: %d\n", current_brake, cmd_brake, target_brake); 
    //fprintf(stderr,"cur: %d cmd: %d target: %d\n", current_brake, cmd_brake, target_brake); 
  }
  
  fprintf(stdout,"setting brake pedal to %d\n", cmd_brake);
  //fprintf(stderr,"setting brake pedal to %d\n", cmd_brake);

  fprintf(stdout,"calling SetBrakeStroke\n");
  //fprintf(stderr,"calling SetBrakeStroke\n");

  hev->SetBrakeStroke(cmd_brake);
#ifdef DEBUG
  nbrk = cmd_brake;
#endif
 
  old_brake_cmd = cmd_brake;

  if (target_brake == current_brake){ 
    old_brake_cmd = -1;
    return true;

  }
  return false;
}

void MainWindow::SteeringControl(double cmd_wheel_angle){
  
  // correct the angle error that often comes from the LIDAR position.
  cmd_wheel_angle += ANGLE_ERROR;

  // transform the wheel angle to the steering angle.
  int wheel = (int)((cmd_wheel_angle / M_PI) * 180.0);
  int str = (wheel * WHEEL_TO_STEERING) * STEERING_ADDITIONAL_WEIGHT;

  cout << "wheel = " << wheel << endl;
  cout << "str = "<< str << endl;

  static double prev_str = str;

  if (str - prev_str > TYPICAL_STEERING_THRETHOLD) {
    str = prev_str + TYPICAL_STEERING_THRETHOLD;
    cout << "steering angle rounded: str = " <<  str << endl;
  } else if (str - prev_str < -TYPICAL_STEERING_THRETHOLD) {
    str = prev_str - TYPICAL_STEERING_THRETHOLD;
    cout << "steering angle rounded: str = " <<  str << endl;
  }

  if (str > STEERING_ANGLE_LIMIT) {
    cout << "steering angle too large : " <<  str << endl;
    str = STEERING_ANGLE_LIMIT;
  }
  else if (str < -STEERING_ANGLE_LIMIT) {
    cout << "steering angle too large : " <<  str << endl;
    str = -STEERING_ANGLE_LIMIT;  
  }

  // set the steering angle to HEV.
  // note that we need *10 to input the steering.
  // a unit of steering for "set" is 0.1 degree 
  // while that for "get" is 1 degree.
  hev->SetStrAngle(str*10);

#ifdef DEBUG
  nstr = str;
#endif

  // save the current steering.
  // FIXME: we should use the actual steering value.
  prev_str = str;
}

void MainWindow::ChangeShiftMode(double cmd_velocity)
{


  if (_drv_state.drvInf.actualShift == SHIFT_POS_N) {
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
      if (_drv_state.brkInf.pressed == true) {
        printf("in neutral, 0 vel cmd, brake pressed, release\n");
        Brake(0, (int)(((double)HEV_MAX_BRAKE)/0.5*cycle_time));
        
      } // if brake pressed
      
    }
    target_accel_level = 0;
  }
  printf("SHIFT = %d\n",_drv_state.drvInf.actualShift);
}


void MainWindow::AccelerateControl(double current_velocity,double cmd_velocity)
{


  fprintf(stdout,"accelerate here\n");
  //fprintf(stderr,"accelerate here\n");
  // acclerate by releasing brake if pressed
  if (_drv_state.brkInf.pressed == true) {
    fprintf(stdout,"brake pressed, release\n");
    //fprintf(stderr,"brake pressed, release\n");
    Brake(0, (int)(((double)HEV_MAX_BRAKE)/0.5*cycle_time));
    
  } // end if brake pressed
  
  // accelrate using accelrator pedal
  double acc_vel_threshold = KmhToMs(8.0);
  int acc_start_target = ACCEL_PEDAL_SET_START;
        
  // desired vel is high, up first acceleration
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
          
  } // else accel slowly

        
}


void MainWindow::DecelerateControl(double current_velocity,double cmd_velocity)
{


      
  fprintf(stdout,"reduce speed..........................................\n");
  //fprintf(stderr,"reduce speed..........................................\n");
  // decelerate but not stop
  // first try releasing accelerator
  // then try light braking
  double vel_diff = fabs(current_velocity) - fabs(cmd_velocity);

  fprintf(stdout,"vel diff = %.2f\n", vel_diff);
  //fprintf(stderr,"vel diff = %.2f\n", vel_diff);
  double max_vel_diff = KmhToMs(10.0);
  double vel_diff_limit = KmhToMs(3.0);
  if (vel_diff > max_vel_diff) 
    vel_diff = max_vel_diff;

      
      
  if (estimate_accel > -0.05) {
    // not decellerating  so release pedal to decel
	  
    target_accel_level-=ACCEL_PEDAL_RELEASE_STEP;
    if (target_accel_level < 0) target_accel_level = 0;
    Accel(target_accel_level, (int)(((double)target_accel_level)/0.2*cycle_time)); 
    fprintf(stdout,"release pedal target = %d\n", target_accel_level);
    //    fprintf(stderr,"release pedal target = %d\n", target_accel_level);
  }
  fprintf(stdout,"actual pedal = %d\n", _drv_state.drvInf.actualPedalStr);
  //fprintf(stderr,"actual pedal = %d\n", _drv_state.drvInf.actualPedalStr);
  // check if pedal nearly released, if so need to apply brakes to slow
  if (_drv_state.drvInf.actualPedalStr < 200  || vel_diff > vel_diff_limit) {
      

    // try to brake proportionally to difference in vel
    double w = vel_diff/max_vel_diff;
    int br = HEV_LIGHT_BRAKE + (int)(w*(((double)(HEV_MED2_BRAKE-HEV_LIGHT_BRAKE))));
		    
    // ensure br value is between light and med brake 
    if (br < HEV_LIGHT_BRAKE) br = HEV_LIGHT_BRAKE;
    if (br > HEV_MED2_BRAKE) br = HEV_MED2_BRAKE;

    // call brake

    fprintf(stdout,"calling proportional brake.........br = %d\n",br);
    //fprintf(stderr,"calling proportional brake.........br = %d\n",br);
    Brake(br, (int)(((double)HEV_MED2_BRAKE)/0.5*cycle_time));

  }
 
}

void MainWindow::StoppingControl(double current_velocity,double cmd_velocity)
{

  printf("stopping function is here\n");

  // if using acc pedal release
  Accel(0, (int)(((double)1000)/0.2*cycle_time)); 
  // accel goes from pedal pos 1000 to 0 in 0.2s
  // should be faster than break application below
  target_accel_level = 0;
      
      
  // decelerate by using brake	
  if (cmd_velocity == 0.0 && fabs(current_velocity) < 0.1) {
    // nearly at stop/at stop to stop -> apply full brake
    if (Brake(HEV_MAX_BRAKE, (int)(((double)HEV_MAX_BRAKE)/1.0*cycle_time))) {
      // car nearly zero vel with max brake - wait and change to neutral
	    
      //samejima change 
      // simon - uncommented to test vel mode
      //shift to neutral
      sndDrvShiftN(); //HEVの関数
      //hev->SetDrvShiftMode(SHIFT_POS_N);
      
    }
	
  } // if nearly at stop
  else {
    // one second is approximately how fast full brakes applied in sharp stop
    int high_brake = HEV_MAX_BRAKE-500;
    int brake_target = HEV_MED_BRAKE;
    if (fabs(current_velocity) > KmhToMs(16.0)) {
      brake_target = HEV_MED_BRAKE + (int)((fabs(current_velocity) - KmhToMs(16.0)/50.0)*((double)(high_brake-HEV_MED_BRAKE)));
      if (brake_target > high_brake) brake_target = high_brake;
      
    }
    Brake(brake_target, (int)(((double)brake_target)/0.5*cycle_time));
     
  } // else decelerate*/
}

