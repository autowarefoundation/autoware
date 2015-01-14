//activate hev_base 
#include <iostream>
#include <queue>

#include "../utils/timer.h"
#include "../comms/shm_client.h"


using namespace std;
using namespace zmp::hev;

#define HEV_WHEEL_BASE 2.7 //Prius Tire Size
#define HANDLE_ANGLE_MAX 600 //ハンドルを最大まで回したときの角度 
#define WHEEL_ANGLE_MAX 31.28067 //フロントタイヤの最大角度
#define PERIOD 100  

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

void PrintState(); //display hev information
void PrintDrvInf();  //display drv information
void PrintStrInf(); //display str information
void PrintBrkInf(); //display brk information
void PrintMode(int mode); //display drv and str mode
void PrintServo(int servo); //display drv and str servo
void PrintShift(int shift); //display drv shift 

void SharedMemoryUpdate();//update Sharedmemory
vel_data_t GetCommand();//get command from shm
bool HevBaseUpdate(double * x, double * y, double *theta, double *tv, double * sv);//update odometry and current velocity
bool CheckHevMode(); //check HEV Mode
bool Control(double tv,double sv,void*);//function for autonomous driving


//Structure to store HEV State
typedef struct HevState {
  DrvInf drvInf;
  StrInf strInf;
  BrakeInf brkInf;
  long long int tstamp;
} HevState;

HevState _hev_state;


SHMClient _shared_memory; //shm Class
pthread_t __HevBaseThread;

double inline Mod(double x, double y)
{
  if (0 == y) return x;
  return (x - (y * floor(x/y)));
}


double inline WrapPI(double a)
{
  return (Mod(a + M_PI, 2*M_PI) - M_PI);
}


//convert km/h to m/s
static double KmhToMs(double v) {
  return (v*1000.0/(60.0*60.0));
}

//get current time
static double getTime() {
  //returns time in milliseconds 
  struct timeval current_time;
  struct timezone ttz;
  double t;
  // lock();
  gettimeofday(&current_time, &ttz);
  t = ((current_time.tv_sec * 1000.0) +
       (current_time.tv_usec) / 1000.0);
  // unlock();
  return t;
}

//set velocity command to 0
vel_data_t zero_cmd() {
  vel_data_t z;
  z.tv = 0.0;
  z.sv = 0.0;
  z.tstamp = TSTAMP();
  return z;
}


void MainWindow::HevBaseActivate()
{
  printf("\n");
  cout << "HevBase Activate" << endl;

  //initialize odometry
  pose_data_t zero_odom;
  cout << "initialize odom" << endl;
  memset(&zero_odom,0,sizeof(zero_odom));
  _shared_memory.writeOdomPose(zero_odom);

  //launch HevBase thread
  pthread_create(&__HevBaseThread,NULL,HevBaseThreadEntry,this);
 
}

//HEVの制御スレッド
void* MainWindow::HevBaseThreadEntry(void* arg)
{
  cout << "HevBase Thread Create!!!!!!!!!!!!" << endl;

  MainWindow* Main = (MainWindow*)arg;   //refer MainWindow class
  bool change_flag = false;  //HEV Mode Change flag
  bool hev_mode = false; //if program,true. if manual,false

  //Start HevBase
  while(1){

    Main->UpdateState(); //update hev drvinf strinf brkinf 
    SharedMemoryUpdate();  //update odom and current velocity steering in SHM
    hev_mode = CheckHevMode(); //check hev mode manual or program
   
    //judge if HEV mode is program
    if(hev_mode == true){ 
      
      if(change_flag == 0){
        char answer[1];
        cout << "Change Program Mode?(y) : ";
        cin >> answer; 
        
        if(strncmp(answer,"y",1) == 0){
          cout << "Change to Program Mode" << endl; 
          change_flag = true;
   
        } else{
          hev_mode = false;
          Main->sndDrvServoOFF();
          Main->sndStrServoOFF();
          Main->sndDrvModeM();
          Main->sndStrModeM();
          cout << "Fallback to Manual Mode" << endl;
          usleep(PERIOD*10000);
        }

        //loop back to while
        continue;
      }
      
      //reset odometry
      // pose_data_t zero_odom;
      // memset(&zero_odom, 0, sizeof(zero_odom));
      // _shared_memory.writeOdomPose(zero_odom);
      
      //Get velocity and steering command
      vel_data_t cmd = GetCommand();
      
      //prius control 
      Control(cmd.tv,cmd.sv,Main);
      
    }else{
      cout << "HEV: Manual Mode" << endl;
    }
    printf("\n\n");
    usleep(PERIOD*1000);
  }

  return NULL;
  
}


//Update HEV State

void MainWindow::UpdateState()
{
  _hev_state.tstamp = (long long int) (getTime());
  hev->GetDrvInf(&_hev_state.drvInf);
  hev->GetStrInf(&_hev_state.strInf);
  hev->GetBrakeInf(&_hev_state.brkInf);
}



//get velocity and steering command
vel_data_t GetCommand()
{
  //get velocity , steering command and timestamp
  vel_data_t cmd = _shared_memory.readControl();
  long long int cmd_time = TSTAMP();
      
  //check if new command
  if((cmd_time - cmd.tstamp) > 500){
    //old command
    cout << "command is old" <<endl;
    cmd = zero_cmd();
  }
  cout << "velocity command : " << cmd.tv << " " << cmd.sv << endl;
  return cmd;
}



void SharedMemoryUpdate()
{
 
  base_data_t data;
  memset(&data,0,sizeof(base_data_t));
  
  
  double x, y,theta,tv,sv;    
  HevBaseUpdate(&x,&y,&theta,&tv,&sv);

  // cout << "===============SharedMemoryUpdate===============" << endl;
  //printf("x\t\t y      \t theta      \t tv      \t sv\n");
  //printf("%lf \t %lf \t %lf \t %lf \t %lf\n\n",x,y,theta,tv,sv);
  
  data.odom_pose.x = x;
  data.odom_pose.y = y;
  data.odom_pose.Y = theta;
  
  data.vel_cur.tv = tv;
  data.vel_cur.sv = sv;
  
  //write to shm
  _shared_memory.writeBaseState(data);
}

bool CheckHevMode()
{
  
  int drv_mode = _hev_state.drvInf.mode; // 0x00 : manual ; 0x10 : program
  int drv_servo = _hev_state.drvInf.servo; // 0x00 : OFF 0x10 :ON
  int str_mode = _hev_state.strInf.mode; // 0x00 : manual ; 0x10 : program
  int str_servo = _hev_state.strInf.servo; // 0x00 : OFF 0x10 :ON

  // cout <<  "===============CheckHevMode===============" << endl;

  if(drv_mode == 0x10 && drv_servo == 0x10 && str_mode == 0x10 && str_servo == 0x10){
    return true;
  }else{
    return false;
  }

}

//use for HevBaseUpdate


static long long int old_tstamp= 0;
int hevbaseupdate_count = 0;

double cycle_time = 0;
bool HevBaseUpdate(double * x, double * y, double *theta, double *tv, double * sv)
{
  
  bool reverse = false;
  cout << "===============HevBaseUpdate :"  << hevbaseupdate_count <<  "==============="<<endl;
   
  double dt = 0;
  double str_angle = 0.0;
  double vel = 0.0;
  
  pose_data_t odometry = _shared_memory.readOdomPose();
  double odometry_x = odometry.x;
  double odometry_y = odometry.y;
  double odometry_theta = odometry.Y;
  
  if (old_tstamp != 0) {

    if (_hev_state.drvInf.actualShift == SHIFT_POS_R) {
      //	fprintf(stderr,"In REVERSE\n");
      reverse = true;
    }

    dt = (_hev_state.tstamp - old_tstamp)/1000.0;

    // drvInf.veloc [km/h*100] , ((veloc/100)*1000)/(60*60) m/s 
    // *** actually drvInf.veloc [km/h*100] , ((veloc)*1000)/(60*60) m/s 
    vel = _hev_state.drvInf.veloc*10 / 36.0;
    if (fabs(vel) < 0.03) vel = 0.0;

    if (reverse) vel*=-1.0;

    // strInf.angle [deg*10] , ((angle/10)/180)*pi radians
    // and steering ratio 2 (well really 20 but already dvide by 10)
    str_angle = (_hev_state.strInf.angle / 180.0 * M_PI 
                 /(HANDLE_ANGLE_MAX/WHEEL_ANGLE_MAX)); 
    //  fprintf(stderr,"steering angle = %.2f\n",str_angle);

    // calculate odom from state information
    odometry_theta = WrapPI(odometry_theta + dt*vel/HEV_WHEEL_BASE*tan(str_angle));
    odometry_x = odometry_x + dt*vel*cos(odometry_theta);
    odometry_y = odometry_y + dt*vel*sin(odometry_theta);
      
    cycle_time = dt;
  }
  old_tstamp = _hev_state.tstamp;
    
  

  *tv = vel;
  *sv =str_angle;
  *x = odometry_x;
  *y = odometry_y;
  *theta = odometry_theta;
  

  PrintState();
  hevbaseupdate_count++;
 
  return true;
}


void PrintState()
{
  printf("\nHev State:\n");
  PrintDrvInf();
  PrintStrInf();
  PrintBrkInf();
  printf("\n");

}

void PrintMode(int mode){
  
  switch(mode){
  case MODE_MANUAL:
    printf("MANUAL\t");
    break;
  case MODE_PROGRAM:
    printf("PROGRAM\t");
    break;
  default:
    printf("UNKNOWN\t");
    break;
  }
}


void PrintServo(int servo){
  
  switch(servo){
  case SERVO_TRUE:
    printf("ON\t");
    break;
  case SERVO_FALSE:
    printf("OFF\t");
    break;
  default:
    printf("UNKNOWN\t");
    break;
  }
}


void PrintShift(int shift){

  switch(shift){
  case SHIFT_POS_D:
    printf("D\t");
    break;
  case SHIFT_POS_R:
    printf("R\t");
    break;
  case SHIFT_POS_N:
    printf("N\t");
    break;
  case SHIFT_POS_B:
    printf("B\t");
    break;
  case SHIFT_POS_P:
    printf("P\t");
    break;
  default:
    printf("UNKNOWN\t");
    break;
  }
} 



void PrintDrvInf()
{
  int mode = _hev_state.drvInf.mode;
  int servo = _hev_state.drvInf.servo;
  int shift = _hev_state.drvInf.actualShift;

  printf("---: MODE\tSERVO\tSHIFT\tVELOC\t\n");

  printf("Drv: ");
  PrintMode(mode);
  PrintServo(servo);
  PrintShift(shift);
  printf("%.4f\t\n\n", _hev_state.drvInf.veloc);  
}

void PrintStrInf()
{
  int mode = _hev_state.strInf.mode;
  int servo = _hev_state.strInf.servo;

  printf("---: MODE\tSERVO\tANGLE\n"); 
  printf("Str: "); 
  PrintMode(mode);
  PrintServo(servo);

  printf("%.4f\t\n\n",_hev_state.strInf.angle);  
}

void PrintBrkInf() 
{
  bool press = _hev_state.brkInf.pressed;

  printf("---: PRESS\tACTUAL\tINPUT\n");
  printf("Brk: ");
  switch(press){
  case true:
    printf("ON \t");
    break;
  case false:
    printf("OFF\t");
    break;
  default:
    printf("unknown\t");
    break;
  }
  printf("%d\t%d\n",_hev_state.brkInf.actualPedalStr, _hev_state.brkInf.inputPedalStr);
}


static int old_accel_cmd = -1;
bool MainWindow::Accel(int target_accel, int gain) {

  fprintf(stderr,"calling _accel(%d %d)\n", target_accel, gain);
  int current_accel = _hev_state.drvInf.actualPedalStr;

  // for small increments of change in accel pedal,
  // prius sometimes does not apply 
  // assume  cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state


  int cmd_accel = current_accel;

  if (old_accel_cmd != -1) 
    cmd_accel = old_accel_cmd;

  fprintf(stderr,"old accel command = %d\n", old_accel_cmd);
  fprintf(stderr,"cmd_accel now %d\n", cmd_accel);
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
    fprintf(stderr,"accel: cur: %d cmd: %d target: %d\n", current_accel, cmd_accel, target_accel); 
  }

  fprintf(stderr,"setting accel pedal to %d\n", cmd_accel);

  if (cmd_accel >= 0 && cmd_accel < ACCEL_PEDAL_MAX) {
    fprintf(stderr,"calling SetPedalStroke\n");
    hev->SetDrvStroke(cmd_accel);
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


  int current_brake = _hev_state.brkInf.actualPedalStr;

  // for small increments of change in brake pedal,
  // prius sometimes does not apply brake
  // assume brake cmd is acheived
  // new cmd then is calculated directly from old cmd not relative
  // to actual state


  int cmd_brake = current_brake;

  if (old_brake_cmd != -1) 
    cmd_brake = old_brake_cmd;

  fprintf(stderr,"old brake command = %d\n", old_brake_cmd);
  fprintf(stderr,"cmd_brake now %d\n", cmd_brake);
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
    fprintf(stderr,"cur: %d cmd: %d target: %d\n", current_brake, cmd_brake, target_brake); 
  }
  
  fprintf(stderr,"setting brake pedal to %d\n", cmd_brake);

  fprintf(stderr,"calling SetBrakeStroke\n");
  hev->SetBrakeStroke(cmd_brake);

 
  old_brake_cmd = cmd_brake;

  if (target_brake == current_brake){ 
    old_brake_cmd = -1;
    return true;

  }
  return false;
}



static int target_accel_level = 0;

double estimate_accel = 0.0;

bool Control(double tv, double sv,void* p) 
{
  MainWindow* Main = (MainWindow*)p;

  queue<double> vel_buffer;
  static int vel_buffer_size = 10; 
 

  double old_velocity = 0.0;
  vel_data_t current = _shared_memory.readCurVel();

  double current_velocity = current.tv*3.6;
  double current_steering_angle = current.sv;
  
  
  double cmd_velocity = tv *3.6;
  double cmd_steering_angle = sv;
 
  // estimate current acceleration
  vel_buffer.push(fabs(current_velocity));
  
  if (vel_buffer.size() > vel_buffer_size) {
    old_velocity = vel_buffer.front();
    vel_buffer.pop();
    estimate_accel = (fabs(current_velocity)-old_velocity)/(cycle_time*vel_buffer_size);
    
  }
  cout << endl << "Current " << "tv : " << current_velocity << " sv : "<< current_steering_angle << endl; 
  cout << endl << "Command " << "tv : " << tv << " sv : "<< sv << endl; 
  cout << "Estimate Accel : " << estimate_accel << endl; 

  // TRY TO INCREASE STEERING
  //  sv +=0.1*sv;

  // if tv non zero then check if in drive gear first
  //--------------------------------------------------------------
  // if in neutral and get +'ve cmd vel
  //    - put brake on
  //    - change shift to drive
  // if in neutral and vel cmd 0
  //    - release brake if pressed
  
  //ギアがNの場合DかRに
  Main->ChangeShiftMode(cmd_velocity);
   
  //------------------------------------------------------
  // if shift in drive
  //     - if cmd vel > 0 && brake pressed
  //              - release brake
  //     - if cmd vel == 0
  //              - if cur_vel < 0.1, nearly stopped
  //                     - full brake too stop, wait, and shift to neutral
  //              - else - brake to HEV_MED_BRAKE
  //                       - decellerate until nearly stopped
  // now motion if in drift mode and shift is in drive
  
 
  
  if ( _hev_state.drvInf.actualShift == SHIFT_POS_D 
       //|| _hev_state.drvInf.actualShift == SHIFT_POS_R
       ) {
    fprintf(stderr,"In Drive or Reverse\n");
         
    // if accell 
    if (fabs(cmd_velocity) >= fabs(current_velocity) 
        && fabs(cmd_velocity) > 0.0 
        && fabs(current_velocity) <= KmhToMs(51.0) ) {

      //accelerate !!!!!!!!!!!!!!!!
      cout << "Acceleration" << endl;
      Main->AccelerateControl(current_velocity,cmd_velocity);
      
    }else if (fabs(cmd_velocity) < fabs(current_velocity) 
              && fabs(cmd_velocity) > 0.0) 
      {
        //decelerate!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        cout << "Deceleration" << endl;
        Main->DecelerateControl(current_velocity,cmd_velocity);
      }
    
    else if (cmd_velocity == 0.0) {
	
      //Stopping!!!!!!!!!!!
       
      Main->StoppingControl(current_velocity,cmd_velocity);
  
    } // if cmdvel < curvel
    
  } //if shift in drive
  else if( _hev_state.drvInf.actualShift == SHIFT_POS_N ){
    cout << "Shift value unknown" << endl;
  }
  else {
    fprintf(stderr,"Shift value unknown\n");
  }
  
  // set steering angle
  Main->SteeringControl(cmd_steering_angle);
      
  
  return true;
  
}

void MainWindow::SteeringControl(double cmd_steering_angle){
    
  int str = (int)(cmd_steering_angle/M_PI*180.0 * (HANDLE_ANGLE_MAX/WHEEL_ANGLE_MAX));    
  cout << "setting str angle to " << str << endl;
  if (abs(str) > 600) {
    cout << "steering angle too large : " <<  str << endl;
    if (str < 0) 
      str = -600;
    else str = 600;
  }
  else 
    hev->SetStrAngle(str*10);
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


void MainWindow::AccelerateControl(double current_velocity,double cmd_velocity)
{


  fprintf(stderr,"accelerate here\n");
  // acclerate by releasing brake if pressed
  if (_hev_state.brkInf.pressed == true) {
    fprintf(stderr,"brake pressed, release\n");
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


      
  fprintf(stderr,"reduce speed..........................................\n");
  // decelerate but not stop
  // first try releasing accelerator
  // then try light braking
  double vel_diff = fabs(current_velocity) - fabs(cmd_velocity);

  fprintf(stderr,"vel diff = %.2f\n", vel_diff);
  double max_vel_diff = KmhToMs(10.0);
  double vel_diff_limit = KmhToMs(3.0);
  if (vel_diff > max_vel_diff) 
    vel_diff = max_vel_diff;

      
      
  if (estimate_accel > -0.05) {
    // not decellerating  so release pedal to decel
	  
    target_accel_level-=ACCEL_PEDAL_RELEASE_STEP;
    if (target_accel_level < 0) target_accel_level = 0;
    Accel(target_accel_level, (int)(((double)target_accel_level)/0.2*cycle_time)); 
    fprintf(stderr,"release pedal target = %d\n", target_accel_level);
  }
  fprintf(stderr,"actual pedal = %d\n", _hev_state.drvInf.actualPedalStr);
  // check if pedal nearly released, if so need to apply brakes to slow
  if (_hev_state.drvInf.actualPedalStr < 200  || vel_diff > vel_diff_limit) {
      

    // try to brake proportionally to difference in vel
    double w = vel_diff/max_vel_diff;
    int br = HEV_LIGHT_BRAKE + (int)(w*(((double)(HEV_MED2_BRAKE-HEV_LIGHT_BRAKE))));
		    
    // ensure br value is between light and med brake 
    if (br < HEV_LIGHT_BRAKE) br = HEV_LIGHT_BRAKE;
    if (br > HEV_MED2_BRAKE) br = HEV_MED2_BRAKE;

    // call brake

    fprintf(stderr,"calling proportional brake.........br = %d\n",br);
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
