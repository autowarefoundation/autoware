#ifndef __AUTOWARE_SOCKET_H__
#define __AUTOWARE_SOCKET_H__

#include <string>
#include "mainwindow.h"

using namespace zmp::hev;

#define NO_TSTAMP 0

#define CMD_MODE_MANUAL 0 
#define CMD_MODE_PROGRAM 1
#define CMD_GEAR_D 1
#define CMD_GEAR_R 2
#define CMD_GEAR_B 3
#define CMD_GEAR_N 4

#define WHEEL_BASE 2.7 // tire-to-tire size of Prius.
#define STEERING_ANGLE_MAX 666 // max angle of steering
#define STEERING_ANGLE_LIMIT 600 // could be STEERING_ANGLE_MAX but...
#define WHEEL_ANGLE_MAX 31.28067 // max angle of front tires.
#define WHEEL_TO_STEERING (STEERING_ANGLE_MAX/WHEEL_ANGLE_MAX)
#define ANGLE_ERROR 0.025 // for tuning when LIDAR is not aligned
#define MILLISECOND 1000  
#define SPEED_LIMIT 60.0 // km/h
#define STEERING_ANGLE_INC_INC 5 // degree
#define STEERING_INTERNAL_PERIOD 20 // ms (10ms is too fast for HEV)
#define NORMAL_STEERING_THRETHOLD 60 // for tuning (degree)

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
#define ACCEL_LIMIT 8.0 // km/h

#define HEV_LIGHT_BRAKE 400 //brake pedal
#define HEV_MED_BRAKE 1000
#define HEV_MED2_BRAKE 2000
#define HEV_MAX_BRAKE 4096 //max brake pedal value 

typedef struct pose_data_t {
  double x;
  double y;
  double z;
  double R;
  double P;
  double Y;
  long long int tstamp;
} pose_data_t;

typedef struct vel_data_t {
  double tv;
  double sv;
  long long int tstamp;
} vel_data_t;

typedef struct vel_cmd_data_t {
  vel_data_t vel;
  int mode;
} vel_cmd_data_t;

typedef struct base_data_t {
  pose_data_t odom_pose;
  vel_data_t vel_cur;
  unsigned char batt_voltage;
  bool motor_state;
  long long int tstamp;
} base_data_t;

typedef struct state_data_t {
  long long int update_tstamp;
  long long int control_tstamp;

  // state data reading from base
  base_data_t base;

  // control info
  pose_data_t est_pose;
  vel_data_t vel_cmd;
  int control_mode;

} state_data_t;

typedef struct lrf_config_t {
  int N;
  double fov; // field of view radians
  double res;
  double start_a;
  double stop_a;
} lrf_config_t;

typedef int lrf_data_t; // range data type

typedef struct _CMDDATA {
    vel_data_t vel;
    int mode;
    int gear;
    int accel;
    int steer;
    int brake;
} CMDDATA;

// Structure to store HEV State
typedef struct _DrvState {
  DrvInf drvInf;
  StrInf strInf;
  BrakeInf brkInf;
} DrvState;

// Structure to store HEV State
typedef struct HevState {
  DrvInf drvInf;
  StrInf strInf;
  BrakeInf brkInf;
  long long int tstamp;
} HevState;

extern DrvState _drv_state;
extern HevState _hev_state;

extern int can_tx_interval; // ms
extern int cmd_rx_interval; // ms
extern std::string ros_ip_address;
extern std::string candata;
extern int drvmode;
extern CMDDATA cmddata;
extern double estimate_accel;
extern double cycle_time;
extern int ndrv;
extern int pdrv;
extern int nbrk;
extern int pbrk;
extern double nstr;
extern double pstr;

int CheckDrvMode();
bool Control(vel_data_t vel,vel_data_t &current,void* p);

// convert km/h to m/s
static inline double KmhToMs(double v) {
  return (v*1000.0/(60.0*60.0));
}

// get current time
static inline long long int getTime() {
  //returns time in milliseconds 
  struct timeval current_time;
  struct timezone ttz;
  double t;
  // lock();
  gettimeofday(&current_time, &ttz);
  t = ((current_time.tv_sec * 1000.0) +
       (current_time.tv_usec) / 1000.0);
  // unlock();
  return static_cast<long long int>(t);
}



#endif //__AUTOWARE_SOCKET_H__

