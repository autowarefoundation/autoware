#ifndef __SHM_COMMON_H__
#define __SHM_COMMON_H__

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/types.h>

#pragma pack(push, 4)

#define SHM_CONTROL_MODE_MANUAL 10
#define SHM_CONTROL_MODE_AUTO 9
#define SHM_CONTROL_MODE_AUTO_REQUEST 8
#define SHM_CONTROL_MODE_IDLE 0

#define NO_TSTAMP 0

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

typedef struct base_data_t{
  
  pose_data_t odom_pose;
  vel_data_t vel_cur;
  unsigned char batt_voltage;
  bool motor_state;
  long long int tstamp;
} base_data_t;

typedef struct state_data_t{
  
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

#define SHM_STATE_SZ     (sizeof(state_data_t))
#define SHM_P2OS_SZ      (sizeof(p2os_data_t))
#define SHM_STATE_KEY   (0x8888)
#define SHM_LRF_CONFIG_KEY (0x9999)
#define SHM_LRF_DATA_KEY     (0xffff)

#pragma pack(pop)

#endif
