
#ifndef __SHM_CLIENT_H__
#define __SHM_CLIENT_H__

#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/types.h>
#include "../common/shm_common.h"

  
class SHMClient {
 public:
  SHMClient();
  ~SHMClient();

  state_data_t readState();
  pose_data_t readOdomPose();
  pose_data_t readEstPose();
  vel_data_t readCurVel();

  int writeEstPose(pose_data_t bpose);
  int writeOdomPose(pose_data_t pose);
  int writeBaseState(base_data_t data);
  int writeControl(int mode, vel_data_t control);
  int writeCurVel(vel_data_t vel);
  vel_data_t readControl();
  vel_cmd_data_t readVelCmd();
  int readControlMode() { return _data->control_mode; }
private:
  
  int _id;
  key_t _key;
  struct state_data_t * _data;	

};



#endif
