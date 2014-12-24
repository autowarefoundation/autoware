
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <sys/shm.h>
#include <string.h>
#include "shm_client.h"

#include "timer.h"


SHMClient::SHMClient()
{	
     
  _key = SHM_STATE_KEY;
  
  /* Create the segment. */
  if ((_id = shmget(_key, SHM_STATE_SZ, 0666)) < 0) {
    perror("shmget");
    exit(1);
  }
       
  /* attach the segment to data space. */
  _data = (struct state_data_t *)shmat(_id, (void *)0, 0);
  if (_data == (void *)-1) {
    perror("shmat");
    exit(1);
  }
  fprintf(stderr,"SHMClient constructor returning\n");
}

SHMClient::~SHMClient()
{
  int ret = shmdt(_data);
  if (ret < 0) {
    perror("shmdt");
  }
}

//read cmd velocity and steering angle
//velocity : km/h
//steering angle : rad/s
vel_data_t SHMClient::readControl()
{
  vel_data_t cmd;
  memcpy(&cmd, &(_data->vel_cmd), sizeof(vel_data_t));
  // fprintf(stderr,"SHMClient::readControl() tstmap = %lld\n", cmd.tstamp);
  return cmd;
}

vel_cmd_data_t SHMClient::readVelCmd()
{
  vel_cmd_data_t cmd;
  memcpy(&cmd.vel, &(_data->vel_cmd), sizeof(vel_data_t));
  memcpy(&cmd.mode, &(_data->control_mode), sizeof(int));
  
  // fprintf(stderr,"SHMClient::readControl() tstmap = %lld\n", cmd.tstamp);
  return cmd;
}

state_data_t SHMClient::readState()
{
  state_data_t data;
  memcpy(&data, _data, SHM_STATE_SZ);
  return data;
}  

//read odomerty
//x,y,z : m/s
//R,P,Y : rad/s
pose_data_t SHMClient::readOdomPose()
{
  pose_data_t pose;
  memcpy(&pose, &(_data->base.odom_pose), sizeof(pose_data_t));
  return pose;
}  
pose_data_t SHMClient::readEstPose()
{
  pose_data_t pose;
  
  memcpy(&pose, &(_data->est_pose), sizeof(pose_data_t));
  return pose;
}  

//read current velocity and steering angle
//velocity : m/s
//steering angle : rad/s
vel_data_t SHMClient::readCurVel()
{
  vel_data_t vel;
  memcpy(&vel, &(_data->base.vel_cur), sizeof(vel_data_t));
  return  vel;
}

//write odomerty
//x,y,z : m/s
//R,P,Y : rad/s
int SHMClient::writeOdomPose(pose_data_t pose)
{
  // if (pose.tstamp == NO_TSTAMP) 
  pose.tstamp = TSTAMP();
  memcpy(&(_data->base.odom_pose), &(pose), sizeof(pose_data_t));
}
int SHMClient::writeEstPose(pose_data_t pose)
{
  //if (pose.tstamp == NO_TSTAMP) 
  pose.tstamp = TSTAMP();
  memcpy(&(_data->est_pose), &(pose), sizeof(pose_data_t));
}
int SHMClient::writeBaseState(base_data_t data)
{

  data.tstamp = TSTAMP();
  
  memcpy(&(_data->base), &(data), sizeof(base_data_t));
}

//write current velocity and steering angle
//velocity : m/s
//steering angle : rad/s
int SHMClient::writeCurVel(vel_data_t vel) 
{
 
  vel.tstamp = TSTAMP();
  memcpy(&(_data->base.vel_cur), &vel, sizeof(vel_data_t));
}

//read cmd velocity and steering angle
//velocity : km/h
//steering angle : rad/s
int SHMClient::writeControl(int mode, vel_data_t control) 
{
 
  control.tstamp = TSTAMP();
  //  fprintf(stderr,"SHMClient::writeControl() tstamp = %lld\n", control.tstamp);
  // if control mode not in automatic mode refuse auto requests
  if (mode == SHM_CONTROL_MODE_AUTO_REQUEST &&
      _data->control_mode != SHM_CONTROL_MODE_AUTO) {
    fprintf(stderr,"Requesting auto control when not allowed\n");
    return 0;
  }

  // if manual or auto control mode modify control mode setting
  if (mode != SHM_CONTROL_MODE_AUTO_REQUEST) 
    _data->control_mode = mode;

  // finally write vel command to shared memory
  //fprintf(stderr,"writing control mode to shared memory: %d %.2f %.2f\n", mode, control.tv, control.sv);
  memcpy(&(_data->vel_cmd), &control, sizeof(vel_data_t)); 
  return true;
}

