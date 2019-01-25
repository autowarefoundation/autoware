/*
 * Copyright 2015 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mainwindow.h"
#include "autoware_socket.h"

#include <sys/shm.h>
#include <sys/types.h>
#include <sys/ipc.h>

struct struct_PID_controller *shm_ptr;

// these parameters can be configured later.
int can_tx_interval = 10; // ms
int cmd_rx_interval = 100; // ms
std::string ros_ip_address = "192.168.1.101";

// vehicle state.
vehicle_state_t vstate;

bool MainWindow::ConfigSocket(void)
{
  std::ifstream ifs("./config");
  std::string str;
  if (ifs.fail()) {
    return false;
  }

  if (getline(ifs,str)) {
    can_tx_interval = atoi(str.c_str());
    cout << "CAN Interval = " << can_tx_interval << " ms" << endl;
  } else {
    return false;
  }

  if (getline(ifs, str)) {
    cmd_rx_interval = atoi(str.c_str());
    cout << "CMD Interval = " << cmd_rx_interval << " ms" << endl;
  } else {
    return false;
  }

  if (getline(ifs,str)) {
    ros_ip_address = str;
    cout << "ROS IP Address = " << ros_ip_address << endl;
  } else {
    return false;
  }

  // key generation
  key_t shm_key = ftok(SHM_SEED_PATH.c_str(), 1);

  // open shared memory
  int shm_id = shmget(shm_key, sizeof(struct_PID_controller), 0444); // read only

  // get address of shared memory
  shm_ptr = (struct struct_PID_controller *)shmat(shm_id, NULL, 0);

  return true;
}

void MainWindow::UpdateState(void)
{
  ZMP_UPDATE_STATE();
  vstate.accel_stroke = ZMP_ACCEL_STROKE();
  vstate.brake_stroke = ZMP_BRAKE_STROKE();
  vstate.steering_torque = ZMP_STEERING_TORQUE();
  vstate.steering_angle = ZMP_STEERING_ANGLE();
  vstate.velocity = ZMP_VELOCITY();
  vstate.tstamp = (long long int) (getTime());
}

void MainWindow::ClearCntDiag(void)
{
  ZMP_CLEAR_CNT_DIAG();
}
