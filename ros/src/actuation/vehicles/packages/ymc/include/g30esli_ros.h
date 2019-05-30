/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#ifndef G30ESLI_ROS_H
#define G30ESLI_ROS_H

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <ds4_msgs/DS4.h>

#include "cansend.h"
#include "g30esli.h"

enum struct MODE
{
  AUTO,
  JOYSTICK,
};

class G30esliROS
{
private:
  struct Command
  {
    bool received = false;
    ros::Time time;
    ymc::G30esli::Command command;
  };

  struct Status
  {
    bool received = false;
    ros::Time time;
    ymc::G30esli::Status status;
  };

  ymc::G30esli g30esli_;

  Command commands_[2];
  Status status_;

  unsigned char alive_;
  geometry_msgs::TwistStamped current_twist_;
  autoware_msgs::VehicleStatus vehicle_status_;

  bool reset_command_;
  const double stop_keep_secs_, reset_keep_secs_, disable_reset_secs_;

public:
  G30esliROS();
  ~G30esliROS();

  bool openDevice(const std::string& device);
  void receiveStatus(const double& steering_offset_deg);
  void sendCommand(const MODE& mode);

  void updateAutoCommand(const autoware_msgs::VehicleCmd& msg, const bool& engage, const double& steering_offset_deg,
                         const double& brake_threshold);
  void updateJoystickCommand(const ds4_msgs::DS4& msg, const bool& engage, const double& steering_offset_deg);
  void updateAliveCounter();

  bool checkOverride();
  void checkRestart(const MODE& mode);
  bool checkTimeout(const MODE& mode, const double& timeout);
  bool emergencyStop(const MODE& mode);

  std::string dumpDebug(const MODE& mode);

  autoware_msgs::VehicleStatus& getVehicleStatus()
  {
    return vehicle_status_;
  }

  geometry_msgs::TwistStamped& getCurrentTwist()
  {
    return current_twist_;
  }

  double getBatteryCharge()
  {
    return status_.status.battery.charge;
  }
};

#endif
