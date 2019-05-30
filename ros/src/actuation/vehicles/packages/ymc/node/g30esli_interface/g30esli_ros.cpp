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

#include "g30esli_ros.h"

G30esliROS::G30esliROS()
  : alive_(0), reset_command_(false), stop_keep_secs_(2.0), reset_keep_secs_(0.6), disable_reset_secs_(3.0)
{
}

G30esliROS::~G30esliROS()
{
}

bool G30esliROS::openDevice(const std::string& device)
{
  return g30esli_.openDevice(device);
}

void G30esliROS::updateAutoCommand(const autoware_msgs::VehicleCmd& msg, const bool& engage,
                                   const double& steering_offset_deg, const double& brake_threshold)
{
  Command& cmd = commands_[(int)MODE::AUTO];

  cmd.time = ros::Time::now();
  cmd.received = true;

  // speed
  double speed_kmph = msg.ctrl_cmd.linear_velocity * 3.6;  // [m/s] -> [km/h]
  cmd.command.speed = (engage && !reset_command_) ? speed_kmph : 0.0;

  // steer
  double steering_angle_deg = msg.ctrl_cmd.steering_angle / M_PI * 180.0;  // [rad] -> [deg]
  cmd.command.steer = -(steering_angle_deg + steering_offset_deg);

  // mode
  cmd.command.mode = engage ? G30ESLI_MODE_AUTO : G30ESLI_MODE_MANUAL;

  // brake
  if (msg.emergency == 1)
  {
    cmd.command.brake = G30ESLI_BRAKE_SEMIEMG;
  }
  else if (cmd.command.speed < brake_threshold)
  {
    cmd.command.brake = G30ESLI_BRAKE_SMOOTH;
  }
  else
  {
    cmd.command.brake = G30ESLI_BRAKE_NONE;
  }

  // shift
  if (msg.gear == 1)
  {
    cmd.command.shift = G30ESLI_SHIFT_DRIVE;
  }
  else if (msg.gear == 2)
  {
    cmd.command.shift = G30ESLI_SHIFT_REVERSE;
  }
  else if (msg.gear == 4)
  {
    cmd.command.shift = G30ESLI_SHIFT_NEUTRAL;
  }

  // flasher
  if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 0)
  {
    cmd.command.flasher = G30ESLI_FLASHER_CLEAR;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 0)
  {
    cmd.command.flasher = G30ESLI_FLASHER_LEFT;
  }
  else if (msg.lamp_cmd.l == 0 && msg.lamp_cmd.r == 1)
  {
    cmd.command.flasher = G30ESLI_FLASHER_RIGHT;
  }
  else if (msg.lamp_cmd.l == 1 && msg.lamp_cmd.r == 1)
  {
    cmd.command.flasher = G30ESLI_FLASHER_HAZARD;
  }
}

void G30esliROS::updateJoystickCommand(const ds4_msgs::DS4& msg, const bool& engage, const double& steering_offset_deg)
{
  Command& cmd = commands_[(int)MODE::JOYSTICK];

  cmd.time = ros::Time::now();
  cmd.received = true;

  // check connection
  if (!msg.connected)
  {
    cmd.command.speed = 0.0;
    cmd.command.brake = G30ESLI_BRAKE_SEMIEMG;
    return;
  }

  // speed
  bool send_speed = msg.cross && engage && !reset_command_;
  cmd.command.speed = send_speed ? 16.0 * msg.r2 + 3.0 : 0.0;

  // steer
  cmd.command.steer = -((17.0 * msg.l2 + 20.0) * msg.left_y + steering_offset_deg);

  cmd.command.mode = engage ? G30ESLI_MODE_AUTO : G30ESLI_MODE_MANUAL;

  // brake
  if (!(msg.square || msg.circle || msg.triangle))
  {
    cmd.command.brake = G30ESLI_BRAKE_NONE;
  }
  else
  {
    cmd.command.brake = msg.square ? G30ESLI_BRAKE_SMOOTH : cmd.command.brake;
    cmd.command.brake = msg.circle ? G30ESLI_BRAKE_SEMIEMG : cmd.command.brake;
    cmd.command.brake = msg.triangle ? G30ESLI_BRAKE_EMERGENCY : cmd.command.brake;
  }

  // shift
  if (!(msg.r1 || msg.l1))
  {
    cmd.command.shift = G30ESLI_SHIFT_DRIVE;
  }
  else
  {
    cmd.command.shift = msg.r1 ? G30ESLI_SHIFT_REVERSE : cmd.command.shift;
    cmd.command.shift = msg.l1 ? G30ESLI_SHIFT_NEUTRAL : cmd.command.shift;
  }

  // flasher
  if (!(msg.up || msg.right || msg.left || msg.down))
  {
    cmd.command.flasher = G30ESLI_FLASHER_NONE;
  }
  else
  {
    cmd.command.flasher = msg.right ? G30ESLI_FLASHER_RIGHT : cmd.command.flasher;
    cmd.command.flasher = msg.left ? G30ESLI_FLASHER_LEFT : cmd.command.flasher;
    cmd.command.flasher = msg.down ? G30ESLI_FLASHER_HAZARD : cmd.command.flasher;
    cmd.command.flasher = msg.up ? G30ESLI_FLASHER_CLEAR : cmd.command.flasher;
  }
}

void G30esliROS::receiveStatus(const double& steering_offset_deg)
{
  g30esli_.readStatus(status_.status);

  ros::Time now = ros::Time::now();

  // update twist
  double lv = status_.status.speed.actual / 3.6;                                    // [km/h] -> [m/s]
  double th = (-status_.status.steer.actual - steering_offset_deg) * M_PI / 180.0;  // [deg] -> [rad]
  double az = std::tan(th) * lv / G30ESLI_WHEEL_BASE;                               // [rad] -> [rad/s]
  current_twist_.header.frame_id = "base_link";
  current_twist_.header.stamp = now;
  current_twist_.twist.linear.x = lv;
  current_twist_.twist.angular.z = az;

  // update vehicle status
  vehicle_status_.header = current_twist_.header;

  // drive/steeringmode
  if (status_.status.mode == G30ESLI_MODE_MANUAL)
  {
    vehicle_status_.drivemode = autoware_msgs::VehicleStatus::MODE_MANUAL;
    vehicle_status_.steeringmode = autoware_msgs::VehicleStatus::MODE_MANUAL;
  }
  else if (status_.status.mode == G30ESLI_MODE_AUTO)
  {
    vehicle_status_.drivemode = autoware_msgs::VehicleStatus::MODE_AUTO;
    vehicle_status_.steeringmode = autoware_msgs::VehicleStatus::MODE_AUTO;
  }

  // gearshift
  if (status_.status.shift == G30ESLI_SHIFT_DRIVE)
  {
    vehicle_status_.gearshift = 1;
  }
  else if (status_.status.shift == G30ESLI_SHIFT_REVERSE)
  {
    vehicle_status_.gearshift = 2;
  }
  else if (status_.status.shift == G30ESLI_SHIFT_NEUTRAL)
  {
    vehicle_status_.gearshift = 4;
  }

  // speed
  vehicle_status_.speed = status_.status.speed.actual;  // [kmph]

  // drivepedal
  vehicle_status_.drivepedal = status_.status.override.accel;  // [-]

  // brakepedal
  vehicle_status_.brakepedal = status_.status.override.brake;  // [-]

  // angle
  vehicle_status_.angle = -status_.status.steer.actual - steering_offset_deg;  // [deg]

  // lamp
  if (status_.status.override.flasher == G30ESLI_FLASHER_NONE)
  {
    vehicle_status_.lamp = 0;
  }
  else if (status_.status.override.flasher == G30ESLI_FLASHER_RIGHT)
  {
    vehicle_status_.lamp = autoware_msgs::VehicleStatus::LAMP_RIGHT;
  }
  else if (status_.status.override.flasher == G30ESLI_FLASHER_LEFT)
  {
    vehicle_status_.lamp = autoware_msgs::VehicleStatus::LAMP_LEFT;
  }
  else if (status_.status.override.flasher == G30ESLI_FLASHER_HAZARD)
  {
    vehicle_status_.lamp = autoware_msgs::VehicleStatus::LAMP_HAZARD;
  }

  // light
  vehicle_status_.light = 0;  // not used
}

bool G30esliROS::checkOverride()
{
  return (status_.status.override.accel == 1 || status_.status.override.brake == 1);
}

bool G30esliROS::checkTimeout(const MODE& mode, const double& timeout)
{
  Command& cmd = commands_[(int)mode];
  double dtms = (ros::Time::now() - cmd.time).toSec() * 1000;
  bool timeouted = (dtms > timeout);
  return (cmd.received && timeouted);
}

void G30esliROS::checkRestart(const MODE& mode)
{
  static bool initialize = false;
  static double stop_keep_dt = 0.0;
  static ros::Time prev_time(0), start_reset_time(0), end_reset_time(0);

  ros::Time curr_time = ros::Time::now();

  if (!initialize)
  {
    prev_time = curr_time;
    initialize = true;
    return;
  }

  Command& cmd = commands_[(int)mode];
  bool is_positive_command = (cmd.command.speed > 0.0);
  bool is_stopped = (vehicle_status_.speed < 0.01);

  if (!reset_command_)
  {
    if ((curr_time - end_reset_time).toSec() > disable_reset_secs_)
    {
      if (is_positive_command && is_stopped)
      {
        stop_keep_dt += (curr_time - prev_time).toSec();
      }
      else
      {
        stop_keep_dt = 0.0;
      }
      if (stop_keep_dt > stop_keep_secs_)
      {
        reset_command_ = true;
        start_reset_time = curr_time;
        stop_keep_dt = 0.0;
      }
    }
  }
  else
  {
    double reset_keep_dt = (curr_time - start_reset_time).toSec();
    if (reset_keep_dt > reset_keep_secs_)
    {
      reset_command_ = false;
      end_reset_time = curr_time;
    }
  }

  prev_time = curr_time;
}

bool G30esliROS::emergencyStop(const MODE& mode)
{
  Command& cmd = commands_[(int)mode];
  cmd.command.speed = 0.0;
  cmd.command.brake = G30ESLI_BRAKE_SEMIEMG;
}

void G30esliROS::sendCommand(const MODE& mode)
{
  Command& cmd = commands_[(int)mode];
  g30esli_.sendCommand(cmd.command);
}

void G30esliROS::updateAliveCounter()
{
  alive_++;
  commands_[(int)MODE::AUTO].command.alive = alive_;
  commands_[(int)MODE::JOYSTICK].command.alive = alive_;
}

std::string G30esliROS::dumpDebug(const MODE& mode)
{
  Command& cmd = commands_[(int)mode];
  std::stringstream ss;
  ss << "Status[" << (int)cmd.command.alive << "]\n" << ymc::G30esli::dumpStatus(status_.status);
  ss << "\nCommand[" << (int)cmd.command.alive << "]\n" << ymc::G30esli::dumpCommand(cmd.command);
  return ss.str();
}
