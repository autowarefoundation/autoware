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

#include <cmath>
#include <thread>

#include <ros/ros.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/VehicleStatus.h>
#include <ds4_msgs/DS4.h>

#include "cansend.h"
#include "g30esli.h"

#include "g30esli_interface_util.h"

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

// ros subscriber
ros::Subscriber vehicle_cmd_sub_;
ros::Subscriber ds4_sub_;

// ros publisher
ros::Publisher current_twist_pub_;
ros::Publisher vehicle_status_pub_;

// ros param
std::string device_;
bool use_ds4_;
double steering_offset_deg_;
double command_timeout_;

// variables
bool joy_;
bool engage_;
bool terminate_thread_;
geometry_msgs::TwistStamped current_twist_;
autoware_msgs::VehicleStatus vehicle_status_;

// ymc g30esli driver
ymc::G30esli g30esli_;

// ymc status/command
Status status_;
Command command_auto_, command_joy_;

// generate command by autoware
void vehicleCmdCallback(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  command_auto_.time = ros::Time::now();
  command_auto_.received = true;

  // speed
  double speed_kmph = msg->ctrl_cmd.linear_velocity * 3.6;  // [m/s] -> [km/h]
  command_auto_.command.speed = engage_ ? speed_kmph : 0.0;

  // steer
  double steering_angle_deg = msg->ctrl_cmd.steering_angle / M_PI * 180.0;  // [rad] -> [deg]
  command_auto_.command.steer = -(steering_angle_deg + steering_offset_deg_);

  // mode
  command_auto_.command.mode = engage_ ? G30ESLI_MODE_AUTO : G30ESLI_MODE_MANUAL;

  // brake
  command_auto_.command.brake = (msg->emergency == 1) ? G30ESLI_BRAKE_SEMIEMG : G30ESLI_BRAKE_NONE;

  // shift
  if (msg->gear == 1)
  {
    command_auto_.command.shift = G30ESLI_SHIFT_DRIVE;
  }
  else if (msg->gear == 2)
  {
    command_auto_.command.shift = G30ESLI_SHIFT_REVERSE;
  }
  else if (msg->gear == 4)
  {
    command_auto_.command.shift = G30ESLI_SHIFT_NEUTRAL;
  }

  // flasher
  if (msg->lamp_cmd.l == 0 && msg->lamp_cmd.r == 0)
  {
    command_auto_.command.flasher = G30ESLI_FLASHER_CLEAR;
  }
  else if (msg->lamp_cmd.l == 1 && msg->lamp_cmd.r == 0)
  {
    command_auto_.command.flasher = G30ESLI_FLASHER_LEFT;
  }
  else if (msg->lamp_cmd.l == 0 && msg->lamp_cmd.r == 1)
  {
    command_auto_.command.flasher = G30ESLI_FLASHER_RIGHT;
  }
  else if (msg->lamp_cmd.l == 1 && msg->lamp_cmd.r == 1)
  {
    command_auto_.command.flasher = G30ESLI_FLASHER_HAZARD;
  }
}

// generate command by ds4 joystick
void ds4Callback(const ds4_msgs::DS4ConstPtr& msg)
{
  command_joy_.time = ros::Time::now();
  command_joy_.received = true;

  // check connection
  if (!msg->connected)
  {
    command_joy_.command.speed = 0.0;
    command_joy_.command.brake = G30ESLI_BRAKE_SEMIEMG;
    return;
  }

  // joystick override = use command_joy
  if ((msg->square || msg->cross) && !joy_)
  {
    ROS_INFO("JOYSTICK: Joystick Manual Mode");
    joy_ = true;
  }

  // autonomous drive mode = use command_auto
  if (msg->ps && joy_)
  {
    ROS_INFO("JOYSTICK: Autonomous Driving Mode");
    joy_ = false;
  }

  // speed
  command_joy_.command.speed = msg->cross ? 16.0 * msg->r2 + 3.0 : 0.0;

  // steer
  command_joy_.command.steer = -((17.0 * msg->l2 + 20.0) * msg->left_y + steering_offset_deg_);

  // mode
  if (msg->option && !engage_)
  {
    engage_ = true;
    ROS_INFO("JOYSTICK: Engaged");
  }
  if (msg->share && engage_)
  {
    engage_ = false;
    ROS_WARN("JOYSTICK: Disengaged");
  }
  command_joy_.command.mode = engage_ ? G30ESLI_MODE_AUTO : G30ESLI_MODE_MANUAL;

  // brake
  if (!(msg->square || msg->circle || msg->triangle))
  {
    command_joy_.command.brake = G30ESLI_BRAKE_NONE;
  }
  else
  {
    command_joy_.command.brake = msg->square ? G30ESLI_BRAKE_SMOOTH : command_joy_.command.brake;
    command_joy_.command.brake = msg->circle ? G30ESLI_BRAKE_SEMIEMG : command_joy_.command.brake;
    command_joy_.command.brake = msg->triangle ? G30ESLI_BRAKE_EMERGENCY : command_joy_.command.brake;
  }

  // shift
  if (!(msg->r1 || msg->l1))
  {
    command_joy_.command.shift = G30ESLI_SHIFT_DRIVE;
  }
  else
  {
    command_joy_.command.shift = msg->r1 ? G30ESLI_SHIFT_REVERSE : command_joy_.command.shift;
    command_joy_.command.shift = msg->l1 ? G30ESLI_SHIFT_NEUTRAL : command_joy_.command.shift;
  }

  // flasher
  if (!(msg->up || msg->right || msg->left || msg->down))
  {
    command_joy_.command.flasher = G30ESLI_FLASHER_NONE;
  }
  else
  {
    command_joy_.command.flasher = msg->right ? G30ESLI_FLASHER_RIGHT : command_joy_.command.flasher;
    command_joy_.command.flasher = msg->left ? G30ESLI_FLASHER_LEFT : command_joy_.command.flasher;
    command_joy_.command.flasher = msg->down ? G30ESLI_FLASHER_HAZARD : command_joy_.command.flasher;
    command_joy_.command.flasher = msg->up ? G30ESLI_FLASHER_CLEAR : command_joy_.command.flasher;
  }
}

// receive input from keyboard
// change the mode to manual mode or auto drive mode
void changeMode()
{
  while (!terminate_thread_)
  {
    if (kbhit())
    {
      char c = getchar();
      if (c == 's')
      {
        if (!engage_)
        {
          engage_ = true;
          ROS_INFO("KEYBOARD: Engaged");
        }
        if (joy_)
        {
          joy_ = false;
          ROS_INFO("KEYBOARD: Autonomous Driving Mode");
        }
      }
      else if (c == ' ')
      {
        if (engage_)
        {
          engage_ = false;
          ROS_WARN("KEYBOARD: Disengaged");
        }
        if (joy_)
        {
          joy_ = false;
          ROS_INFO("KEYBOARD: Autonomous Driving Mode");
        }
      }
    }
    usleep(10);
  }
}

// read vehicle status
void readStatus()
{
  while (!terminate_thread_)
  {
    g30esli_.readStatus(status_.status);

    ros::Time now = ros::Time::now();

    // accel/brake override, switch to manual mode
    if ((status_.status.override.accel == 1 || status_.status.override.brake == 1) && engage_)
    {
      engage_ = false;
      command_auto_.command.mode = G30ESLI_MODE_MANUAL;
      command_joy_.command.mode = G30ESLI_MODE_MANUAL;
      ROS_WARN("OVERRIDE: Disengaged");
    }

    // update twist
    double lv = status_.status.speed.actual / 3.6;             // [km/h] -> [m/s]
    double th = (-status_.status.steer.actual + steering_offset_deg_) * M_PI / 180.0;   // [deg] -> [rad]
    double az = std::tan(th) * lv / G30ESLI_WHEEL_BASE; // [rad] -> [rad/s]
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
    vehicle_status_.drivepedal = status_.status.override.accel;  // TODO: scaling

    // brakepedal
    vehicle_status_.brakepedal = status_.status.override.brake;  // TODO: scaling

    // angle
    vehicle_status_.angle = -status_.status.steer.actual;  // [deg]

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
    vehicle_status_.light = 0; // not used
  }
}

// publish vehicle status
void publishStatus()
{
  while (!terminate_thread_)
  {
    current_twist_pub_.publish(current_twist_);
    vehicle_status_pub_.publish(vehicle_status_);
    usleep(10000);
  }
}

int main(int argc, char* argv[])
{
  // ros initialization
  ros::init(argc, argv, "g30esli_interface");
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh("~");

  // rosparam
  private_nh.param<std::string>("device", device_, "can0");
  private_nh.param<bool>("use_ds4", use_ds4_, false);
  private_nh.param<double>("steering_offset_deg", steering_offset_deg_, 0.0);
  private_nh.param<double>("command_timeout", command_timeout_, 1000);

  // mode at startup
  private_nh.param<bool>("engaged", engage_, true);

  // subscriber
  vehicle_cmd_sub_ = nh_.subscribe<autoware_msgs::VehicleCmd>("vehicle_cmd", 1, vehicleCmdCallback);

  if (use_ds4_)
  {
    ds4_sub_ = nh_.subscribe<ds4_msgs::DS4>("ds4", 1, ds4Callback);
    joy_ = true;  // use ds4 at startup
  }

  // publisher
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("ymc_current_twist", 10);
  vehicle_status_pub_ = nh_.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 10);

  // open can device
  if (!g30esli_.openDevice(device_))
  {
    std::cerr << "Cannot open device" << std::endl;
    return -1;
  }

  // create threads
  terminate_thread_ = false;
  std::thread t1(changeMode);
  std::thread t2(readStatus);
  std::thread t3(publishStatus);

  ros::Rate rate(100);
  unsigned char alive = 0;
  bool timeouted = false;

  while (ros::ok())
  {
    // update subscribers
    ros::spinOnce();

    // select command
    Command& command = !joy_ ? command_auto_ : command_joy_;

    // check timeout
    timeouted = (((ros::Time::now() - command.time).toSec() * 1000) > command_timeout_);

    // semi-emergency stop if timeout occured
    if (command.received && timeouted)
    {
      ROS_ERROR("Timeout > %f [ms], emergency stop...", command_timeout_);
      command.command.speed = 0.0;
      command.command.brake = G30ESLI_BRAKE_SEMIEMG;
    }

    // send vehicle command
    g30esli_.sendCommand(command.command);

    // update heart beat
    alive++;
    command_auto_.command.alive = alive;
    command_joy_.command.alive = alive;

    // debug
    ROS_DEBUG_STREAM("Status[" << (int)command.command.alive << "]\n" << ymc::G30esli::dumpStatus(status_.status));
    ROS_DEBUG_STREAM("Command[" << (int)command.command.alive << "]\n" << ymc::G30esli::dumpCommand(command.command));

    // loop sleep
    rate.sleep();
  }

  terminate_thread_ = true;
  t1.join();
  t2.join();
  t3.join();

  return 0;
}
