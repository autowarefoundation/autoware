/*
 *  Copyright (c) 2017, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

// variables
bool joy_;
bool engage_;
bool terminate_thread_;
geometry_msgs::TwistStamped current_twist_;
autoware_msgs::VehicleStatus vehicle_status_;

// ymc g30esli driver
ymc::G30esli g30esli_;
ymc::G30esli::Status status_;
ymc::G30esli::Command command_auto_, command_joy_;

// generate command by autoware
void vehicleCmdCallback(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  // speed
  double speed_kmph = msg->ctrl_cmd.linear_velocity * 3.6;  // [m/s] -> [km/h]
  command_auto_.speed = engage_ ? speed_kmph : 0.0;

  // steer
  double steering_angle_deg = msg->ctrl_cmd.steering_angle / M_PI * 180.0;  // [rad] -> [deg]
  command_auto_.steer = -(steering_angle_deg + steering_offset_deg_);

  // mode
  command_auto_.mode = engage_ ? G30ESLI_MODE_AUTO : G30ESLI_MODE_MANUAL;

  // brake
  command_auto_.brake = (msg->emergency == 1) ? G30ESLI_BRAKE_SEMIEMG : G30ESLI_BRAKE_NONE;

  // shift
  if (msg->gear == 1)
  {
    command_auto_.shift = G30ESLI_SHIFT_DRIVE;
  }
  else if (msg->gear == 2)
  {
    command_auto_.shift = G30ESLI_SHIFT_REVERSE;
  }
  else if (msg->gear == 4)
  {
    command_auto_.shift = G30ESLI_SHIFT_NEUTRAL;
  }

  // flasher
  if (msg->lamp_cmd.l == 0 && msg->lamp_cmd.r == 0)
  {
    command_auto_.flasher = G30ESLI_FLASHER_CLEAR;
  }
  else if (msg->lamp_cmd.l == 1 && msg->lamp_cmd.r == 0)
  {
    command_auto_.flasher = G30ESLI_FLASHER_LEFT;
  }
  else if (msg->lamp_cmd.l == 0 && msg->lamp_cmd.r == 1)
  {
    command_auto_.flasher = G30ESLI_FLASHER_RIGHT;
  }
  else if (msg->lamp_cmd.l == 1 && msg->lamp_cmd.r == 1)
  {
    command_auto_.flasher = G30ESLI_FLASHER_HAZARD;
  }
}

// generate command by ds4 joystick
void ds4Callback(const ds4_msgs::DS4ConstPtr& msg)
{
  // check connection
  if (!msg->connected)
  {
    command_joy_.speed = 0.0;
    command_joy_.brake = G30ESLI_BRAKE_SEMIEMG;
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
  command_joy_.speed = msg->cross ? 16.0 * msg->r2 + 3.0 : 0.0;

  // steer
  command_joy_.steer = -((17.0 * msg->l2 + 20.0) * msg->left_y + steering_offset_deg_);

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
  command_joy_.mode = engage_ ? G30ESLI_MODE_AUTO : G30ESLI_MODE_MANUAL;

  // brake
  if (!(msg->square || msg->circle || msg->triangle))
  {
    command_joy_.brake = G30ESLI_BRAKE_NONE;
  }
  else
  {
    command_joy_.brake = msg->square ? G30ESLI_BRAKE_SMOOTH : command_joy_.brake;
    command_joy_.brake = msg->circle ? G30ESLI_BRAKE_SEMIEMG : command_joy_.brake;
    command_joy_.brake = msg->triangle ? G30ESLI_BRAKE_EMERGENCY : command_joy_.brake;
  }

  // shift
  if (!(msg->r1 || msg->l1))
  {
    command_joy_.shift = G30ESLI_SHIFT_DRIVE;
  }
  else
  {
    command_joy_.shift = msg->r1 ? G30ESLI_SHIFT_REVERSE : command_joy_.shift;
    command_joy_.shift = msg->l1 ? G30ESLI_SHIFT_NEUTRAL : command_joy_.shift;
  }

  // flasher
  if (!(msg->up || msg->right || msg->left || msg->down))
  {
    command_joy_.flasher = G30ESLI_FLASHER_NONE;
  }
  else
  {
    command_joy_.flasher = msg->right ? G30ESLI_FLASHER_RIGHT : command_joy_.flasher;
    command_joy_.flasher = msg->left ? G30ESLI_FLASHER_LEFT : command_joy_.flasher;
    command_joy_.flasher = msg->down ? G30ESLI_FLASHER_HAZARD : command_joy_.flasher;
    command_joy_.flasher = msg->up ? G30ESLI_FLASHER_CLEAR : command_joy_.flasher;
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
    g30esli_.readStatus(status_);

    ros::Time now = ros::Time::now();

    // accel/brake override, switch to manual mode
    if ((status_.override.accel == 1 || status_.override.brake == 1) && engage_)
    {
      engage_ = false;
      command_auto_.mode = G30ESLI_MODE_MANUAL;
      command_joy_.mode = G30ESLI_MODE_MANUAL;
      ROS_WARN("OVERRIDE: Disengaged");
    }

    // update twist
    double lv = status_.speed.actual / 3.6;             // [km/h] -> [m/s]
    double th = (-status_.steer.actual + steering_offset_deg_) * M_PI / 180.0;   // [deg] -> [rad]
    double az = std::tan(th) * lv / G30ESLI_WHEEL_BASE; // [rad] -> [rad/s]
    current_twist_.header.frame_id = "base_link";
    current_twist_.header.stamp = now;
    current_twist_.twist.linear.x = lv;
    current_twist_.twist.angular.z = az;

    // update vehicle status
    vehicle_status_.header = current_twist_.header;

    // drive/steeringmode
    if (status_.mode == G30ESLI_MODE_MANUAL)
    {
      vehicle_status_.drivemode = autoware_msgs::VehicleStatus::MODE_MANUAL;
      vehicle_status_.steeringmode = autoware_msgs::VehicleStatus::MODE_MANUAL;
    }
    else if (status_.mode == G30ESLI_MODE_AUTO)
    {
      vehicle_status_.drivemode = autoware_msgs::VehicleStatus::MODE_AUTO;
      vehicle_status_.steeringmode = autoware_msgs::VehicleStatus::MODE_AUTO;
    }

    // gearshift
    if (status_.shift == G30ESLI_SHIFT_DRIVE)
    {
      vehicle_status_.gearshift = 1;
    }
    else if (status_.shift == G30ESLI_SHIFT_REVERSE)
    {
      vehicle_status_.gearshift = 2;
    }
    else if (status_.shift == G30ESLI_SHIFT_NEUTRAL)
    {
      vehicle_status_.gearshift = 4;
    }

    // speed
    vehicle_status_.speed = status_.speed.actual;  // [kmph]

    // drivepedal
    vehicle_status_.drivepedal = status_.override.accel;  // TODO: scaling

    // brakepedal
    vehicle_status_.brakepedal = status_.override.brake;  // TODO: scaling

    // angle
    vehicle_status_.angle = -status_.steer.actual;  // [deg]

    // lamp
    if (status_.override.flasher == G30ESLI_FLASHER_NONE)
    {
      vehicle_status_.lamp = 0;
    }
    else if (status_.override.flasher == G30ESLI_FLASHER_RIGHT)
    {
      vehicle_status_.lamp = autoware_msgs::VehicleStatus::LAMP_RIGHT;
    }
    else if (status_.override.flasher == G30ESLI_FLASHER_LEFT)
    {
      vehicle_status_.lamp = autoware_msgs::VehicleStatus::LAMP_LEFT;
    }
    else if (status_.override.flasher == G30ESLI_FLASHER_HAZARD)
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

  while (ros::ok())
  {
    // update subscribers
    ros::spinOnce();

    // select command
    ymc::G30esli::Command& cmd = !joy_ ? command_auto_ : command_joy_;

    // send vehicle command
    g30esli_.sendCommand(cmd);

    // update heart beat
    alive++;
    command_auto_.alive = alive;
    command_joy_.alive = alive;

    // debug
    ROS_DEBUG_STREAM("Status[" << (int)cmd.alive << "]\n" << ymc::G30esli::dumpStatus(status_));
    ROS_DEBUG_STREAM("Command[" << (int)cmd.alive << "]\n" << ymc::G30esli::dumpCommand(cmd));

    // loop sleep
    rate.sleep();
  }

  terminate_thread_ = true;
  t1.join();
  t2.join();
  t3.join();

  return 0;
}
