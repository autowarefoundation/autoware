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

#include "as_interface.h"

namespace pacmod
{
// Constructor
PacmodInterface::PacmodInterface() : private_nh_("~"), control_mode_(false)
{
  initForROS();
}

// Destructor
PacmodInterface::~PacmodInterface()
{
}

void PacmodInterface::initForROS()
{
  // ros parameter settings
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_curvature_rate", max_curvature_rate_, 0.75);
  private_nh_.param<bool>("use_curvature_cmd", use_curvature_cmd_, true);

  // setup subscriber
  if (!use_curvature_cmd_)
  {
    twist_cmd_sub_ = nh_.subscribe("twist_cmd", 1, &PacmodInterface::callbackFromTwistCmd, this);
  }
  else
  {
    curvature_cmd_sub_ = nh_.subscribe("curvature_cmd", 1, &PacmodInterface::callbackFromCurvatureCmd, this);
  }

  control_mode_sub_ = nh_.subscribe("/as/control_mode", 1, &PacmodInterface::callbackFromControlMode, this);
  speed_sub_ = nh_.subscribe("/vehicle/steering_report", 1, &PacmodInterface::callbackFromSteeringReport, this);

  // setup publisher
  steer_mode_pub_ = nh_.advertise<module_comm_msgs::SteerMode>("/as/arbitrated_steering_commands", 1);
  speed_mode_pub_ = nh_.advertise<module_comm_msgs::SpeedMode>("/as/arbitrated_speed_commands", 1);
  turn_signal_pub_ = nh_.advertise<platform_comm_msgs::TurnSignalCommand>("/as/turn_signal_command", 1);
  gear_pub_ = nh_.advertise<platform_comm_msgs::GearCommand>("/as/gear_select", 1, true);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("as_current_twist", 10);
}

void PacmodInterface::run()
{
  ros::spin();
}

void PacmodInterface::callbackFromCurvatureCmd(const autoware_msgs::CurvatureCommandStamped& msg)
{
  int mode;

  if (control_mode_)
  {
    mode = 1;
  }
  else
  {
    mode = 0;
  }

  module_comm_msgs::SpeedMode speed_mode;
  speed_mode.header = msg.header;
  speed_mode.mode = mode;
  speed_mode.speed = msg.cmd.linear_velocity;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit_;

  module_comm_msgs::SteerMode steer_mode;
  steer_mode.header = msg.header;
  steer_mode.mode = mode;
  steer_mode.curvature = msg.cmd.curvature;
  steer_mode.max_curvature_rate = 0.75;

  platform_comm_msgs::TurnSignalCommand turn_signal;
  turn_signal.header = msg.header;
  turn_signal.mode = mode;

  platform_comm_msgs::GearCommand gear_comm;
  gear_comm.header.stamp = ros::Time::now();
  gear_comm.command.gear =
      mode ? platform_comm_msgs::Gear::DRIVE : platform_comm_msgs::Gear::NONE;  // Drive if auto mode is enabled

  std::cout << "mode: " << mode << std::endl;
  std::cout << "speed: " << speed_mode.speed << std::endl;
  std::cout << "steer: " << steer_mode.curvature << std::endl;

  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);

  std::cout << "GEAR COMM: " << gear_comm << "\n";
  gear_pub_.publish(gear_comm);
}

void PacmodInterface::callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr& msg)
{
  int mode;
  if (control_mode_)
  {
    mode = 1;
  }
  else
  {
    mode = 0;
  }

  double _minimum_linear_x = 1e-6;
  double _speed = msg->twist.linear.x >= _minimum_linear_x ? msg->twist.linear.x : _minimum_linear_x;
  double _curvature = msg->twist.angular.z / _speed;

  module_comm_msgs::SpeedMode speed_mode;
  speed_mode.header = msg->header;
  speed_mode.mode = mode;
  speed_mode.speed = _speed;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit_;

  module_comm_msgs::SteerMode steer_mode;
  steer_mode.header = msg->header;
  steer_mode.mode = mode;
  steer_mode.curvature = _curvature;
  steer_mode.max_curvature_rate = 0.75;

  platform_comm_msgs::TurnSignalCommand turn_signal;
  turn_signal.header = msg->header;
  turn_signal.mode = mode;

  platform_comm_msgs::GearCommand gear_comm;
  gear_comm.header.stamp = ros::Time::now();
  gear_comm.command.gear =
      mode ? platform_comm_msgs::Gear::DRIVE : platform_comm_msgs::Gear::NONE;  // Drive if auto mode is enabled

  std::cout << "mode: " << mode << std::endl;
  std::cout << "speed: " << speed_mode.speed << std::endl;
  std::cout << "steer: " << steer_mode.curvature << std::endl;

  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);

  std::cout << "GEAR COMM: " << gear_comm << "\n";
  gear_pub_.publish(gear_comm);
}

void PacmodInterface::callbackFromControlMode(const std_msgs::BoolConstPtr& msg)
{
  control_mode_ = msg->data;
}

void PacmodInterface::callbackFromSteeringReport(const dbw_mkz_msgs::SteeringReportConstPtr& msg)
{
  geometry_msgs::TwistStamped ts;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  ts.header = header;
  ts.twist.linear.x = msg->speed;  // [m/sec]
  // Can we get angular velocity?

  current_twist_pub_.publish(ts);
}

}  // pacmod
