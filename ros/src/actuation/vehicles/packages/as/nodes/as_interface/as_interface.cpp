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
  private_nh_.param<bool>("use_timer_publisher", use_timer_publisher_, false);
  private_nh_.param<double>("publish_frequency", publish_frequency_, 10.0);

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
  speed_sub_ = nh_.subscribe("/as/velocity_accel", 1, &PacmodInterface::callbackFromVelocityAccel, this);
  lamp_cmd_sub_ = nh_.subscribe("/lamp_cmd", 1, &PacmodInterface::callbackFromLampCmd, this);

  // sease_link"up timer
  if (use_timer_publisher_)
  {
    pacmod_timer_ = nh_.createTimer(ros::Rate(publish_frequency_), &PacmodInterface::callbackPacmodTimer, this);
  }

  // setup publisher
  steer_mode_pub_ = nh_.advertise<module_comm_msgs::SteerMode>("/as/arbitrated_steering_commands", 1);
  speed_mode_pub_ = nh_.advertise<module_comm_msgs::SpeedMode>("/as/arbitrated_speed_commands", 1);
  turn_signal_pub_ = nh_.advertise<platform_comm_msgs::TurnSignalCommand>("/as/turn_signal_command", 1);
  gear_pub_ = nh_.advertise<platform_comm_msgs::GearCommand>("/as/gear_select", 1, true);
  velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("as_velocity", 10);
}

void PacmodInterface::run()
{
  ros::spin();
}

void PacmodInterface::callbackFromCurvatureCmd(const autoware_msgs::CurvatureCommandStampedConstPtr& msg)
{
  header_ = msg->header;
  speed_ = msg->cmd.linear_velocity;
  curvature_ = msg->cmd.curvature;
  if (!use_timer_publisher_)
    publishToPacmod();
}

void PacmodInterface::callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr& msg)
{
  header_ = msg->header;
  speed_ = msg->twist.linear.x >= minimum_linear_x_ ? msg->twist.linear.x : minimum_linear_x_;
  curvature_ = msg->twist.angular.z / speed_;
  if (!use_timer_publisher_)
    publishToPacmod();
}

void PacmodInterface::callbackFromControlMode(const std_msgs::BoolConstPtr& msg)
{
  control_mode_ = msg->data;
}

void PacmodInterface::callbackFromLampCmd(const autoware_msgs::lamp_cmdConstPtr& msg)
{
  lamp_cmd_ = *msg;
}

void PacmodInterface::callbackFromVelocityAccel(const module_comm_msgs::VelocityAccelConstPtr& msg)
{
  geometry_msgs::TwistStamped ts;
  std_msgs::Header header;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "base_link";
  ts.twist.linear.x = msg->velocity;  // [m/sec]
  // Can we get angular velocity?
  velocity_pub_.publish(ts);
}

void PacmodInterface::callbackPacmodTimer(const ros::TimerEvent& event)
{
  publishToPacmod();
}

void PacmodInterface::publishToPacmod()
{
  module_comm_msgs::SpeedMode speed_mode;
  speed_mode.header = header_;
  speed_mode.mode = control_mode_ ? 1 : 0;
  speed_mode.speed = speed_;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit_;

  module_comm_msgs::SteerMode steer_mode;
  steer_mode.header = header_;
  steer_mode.mode = speed_mode.mode;
  steer_mode.curvature = curvature_;
  steer_mode.max_curvature_rate = max_curvature_rate_;

  platform_comm_msgs::TurnSignalCommand turn_signal;
  turn_signal.header = header_;
  turn_signal.mode = speed_mode.mode;
  if (lamp_cmd_.l == 1)
  {
    turn_signal.turn_signal = platform_comm_msgs::TurnSignalCommand::LEFT;
  }
  else if (lamp_cmd_.r == 1)
  {
    turn_signal.turn_signal = platform_comm_msgs::TurnSignalCommand::RIGHT;
  }
  else
  {
    turn_signal.turn_signal = platform_comm_msgs::TurnSignalCommand::NONE;
  }

  platform_comm_msgs::GearCommand gear_comm;
  gear_comm.header.stamp = ros::Time::now();
  gear_comm.command.gear = control_mode_ ? platform_comm_msgs::Gear::DRIVE :
                                           platform_comm_msgs::Gear::NONE;  // Drive if auto mode is enabled

  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);
  gear_pub_.publish(gear_comm);

  ROS_INFO_STREAM("mode: " << speed_mode.mode << ", "
                           << "speed: " << speed_mode.speed << ", "
                           << "steer: " << steer_mode.curvature << ", "
                           << "gear: " << (int)gear_comm.command.gear << ", "
                           << "turn_signal: " << (int)turn_signal.turn_signal);
}

}  // pacmod
