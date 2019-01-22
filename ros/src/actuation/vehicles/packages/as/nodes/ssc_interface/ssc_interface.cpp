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

#include "ssc_interface.h"

SSCInterface::SSCInterface() : nh_(), private_nh_("~"), control_mode_(false)
{
  // setup parameters
  private_nh_.param<double>("loop_rate", loop_rate_, 100.0);
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_curvature_rate", max_curvature_rate_, 0.75);

  rate_ = new ros::Rate(loop_rate_);

  // setup subscribers
  twist_cmd_sub_ = nh_.subscribe("twist_cmd", 1, &SSCInterface::callbackFromTwistCmd, this);
  current_velocity_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::VelocityAccel>(nh_, "as/velocity_accel", 1);
  current_curvature_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::CurvatureFeedback>(nh_, "as/curvature_feedback", 1);
  current_twist_sync_ = new message_filters::Synchronizer<CurrentTwistSyncPolicy>(
      CurrentTwistSyncPolicy(10), *current_velocity_sub_, *current_curvature_sub_);
  current_twist_sync_->registerCallback(boost::bind(&SSCInterface::callbackFromSyncedCurrentTwist, this, _1, _2));
  control_mode_sub_ = nh_.subscribe("as/control_mode", 1, &SSCInterface::callbackFromControlMode, this);
  lamp_cmd_sub_ = nh_.subscribe("lamp_cmd", 1, &SSCInterface::callbackFromLampCmd, this);

  // setup publisher
  steer_mode_pub_ = nh_.advertise<automotive_platform_msgs::SteerMode>("as/arbitrated_steering_commands", 1);
  speed_mode_pub_ = nh_.advertise<automotive_platform_msgs::SpeedMode>("as/arbitrated_speed_commands", 1);
  turn_signal_pub_ = nh_.advertise<automotive_platform_msgs::TurnSignalCommand>("as/turn_signal_command", 1);
  gear_pub_ = nh_.advertise<automotive_platform_msgs::GearCommand>("as/gear_select", 1, true);

  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("as_current_twist", 10);
  current_speed_pub_ = nh_.advertise<std_msgs::Float32>("as_current_speed", 10);
}

SSCInterface::~SSCInterface()
{
}

void SSCInterface::run()
{
  while (ros::ok())
  {
    ros::spinOnce();

    publishCommand();

    rate_->sleep();
  }
}

void SSCInterface::callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr& msg)
{
  desired_speed_ = std::max(msg->twist.linear.x, MINIMUM_DESIRED_SPEED);
  desired_curvature_ = msg->twist.angular.z / desired_speed_;
}

void SSCInterface::callbackFromControlMode(const std_msgs::BoolConstPtr& msg)
{
  control_mode_ = msg->data;
}

void SSCInterface::callbackFromLampCmd(const autoware_msgs::LampCmdConstPtr& msg)
{
  lamp_cmd_ = *msg;
}

void SSCInterface::callbackFromSyncedCurrentTwist(
    const automotive_platform_msgs::VelocityAccelConstPtr& msg_velocity,
    const automotive_platform_msgs::CurvatureFeedbackConstPtr& msg_curvature)
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.header.frame_id = "base_link";
  ts.twist.linear.x = msg_velocity->velocity;                         // [m/s]
  ts.twist.angular.z = msg_curvature->curvature * ts.twist.linear.x;  // [rad/s]
  current_twist_pub_.publish(ts);

  // for vizualize
  std_msgs::Float32 msg;
  msg.data = msg_velocity->velocity;
  current_speed_pub_.publish(msg);
}

void SSCInterface::publishCommand()
{
  static std::string frame_id = "base_link";
  ros::Time now = ros::Time::now();

  // speed command
  automotive_platform_msgs::SpeedMode speed_mode;
  speed_mode.header.frame_id = frame_id;
  speed_mode.header.stamp = now;
  speed_mode.mode = control_mode_ ? 1 : 0;
  speed_mode.speed = desired_speed_;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit_;

  // steer command
  automotive_platform_msgs::SteerMode steer_mode;
  steer_mode.header.frame_id = frame_id;
  steer_mode.header.stamp = now;
  steer_mode.mode = speed_mode.mode;
  steer_mode.curvature = desired_curvature_;
  steer_mode.max_curvature_rate = max_curvature_rate_;

  // turn signal command
  automotive_platform_msgs::TurnSignalCommand turn_signal;
  turn_signal.header.frame_id = frame_id;
  turn_signal.header.stamp = now;
  turn_signal.mode = speed_mode.mode;

  if (lamp_cmd_.l == 0 && lamp_cmd_.r == 0)
  {
    turn_signal.turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
  }
  else if (lamp_cmd_.l == 1 && lamp_cmd_.r == 0)
  {
    turn_signal.turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
  }
  else if (lamp_cmd_.l == 0 && lamp_cmd_.r == 1)
  {
    turn_signal.turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
  }
  else if (lamp_cmd_.l == 1 && lamp_cmd_.r == 1)
  {
    // turn_signal.turn_signal = automotive_platform_msgs::TurnSignalCommand::HAZARD;
  }

  // gear command
  automotive_platform_msgs::GearCommand gear_cmd;
  gear_cmd.header.frame_id = frame_id;
  gear_cmd.header.stamp = now;
  gear_cmd.command.gear = control_mode_ ? automotive_platform_msgs::Gear::DRIVE :
                                          automotive_platform_msgs::Gear::NONE;  // Drive if auto mode is enabled

  // publish
  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);
  gear_pub_.publish(gear_cmd);

  ROS_INFO_STREAM("mode: " << speed_mode.mode << ", "
                           << "speed: " << speed_mode.speed << ", "
                           << "steer: " << steer_mode.curvature << ", "
                           << "gear: " << (int)gear_cmd.command.gear << ", "
                           << "turn_signal: " << (int)turn_signal.turn_signal);
}
