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

SSCInterface::SSCInterface() : nh_(), private_nh_("~"), engage_(false)
{
  // setup parameters
  // NOTE: max steering wheel rotation rate = 6.28 [rad/s]
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh_.param<double>("wheel_base", wheel_base_, 2.79);
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_curvature_rate", max_curvature_rate_, 0.15);

  rate_ = new ros::Rate(loop_rate_);

  // subscribers from autoware
  vehicle_cmd_sub_ = nh_.subscribe("vehicle_cmd", 1, &SSCInterface::callbackFromVehicleCmd, this);
  engage_sub_ = nh_.subscribe("vehicle/engage", 1, &SSCInterface::callbackFromEngage, this);

  // subscribers from ssc
  current_velocity_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::VelocityAccel>(nh_, "as/velocity_accel", 10);
  current_curvature_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::CurvatureFeedback>(nh_, "as/curvature_feedback", 10);
  current_twist_sync_ = new message_filters::Synchronizer<CurrentTwistSyncPolicy>(
      CurrentTwistSyncPolicy(10), *current_velocity_sub_, *current_curvature_sub_);
  current_twist_sync_->registerCallback(boost::bind(&SSCInterface::callbackFromSyncedCurrentTwist, this, _1, _2));

  // publishers to autoware
  vehicle_status_pub_ = nh_.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vehicle/twist", 10);

  // publishers to ssc
  steer_mode_pub_ = nh_.advertise<automotive_platform_msgs::SteerMode>("as/arbitrated_steering_commands", 10);
  speed_mode_pub_ = nh_.advertise<automotive_platform_msgs::SpeedMode>("as/arbitrated_speed_commands", 10);
  turn_signal_pub_ = nh_.advertise<automotive_platform_msgs::TurnSignalCommand>("as/turn_signal_command", 10);
  gear_pub_ = nh_.advertise<automotive_platform_msgs::GearCommand>("as/gear_select", 1, true);
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

void SSCInterface::callbackFromVehicleCmd(const autoware_msgs::VehicleCmdConstPtr& msg)
{
  vehicle_cmd_ = *msg;
}

void SSCInterface::callbackFromEngage(const std_msgs::BoolConstPtr& msg)
{
  engage_ = msg->data;
}

void SSCInterface::callbackFromSyncedCurrentTwist(
    const automotive_platform_msgs::VelocityAccelConstPtr& msg_velocity,
    const automotive_platform_msgs::CurvatureFeedbackConstPtr& msg_curvature)
{
  geometry_msgs::TwistStamped ts;
  ts.header.frame_id = BASE_FRAME_ID;
  ts.header.stamp = msg_velocity->header.stamp;
  ts.twist.linear.x = msg_velocity->velocity; // [m/s]
  ts.twist.angular.z = msg_curvature->curvature *  ts.twist.linear.x; // [rad/s]
  current_twist_pub_.publish(ts);
}

void SSCInterface::publishCommand()
{
  ros::Time stamp = ros::Time::now();

  // Desired values
  // Driving mode (If autonomy mode should be active, mode = 1)
  unsigned char desired_mode = engage_ ? 1 : 0;
  // Speed for SSC speed_model
  double desired_speed = vehicle_cmd_.ctrl_cmd.linear_velocity;
  // Curvature for SSC steer_model (TODO: Add adaptive gear ratio)
  double desired_curvature = std::tan(vehicle_cmd_.ctrl_cmd.steering_angle) / wheel_base_;
  // Gear (TODO: Use vehicle_cmd.gear)
  unsigned char desired_gear = engage_ ? automotive_platform_msgs::Gear::DRIVE : automotive_platform_msgs::Gear::NONE;
  // Turn signal
  unsigned char desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;

  if (vehicle_cmd_.lamp_cmd.l == 0 && vehicle_cmd_.lamp_cmd.r == 0)
  {
    desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
  }
  else if (vehicle_cmd_.lamp_cmd.l == 1 && vehicle_cmd_.lamp_cmd.r == 0)
  {
    desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
  }
  else if (vehicle_cmd_.lamp_cmd.l == 0 && vehicle_cmd_.lamp_cmd.r == 1)
  {
    desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
  }
  else if (vehicle_cmd_.lamp_cmd.l == 1 && vehicle_cmd_.lamp_cmd.r == 1)
  {
    // NOTE: HAZARD signal cannot be used in automotive_platform_msgs::TurnSignalCommand
  }

  // speed command
  automotive_platform_msgs::SpeedMode speed_mode;
  speed_mode.header.frame_id = BASE_FRAME_ID;
  speed_mode.header.stamp = stamp;
  speed_mode.mode = desired_mode;
  speed_mode.speed = desired_speed;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit_;

  // steer command
  automotive_platform_msgs::SteerMode steer_mode;
  steer_mode.header.frame_id = BASE_FRAME_ID;
  steer_mode.header.stamp = stamp;
  steer_mode.mode = desired_mode;
  steer_mode.curvature = desired_curvature;
  steer_mode.max_curvature_rate = max_curvature_rate_;

  // turn signal command
  automotive_platform_msgs::TurnSignalCommand turn_signal;
  turn_signal.header.frame_id = BASE_FRAME_ID;
  turn_signal.header.stamp = stamp;
  turn_signal.mode = desired_mode;
  turn_signal.turn_signal = desired_turn_signal;

  // gear command
  automotive_platform_msgs::GearCommand gear_cmd;
  gear_cmd.header.frame_id = BASE_FRAME_ID;
  gear_cmd.header.stamp = stamp;
  gear_cmd.command.gear = desired_gear;

  // publish
  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);
  gear_pub_.publish(gear_cmd);

  ROS_INFO_STREAM("Mode: " << (int)desired_mode << ", "
                           << "Speed: " << speed_mode.speed << ", "
                           << "Curvature: " << steer_mode.curvature << ", "
                           << "Gear: " << (int)gear_cmd.command.gear << ", "
                           << "TurnSignal: " << (int)turn_signal.turn_signal);
}
