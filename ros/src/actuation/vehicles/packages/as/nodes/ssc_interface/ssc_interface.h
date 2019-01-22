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

#ifndef SSC_INTERFACE_H
#define SSC_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <automotive_platform_msgs/VelocityAccel.h>
#include <automotive_platform_msgs/CurvatureFeedback.h>
#include <automotive_platform_msgs/TurnSignalCommand.h>
#include <automotive_platform_msgs/GearCommand.h>
#include <automotive_platform_msgs/Gear.h>

#include <autoware_msgs/LampCmd.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

static const double MINIMUM_DESIRED_SPEED = 1e-6;  // [m/s]

class SSCInterface
{
public:
  SSCInterface();
  ~SSCInterface();

  void run();

private:
  typedef message_filters::sync_policies::ApproximateTime<automotive_platform_msgs::VelocityAccel,
                                                          automotive_platform_msgs::CurvatureFeedback>
      CurrentTwistSyncPolicy;

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscriber
  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Subscriber lamp_cmd_sub_;
  message_filters::Subscriber<automotive_platform_msgs::VelocityAccel>* current_velocity_sub_;
  message_filters::Subscriber<automotive_platform_msgs::CurvatureFeedback>* current_curvature_sub_;
  message_filters::Synchronizer<CurrentTwistSyncPolicy>* current_twist_sync_;

  // publisher
  ros::Publisher steer_mode_pub_;
  ros::Publisher speed_mode_pub_;
  ros::Publisher turn_signal_pub_;
  ros::Publisher gear_pub_;
  ros::Publisher current_twist_pub_;
  ros::Publisher current_speed_pub_;

  // ros param
  double loop_rate_;
  double acceleration_limit_;
  double deceleration_limit_;
  double max_curvature_rate_;

  // variables
  bool control_mode_;
  double desired_speed_;
  double desired_curvature_;
  autoware_msgs::LampCmd lamp_cmd_;
  ros::Rate* rate_;

  // callbacks
  void callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr& msg);
  void callbackFromControlMode(const std_msgs::BoolConstPtr& msg);
  void callbackFromSyncedCurrentTwist(const automotive_platform_msgs::VelocityAccelConstPtr& msg_velocity,
                                      const automotive_platform_msgs::CurvatureFeedbackConstPtr& msg_curvature);
  void callbackFromLampCmd(const autoware_msgs::LampCmdConstPtr& msg);

  // functions
  void publishCommand();
};

#endif  // SSC_INTERFACE_H
