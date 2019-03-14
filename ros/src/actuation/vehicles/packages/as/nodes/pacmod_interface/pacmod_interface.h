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

#ifndef PACMOD_INTERFACE_H
#define PACMOD_INTERFACE_H

// ROS includes
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <automotive_platform_msgs/SteerMode.h>
#include <automotive_platform_msgs/SpeedMode.h>
#include <dbw_mkz_msgs/SteeringReport.h>

namespace pacmod
{
class PacmodInterface
{
public:
  PacmodInterface();
  ~PacmodInterface();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher steer_mode_pub_;
  ros::Publisher speed_mode_pub_;
  ros::Publisher current_twist_pub_;

  // subscriber
  ros::Subscriber twist_cmd_sub_;
  ros::Subscriber control_mode_sub_;
  ros::Subscriber speed_sub_;

  // ros param
  double acceleration_limit_;
  double deceleration_limit_;
  double max_curvature_rate_;

  // variables
  bool control_mode_;

  // callbacks
  void callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg);
  void callbackFromControlMode(const std_msgs::BoolConstPtr &msg);
  void callbackFromSteeringReport(const dbw_mkz_msgs::SteeringReportConstPtr &msg);

  // initializer
  void initForROS();
};
}  // pacmod
#endif  // PACMOD_INTERFACE_H
