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

#include "pacmod_interface.h"

namespace pacmod
{
// Constructor
PacmodInterface::PacmodInterface() :
    private_nh_("~"),
    control_mode_(false)
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

  // setup subscriber
  twist_cmd_sub_    = nh_.subscribe("twist_cmd", 10, &PacmodInterface::callbackFromTwistCmd, this);
  control_mode_sub_ = nh_.subscribe("/as/control_mode", 10, &PacmodInterface::callbackFromControlMode, this);
  speed_sub_        = nh_.subscribe("/vehicle/steering_report", 10, &PacmodInterface::callbackFromSteeringReport, this);

  // setup publisher
  steer_mode_pub_    = nh_.advertise<automotive_platform_msgs::SteerMode>("/as/arbitrated_steering_commands", 10);
  speed_mode_pub_    = nh_.advertise<automotive_platform_msgs::SpeedMode>("/as/arbitrated_speed_commands", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("as_current_twist", 10);
}

void PacmodInterface::run()
{
  ros::spin();
}


void PacmodInterface::callbackFromTwistCmd(const geometry_msgs::TwistStampedConstPtr &msg)
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

  automotive_platform_msgs::SpeedMode speed_mode;
  speed_mode.header = msg->header;
  speed_mode.mode = mode;
  speed_mode.speed = msg->twist.linear.x;
  speed_mode.acceleration_limit = 3.0;
  speed_mode.deceleration_limit = 3.0;

  automotive_platform_msgs::SteerMode steer_mode;
  steer_mode.header = msg->header;
  steer_mode.mode = mode;
  double curvature = msg->twist.angular.z / msg->twist.linear.x;
  steer_mode.curvature = msg->twist.linear.x <= 0 ? 0 : curvature;
  steer_mode.max_curvature_rate = 0.75;

  std::cout << "mode: "  << mode << std::endl;
  std::cout << "speed: " << speed_mode.speed << std::endl;
  std::cout << "steer: " << steer_mode.curvature << std::endl;

  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
}

void PacmodInterface::callbackFromControlMode(const std_msgs::BoolConstPtr &msg)
{
  control_mode_ = msg->data;
}

void PacmodInterface::callbackFromSteeringReport(const dbw_mkz_msgs::SteeringReportConstPtr &msg)
{
  geometry_msgs::TwistStamped ts;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  ts.header = header;
  ts.twist.linear.x = msg->speed; // [m/sec]
  // Can we get angular velocity?

  current_twist_pub_.publish(ts);
}

}  // pacmod
