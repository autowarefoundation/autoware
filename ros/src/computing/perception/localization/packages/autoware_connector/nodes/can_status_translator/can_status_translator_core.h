/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef CAN_STATUS_TRANSLATOR_CORE_H
#define CAN_STATUS_TRANSLATOR_CORE_H

// ROS includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

// User Defined Includes
#include "autoware_can_msgs/CANInfo.h"
#include "autoware_msgs/VehicleStatus.h"

namespace autoware_connector
{
inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}

inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

// convert degree to radian
inline double deg2rad(double deg)
{
  return deg * M_PI / 180;
}

// convert degree to radian
inline double rad2deg(double rad)
{
  return rad * 180 / M_PI;
}
struct VehicleInfo
{
  bool is_stored;
  double wheel_base;
  double minimum_turning_radius;
  double maximum_steering_angle;

  VehicleInfo()
  {
    is_stored = false;
    wheel_base = 0.0;
    minimum_turning_radius = 0.0;
    maximum_steering_angle = 0.0;
  }
  double convertSteeringAngleToAngularVelocity(const double cur_vel_mps, const double cur_angle_deg)  // rad/s
  {
    return is_stored ? tan(deg2rad(getCurrentTireAngle(cur_angle_deg))) * cur_vel_mps / wheel_base : 0;
  }
  double getCurrentTireAngle(const double angle_deg)  // steering [degree] -> tire [degree]
  {
    return is_stored ? angle_deg * getMaximumTireAngle() / maximum_steering_angle : 0;
  }
  double getMaximumTireAngle()  // degree
  {
    return is_stored ? rad2deg(asin(wheel_base / minimum_turning_radius)) : 0;
  }
};

class CanStatusTranslatorNode
{
    enum class GearShift{
        Drive = 16,
        Neutral = 32,
        Reverse = 64,
        Parking = 128,
    };

public:
  CanStatusTranslatorNode();
  ~CanStatusTranslatorNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_, pub2_, pub3_;

  // subscriber
  ros::Subscriber sub1_, sub2_;

  // variables
  VehicleInfo v_info_;

  // callbacks
  void callbackFromCANInfo(const autoware_can_msgs::CANInfoConstPtr& msg);
  void callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publishVelocity(const autoware_msgs::VehicleStatusConstPtr& msg);
  void publishVelocityViz(const autoware_msgs::VehicleStatusConstPtr& msg);
  void publishVehicleStatus(const autoware_can_msgs::CANInfoConstPtr& msg);
};
}
#endif  // CAN_STATUS_TRANSLATOR_CORE_H
