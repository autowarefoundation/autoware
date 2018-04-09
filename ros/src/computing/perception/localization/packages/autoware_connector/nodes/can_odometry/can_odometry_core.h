/*
// *  Copyright (c) 2015, Nagoya University
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

#ifndef CAN_ODOMETRY_CORE_H
#define CAN_ODOMETRY_CORE_H

// ROS includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>

// User Defined Includes
#include "autoware_msgs/CanInfo.h"
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

struct Odometry
{
  double x;
  double y;
  double th;
  ros::Time stamp;

  Odometry(const ros::Time &time)
  {
    x = 0.0;
    y = 0.0;
    th = 0.0;
    stamp = time;
  }

  void updateOdometry(const double vx, const double vth, const ros::Time &cur_time)
  {
    if (stamp.sec == 0 && stamp.nsec == 0)
    {
      stamp = cur_time;
    }
    double dt = (cur_time - stamp).toSec();
    double delta_x = (vx * cos(th)) * dt;
    double delta_y = (vx * sin(th)) * dt;
    double delta_th = vth * dt;

    ROS_INFO("dt : %f delta (x y th) : (%f %f %f %f)", dt, delta_x, delta_y, delta_th);

    x += delta_x;
    y += delta_y;
    th += delta_th;
    stamp = cur_time;
  }
};

class CanOdometryNode
{
public:
  CanOdometryNode();
  ~CanOdometryNode();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher pub1_;

  // subscriber
  ros::Subscriber sub1_;

  // variables
  VehicleInfo v_info_;
  Odometry odom_;

  // callbacks
  void callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr &msg);

  // initializer
  void initForROS();

  // functions
  void publishOdometry(const autoware_msgs::VehicleStatusConstPtr &msg);
};
}
#endif  // CAN_ODOMETRY_CORE_H
