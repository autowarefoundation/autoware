/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include "twist_gate.h"

class TwistGateTestClass
{
public:
  TwistGateTestClass()
  {
    twist_cmd_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 0);
    control_cmd_publisher = nh.advertise<autoware_msgs::ControlCommandStamped>("ctrl_cmd", 0);
    vehicle_cmd_subscriber = nh.subscribe("/vehicle_cmd", 1, &TwistGateTestClass::vehicleCmdCallback, this);
  }

  TwistGate *tg;
  autoware_msgs::VehicleCmd cb_vehicle_cmd;

  ros::NodeHandle nh;
  ros::Publisher twist_cmd_publisher;
  ros::Publisher control_cmd_publisher;
  ros::Subscriber vehicle_cmd_subscriber;

  void tgSpinOnce(){ tg->spinOnce(); }

  void publishTwistCmd(double linear_x, double angular_z)
  {
    geometry_msgs::TwistStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.twist.linear.x = linear_x;
    msg.twist.angular.z = angular_z;

    twist_cmd_publisher.publish(msg);
  }

  void publishControlCmd(double linear_vel, double linear_acc, double steer_angle)
  {
    autoware_msgs::ControlCommandStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.cmd.linear_velocity = linear_vel;
    msg.cmd.linear_acceleration = linear_acc;
    msg.cmd.steering_angle = steer_angle;

    control_cmd_publisher.publish(msg);
  }

  void vehicleCmdCallback(autoware_msgs::VehicleCmd msg)
  {
    cb_vehicle_cmd = msg;
  }
};
