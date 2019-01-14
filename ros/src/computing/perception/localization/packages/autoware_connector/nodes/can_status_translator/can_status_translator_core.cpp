/*
 * Copyright 2015-2018 Autoware Foundation. All rights reserved.
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

#include "can_status_translator_core.h"

namespace autoware_connector
{
// Constructor
CanStatusTranslatorNode::CanStatusTranslatorNode() : private_nh_("~"), v_info_()
{
  initForROS();
}

// Destructor
CanStatusTranslatorNode::~CanStatusTranslatorNode()
{
}

void CanStatusTranslatorNode::initForROS()
{
  // ros parameter settings
  if (!nh_.hasParam("/vehicle_info/wheel_base") || !nh_.hasParam("/vehicle_info/minimum_turning_radius") ||
      !nh_.hasParam("/vehicle_info/maximum_steering_angle"))
  {
    v_info_.is_stored = false;
    ROS_INFO("vehicle_info is not set");
  }
  else
  {
    private_nh_.getParam("/vehicle_info/wheel_base", v_info_.wheel_base);
    // ROS_INFO_STREAM("wheel_base : " << wheel_base);

    private_nh_.getParam("/vehicle_info/minimum_turning_radius", v_info_.minimum_turning_radius);
    // ROS_INFO_STREAM("minimum_turning_radius : " << minimum_turning_radius);

    private_nh_.getParam("/vehicle_info/maximum_steering_angle", v_info_.maximum_steering_angle);  //[degree]:
    // ROS_INFO_STREAM("maximum_steering_angle : " << maximum_steering_angle);

    v_info_.is_stored = true;
  }
  // setup subscriber
  sub1_ = nh_.subscribe("can_info", 100, &CanStatusTranslatorNode::callbackFromCANInfo, this);
  sub2_ = nh_.subscribe("vehicle_status", 10, &CanStatusTranslatorNode::callbackFromVehicleStatus, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("can_velocity", 10);
  pub2_ = nh_.advertise<std_msgs::Float32>("linear_velocity_viz", 10);
  pub3_ = nh_.advertise<autoware_msgs::VehicleStatus>("vehicle_status", 10);
}

void CanStatusTranslatorNode::run()
{
  ros::spin();
}

void CanStatusTranslatorNode::publishVelocity(const autoware_msgs::VehicleStatusConstPtr& msg)
{
  geometry_msgs::TwistStamped tw;
  tw.header = msg->header;

  // linear velocity
  tw.twist.linear.x = kmph2mps(msg->speed);  // km/h -> m/s

  // angular velocity
  tw.twist.angular.z = v_info_.convertSteeringAngleToAngularVelocity(kmph2mps(msg->speed), msg->angle);

  pub1_.publish(tw);
}

void CanStatusTranslatorNode::publishVelocityViz(const autoware_msgs::VehicleStatusConstPtr& msg)
{
  std_msgs::Float32 fl;
  fl.data = msg->speed;
  pub2_.publish(fl);
}

void CanStatusTranslatorNode::publishVehicleStatus(const autoware_can_msgs::CANInfoConstPtr& msg)
{
  // currently, this function is only support to autoware_socket format.
  autoware_msgs::VehicleStatus vs;
  vs.header = msg->header;
  vs.tm = msg->tm;
  vs.drivemode = msg->devmode;  // I think devmode is typo in CANInfo...
  vs.steeringmode = msg->strmode;
  vs.gearshift = msg->driveshift;

  vs.speed = msg->speed;
  if(vs.gearshift == static_cast<int>(GearShift::Reverse)) {
      vs.speed *= -1.0;
  }

  vs.drivepedal = msg->drivepedal;
  vs.brakepedal = msg->brakepedal;
  vs.angle = msg->angle;
  vs.lamp = 0;
  vs.light = msg->light;

  pub3_.publish(vs);
}

void CanStatusTranslatorNode::callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr& msg)
{
  publishVelocity(msg);
  publishVelocityViz(msg);
}

void CanStatusTranslatorNode::callbackFromCANInfo(const autoware_can_msgs::CANInfoConstPtr& msg)
{
  publishVehicleStatus(msg);
}

}  // autoware_connector
