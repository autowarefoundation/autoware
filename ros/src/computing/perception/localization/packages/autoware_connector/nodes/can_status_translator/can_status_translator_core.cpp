/*
 *  Copyright (c) 2018, TierIV Inc.
 *  Copyright (c) 2015-2018, Nagoya University
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
  sub1_ = nh_.subscribe("can_info", 100, &CanStatusTranslatorNode::callbackFromCanInfo, this);
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

void CanStatusTranslatorNode::publishVehicleStatus(const autoware_msgs::CanInfoConstPtr& msg)
{
  // currently, this function is only support to autoware_socket format.
  autoware_msgs::VehicleStatus vs;
  vs.header = msg->header;
  vs.tm = msg->tm;
  vs.drivemode = msg->devmode;  // I think devmode is typo in CanInfo...
  vs.steeringmode = msg->strmode;
  vs.gearshift = msg->driveshift;
  vs.speed = msg->speed;
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

void CanStatusTranslatorNode::callbackFromCanInfo(const autoware_msgs::CanInfoConstPtr& msg)
{
  publishVehicleStatus(msg);
}

}  // autoware_connector
