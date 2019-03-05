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

#include "can_odometry_core.h"

namespace autoware_connector
{
// Constructor
CanOdometryNode::CanOdometryNode() : private_nh_("~"), v_info_(), odom_(ros::Time::now())
{
  initForROS();
}

// Destructor
CanOdometryNode::~CanOdometryNode()
{
}

void CanOdometryNode::initForROS()
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
  sub1_ = nh_.subscribe("vehicle_status", 10, &CanOdometryNode::callbackFromVehicleStatus, this);

  // setup publisher
  pub1_ = nh_.advertise<nav_msgs::Odometry>("/vehicle/odom", 10);
}

void CanOdometryNode::run()
{
  ros::spin();
}

void CanOdometryNode::publishOdometry(const autoware_msgs::VehicleStatusConstPtr &msg)
{
  double vx = kmph2mps(msg->speed);
  double vth = v_info_.convertSteeringAngleToAngularVelocity(kmph2mps(msg->speed), msg->angle);
  odom_.updateOdometry(vx, vth, msg->header.stamp);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_.th);

  // first, we'll publish the transform over tf
  /*geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = msg->header.stamp;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odom_.x;
  odom_trans.transform.translation.y = odom_.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster_.sendTransform(odom_trans);
  */

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = odom_.x;
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.angular.z = vth;

  // publish the message
  pub1_.publish(odom);
}

void CanOdometryNode::callbackFromVehicleStatus(const autoware_msgs::VehicleStatusConstPtr &msg)
{
  publishOdometry(msg);
}

}  // autoware_connector
