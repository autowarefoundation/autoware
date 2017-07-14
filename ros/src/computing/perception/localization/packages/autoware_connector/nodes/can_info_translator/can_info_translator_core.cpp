/*
 *  Copyright (c) 2015, Nagoya University

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

#include "can_info_translator_core.h"

namespace autoware_connector
{
// Constructor
VelPoseConnectNode::VelPoseConnectNode() : private_nh_("~"), v_info_(), odom_(ros::Time::now())
{
  initForROS();
}

// Destructor
VelPoseConnectNode::~VelPoseConnectNode()
{
}

void VelPoseConnectNode::initForROS()
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
  sub1_ = nh_.subscribe("can_info", 100, &VelPoseConnectNode::callbackFromCanInfo, this);

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("can_velocity",10);
  pub2_ = nh_.advertise<std_msgs::Float32>("linear_velocity_viz", 10);
  pub3_ = nh_.advertise<nav_msgs::Odometry>("odom_pose",10);

}

void VelPoseConnectNode::run()
{
  ros::spin();
}

void VelPoseConnectNode::publishVelocity(const autoware_msgs::CanInfoConstPtr &msg)
{
  geometry_msgs::TwistStamped tw;
  tw.header = msg->header;

  // linear velocity
  tw.twist.linear.x = kmph2mps(msg->speed);  // km/h -> m/s

  // angular velocity
  if (v_info_.is_stored)
    tw.twist.angular.z = v_info_.convertSteeringAngleToAngularVelocity(kmph2mps(msg->speed), msg->angle);

  pub1_.publish(tw);
}

void VelPoseConnectNode::publishVelocityViz(const autoware_msgs::CanInfoConstPtr &msg)
{
  std_msgs::Float32 fl;
  fl.data = msg->speed;
  pub2_.publish(fl);
}

void VelPoseConnectNode::publishOdometry(const autoware_msgs::CanInfoConstPtr &msg)
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
  pub3_.publish(odom);
}

void VelPoseConnectNode::callbackFromCanInfo(const autoware_msgs::CanInfoConstPtr &msg)
{
  publishVelocity(msg);
  publishVelocityViz(msg);
  publishOdometry(msg);



}



} // autoware_connector
