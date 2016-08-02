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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>

#include "waypoint_follower/libwaypoint_follower.h"

namespace
{
geometry_msgs::Twist _current_velocity;

const std::string SIMULATION_FRAME = "sim_base_link";
const std::string MAP_FRAME = "map";

geometry_msgs::Pose _initial_pose;
bool _initial_set = false;
bool _pose_set = false;
bool _waypoint_set = false;
WayPoints _current_waypoints;
ros::Publisher g_odometry_publisher;
ros::Publisher g_velocity_publisher;

void CmdCallBack(const geometry_msgs::TwistStampedConstPtr &msg)
{
  _current_velocity = msg->twist;
}

void getTransformFromTF(const std::string parent_frame, const std::string child_frame, tf::StampedTransform &transform)
{
  static tf::TransformListener listener;

  while (1)
  {
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      break;
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
  }
}

void initialposeCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &input)
{
  tf::StampedTransform transform;
  getTransformFromTF(MAP_FRAME, "world", transform);

  _initial_pose.position.x = input->pose.pose.position.x + transform.getOrigin().x();
  _initial_pose.position.y = input->pose.pose.position.y + transform.getOrigin().y();
  _initial_pose.position.z = input->pose.pose.position.z + transform.getOrigin().z();
  _initial_pose.orientation = input->pose.pose.orientation;

  _initial_set = true;
  _pose_set = false;
}

void callbackFromPoseStamped(const geometry_msgs::PoseStampedConstPtr &msg)
{
  _initial_pose = msg->pose;
  _initial_set = true;
}

void waypointCallback(const waypoint_follower::laneConstPtr &msg)
{
  // _path_og.setPath(msg);
  _current_waypoints.setPath(*msg);
  _waypoint_set = true;
  ROS_INFO_STREAM("waypoint subscribed");
}

void publishOdometry()
{
  static ros::Time current_time = ros::Time::now();
  static ros::Time last_time = ros::Time::now();
  static geometry_msgs::Pose pose;
  static double th = 0;
  static tf::TransformBroadcaster odom_broadcaster;

  if (!_pose_set)
  {
    pose.position = _initial_pose.position;
    pose.orientation = _initial_pose.orientation;
    th = tf::getYaw(pose.orientation);
    ROS_INFO_STREAM("pose set : (" << pose.position.x << " " << pose.position.y << " " << pose.position.z << " " << th
                                   << ")");
    _pose_set = true;
  }

  int closest_waypoint = getClosestWaypoint(_current_waypoints.getCurrentWaypoints(), pose);
  if (closest_waypoint == -1)
  {
    ROS_INFO("cannot publish odometry because closest waypoint is -1.");
    return;
  }
  else
  {
    pose.position.z = _current_waypoints.getWaypointPosition(closest_waypoint).z;
  }

  double vx = _current_velocity.linear.x;
  double vth = _current_velocity.angular.z;
  current_time = ros::Time::now();

  // compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th)) * dt;
  double delta_y = (vx * sin(th)) * dt;
  double delta_th = vth * dt;

  pose.position.x += delta_x;
  pose.position.y += delta_y;
  th += delta_th;
  pose.orientation = tf::createQuaternionMsgFromYaw(th);

  // std::cout << "delta (x y th) : (" << delta_x << " " << delta_y << " " << delta_th << ")" << std::endl;
  // std::cout << "current_velocity(linear.x angular.z) : (" << _current_velocity.linear.x << " " <<
  // _current_velocity.angular.z << ")"<< std::endl;
  //    std::cout << "current_pose : (" << pose.position.x << " " << pose.position.y<< " " << pose.position.z << " " <<
  //    th << ")" << std::endl << std::endl;

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = MAP_FRAME;
  odom_trans.child_frame_id = SIMULATION_FRAME;

  odom_trans.transform.translation.x = pose.position.x;
  odom_trans.transform.translation.y = pose.position.y;
  odom_trans.transform.translation.z = pose.position.z;
  odom_trans.transform.rotation = pose.orientation;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  std_msgs::Header h;
  h.stamp = current_time;
  h.frame_id = MAP_FRAME;

  geometry_msgs::PoseStamped ps;
  ps.header = h;
  ps.pose = pose;

  geometry_msgs::TwistStamped ts;
  ts.header = h;
  ts.twist.linear.x = vx;
  ts.twist.angular.z = vth;

  // publish the message
  g_odometry_publisher.publish(ps);
  g_velocity_publisher.publish(ts);

  last_time = current_time;
}
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "wf_simulator");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string initialize_source;
  private_nh.getParam("initialize_source", initialize_source);
  ROS_INFO_STREAM("initialize_source : " << initialize_source);

  // publish topic
  g_odometry_publisher = nh.advertise<geometry_msgs::PoseStamped>("sim_pose", 10);
  g_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("sim_velocity", 10);

  // subscribe topic
  ros::Subscriber cmd_subscriber = nh.subscribe("twist_cmd", 10, CmdCallBack);
  ros::Subscriber waypoint_subcscriber = nh.subscribe("base_waypoints", 10, waypointCallback);
  ros::Subscriber initialpose_subscriber;

  if (initialize_source == "Rviz")
  {
    initialpose_subscriber = nh.subscribe("initialpose", 10, initialposeCallback);
  }
  else if (initialize_source == "ndt_localizer")
  {
    initialpose_subscriber = nh.subscribe("ndt_pose", 10, callbackFromPoseStamped);
  }
  else if (initialize_source == "GNSS")
  {
    initialpose_subscriber = nh.subscribe("gnss_pose", 10, callbackFromPoseStamped);
  }
  else
  {
    ROS_INFO("Set pose initializer!!");
  }

  ros::Rate loop_rate(50);  // 50Hz
  while (ros::ok())
  {
    ros::spinOnce();  // check subscribe topic

    if (!_waypoint_set)
    {
      loop_rate.sleep();
      continue;
    }

    if (!_initial_set)
    {
      loop_rate.sleep();
      continue;
    }

    publishOdometry();

    loop_rate.sleep();
  }

  return 0;
}
