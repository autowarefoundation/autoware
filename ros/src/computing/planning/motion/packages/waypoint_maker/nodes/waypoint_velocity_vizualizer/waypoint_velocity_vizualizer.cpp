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
#include <ros/console.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <vector>
#include <string>

#include "waypoint_follower/libwaypoint_follower.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/ConfigLaneStop.h"
#include "autoware_msgs/traffic_light.h"

namespace
{
ros::Publisher g_local_mark_pub;
ros::Publisher g_global_mark_pub;


std_msgs::ColorRGBA _initial_color;
std_msgs::ColorRGBA _global_color;
std_msgs::ColorRGBA g_local_color;
const double g_global_alpha = 0.2;
const double g_local_alpha = 1.0;
int _closest_waypoint = -1;

bool g_current_pose_subscribed = false;
bool g_current_vel_subscribed = false;
bool g_command_vel_subscribed = false;
geometry_msgs::PoseStamped g_current_pose;
geometry_msgs::TwistStamped g_current_vel, g_command_vel;
autoware_msgs::lane g_current_vel_array, g_command_vel_array;

visualization_msgs::MarkerArray g_global_marker_array;
visualization_msgs::MarkerArray g_local_waypoints_marker_array;

bool g_config_manual_detection = true;
bool g_use_velocity_visualizer;
double g_graph_height_ratio;
std::vector<double> g_local_vel_graph_color = { 0.0, 1.0, 0.0, 0.5 };
std::vector<double> g_current_vel_graph_color = { 1.0, 0.0, 0.0, 0.5 };
std::vector<double> g_command_vel_graph_color = { 0.0, 0.0, 1.0, 0.5 };

void publishLocalMarker()
{
  visualization_msgs::MarkerArray marker_array;

  // insert local marker
  marker_array.markers.insert(marker_array.markers.end(), g_local_waypoints_marker_array.markers.begin(),
                              g_local_waypoints_marker_array.markers.end());

  g_local_mark_pub.publish(marker_array);
}

void createGlobalLaneArrayVelocityMarker(const autoware_msgs::LaneArray& lane_waypoints_array)
{
  visualization_msgs::MarkerArray tmp_marker_array;
  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity_marker;
  velocity_marker.header.frame_id = "map";
  velocity_marker.header.stamp = ros::Time::now();
  velocity_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity_marker.action = visualization_msgs::Marker::ADD;
  velocity_marker.scale.z = 0.4;
  velocity_marker.color.r = 1;
  velocity_marker.color.g = 1;
  velocity_marker.color.b = 1;
  velocity_marker.color.a = 1.0;
  velocity_marker.frame_locked = true;

  int count = 1;
  for (auto lane : lane_waypoints_array.lanes)
  {
    velocity_marker.ns = "global_velocity_lane_" + std::to_string(count);
    for (int i = 0; i < static_cast<int>(lane.waypoints.size()); i++)
    {
      velocity_marker.id = i;
      geometry_msgs::Point relative_p;
      relative_p.y = 0.5;
      velocity_marker.pose.position = calcAbsoluteCoordinate(relative_p, lane.waypoints[i].pose.pose);
      velocity_marker.pose.position.z += 0.2;

      // double to string
      std::string vel = std::to_string(mps2kmph(lane.waypoints[i].twist.twist.linear.x));
      velocity_marker.text = vel.erase(vel.find_first_of(".") + 2);

      tmp_marker_array.markers.push_back(velocity_marker);
    }
    count++;
  }

  g_global_marker_array.markers.insert(g_global_marker_array.markers.end(), tmp_marker_array.markers.begin(),
                                       tmp_marker_array.markers.end());
}

void createLocalWaypointVelocityMarker(std_msgs::ColorRGBA color, int closest_waypoint,
                                       const autoware_msgs::lane& lane_waypoint)
{
  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity;
  velocity.header.frame_id = "map";
  velocity.header.stamp = ros::Time::now();
  velocity.ns = "local_waypoint_velocity";
  velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity.action = visualization_msgs::Marker::ADD;
  velocity.scale.z = 0.4;
  velocity.color = color;
  velocity.frame_locked = true;

  for (int i = 0; i < static_cast<int>(lane_waypoint.waypoints.size()); i++)
  {
    velocity.id = i;
    geometry_msgs::Point relative_p;
    relative_p.y = -0.65;
    velocity.pose.position = calcAbsoluteCoordinate(relative_p, lane_waypoint.waypoints[i].pose.pose);
    velocity.pose.position.z += 0.2;

    // double to string
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x);
    velocity.text = oss.str();

    g_local_waypoints_marker_array.markers.push_back(velocity);
  }
}

void createVelocityBarGraphMarker(const autoware_msgs::lane& lane_waypoint, const std::string str_name_space, const std::vector<double>& flat_color)
{
  visualization_msgs::Marker velocity_bar_graph_marker;
  velocity_bar_graph_marker.header.frame_id = "map";
  velocity_bar_graph_marker.header.stamp = ros::Time::now();
  velocity_bar_graph_marker.ns = str_name_space + "_velocity_bar_graph_marker";
  velocity_bar_graph_marker.type = visualization_msgs::Marker::CYLINDER;
  velocity_bar_graph_marker.action = visualization_msgs::Marker::ADD;
  velocity_bar_graph_marker.scale.x = 0.2;
  velocity_bar_graph_marker.scale.y = 0.2;
  velocity_bar_graph_marker.color.r = flat_color[0];
  velocity_bar_graph_marker.color.g = flat_color[1];
  velocity_bar_graph_marker.color.b = flat_color[2];
  velocity_bar_graph_marker.color.a = flat_color[3];
  velocity_bar_graph_marker.frame_locked = true;

  unsigned int count = 0;
  for (auto el : lane_waypoint.waypoints)
  {
    double bar_graph_height = g_graph_height_ratio * el.twist.twist.linear.x;
    velocity_bar_graph_marker.id = count++;
    velocity_bar_graph_marker.pose = el.pose.pose;
    velocity_bar_graph_marker.pose.position.z += bar_graph_height / 2.0;
    // When the the cylinder height is 0 or less, a warning occurs in RViz.
    velocity_bar_graph_marker.scale.z = fabs(bar_graph_height) + 1e-6;
    g_local_waypoints_marker_array.markers.push_back(velocity_bar_graph_marker);
  }
}

void createVelocityLineGraphMarker(const autoware_msgs::lane& lane_waypoint, const std::string str_name_space, const std::vector<double>& flat_color)
{
  visualization_msgs::Marker velocity_line_graph_marker;
  velocity_line_graph_marker.header.frame_id = "map";
  velocity_line_graph_marker.header.stamp = ros::Time::now();
  velocity_line_graph_marker.ns = str_name_space + "_velocity_line_graph_marker";
  velocity_line_graph_marker.type = visualization_msgs::Marker::LINE_STRIP;
  velocity_line_graph_marker.action = visualization_msgs::Marker::ADD;
  velocity_line_graph_marker.scale.x = 0.25;
  velocity_line_graph_marker.color.r = flat_color[0];
  velocity_line_graph_marker.color.g = flat_color[1];
  velocity_line_graph_marker.color.b = flat_color[2];
  velocity_line_graph_marker.color.a = flat_color[3];
  velocity_line_graph_marker.frame_locked = true;

  for (auto el : lane_waypoint.waypoints)
  {
    geometry_msgs::Point point = el.pose.pose.position;
    point.z += g_graph_height_ratio * el.twist.twist.linear.x;
    velocity_line_graph_marker.points.push_back(point);
  }
  g_local_waypoints_marker_array.markers.push_back(velocity_line_graph_marker);
}

void laneArrayCallback(const autoware_msgs::LaneArrayConstPtr& msg)
{
  g_global_marker_array.markers.clear();
}

void finalCallback(const autoware_msgs::laneConstPtr& msg)
{
  g_local_waypoints_marker_array.markers.clear();
  createVelocityBarGraphMarker(*msg, "local", g_local_vel_graph_color);
  createVelocityLineGraphMarker(*msg, "local", g_local_vel_graph_color);
  createVelocityBarGraphMarker(g_current_vel_array, "current", g_current_vel_graph_color);
  createVelocityLineGraphMarker(g_current_vel_array, "current", g_current_vel_graph_color);
  createVelocityBarGraphMarker(g_command_vel_array, "command", g_command_vel_graph_color);
  createVelocityLineGraphMarker(g_command_vel_array, "command", g_command_vel_graph_color);
  publishLocalMarker();
}

const bool isSyncTimeOn10Hz()
{
  ros::Duration duration_diff[3];
  ros::Time cpose_stamp = g_current_pose.header.stamp;
  ros::Time cvel_stamp = g_current_vel.header.stamp;
  ros::Time pvel_stamp = g_command_vel.header.stamp;
  duration_diff[0] = cpose_stamp - cvel_stamp;
  duration_diff[1] = cpose_stamp - pvel_stamp;
  duration_diff[2] = cvel_stamp - pvel_stamp;
  for(int i=0;i<3;i++)
  {
    double diff_sec = fabs(1e-9 * duration_diff[i].nsec + duration_diff[i].sec);
    if(diff_sec >= 0.2)return false;//rejecting over 0.2sec delay
  }
  return true;
}

void pushVelocityArray()
{
  const unsigned int pose_and_velocity_array_limits = 100;
  std::vector<autoware_msgs::waypoint>& cwp_array = g_current_vel_array.waypoints;
  std::vector<autoware_msgs::waypoint>& pwp_array = g_command_vel_array.waypoints;

  autoware_msgs::waypoint current_wp, command_wp;
  current_wp.pose = command_wp.pose = g_current_pose;
  current_wp.twist = g_current_vel;
  command_wp.twist = g_command_vel;
  cwp_array.push_back(current_wp);
  pwp_array.push_back(command_wp);
  if(cwp_array.size() > pose_and_velocity_array_limits)cwp_array.erase(cwp_array.begin());
  if(pwp_array.size() > pose_and_velocity_array_limits)pwp_array.erase(pwp_array.begin());
}

void currentPoseCallback(const geometry_msgs::PoseStamped& pose)
{
  g_current_pose = pose;
  g_current_pose_subscribed = true;

  bool is_sync_pose_and_velocity = (g_current_pose_subscribed && g_current_vel_subscribed && g_command_vel_subscribed);
  if(!is_sync_pose_and_velocity)return;
  if(!isSyncTimeOn10Hz())return;
  pushVelocityArray();
  g_current_pose_subscribed = g_current_vel_subscribed = g_command_vel_subscribed =false;
}

void currentVelocityCallback(const geometry_msgs::TwistStamped& vel)
{
  g_current_vel = vel;
  g_current_vel_subscribed = true;

  bool is_sync_pose_and_velocity = (g_current_pose_subscribed && g_current_vel_subscribed && g_command_vel_subscribed);
  if(!is_sync_pose_and_velocity)return;
  if(!isSyncTimeOn10Hz())return;
  pushVelocityArray();
  g_current_pose_subscribed = g_current_vel_subscribed = g_command_vel_subscribed =false;
}

void commandVelocityCallback(const geometry_msgs::TwistStamped& vel)
{
  g_command_vel = vel;
  g_command_vel_subscribed = true;

  bool is_sync_pose_and_velocity = (g_current_pose_subscribed && g_current_vel_subscribed && g_command_vel_subscribed);
  if(!is_sync_pose_and_velocity)return;
  if(!isSyncTimeOn10Hz())return;
  pushVelocityArray();
  g_current_pose_subscribed = g_current_vel_subscribed = g_command_vel_subscribed =false;
}

void closestCallback(const std_msgs::Int32ConstPtr& msg)
{
  _closest_waypoint = msg->data;
}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_marker_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<bool>("use_velocity_visualizer", g_use_velocity_visualizer, false);
  private_nh.param<double>("graph_height_ratio", g_graph_height_ratio, 1.0);
  private_nh.param<std::vector<double> >("local_velocity_graph_color", g_local_vel_graph_color, g_local_vel_graph_color);
  private_nh.param<std::vector<double> >("current_velocity_graph_color", g_current_vel_graph_color, g_current_vel_graph_color);
  private_nh.param<std::vector<double> >("command_velocity_graph_color", g_command_vel_graph_color, g_command_vel_graph_color);

  // subscribe local waypoints
  ros::Subscriber final_sub = nh.subscribe("final_waypoints", 10, finalCallback);
  ros::Subscriber closest_sub = nh.subscribe("closest_waypoint", 10, closestCallback);

  // subscribe current pose & velocity
  ros::Subscriber current_pose_sub = nh.subscribe("current_pose", 10, currentPoseCallback);
  ros::Subscriber current_vel_sub = nh.subscribe("current_velocity", 10, currentVelocityCallback);

  // subscribe velocity command
  ros::Subscriber command_vel_sub = nh.subscribe("twist_cmd", 10, commandVelocityCallback);

  g_local_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("local_waypoints_velocity", 10, true);
  g_global_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("global_waypoints_velocity", 10, true);

  // initialize path color
  _initial_color.g = 0.7;
  _initial_color.b = 1.0;
  _global_color = _initial_color;
  _global_color.a = g_global_alpha;
  g_local_color = _initial_color;
  g_local_color.a = g_local_alpha;

  ros::spin();
}
