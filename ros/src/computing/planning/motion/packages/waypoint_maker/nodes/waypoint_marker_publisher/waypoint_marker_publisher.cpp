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

#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"
#include "runtime_manager/traffic_light.h"

static ros::Publisher _lane_mark_pub;

static constexpr int32_t TRAFFIC_LIGHT_RED     = 0;
static constexpr int32_t TRAFFIC_LIGHT_GREEN   = 1;
static constexpr int32_t TRAFFIC_LIGHT_UNKNOWN = 2;

static std_msgs::ColorRGBA _initial_color;
static std_msgs::ColorRGBA _global_color;
static std_msgs::ColorRGBA g_local_color;
static const double g_global_alpha = 0.2;
static const double g_local_alpha = 1.0;
static int _closest_waypoint = -1;

void createGlobalWaypointVelocityMarker(const waypoint_follower::lane &lane_waypoint, visualization_msgs::MarkerArray *marker_array)
{

  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity;
  velocity.header.frame_id = "map";
  velocity.header.stamp = ros::Time();
  velocity.ns = "global_waypoint_velocity";
  velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity.action = visualization_msgs::Marker::ADD;
  velocity.scale.z = 0.4;
  velocity.color.a = 1.0;
  velocity.color.r = 1;
  velocity.color.g = 1;
  velocity.color.b = 1;
  velocity.frame_locked = true;

  for ( int i = 0; i < static_cast<int>(lane_waypoint.waypoints.size()); i++)
  {

    //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
    velocity.id = i;
    double yaw = 0;
    if(i == static_cast<int>(lane_waypoint.waypoints.size()) -1){
      yaw = atan2(lane_waypoint.waypoints[i -1].pose.pose.position.y - lane_waypoint.waypoints[i].pose.pose.position.y,
          lane_waypoint.waypoints[i -1].pose.pose.position.x - lane_waypoint.waypoints[i].pose.pose.position.x);
      yaw -= M_PI;
    }
    else
      yaw = atan2(lane_waypoint.waypoints[i + 1].pose.pose.position.y - lane_waypoint.waypoints[i].pose.pose.position.y,
          lane_waypoint.waypoints[i + 1].pose.pose.position.x - lane_waypoint.waypoints[i].pose.pose.position.x);

    geometry_msgs::Pose waypoint_pose;
    waypoint_pose.position = lane_waypoint.waypoints[i].pose.pose.position;
    waypoint_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    geometry_msgs::Point relative_p;
    relative_p.y = 0.65;
    velocity.pose.position = calcAbsoluteCoordinate(relative_p,waypoint_pose);
    velocity.pose.position.z += 0.2;

    // double to string
    std::ostringstream oss;
   // oss << std::fixed << std::setprecision(2) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x) << " km/h";
    oss << std::fixed << std::setprecision(1) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x) ;
    velocity.text = oss.str();

    //C++11 version
    //std::string velocity = std::to_string(test_pose.velocity_kmh);
    //velocity.erase(velocity.find_first_of(".") + 3);
    //std::string kmh = " km/h";
    //std::string text = velocity + kmh;
    //marker.text = text;

    marker_array->markers.push_back(velocity);
  }
}

void createLocalWaypointVelocityMarker(int closest_waypoint,const waypoint_follower::lane &lane_waypoint, visualization_msgs::MarkerArray *marker_array)
{

  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity;
  velocity.header.frame_id = "map";
  velocity.header.stamp = ros::Time();
  velocity.ns = "local_waypoint_velocity";
  velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity.action = visualization_msgs::Marker::ADD;
  velocity.scale.z = 0.4;
  velocity.color.a = 1.0;
  velocity.color.r = 1;
  velocity.color.g = 0;
  velocity.color.b = 0;
  velocity.frame_locked = true;

  for (int i = 0; i < static_cast<int>(lane_waypoint.waypoints.size()); i++)
  {
    velocity.id = closest_waypoint+i;
    double yaw = 0;
        if(i == static_cast<int>(lane_waypoint.waypoints.size()) -1){
          yaw = atan2(lane_waypoint.waypoints[i -1].pose.pose.position.y - lane_waypoint.waypoints[i].pose.pose.position.y,
              lane_waypoint.waypoints[i -1].pose.pose.position.x - lane_waypoint.waypoints[i].pose.pose.position.x);
          yaw -= M_PI;
        }
        else
          yaw = atan2(lane_waypoint.waypoints[i + 1].pose.pose.position.y - lane_waypoint.waypoints[i].pose.pose.position.y,
              lane_waypoint.waypoints[i + 1].pose.pose.position.x - lane_waypoint.waypoints[i].pose.pose.position.x);

        geometry_msgs::Pose waypoint_pose;
        waypoint_pose.position = lane_waypoint.waypoints[i].pose.pose.position;
        waypoint_pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

        geometry_msgs::Point relative_p;
        relative_p.y = - 0.65;
        velocity.pose.position = calcAbsoluteCoordinate(relative_p,waypoint_pose);
    velocity.pose.position.z += 0.2;

    // double to string
    std::ostringstream oss;
   // oss << std::fixed << std::setprecision(2) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x) << " km/h";
    oss << std::fixed << std::setprecision(1) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x) ;
    velocity.text = oss.str();

    //C++11 version
    //std::string velocity = std::to_string(test_pose.velocity_kmh);
    //velocity.erase(velocity.find_first_of(".") + 3);
    //std::string kmh = " km/h";
    //std::string text = velocity + kmh;
    //marker.text = text;

    marker_array->markers.push_back(velocity);
  }

  for (int i = closest_waypoint -1 ; i > -1 ; i--)
    {
      velocity.id = i;
      velocity.action = visualization_msgs::Marker::DELETE;
      marker_array->markers.push_back(velocity);
    }

}

void createGlobalWaypointMarker(waypoint_follower::lane lane_waypoint,visualization_msgs::MarkerArray *marker_array)
{
  visualization_msgs::Marker waypoint_mark;
  waypoint_mark.header.frame_id =  "map";
  waypoint_mark.header.stamp = ros::Time();
  waypoint_mark.ns = "waypoint_marker";
  waypoint_mark.id = 0;
  waypoint_mark.type = visualization_msgs::Marker::POINTS;
  waypoint_mark.action = visualization_msgs::Marker::ADD;
  waypoint_mark.scale.x = 0.1;
  waypoint_mark.scale.y = 0.1;
  waypoint_mark.color.r = 1.0;
  waypoint_mark.color.g = 1.0;
  waypoint_mark.color.b = 1.0;
  waypoint_mark.color.a = 1.0;
  waypoint_mark.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.waypoints.size(); i++)
  {
    geometry_msgs::Point point;
    point = lane_waypoint.waypoints[i].pose.pose.position;
    waypoint_mark.points.push_back(point);

  }
  marker_array->markers.push_back(waypoint_mark);
}

void createGlobalPathMarker(std_msgs::ColorRGBA color , const waypoint_follower::lane &lane_waypoint,visualization_msgs::MarkerArray *marker_array)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "global_path_marker";
  lane_waypoint_marker.id = 0;
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 2.0;
  lane_waypoint_marker.color = color;
  lane_waypoint_marker.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.waypoints.size(); i++)
    {
      geometry_msgs::Point point;
      point = lane_waypoint.waypoints[i].pose.pose.position;
      lane_waypoint_marker.points.push_back(point);

    }
  marker_array->markers.push_back(lane_waypoint_marker);
}

void createLocalPathMarker(std_msgs::ColorRGBA color , const waypoint_follower::lane &lane_waypoint,visualization_msgs::MarkerArray *marker_array)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "local_path_marker";
  lane_waypoint_marker.id = 0;
  lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
  lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
  lane_waypoint_marker.scale.x = 0.2;
  lane_waypoint_marker.color = color;
  lane_waypoint_marker.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.waypoints.size(); i++)
    {
      geometry_msgs::Point point;
      point = lane_waypoint.waypoints[i].pose.pose.position;
      lane_waypoint_marker.points.push_back(point);

    }
  marker_array->markers.push_back(lane_waypoint_marker);
}

static void laneCallback(const waypoint_follower::laneConstPtr &msg)
{
  visualization_msgs::MarkerArray marker_array;
  createGlobalWaypointVelocityMarker(*msg, &marker_array);
  //createGlobalWaypointMarker(*msg, &marker_array);
  createGlobalPathMarker(_global_color, *msg, &marker_array);

  _lane_mark_pub.publish(marker_array);
}

static void lightCallback(const runtime_manager::traffic_lightConstPtr& msg)
{
  std_msgs::ColorRGBA global_color;
  global_color.a = g_global_alpha;

  std_msgs::ColorRGBA local_color;
  local_color.a = g_local_alpha;

  switch (msg->traffic_light)
  {
  case TRAFFIC_LIGHT_RED:
    global_color.r = 1.0;
    _global_color = global_color;
    local_color.r = 1.0;
    g_local_color = local_color;
    break;
  case TRAFFIC_LIGHT_GREEN:
    global_color.g = 1.0;
    _global_color = global_color;
    local_color.g = 1.0;
    g_local_color = local_color;
    break;
  case TRAFFIC_LIGHT_UNKNOWN:
    _global_color = _initial_color;
    g_local_color = _initial_color;
    break;
  default:
    ROS_ERROR("unknown traffic_light");
    return;
  }
}

static void trafficCallback(const waypoint_follower::laneConstPtr &msg)
{
  visualization_msgs::MarkerArray marker_array;
  createGlobalWaypointVelocityMarker(*msg, &marker_array);
  //createGlobalWaypointMarker(*msg, &marker_array);
  createGlobalPathMarker(_global_color, *msg, &marker_array);

  _lane_mark_pub.publish(marker_array);

}

static void temporalCallback(const waypoint_follower::laneConstPtr &msg)
{
  visualization_msgs::MarkerArray marker_array;
  if(_closest_waypoint != -1)
    createLocalWaypointVelocityMarker(_closest_waypoint,*msg, &marker_array);
  //createLocalWaypointMarker(*msg, &marker_array);
  createLocalPathMarker(g_local_color, *msg, &marker_array);

  _lane_mark_pub.publish(marker_array);

}

static void closestCallback(const std_msgs::Int32ConstPtr &msg)
{
  _closest_waypoint = msg->data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoints_marker_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  //subscribe traffic light
  ros::Subscriber light_sub = nh.subscribe("traffic_light",10,lightCallback);

  //subscribe global waypoints
  ros::Subscriber lane_sub = nh.subscribe("lane_waypoints",10,laneCallback);
  ros::Subscriber traffic_sub = nh.subscribe("traffic_waypoints",10,trafficCallback);

  //subscribe local waypoints
  ros::Subscriber temporal_sub = nh.subscribe("temporal_waypoints",10,temporalCallback);
  ros::Subscriber closest_sub = nh.subscribe("closest_waypoint",10,closestCallback);

  _lane_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("lane_waypoint_mark", 10, true);

  //initialize path color
  _initial_color.b = 1.0;
  _initial_color.g = 0.7;
  _global_color = _initial_color;
  _global_color.a = g_global_alpha;
  g_local_color = _initial_color;
  g_local_color.a = g_local_alpha;

  ros::spin();

}
