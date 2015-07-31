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
static std_msgs::ColorRGBA _color;

void createWaypointVelocityMarker(waypoint_follower::lane lane_waypoint, visualization_msgs::MarkerArray *marker_array)
{

  // display by markers the velocity of each waypoint.
  visualization_msgs::Marker velocity;
  velocity.header.frame_id = "map";
  velocity.header.stamp = ros::Time();
  velocity.ns = "waypoint_velocity";
  velocity.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity.action = visualization_msgs::Marker::ADD;
  velocity.scale.z = 0.4;
  velocity.color.a = 1.0;
  velocity.color.r = 1;
  velocity.color.g = 1;
  velocity.color.b = 1;
  velocity.frame_locked = true;

  for (unsigned int i = 0; i < lane_waypoint.waypoints.size(); i++)
  {

    //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
    velocity.id = i;
    velocity.pose.position = lane_waypoint.waypoints[i].pose.pose.position;
    velocity.pose.position.z += 0.2;
    velocity.pose.orientation.w = 1.0;

    // double to string
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x) << " km/h";
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

void createWaypointMarker(waypoint_follower::lane lane_waypoint,visualization_msgs::MarkerArray *marker_array)
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

void createPathMarker(std_msgs::ColorRGBA color , waypoint_follower::lane lane_waypoint,visualization_msgs::MarkerArray *marker_array)
{
  visualization_msgs::Marker lane_waypoint_marker;
  lane_waypoint_marker.header.frame_id = "map";
  lane_waypoint_marker.header.stamp = ros::Time();
  lane_waypoint_marker.ns = "path_marker";
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
  createWaypointVelocityMarker(*msg, &marker_array);
  createWaypointMarker(*msg, &marker_array);
  createPathMarker(_color, *msg, &marker_array);

  _lane_mark_pub.publish(marker_array);
}

static void lightCallback(const runtime_manager::traffic_lightConstPtr& msg)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;

  switch (msg->traffic_light)
  {
  case TRAFFIC_LIGHT_RED:
    color.r = 1.0;
    _color = color;
    break;
  case TRAFFIC_LIGHT_GREEN:
    color.g = 1.0;
    _color = color;
    break;
  case TRAFFIC_LIGHT_UNKNOWN:
    _color = _initial_color;
    break;
  default:
    ROS_ERROR("unknown traffic_light");
    return;
  }
}

static void trafficCallback(const waypoint_follower::laneConstPtr &msg)
{
  visualization_msgs::MarkerArray marker_array;
  createWaypointVelocityMarker(*msg, &marker_array);
  createWaypointMarker(*msg, &marker_array);
  createPathMarker(_color, *msg, &marker_array);

  _lane_mark_pub.publish(marker_array);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber lane_sub = nh.subscribe("lane_waypoint",10,laneCallback);
  ros::Subscriber traffic_sub = nh.subscribe("traffic_waypoint",10,trafficCallback);
  ros::Subscriber light_sub = nh.subscribe("traffic_light",10,lightCallback);

  _lane_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("lane_waypoint_mark", 10, true);

  //initialize path color
  _initial_color.b = 1.0;
  _initial_color.g = 0.7;
  _initial_color.a = 1.0;
  _color = _initial_color;

  ros::spin();

}
