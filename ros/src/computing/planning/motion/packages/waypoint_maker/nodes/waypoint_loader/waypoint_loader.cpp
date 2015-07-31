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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include "runtime_manager/ConfigWaypointLoader.h"
#include "waypoint_follower/lane.h"
#include "waypoint_follower/libwaypoint_follower.h"

#include "vmap_utility.hpp"

struct WP {
    geometry_msgs::Point point;
    double velocity_kmh;
};

std::string PATH_FRAME = "/map";
static const std::string LANE_WAYPOINT_CSV = "/tmp/lane_waypoint.csv";

static std::vector<WP> _waypoints;
static ros::Publisher _lane_mark_pub;
static ros::Publisher _lane_pub;

static WP parseWaypoint(const std::string& line)
{
    std::istringstream ss(line);
    std::vector<std::string> columns;

    std::string column;
    while (std::getline(ss, column, ',')) {
        columns.push_back(column);
    }

    WP waypoint;
    waypoint.point.x = std::stod(columns[0]);
    waypoint.point.y = std::stod(columns[1]);
    waypoint.point.z = std::stod(columns[2]);
    waypoint.velocity_kmh = std::stod(columns[3]);

    return waypoint;

}

static std::vector<WP> readWaypoint(const char *filename)
{
    std::ifstream ifs(filename);
    std::string line;

    std::getline(ifs, line); // Remove first line

    std::vector<WP> waypoints;
    while (std::getline(ifs, line)) {
      waypoints.push_back(parseWaypoint(line));

    }

    return waypoints;
}

double decelerate(tf::Vector3 v1 ,tf::Vector3 v2 , double original_velocity_kmh){

  double a = 0.5; //m/s^2
  double distance = tf::tfDistance(v1,v2);
  double vel = mps2kmph(sqrt(2 * a * distance)); //km/h
  if(vel < 1.0)
    vel = 0;
  if (vel > original_velocity_kmh)
  {
    vel = original_velocity_kmh;
  }
  return vel;
}

void createWaypointVelocity(visualization_msgs::MarkerArray *marker_array){

  // display by markers the velocity of each waypoint.
    visualization_msgs::Marker velocity;
    velocity.header.frame_id = PATH_FRAME;
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

    for (unsigned int i = 0; i < _waypoints.size(); i++) {

        //std::cout << _waypoints[i].GetX() << " " << _waypoints[i].GetY() << " " << _waypoints[i].GetZ() << " " << _waypoints[i].GetVelocity_kmh() << std::endl;
        velocity.id = i;
        velocity.pose.position = _waypoints[i].point;
        velocity.pose.position.z += 0.2;
        velocity.pose.orientation.w = 1.0;

        double vel = decelerate(point2vector(_waypoints[i].point),
              point2vector(_waypoints[_waypoints.size() - 1].point),_waypoints[i].velocity_kmh);

        // double to string
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << vel << " km/h";
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

void createWaypointMark(visualization_msgs::MarkerArray *marker_array)
{
    visualization_msgs::Marker waypoint_mark;
    waypoint_mark.header.frame_id = PATH_FRAME;
    waypoint_mark.header.stamp = ros::Time();
    waypoint_mark.ns = "waypoint_mark";
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

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
        geometry_msgs::Point point;
        point = _waypoints[i].point;
        waypoint_mark.points.push_back(point);

    }
    marker_array->markers.push_back(waypoint_mark);
}

void createLaneWaypointMarker(visualization_msgs::MarkerArray *marker_array)
{
    static visualization_msgs::Marker lane_waypoint_marker;
    lane_waypoint_marker.header.frame_id = PATH_FRAME;
    lane_waypoint_marker.header.stamp = ros::Time();
    lane_waypoint_marker.ns = "lane_waypoint_marker";
    lane_waypoint_marker.id = 0;
    lane_waypoint_marker.type = visualization_msgs::Marker::LINE_STRIP;
    lane_waypoint_marker.action = visualization_msgs::Marker::ADD;
    lane_waypoint_marker.scale.x = 0.2;
    lane_waypoint_marker.pose.orientation.w = 1.0;
    lane_waypoint_marker.color.b = 1.0;
    lane_waypoint_marker.color.g = 0.5;
    lane_waypoint_marker.color.a = 1.0;
    lane_waypoint_marker.frame_locked = true;

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
        geometry_msgs::Point point;
        point = _waypoints[i].point;
        lane_waypoint_marker.points.push_back(point);

    }
    marker_array->markers.push_back(lane_waypoint_marker);
}

void displayLaneWaypoint(){


     visualization_msgs::MarkerArray lane_waypoint;
     createWaypointVelocity(&lane_waypoint);
     createWaypointMark(&lane_waypoint);
     createLaneWaypointMarker(&lane_waypoint);
     _lane_mark_pub.publish(lane_waypoint);

}

void publishLaneWaypoint()
{
  waypoint_follower::lane lane_waypoint;
    lane_waypoint.header.frame_id = PATH_FRAME;
     lane_waypoint.header.stamp = ros::Time(0);

    for (unsigned int i = 0; i < _waypoints.size(); i++) {
      waypoint_follower::waypoint waypoint;

     waypoint.pose.header = lane_waypoint.header;
     waypoint.pose.pose.position = _waypoints[i].point;
     waypoint.pose.pose.orientation.w = 1.0;

     waypoint.twist.header = lane_waypoint.header;
     waypoint.twist.twist.linear.x = kmph2mps(_waypoints[i].velocity_kmh);

    lane_waypoint.waypoints.push_back(waypoint);
     }
     _lane_pub.publish(lane_waypoint);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypoint_loader");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string lane_waypoint_csv;

    private_nh.param<std::string>("lane_waypoint_csv",
                          lane_waypoint_csv, LANE_WAYPOINT_CSV);

    _lane_pub = nh.advertise<waypoint_follower::lane>("lane_waypoint", 10, true);
    _lane_mark_pub = nh.advertise<visualization_msgs::MarkerArray>("lane_waypoint_mark", 10, true);

    _waypoints = readWaypoint(lane_waypoint_csv.c_str());
    displayLaneWaypoint();
    publishLaneWaypoint();

    ros::spin();

}
