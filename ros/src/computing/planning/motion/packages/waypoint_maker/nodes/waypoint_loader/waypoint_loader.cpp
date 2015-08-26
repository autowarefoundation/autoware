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
#include <geometry_msgs/Point.h>

#include <iostream>
#include <vector>
#include <string>
#include "waypoint_follower/libwaypoint_follower.h"

struct WP
{
  geometry_msgs::Point point;
  double velocity_kmh;
};


static const std::string LANE_WAYPOINT_CSV = "/tmp/lane_waypoint.csv";
static double _decelerate = 1.0;

static std::vector<WP> _waypoints;

static WP parseWaypoint(const std::string& line)
{
  std::istringstream ss(line);
  std::vector<std::string> columns;

  std::string column;
  while (std::getline(ss, column, ','))
  {
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
  while (std::getline(ifs, line))
  {
    waypoints.push_back(parseWaypoint(line));

  }

  return waypoints;
}

double decelerate(tf::Vector3 v1, tf::Vector3 v2, double original_velocity_kmh)
{

  double distance = tf::tfDistance(v1, v2);
  double vel = mps2kmph(sqrt(2 * _decelerate * distance)); //km/h
  if (vel < 1.0)
    vel = 0;
  if (vel > original_velocity_kmh)
  {
    vel = original_velocity_kmh;
  }
  return vel;
}

waypoint_follower::lane createLaneWaypoint(std::vector<WP> waypoints)
{
  waypoint_follower::lane lane_waypoint;
  lane_waypoint.header.frame_id = "/map";
  lane_waypoint.header.stamp = ros::Time(0);

  for (unsigned int i = 0; i < waypoints.size(); i++)
  {
    waypoint_follower::waypoint wp;

    wp.pose.header = lane_waypoint.header;
    wp.pose.pose.position = waypoints[i].point;
    wp.pose.pose.orientation.w = 1.0;

    wp.twist.header = lane_waypoint.header;
    double vel_kmh = decelerate(point2vector(waypoints[i].point),
        point2vector(waypoints[waypoints.size() -1 ].point),
        waypoints[i].velocity_kmh);
    wp.twist.twist.linear.x = kmph2mps(vel_kmh);

    lane_waypoint.waypoints.push_back(wp);
  }
  return lane_waypoint;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_loader");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string lane_waypoint_csv;

  private_nh.param<std::string>("lane_waypoint_csv", lane_waypoint_csv, LANE_WAYPOINT_CSV);
  private_nh.getParam("decelerate", _decelerate);
  ROS_INFO_STREAM("decelerate :" << _decelerate);

  ros::Publisher lane_pub = nh.advertise<waypoint_follower::lane>("lane_waypoint", 10, true);

  std::vector<WP> waypoints = readWaypoint(lane_waypoint_csv.c_str());
  waypoint_follower::lane lane_waypoint = createLaneWaypoint(waypoints);

   lane_pub.publish(lane_waypoint);

  ros::spin();

}
