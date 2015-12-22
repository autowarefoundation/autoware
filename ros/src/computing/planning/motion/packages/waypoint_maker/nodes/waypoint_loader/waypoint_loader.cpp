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
#include "waypoint_follower/LaneArray.h"

struct WP
{
  geometry_msgs::Pose pose;
  double velocity_kmh;
};


static const std::string DRIVING_LANE_CSV = "/tmp/driving_lane.csv";
static const std::string PASSING_LANE_CSV = "/tmp/passing_lane.csv";
static double _decelerate = 1.0;

static std::vector<WP> _waypoints;

static WP parseWaypoint(const std::string& line, bool yaw)
{
  std::istringstream ss(line);
  std::vector<std::string> columns;

  std::string column;
  while (std::getline(ss, column, ','))
  {
    columns.push_back(column);
  }

  WP waypoint;
  if (yaw)
  {
    waypoint.pose.position.x = std::stod(columns[0]);
    waypoint.pose.position.y = std::stod(columns[1]);
    waypoint.pose.position.z = std::stod(columns[2]);
    waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(columns[3]));
    waypoint.velocity_kmh = std::stod(columns[4]);
  }
  else
  {
    waypoint.pose.position.x = std::stod(columns[0]);
    waypoint.pose.position.y = std::stod(columns[1]);
    waypoint.pose.position.z = std::stod(columns[2]);
    waypoint.velocity_kmh = std::stod(columns[3]);
  }

  return waypoint;
}

static size_t countColumn(const std::string& line)
{
  std::istringstream ss(line);
  size_t ncol = 0;

  std::string column;
  while (std::getline(ss, column, ','))
  {
    ++ncol;
  }

  return ncol;
}

static std::vector<WP> readWaypoint(const char *filename)
{
  std::ifstream ifs(filename);
  std::string line;

  std::getline(ifs, line); // Remove first line
  size_t ncol = countColumn(line);

  std::vector<WP> waypoints;
  if (ncol == 3)
  {
    while (std::getline(ifs, line))
    {
      waypoints.push_back(parseWaypoint(line, false));
    }

    size_t last = waypoints.size() - 1;
    for (size_t i = 0; i < waypoints.size(); ++i)
    {
      double yaw;
      if (i == last)
      {
        yaw = atan2(waypoints[i-1].pose.position.y - waypoints[i].pose.position.y,
                    waypoints[i-1].pose.position.x - waypoints[i].pose.position.x);
        yaw -= M_PI;
      }
      else
      {
        yaw = atan2(waypoints[i+1].pose.position.y - waypoints[i].pose.position.y,
                    waypoints[i+1].pose.position.x - waypoints[i].pose.position.x);
      }
      waypoints[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
  }
  else if (ncol == 4)
  {
    while (std::getline(ifs, line))
    {
      waypoints.push_back(parseWaypoint(line, true));
    }
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


bool verifyFileConsistency(const char *filename)
{

  std::ifstream ifs(filename);

  if (!ifs)
  {
    return false;
  }

  std::string line;
  std::getline(ifs, line);
  size_t ncol = countColumn(line);

  while (std::getline(ifs, line))
  {
    if (countColumn(line) != ncol + 1)
      return false;
  }
  return true;
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
    wp.pose.pose.position = waypoints[i].pose.position;
    wp.pose.pose.orientation = waypoints[i].pose.orientation;

    wp.twist.header = lane_waypoint.header;
    double vel_kmh = decelerate(point2vector(waypoints[i].pose.position),
        point2vector(waypoints[waypoints.size() -1 ].pose.position),
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

  std::string driving_lane_csv;
  std::string passing_lane_csv;

  private_nh.param<std::string>("driving_lane_csv", driving_lane_csv, DRIVING_LANE_CSV);
  private_nh.param<std::string>("passing_lane_csv", passing_lane_csv, PASSING_LANE_CSV);
  private_nh.getParam("decelerate", _decelerate);
  ROS_INFO_STREAM("decelerate :" << _decelerate);

  ros::Publisher lane_pub = nh.advertise<waypoint_follower::LaneArray>("lane_waypoints_array", 10, true);
  waypoint_follower::LaneArray lane_array;

  if (!verifyFileConsistency(driving_lane_csv.c_str()))
  {
    ROS_ERROR("driving lane data is something wrong...");
    exit(-1);
  }
  else
  {
    ROS_INFO("driving lane data is valid. publishing...");
    lane_array.lanes.push_back(createLaneWaypoint(readWaypoint(driving_lane_csv.c_str())));
  }

  if (!verifyFileConsistency(passing_lane_csv.c_str()))
  {
    ROS_INFO("no passing lane data...");
  }
  else
  {
    ROS_INFO("passing lane data is valid. publishing...");
    lane_array.lanes.push_back(createLaneWaypoint(readWaypoint(passing_lane_csv.c_str())));
  }

  lane_pub.publish(lane_array);

  ros::spin();

}
