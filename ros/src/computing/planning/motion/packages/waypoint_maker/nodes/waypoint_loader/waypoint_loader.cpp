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
#include <cctype>
#include <algorithm>
#include "waypoint_follower/libwaypoint_follower.h"
#include "waypoint_follower/LaneArray.h"

namespace {

enum class FileFormat
{
  ver1,  //x,y,z,(velocity)
  ver2,  //x,y,z,yaw,(velocity)
  ver3,  //first line consists on explanation of values
  unknown,
};

struct WP
{
  geometry_msgs::Pose pose;
  double velocity_kmh;
  int32_t change_flag;
};

double g_decelerate = 1.0;
const std::string MULTI_LANE_CSV = "/tmp/driving_lane.csv";

void parseColumns(const std::string &line, std::vector<std::string> *columns)
{
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ','))
    {
      columns->push_back(column);
    }
}

void parseWaypoint(const std::string& line, FileFormat format, WP *waypoint)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);

  waypoint->pose.position.x = std::stod(columns[0]);
  waypoint->pose.position.y = std::stod(columns[1]);
  waypoint->pose.position.z = std::stod(columns[2]);

  if (format == FileFormat::ver2)
  {
    waypoint->pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(columns[3]));
    waypoint->velocity_kmh = std::stod(columns[4]);
  }
  else if (format == FileFormat::ver1)
  {
    waypoint->velocity_kmh = std::stod(columns[3]);
  }
  else if (format == FileFormat::ver3)
  {
    waypoint->pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(columns[3]));
    waypoint->velocity_kmh = std::stod(columns[4]);
    waypoint->change_flag = std::stoi(columns[5]);
  }
}

size_t countColumns(const std::string& line)
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

bool readWaypoint(const char *filename, FileFormat format, std::vector<WP> *waypoints)
{
  std::ifstream ifs(filename);

  if(!ifs)
  {
    return false;
  }

  std::string line;
  std::getline(ifs, line); // Remove first line

  if(format == FileFormat::ver3)
  {
    std::getline(ifs, line);  //remove second line
  }

  while (std::getline(ifs, line))
  {
    WP wp;
    parseWaypoint(line, format, &wp);
    waypoints->push_back(wp);
  }

  if (format == FileFormat::ver1)
  {
    size_t last = waypoints->size() - 1;
    for (size_t i = 0; i < waypoints->size(); ++i)
    {
      if (i != last)
      {
        double yaw = atan2(waypoints->at(i+1).pose.position.y - waypoints->at(i).pose.position.y,
                   waypoints->at(i+1).pose.position.x - waypoints->at(i).pose.position.x);
        waypoints->at(i).pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      }
      else
      {
        waypoints->at(i).pose.orientation = waypoints->at(i-1).pose.orientation;
      }

    }
  }

  return true;
}

double decelerate(tf::Vector3 v1, tf::Vector3 v2, double original_velocity_kmh)
{

  double distance = tf::tfDistance(v1, v2);
  double vel = mps2kmph(sqrt(2 * g_decelerate * distance)); //km/h
  if (vel < 1.0)
    vel = 0;
  if (vel > original_velocity_kmh)
  {
    vel = original_velocity_kmh;
  }
  return vel;
}


bool verifyFileConsistency(const char *filename, FileFormat format)
{
  ROS_INFO("verify...");
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return false;
  }

  std::string line;
  std::getline(ifs, line); //get first line
  std::getline(ifs, line); //get second line

  size_t ncol = countColumns(line);

   while (std::getline(ifs, line))
  {
    if (countColumns(line) != ncol)
      return false;
  }
  return true;
}


waypoint_follower::lane createLaneWaypoint(const std::vector<WP> &waypoints)
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
    wp.change_flag = waypoints[i].change_flag;

    lane_waypoint.waypoints.push_back(wp);
  }
  return lane_waypoint;
}

FileFormat checkFileFormat(const char* filename)
{

  std::ifstream ifs(filename);

  if (!ifs)
  {
    return FileFormat::unknown;
  }

  // get first line
  std::string line;
  std::getline(ifs, line);

  //parse first line
  std::vector<std::string> parsed_columns;
  parseColumns(line, &parsed_columns);

  //check if first element in the first column does not include digit
  if (!std::any_of(parsed_columns.at(0).cbegin(),parsed_columns.at(0).cend(),isdigit))
  {
    return FileFormat::ver3;
  }

  //if element consists only digit
  int num_of_columns = countColumns(line);
  ROS_INFO("columns size: %d",num_of_columns);

  return (num_of_columns == 3 ? FileFormat::ver1  // if data consists "x y z (velocity)"
         : num_of_columns == 4 ? FileFormat::ver2  // if data consists "x y z yaw (velocity)
                               : FileFormat::unknown
          );
}

} //namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_loader");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string multi_lane_csv;

  private_nh.getParam("decelerate", g_decelerate);
  ROS_INFO_STREAM("decelerate :" << g_decelerate);
  private_nh.param<std::string>("multi_lane_csv", multi_lane_csv, MULTI_LANE_CSV);

  ros::Publisher lane_pub = nh.advertise<waypoint_follower::LaneArray>("lane_waypoints_array", 10, true);
  waypoint_follower::LaneArray lane_array;

  std::vector<std::string> multi_file_path;
  parseColumns(multi_lane_csv, &multi_file_path);

  for (auto el : multi_file_path)
  {
    FileFormat file_format = checkFileFormat(el.c_str());
    ROS_INFO("Format: %d", file_format);

    if (!verifyFileConsistency(el.c_str(),file_format))
    {
      ROS_ERROR("lane data is something wrong...");
      exit(-1);
    }

    ROS_INFO("lane data is valid. publishing...");
    std::vector<WP> wps;
    readWaypoint(el.c_str(), file_format, &wps);
    lane_array.lanes.push_back(createLaneWaypoint(wps));
  }

  lane_pub.publish(lane_array);

  ros::spin();

}
