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

#include "waypoint_loader_core.h"

namespace waypoint_maker
{
// Constructor
WaypointLoaderNode::WaypointLoaderNode() : private_nh_("~")
{
  initParameter();
  initPublisher();
}

// Destructor
WaypointLoaderNode::~WaypointLoaderNode()
{
}

void WaypointLoaderNode::initPublisher()
{
  // setup publisher
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 10, true);
}

void WaypointLoaderNode::initParameter()
{
  // parameter settings
  private_nh_.param<double>("decelerate", decelerate_, double(0));
  ROS_INFO_STREAM("decelerate :" << decelerate_);
  private_nh_.param<std::string>("multi_lane_csv", multi_lane_csv_, MULTI_LANE_CSV);
}

void WaypointLoaderNode::publishLaneArray()
{
  // extract file paths
  std::vector<std::string> multi_file_path;
  parseColumns(multi_lane_csv_, &multi_file_path);
  autoware_msgs::LaneArray lane_array;
  createLaneArray(multi_file_path, &lane_array);
  lane_pub_.publish(lane_array);
}

void WaypointLoaderNode::createLaneArray(const std::vector<std::string> &paths,
                                         autoware_msgs::LaneArray *lane_array)
{
  for (auto el : paths)
  {
    autoware_msgs::lane lane;
    createLaneWaypoint(el, &lane);
    lane_array->lanes.push_back(lane);
  }
}

void WaypointLoaderNode::createLaneWaypoint(const std::string &file_path, autoware_msgs::lane *lane)
{
  if (!verifyFileConsistency(file_path.c_str()))
  {
    ROS_ERROR("lane data is something wrong...");
    return;
  }

  ROS_INFO("lane data is valid. publishing...");
  FileFormat format = checkFileFormat(file_path.c_str());
  std::vector<autoware_msgs::waypoint> wps;
  if (format == FileFormat::ver1)
    loadWaypointsForVer1(file_path.c_str(), &wps);
  else if (format == FileFormat::ver2)
    loadWaypointsForVer2(file_path.c_str(), &wps);
  else
    loadWaypoints(file_path.c_str(), &wps);

  lane->header.frame_id = "/map";
  lane->header.stamp = ros::Time(0);
  lane->waypoints = wps;
}

void WaypointLoaderNode::loadWaypointsForVer1(const char *filename, std::vector<autoware_msgs::waypoint> *wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
    return;

  std::string line;
  std::getline(ifs, line);  // Remove first line

  while (std::getline(ifs, line))
  {
    autoware_msgs::waypoint wp;
    parseWaypointForVer1(line, &wp);
    wps->push_back(wp);
  }

  size_t last = wps->size() - 1;
  for (size_t i = 0; i < wps->size(); ++i)
  {
    if (i != last)
    {
      double yaw = atan2(wps->at(i + 1).pose.pose.position.y - wps->at(i).pose.pose.position.y,
                         wps->at(i + 1).pose.pose.position.x - wps->at(i).pose.pose.position.x);
      wps->at(i).pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    }
    else
    {
      wps->at(i).pose.pose.orientation = wps->at(i - 1).pose.pose.orientation;
    }

    wps->at(i).twist.twist.linear.x = decelerate(
        wps->at(i).pose.pose.position, wps->at(wps->size() - 1).pose.pose.position, wps->at(i).twist.twist.linear.x);
  }
}

void WaypointLoaderNode::parseWaypointForVer1(const std::string &line, autoware_msgs::waypoint *wp)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);

  wp->pose.pose.position.x = std::stod(columns[0]);
  wp->pose.pose.position.y = std::stod(columns[1]);
  wp->pose.pose.position.z = std::stod(columns[2]);
  wp->twist.twist.linear.x = kmph2mps(std::stod(columns[3]));
}

void WaypointLoaderNode::loadWaypointsForVer2(const char *filename, std::vector<autoware_msgs::waypoint> *wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
    return;

  std::string line;
  std::getline(ifs, line);  // Remove first line

  while (std::getline(ifs, line))
  {
    autoware_msgs::waypoint wp;
    parseWaypointForVer2(line, &wp);
    wps->push_back(wp);
  }
  planningVelocity(&*wps);
}

void WaypointLoaderNode::parseWaypointForVer2(const std::string &line, autoware_msgs::waypoint *wp)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);

  wp->pose.pose.position.x = std::stod(columns[0]);
  wp->pose.pose.position.y = std::stod(columns[1]);
  wp->pose.pose.position.z = std::stod(columns[2]);
  wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(columns[3]));
  wp->twist.twist.linear.x = kmph2mps(std::stod(columns[4]));
}

void WaypointLoaderNode::loadWaypoints(const char *filename, std::vector<autoware_msgs::waypoint> *wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
    return;

  std::string line;
  std::getline(ifs, line);  // get first line
  std::vector<std::string> contents;
  parseColumns(line, &contents);

  std::getline(ifs, line);  // remove second line
  while (std::getline(ifs, line))
  {
    autoware_msgs::waypoint wp;
    parseWaypoint(line, contents, &wp);
    wps->push_back(wp);
  }
  planningVelocity(&*wps);
}

void WaypointLoaderNode::parseWaypoint(const std::string &line, const std::vector<std::string> &contents,
                                       autoware_msgs::waypoint *wp)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);
  std::unordered_map<std::string, std::string> map;
  for (size_t i = 0; i < contents.size(); i++)
  {
    map[contents.at(i)] = columns.at(i);
  }

  wp->pose.pose.position.x = std::stod(map["x"]);
  wp->pose.pose.position.y = std::stod(map["y"]);
  wp->pose.pose.position.z = std::stod(map["z"]);
  wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(map["yaw"]));
  wp->twist.twist.linear.x = kmph2mps(std::stod(map["velocity"]));
  wp->change_flag = std::stoi(map["change_flag"]);
}

FileFormat WaypointLoaderNode::checkFileFormat(const char *filename)
{
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return FileFormat::unknown;
  }

  // get first line
  std::string line;
  std::getline(ifs, line);

  // parse first line
  std::vector<std::string> parsed_columns;
  parseColumns(line, &parsed_columns);

  // check if first element in the first column does not include digit
  if (!std::any_of(parsed_columns.at(0).cbegin(), parsed_columns.at(0).cend(), isdigit))
  {
    return FileFormat::ver3;
  }

  // if element consists only digit
  int num_of_columns = countColumns(line);
  ROS_INFO("columns size: %d", num_of_columns);

  return ( num_of_columns == 3 ? FileFormat::ver1  // if data consists "x y z (velocity)"
         : num_of_columns == 4 ? FileFormat::ver2  // if data consists "x y z yaw (velocity)
                               : FileFormat::unknown
          );
}

void WaypointLoaderNode::planningVelocity(std::vector<autoware_msgs::waypoint> *wps)
{
  for (size_t i = 0; i < wps->size(); ++i)
  {
    wps->at(i).twist.twist.linear.x = decelerate(
      wps->at(i).pose.pose.position, wps->at(wps->size() - 1).pose.pose.position, wps->at(i).twist.twist.linear.x);
  }
}

double WaypointLoaderNode::decelerate(geometry_msgs::Point p1, geometry_msgs::Point p2, double original_velocity_mps)
{
  double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
  double vel = sqrt(2 * decelerate_ * distance);  // km/h

  if (mps2kmph(vel) < 1.0)
    vel = 0;

  if (vel > original_velocity_mps)
    vel = original_velocity_mps;

  return vel;
}

bool WaypointLoaderNode::verifyFileConsistency(const char *filename)
{
  ROS_INFO("verify...");
  std::ifstream ifs(filename);

  if (!ifs)
    return false;

  FileFormat format = checkFileFormat(filename);
  ROS_INFO("format: %d", static_cast<FileFormat>(format));
  if (format == FileFormat::unknown)
  {
    ROS_ERROR("unknown file format");
    return false;
  }

  std::string line;
  std::getline(ifs, line);  // remove first line

  size_t ncol = format == FileFormat::ver1 ? 4 //x,y,z,velocity
              : format == FileFormat::ver2 ? 5 //x,y,z,yaw,velocity
              : countColumns(line);

  while (std::getline(ifs, line))  // search from second line
  {
    if (countColumns(line) != ncol)
      return false;
  }
  return true;
}

void parseColumns(const std::string &line, std::vector<std::string> *columns)
{
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ','))
  {
    columns->push_back(column);
  }
}

size_t countColumns(const std::string &line)
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

}  // waypoint_maker
