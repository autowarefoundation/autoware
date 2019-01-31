/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "waypoint_loader_core.h"

namespace waypoint_maker
{
// Constructor
WaypointLoaderNode::WaypointLoaderNode() : private_nh_("~")
{
  initPubSub();
}

// Destructor
WaypointLoaderNode::~WaypointLoaderNode()
{
}

void WaypointLoaderNode::initPubSub()
{
  private_nh_.param<std::string>("multi_lane_csv", multi_lane_csv_, "/tmp/driving_lane.csv");
  // setup publisher
  lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("/based/lane_waypoints_raw", 10, true);
}

void WaypointLoaderNode::run()
{
  multi_file_path_.clear();
  parseColumns(multi_lane_csv_, &multi_file_path_);
  autoware_msgs::LaneArray lane_array;
  createLaneArray(multi_file_path_, &lane_array);
  lane_pub_.publish(lane_array);
  output_lane_array_ = lane_array;
  ros::spin();
}

void WaypointLoaderNode::createLaneArray(const std::vector<std::string>& paths, autoware_msgs::LaneArray* lane_array)
{
  for (const auto& el : paths)
  {
    autoware_msgs::Lane lane;
    createLaneWaypoint(el, &lane);
    lane_array->lanes.emplace_back(lane);
  }
}

void WaypointLoaderNode::createLaneWaypoint(const std::string& file_path, autoware_msgs::Lane* lane)
{
  if (!verifyFileConsistency(file_path.c_str()))
  {
    ROS_ERROR("lane data is something wrong...");
    return;
  }

  ROS_INFO("lane data is valid. publishing...");
  FileFormat format = checkFileFormat(file_path.c_str());
  std::vector<autoware_msgs::Waypoint> wps;
  if (format == FileFormat::ver1)
  {
    loadWaypointsForVer1(file_path.c_str(), &wps);
  }
  else if (format == FileFormat::ver2)
  {
    loadWaypointsForVer2(file_path.c_str(), &wps);
  }
  else
  {
    loadWaypointsForVer3(file_path.c_str(), &wps);
  }
  lane->header.frame_id = "/map";
  lane->header.stamp = ros::Time(0);
  lane->waypoints = wps;
}

void WaypointLoaderNode::loadWaypointsForVer1(const char* filename, std::vector<autoware_msgs::Waypoint>* wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return;
  }

  std::string line;
  std::getline(ifs, line);  // Remove first line

  while (std::getline(ifs, line))
  {
    autoware_msgs::Waypoint wp;
    parseWaypointForVer1(line, &wp);
    wps->emplace_back(wp);
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
  }
}

void WaypointLoaderNode::parseWaypointForVer1(const std::string& line, autoware_msgs::Waypoint* wp)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);

  wp->pose.pose.position.x = std::stod(columns[0]);
  wp->pose.pose.position.y = std::stod(columns[1]);
  wp->pose.pose.position.z = std::stod(columns[2]);
  wp->twist.twist.linear.x = kmph2mps(std::stod(columns[3]));
}

void WaypointLoaderNode::loadWaypointsForVer2(const char* filename, std::vector<autoware_msgs::Waypoint>* wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return;
  }

  std::string line;
  std::getline(ifs, line);  // Remove first line

  while (std::getline(ifs, line))
  {
    autoware_msgs::Waypoint wp;
    parseWaypointForVer2(line, &wp);
    wps->emplace_back(wp);
  }
}

void WaypointLoaderNode::parseWaypointForVer2(const std::string& line, autoware_msgs::Waypoint* wp)
{
  std::vector<std::string> columns;
  parseColumns(line, &columns);

  wp->pose.pose.position.x = std::stod(columns[0]);
  wp->pose.pose.position.y = std::stod(columns[1]);
  wp->pose.pose.position.z = std::stod(columns[2]);
  wp->pose.pose.orientation = tf::createQuaternionMsgFromYaw(std::stod(columns[3]));
  wp->twist.twist.linear.x = kmph2mps(std::stod(columns[4]));
}

void WaypointLoaderNode::loadWaypointsForVer3(const char* filename, std::vector<autoware_msgs::Waypoint>* wps)
{
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return;
  }

  std::string line;
  std::getline(ifs, line);  // get first line
  std::vector<std::string> contents;
  parseColumns(line, &contents);

  // std::getline(ifs, line);  // remove second line
  while (std::getline(ifs, line))
  {
    autoware_msgs::Waypoint wp;
    parseWaypointForVer3(line, contents, &wp);
    wps->emplace_back(wp);
  }
}

void WaypointLoaderNode::parseWaypointForVer3(const std::string& line, const std::vector<std::string>& contents,
                                              autoware_msgs::Waypoint* wp)
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
  wp->wpstate.steering_state = (map.find("steering_flag") != map.end()) ? std::stoi(map["steering_flag"]) : 0;
  wp->wpstate.accel_state = (map.find("accel_flag") != map.end()) ? std::stoi(map["accel_flag"]) : 0;
  wp->wpstate.stop_state = (map.find("stop_flag") != map.end()) ? std::stoi(map["stop_flag"]) : 0;
  wp->wpstate.event_state = (map.find("event_flag") != map.end()) ? std::stoi(map["event_flag"]) : 0;
}

FileFormat WaypointLoaderNode::checkFileFormat(const char* filename)
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

  return (num_of_columns == 3 ? FileFormat::ver1  // if data consists "x y z (velocity)"
                                :
                                num_of_columns == 4 ? FileFormat::ver2  // if data consists "x y z yaw (velocity)
                                                      :
                                                      FileFormat::unknown);
}

bool WaypointLoaderNode::verifyFileConsistency(const char* filename)
{
  ROS_INFO("verify...");
  std::ifstream ifs(filename);

  if (!ifs)
  {
    return false;
  }

  FileFormat format = checkFileFormat(filename);
  ROS_INFO("format: %d", static_cast<int>(format));
  if (format == FileFormat::unknown)
  {
    ROS_ERROR("unknown file format");
    return false;
  }

  std::string line;
  std::getline(ifs, line);  // remove first line

  size_t ncol = format == FileFormat::ver1 ? 4  // x,y,z,velocity
                                             :
                                             format == FileFormat::ver2 ? 5  // x,y,z,yaw,velocity
                                                                          :
                                                                          countColumns(line);

  while (std::getline(ifs, line))  // search from second line
  {
    if (countColumns(line) != ncol)
    {
      return false;
    }
  }
  return true;
}

void parseColumns(const std::string& line, std::vector<std::string>* columns)
{
  std::istringstream ss(line);
  std::string column;
  while (std::getline(ss, column, ','))
  {
    while (1)
    {
      auto res = std::find(column.begin(), column.end(), ' ');
      if (res == column.end())
      {
        break;
      }
      column.erase(res);
    }
    if (!column.empty())
    {
      columns->emplace_back(column);
    }
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

}  // waypoint_maker
