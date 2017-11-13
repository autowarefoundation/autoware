/*
// *  Copyright (c) 2015, Nagoya University
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

#ifndef WAYPOINT_LOADER_CORE_H
#define WAYPOINT_LOADER_CORE_H

// ROS includes
#include <ros/ros.h>

// C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <tf/transform_datatypes.h>
#include <unordered_map>

#include "autoware_msgs/LaneArray.h"

namespace waypoint_maker
{

const std::string MULTI_LANE_CSV = "/tmp/driving_lane.csv";

enum class FileFormat : int32_t
{
  ver1,  //x,y,z,(velocity)
  ver2,  //x,y,z,yaw,(velocity)
  ver3,  //first line consists on explanation of values

  unknown = -1,
};

typedef std::underlying_type<FileFormat>::type FileFormatInteger;

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

class WaypointLoaderNode
{
public:

  WaypointLoaderNode();
  ~WaypointLoaderNode();

  void publishLaneArray();

private:

  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher lane_pub_;

  // variables
  std::string multi_lane_csv_;
  double decelerate_;

  // initializer
  void initPublisher();
  void initParameter();

  // functions

  void createLaneWaypoint(const std::string &file_path, autoware_msgs::lane *lane);
  void createLaneArray(const std::vector<std::string> &paths, autoware_msgs::LaneArray *lane_array);

  FileFormat checkFileFormat(const char *filename);
  bool verifyFileConsistency(const char *filename);
  void loadWaypointsForVer1(const char *filename, std::vector<autoware_msgs::waypoint> *wps);
  void parseWaypointForVer1(const std::string &line, autoware_msgs::waypoint *wp);
  void loadWaypointsForVer2(const char *filename, std::vector<autoware_msgs::waypoint> *wps);
  void parseWaypointForVer2(const std::string &line, autoware_msgs::waypoint *wp);
  void loadWaypoints(const char *filename, std::vector<autoware_msgs::waypoint> *wps);
  void parseWaypoint(const std::string &line, const std::vector<std::string> &contents,
                            autoware_msgs::waypoint *wp);
  void planningVelocity(std::vector<autoware_msgs::waypoint> *wps);
  double decelerate(geometry_msgs::Point p1, geometry_msgs::Point p2, double original_velocity_mps);

};

void parseColumns(const std::string &line, std::vector<std::string> *columns);
size_t countColumns(const std::string& line);

}
#endif  // WAYPOINT_LOADER_CORE_H
