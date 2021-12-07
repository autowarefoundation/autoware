// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANNING_ERROR_MONITOR__DEBUG_MARKER_HPP_
#define PLANNING_ERROR_MONITOR__DEBUG_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class PlanningErrorMonitorDebugNode
{
public:
  PlanningErrorMonitorDebugNode();

  void initialize(rclcpp::Node * node);
  void pushPoseMarker(const geometry_msgs::msg::Pose & pose, const std::string & ns, int id = 0);
  void clearPoseMarker(const std::string & ns);
  void publish();

private:
  rclcpp::Node * node_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  std::map<std::string, int> marker_id_;
  bool initialized = false;

  int getMarkerId(const std::string & ns)
  {
    if (marker_id_.count(ns) == 0) {
      marker_id_[ns] = 0.0;
    }
    return marker_id_[ns]++;
  }

  void clearMarkerId(const std::string & ns) { marker_id_[ns] = 0; }
};

#endif  // PLANNING_ERROR_MONITOR__DEBUG_MARKER_HPP_
