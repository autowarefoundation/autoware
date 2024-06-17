// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR__DEBUG_MARKER_HPP_
#define AUTOWARE__PLANNING_VALIDATOR__DEBUG_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

class PlanningValidatorDebugMarkerPublisher
{
public:
  explicit PlanningValidatorDebugMarkerPublisher(rclcpp::Node * node);

  void pushPoseMarker(
    const autoware_planning_msgs::msg::TrajectoryPoint & p, const std::string & ns, int id = 0);
  void pushPoseMarker(const geometry_msgs::msg::Pose & pose, const std::string & ns, int id = 0);
  void pushVirtualWall(const geometry_msgs::msg::Pose & pose);
  void pushWarningMsg(const geometry_msgs::msg::Pose & pose, const std::string & msg);
  void publish();

  void clearMarkers();

private:
  rclcpp::Node * node_;
  visualization_msgs::msg::MarkerArray marker_array_;
  visualization_msgs::msg::MarkerArray marker_array_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_pub_;
  std::map<std::string, int> marker_id_;

  int getMarkerId(const std::string & ns)
  {
    if (marker_id_.count(ns) == 0) {
      marker_id_[ns] = 0.0;
    }
    return marker_id_[ns]++;
  }
};

#endif  // AUTOWARE__PLANNING_VALIDATOR__DEBUG_MARKER_HPP_
