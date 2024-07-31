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

#ifndef AUTOWARE__CONTROL_VALIDATOR__DEBUG_MARKER_HPP_
#define AUTOWARE__CONTROL_VALIDATOR__DEBUG_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>

/**
 * @brief Class for publishing debug markers
 */
class ControlValidatorDebugMarkerPublisher
{
public:
  /**
   * @brief Constructor
   */
  explicit ControlValidatorDebugMarkerPublisher(rclcpp::Node * node);

  /**
   * @brief Push a virtual wall
   * @param pose pose of the virtual wall
   */
  void push_virtual_wall(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Push a warning message
   * @param pose pose of the warning message
   * @param msg warning message
   */
  void push_warning_msg(const geometry_msgs::msg::Pose & pose, const std::string & msg);

  /**
   * @brief Publish markers
   */
  void publish();

  /**
   * @brief Clear markers
   */
  void clear_markers();

private:
  rclcpp::Node * node_;
  visualization_msgs::msg::MarkerArray marker_array_;
  visualization_msgs::msg::MarkerArray marker_array_virtual_wall_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_pub_;
  std::map<std::string, int> marker_id_;
};

#endif  // AUTOWARE__CONTROL_VALIDATOR__DEBUG_MARKER_HPP_
