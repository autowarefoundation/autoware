// Copyright 2023 Autoware Foundation
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

#ifndef LANDMARK_PARSER__LANDMARK_PARSER_CORE_HPP_
#define LANDMARK_PARSER__LANDMARK_PARSER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>

std::map<std::string, geometry_msgs::msg::Pose> parse_landmark(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg,
  const std::string & target_subtype, const rclcpp::Logger & logger);

visualization_msgs::msg::MarkerArray convert_to_marker_array_msg(
  const std::map<std::string, geometry_msgs::msg::Pose> & landmarks);

#endif  // LANDMARK_PARSER__LANDMARK_PARSER_CORE_HPP_
