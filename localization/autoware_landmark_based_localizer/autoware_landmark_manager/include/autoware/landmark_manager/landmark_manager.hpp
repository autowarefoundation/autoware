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

#ifndef AUTOWARE__LANDMARK_MANAGER__LANDMARK_MANAGER_HPP_
#define AUTOWARE__LANDMARK_MANAGER__LANDMARK_MANAGER_HPP_

#include "autoware_lanelet2_extension/localization/landmark.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <string>
#include <vector>

namespace landmark_manager
{

struct Landmark
{
  std::string id;
  geometry_msgs::msg::Pose pose;
};

class LandmarkManager
{
public:
  void parse_landmarks(
    const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr & msg,
    const std::string & target_subtype);

  [[nodiscard]] std::vector<landmark_manager::Landmark> get_landmarks() const;

  [[nodiscard]] visualization_msgs::msg::MarkerArray get_landmarks_as_marker_array_msg() const;

  [[nodiscard]] geometry_msgs::msg::Pose calculate_new_self_pose(
    const std::vector<Landmark> & detected_landmarks, const geometry_msgs::msg::Pose & self_pose,
    const bool consider_orientation) const;

private:
  // To allow multiple landmarks with the same id to be registered on a vector_map,
  // manage vectors by having them in a std::map.
  // landmarks_map_["<id>"] = [pose0, pose1, ...]
  std::map<std::string, std::vector<geometry_msgs::msg::Pose>> landmarks_map_;
};

}  // namespace landmark_manager

#endif  // AUTOWARE__LANDMARK_MANAGER__LANDMARK_MANAGER_HPP_
