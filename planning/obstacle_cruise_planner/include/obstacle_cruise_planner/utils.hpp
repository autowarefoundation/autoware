// Copyright 2022 TIER IV, Inc.
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

#ifndef OBSTACLE_CRUISE_PLANNER__UTILS_HPP_
#define OBSTACLE_CRUISE_PLANNER__UTILS_HPP_

#include "common_structs.hpp"
#include "motion_utils/motion_utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <boost/optional.hpp>

#include <limits>
#include <string>
#include <vector>

namespace obstacle_cruise_utils
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

bool isVehicle(const uint8_t label);

visualization_msgs::msg::Marker getObjectMarker(
  const geometry_msgs::msg::Pose & obstacle_pose, size_t idx, const std::string & ns,
  const double r, const double g, const double b);

boost::optional<geometry_msgs::msg::Pose> calcForwardPose(
  const autoware_auto_planning_msgs::msg::Trajectory & traj, const size_t start_idx,
  const double target_length);

boost::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPath(
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time);

boost::optional<geometry_msgs::msg::Pose> getCurrentObjectPoseFromPredictedPaths(
  const std::vector<autoware_auto_perception_msgs::msg::PredictedPath> & predicted_paths,
  const rclcpp::Time & obj_base_time, const rclcpp::Time & current_time);

geometry_msgs::msg::PoseStamped getCurrentObjectPose(
  const autoware_auto_perception_msgs::msg::PredictedObject & predicted_object,
  const std_msgs::msg::Header & obj_header, const rclcpp::Time & current_time,
  const bool use_prediction);

boost::optional<TargetObstacle> getClosestStopObstacle(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const std::vector<TargetObstacle> & target_obstacles);

std::string toHexString(const unique_identifier_msgs::msg::UUID & id);

template <class T>
size_t getIndexWithLongitudinalOffset(
  const T & points, const double longitudinal_offset, boost::optional<size_t> start_idx)
{
  if (points.empty()) {
    throw std::logic_error("points is empty.");
  }

  if (start_idx) {
    if (/*start_idx.get() < 0 || */ points.size() <= start_idx.get()) {
      throw std::out_of_range("start_idx is out of range.");
    }
  } else {
    if (longitudinal_offset > 0) {
      start_idx = 0;
    } else {
      start_idx = points.size() - 1;
    }
  }

  double sum_length = 0.0;
  if (longitudinal_offset > 0) {
    for (size_t i = start_idx.get(); i < points.size() - 1; ++i) {
      const double segment_length =
        tier4_autoware_utils::calcDistance2d(points.at(i), points.at(i + 1));
      sum_length += segment_length;
      if (sum_length >= longitudinal_offset) {
        const double back_length = sum_length - longitudinal_offset;
        const double front_length = segment_length - back_length;
        if (front_length < back_length) {
          return i;
        } else {
          return i + 1;
        }
      }
    }
    return points.size() - 1;
  }

  for (size_t i = start_idx.get(); i > 0; --i) {
    const double segment_length =
      tier4_autoware_utils::calcDistance2d(points.at(i - 1), points.at(i));
    sum_length += segment_length;
    if (sum_length >= -longitudinal_offset) {
      const double back_length = sum_length + longitudinal_offset;
      const double front_length = segment_length - back_length;
      if (front_length < back_length) {
        return i;
      } else {
        return i - 1;
      }
    }
  }
  return 0;
}
}  // namespace obstacle_cruise_utils

#endif  // OBSTACLE_CRUISE_PLANNER__UTILS_HPP_
