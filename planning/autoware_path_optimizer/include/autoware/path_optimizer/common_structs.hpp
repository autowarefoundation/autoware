// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_OPTIMIZER__COMMON_STRUCTS_HPP_
#define AUTOWARE__PATH_OPTIMIZER__COMMON_STRUCTS_HPP_

#include "autoware/path_optimizer/type_alias.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::path_optimizer
{
struct ReferencePoint;
struct Bounds;

struct PlannerData
{
  // input
  Header header;
  std::vector<TrajectoryPoint> traj_points;  // converted from the input path
  std::vector<geometry_msgs::msg::Point> left_bound;
  std::vector<geometry_msgs::msg::Point> right_bound;

  // ego
  geometry_msgs::msg::Pose ego_pose;
  double ego_vel{};
};

struct DebugData
{
  // settting
  size_t mpt_visualize_sampling_num;
  geometry_msgs::msg::Pose ego_pose;
  std::vector<double> vehicle_circle_radiuses;
  std::vector<double> vehicle_circle_longitudinal_offsets;

  // mpt
  std::vector<ReferencePoint> ref_points;
  std::vector<std::vector<geometry_msgs::msg::Pose>> vehicle_circles_pose;

  std::vector<TrajectoryPoint> extended_traj_points;
  std::optional<geometry_msgs::msg::Pose> stop_pose_by_drivable_area = std::nullopt;
};

struct TrajectoryParam
{
  TrajectoryParam() = default;
  explicit TrajectoryParam(rclcpp::Node * node)
  {
    output_backward_traj_length =
      node->declare_parameter<double>("common.output_backward_traj_length");
    output_delta_arc_length = node->declare_parameter<double>("common.output_delta_arc_length");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using autoware::universe_utils::updateParam;

    // common
    updateParam<double>(
      parameters, "common.output_backward_traj_length", output_backward_traj_length);
    updateParam<double>(parameters, "common.output_delta_arc_length", output_delta_arc_length);
  }

  double output_delta_arc_length;
  double output_backward_traj_length;
};

struct EgoNearestParam
{
  EgoNearestParam() = default;
  explicit EgoNearestParam(rclcpp::Node * node)
  {
    dist_threshold = node->declare_parameter<double>("ego_nearest_dist_threshold");
    yaw_threshold = node->declare_parameter<double>("ego_nearest_yaw_threshold");
  }

  void onParam(const std::vector<rclcpp::Parameter> & parameters)
  {
    using autoware::universe_utils::updateParam;
    updateParam<double>(parameters, "ego_nearest_dist_threshold", dist_threshold);
    updateParam<double>(parameters, "ego_nearest_yaw_threshold", yaw_threshold);
  }

  double dist_threshold{0.0};
  double yaw_threshold{0.0};
};
}  // namespace autoware::path_optimizer

#endif  // AUTOWARE__PATH_OPTIMIZER__COMMON_STRUCTS_HPP_
