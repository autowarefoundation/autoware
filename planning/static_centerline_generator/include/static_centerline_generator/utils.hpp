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

#ifndef STATIC_CENTERLINE_GENERATOR__UTILS_HPP_
#define STATIC_CENTERLINE_GENERATOR__UTILS_HPP_

#include "route_handler/route_handler.hpp"
#include "static_centerline_generator/type_alias.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>

namespace static_centerline_generator
{
namespace utils
{
rclcpp::QoS create_transient_local_qos();

lanelet::ConstLanelets get_lanelets_from_ids(
  const RouteHandler & route_handler, const std::vector<lanelet::Id> & lane_ids);

geometry_msgs::msg::Pose get_center_pose(
  const RouteHandler & route_handler, const size_t lanelet_id);

PathWithLaneId get_path_with_lane_id(
  const RouteHandler & route_handler, const lanelet::ConstLanelets lanelets,
  const geometry_msgs::msg::Pose & start_pose, const double nearest_ego_dist_threshold,
  const double nearest_ego_yaw_threshold);

void update_centerline(
  RouteHandler & route_handler, const lanelet::ConstLanelets & lanelets,
  const std::vector<TrajectoryPoint> & new_centerline);

MarkerArray create_footprint_marker(
  const LinearRing2d & footprint_poly, const std::array<double, 3> & marker_color,
  const rclcpp::Time & now, const size_t idx);

MarkerArray create_distance_text_marker(
  const geometry_msgs::msg::Pose & pose, const double dist,
  const std::array<double, 3> & marker_color, const rclcpp::Time & now, const size_t idx);
}  // namespace utils
}  // namespace static_centerline_generator

#endif  // STATIC_CENTERLINE_GENERATOR__UTILS_HPP_
