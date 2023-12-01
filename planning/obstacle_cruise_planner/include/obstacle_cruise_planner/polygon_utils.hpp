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

#ifndef OBSTACLE_CRUISE_PLANNER__POLYGON_UTILS_HPP_
#define OBSTACLE_CRUISE_PLANNER__POLYGON_UTILS_HPP_

#include "obstacle_cruise_planner/common_structs.hpp"
#include "obstacle_cruise_planner/type_alias.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "vehicle_info_util/vehicle_info_util.hpp"

#include <boost/geometry.hpp>

#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace polygon_utils
{
namespace bg = boost::geometry;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

Polygon2d createOneStepPolygon(
  const std::vector<geometry_msgs::msg::Pose> & last_poses,
  const std::vector<geometry_msgs::msg::Pose> & current_poses,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double lat_margin);

std::optional<std::pair<geometry_msgs::msg::Point, double>> getCollisionPoint(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const Obstacle & obstacle, const bool is_driving_forward,
  const vehicle_info_util::VehicleInfo & vehicle_info);

std::vector<PointWithStamp> getCollisionPoints(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index,
  const double max_lat_dist = std::numeric_limits<double>::max(),
  const double max_prediction_time_for_collision_check = std::numeric_limits<double>::max());

}  // namespace polygon_utils

#endif  // OBSTACLE_CRUISE_PLANNER__POLYGON_UTILS_HPP_
