// Copyright 2020 Tier IV, Inc., Leo Drive Teknoloji A.Ş.
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

#include "scene.hpp"

#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "util.hpp"

#include <rclcpp/rclcpp.hpp>

namespace behavior_velocity_planner
{
using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::createPoint;

using geometry_msgs::msg::Point32;

SpeedBumpModule::SpeedBumpModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::SpeedBump & speed_bump_reg_elem, const PlannerParam & planner_param,
  const rclcpp::Logger & logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  module_id_(module_id),
  lane_id_(lane_id),
  speed_bump_reg_elem_(std::move(speed_bump_reg_elem)),
  planner_param_(planner_param)
{
  // Read speed bump height [m] from map
  const auto speed_bump_height =
    static_cast<float>(speed_bump_reg_elem_.speedBump().attributeOr("height", 0.5));

  // If slow_down_speed is specified on speed_bump annotation use it instead of calculating it
  if (speed_bump_reg_elem_.speedBump().hasAttribute("slow_down_speed")) {
    speed_bump_slow_down_speed_ = static_cast<float>(
      speed_bump_reg_elem_.speedBump().attribute("slow_down_speed").asDouble().get() / 3.6);
  } else {
    // point.x : height [m] -- point.y : speed [m/s]
    Point32 p1;
    Point32 p2;

    p1.x = planner_param_.speed_calculation_min_height;
    p2.x = planner_param_.speed_calculation_max_height;

    p1.y = planner_param_.speed_calculation_max_speed;
    p2.y = planner_param_.speed_calculation_min_speed;

    // Calculate the speed [m/s] for speed bump
    speed_bump_slow_down_speed_ = calcSlowDownSpeed(p1, p2, speed_bump_height);
  }

  if (planner_param_.print_debug_info) {
    std::cout << "------------------------------" << std::endl;
    std::cout << "Speed Bump ID: " << module_id_ << std::endl;
    std::cout << "Speed Bump Height [cm]: " << speed_bump_height * 100 << std::endl;
    std::cout << "Slow Down Speed [kph]: " << speed_bump_slow_down_speed_ * 3.6 << std::endl;
    std::cout << "------------------------------" << std::endl;
  }
}

bool SpeedBumpModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  if (path->points.empty()) {
    return false;
  }

  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const auto & ego_pos = planner_data_->current_odometry->pose.position;
  const auto & speed_bump = speed_bump_reg_elem_.speedBump();
  const auto & speed_bump_polygon = lanelet::utils::to2D(speed_bump).basicPolygon();

  const auto & ego_path = *path;
  const auto & path_polygon_intersection_status =
    getPathPolygonIntersectionStatus(ego_path, speed_bump_polygon, ego_pos, 2);

  debug_data_.path_polygon_intersection_status = path_polygon_intersection_status;

  for (const auto & p : speed_bump_reg_elem_.speedBump().basicPolygon()) {
    debug_data_.speed_bump_polygon.push_back(createPoint(p.x(), p.y(), ego_pos.z));
  }

  return applySlowDownSpeed(*path, speed_bump_slow_down_speed_, path_polygon_intersection_status);
}

bool SpeedBumpModule::applySlowDownSpeed(
  PathWithLaneId & output, const float speed_bump_speed,
  const PathPolygonIntersectionStatus & path_polygon_intersection_status)
{
  if (isNoRelation(path_polygon_intersection_status)) {
    return false;
  }

  // decide slow_start_point index
  size_t slow_start_point_idx{};
  // if first intersection point exists
  if (path_polygon_intersection_status.first_intersection_point) {
    // calculate & insert slow_start_point position wrt the first intersection point between
    // path and the speed bump polygon
    const auto & src_point = *path_polygon_intersection_status.first_intersection_point;
    const auto & slow_start_margin_to_base_link =
      -1 *
      (planner_data_->vehicle_info_.max_longitudinal_offset_m + planner_param_.slow_start_margin);
    auto slow_start_point_idx_candidate =
      insertPointWithOffset(src_point, slow_start_margin_to_base_link, output.points, 5e-2);
    if (slow_start_point_idx_candidate) {
      slow_start_point_idx = *slow_start_point_idx_candidate;
      debug_data_.slow_start_poses.push_back(output.points.at(slow_start_point_idx).point.pose);
    } else if (calcSignedArcLength(output.points, src_point, 0) > slow_start_margin_to_base_link) {
      // There is first intersection point but slow_start_point can not be inserted because it is
      // behind the first path point (assign virtual slow_start_point_idx)
      slow_start_point_idx = 0;
    } else {
      return false;
    }
  } else if (
    path_polygon_intersection_status.second_intersection_point ||
    path_polygon_intersection_status.is_path_inside_of_polygon) {
    // assign virtual slow_start_point_idx
    slow_start_point_idx = 0;
  }

  // decide slow_end_point index
  size_t slow_end_point_idx{};
  // if second intersection point exists
  if (path_polygon_intersection_status.second_intersection_point) {
    // calculate & insert slow_end_point position wrt the second intersection point between path
    // and the speed bump polygon
    const auto & src_point = *path_polygon_intersection_status.second_intersection_point;
    const auto & slow_end_margin_to_base_link =
      planner_data_->vehicle_info_.rear_overhang_m + planner_param_.slow_end_margin;
    auto slow_end_point_idx_candidate =
      insertPointWithOffset(src_point, slow_end_margin_to_base_link, output.points, 5e-2);
    if (slow_end_point_idx_candidate) {
      slow_end_point_idx = *slow_end_point_idx_candidate;
      debug_data_.slow_end_points.push_back(
        output.points.at(slow_end_point_idx).point.pose.position);
    } else if (
      calcSignedArcLength(output.points, src_point, output.points.size() - 1) <
      slow_end_margin_to_base_link) {
      // There is second intersection point but slow_end_point can not be inserted because it is in
      // front of the last path point (assign virtual slow_end_point_idx)
      slow_end_point_idx = output.points.size() - 1;
    } else {
      return false;
    }
  } else if (
    path_polygon_intersection_status.first_intersection_point ||
    path_polygon_intersection_status.is_path_inside_of_polygon) {
    // assign virtual slow_end_point_idx
    slow_end_point_idx = output.points.size() - 1;
  }

  // insert constant speed to path points that intersects with speed bump area
  return insertConstSpeedToPathSection(
    output.points, slow_start_point_idx, slow_end_point_idx, speed_bump_speed);
}

}  // namespace behavior_velocity_planner
