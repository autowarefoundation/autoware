// Copyright 2023 TIER IV, Inc. All rights reserved.
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

#include "scene_out_of_lane.hpp"

#include "debug.hpp"
#include "decisions.hpp"
#include "filter_predicted_objects.hpp"
#include "footprint.hpp"
#include "lanelets_selection.hpp"
#include "overlapping_range.hpp"
#include "types.hpp"

#include <behavior_velocity_planner_common/utilization/debug.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <lanelet2_core/geometry/LaneletMap.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::out_of_lane
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

OutOfLaneModule::OutOfLaneModule(
  const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), params_(std::move(planner_param))
{
  velocity_factor_.init(VelocityFactor::UNKNOWN);
}

bool OutOfLaneModule::modifyPathVelocity(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  debug_data_.reset_data();
  *stop_reason = planning_utils::initializeStopReason(StopReason::OUT_OF_LANE);
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  EgoData ego_data;
  ego_data.pose = planner_data_->current_odometry->pose;
  ego_data.path = path;
  ego_data.first_path_idx =
    motion_utils::findNearestSegmentIndex(path->points, ego_data.pose.position);
  ego_data.velocity = planner_data_->current_velocity->twist.linear.x;
  ego_data.max_decel = -planner_data_->max_stop_acceleration_threshold;
  stopwatch.tic("calculate_path_footprints");
  const auto current_ego_footprint = calculate_current_ego_footprint(ego_data, params_, true);
  const auto path_footprints = calculate_path_footprints(ego_data, params_);
  const auto calculate_path_footprints_us = stopwatch.toc("calculate_path_footprints");
  // Calculate lanelets to ignore and consider
  const auto path_lanelets = planning_utils::getLaneletsOnPath(
    *path, planner_data_->route_handler_->getLaneletMapPtr(),
    planner_data_->current_odometry->pose);
  const auto ignored_lanelets =
    calculate_ignored_lanelets(ego_data, path_lanelets, *planner_data_->route_handler_, params_);
  const auto other_lanelets = calculate_other_lanelets(
    ego_data, path_lanelets, ignored_lanelets, *planner_data_->route_handler_, params_);

  debug_data_.footprints = path_footprints;
  debug_data_.path_lanelets = path_lanelets;
  debug_data_.ignored_lanelets = ignored_lanelets;
  debug_data_.other_lanelets = other_lanelets;

  if (params_.skip_if_already_overlapping) {
    debug_data_.current_footprint = current_ego_footprint;
    const auto overlapped_lanelet_it =
      std::find_if(other_lanelets.begin(), other_lanelets.end(), [&](const auto & ll) {
        return boost::geometry::intersects(ll.polygon2d().basicPolygon(), current_ego_footprint);
      });
    if (overlapped_lanelet_it != other_lanelets.end()) {
      debug_data_.current_overlapped_lanelets.push_back(*overlapped_lanelet_it);
      RCLCPP_DEBUG(logger_, "Ego is already overlapping a lane, skipping the module ()\n");
      return true;
    }
  }
  // Calculate overlapping ranges
  stopwatch.tic("calculate_overlapping_ranges");
  const auto ranges =
    calculate_overlapping_ranges(path_footprints, path_lanelets, other_lanelets, params_);
  const auto calculate_overlapping_ranges_us = stopwatch.toc("calculate_overlapping_ranges");
  // Calculate stop and slowdown points
  stopwatch.tic("calculate_decisions");
  DecisionInputs inputs;
  inputs.ranges = ranges;
  inputs.ego_data = ego_data;
  inputs.objects = filter_predicted_objects(*planner_data_->predicted_objects, ego_data, params_);
  inputs.route_handler = planner_data_->route_handler_;
  inputs.lanelets = other_lanelets;
  auto decisions = calculate_decisions(inputs, params_, logger_);
  const auto calculate_decisions_us = stopwatch.toc("calculate_decisions");
  stopwatch.tic("calc_slowdown_points");
  const auto points_to_insert = calculate_slowdown_points(ego_data, decisions, params_);
  debug_data_.slowdowns = points_to_insert;
  const auto calc_slowdown_points_us = stopwatch.toc("calc_slowdown_points");
  stopwatch.tic("insert_slowdown_points");
  for (const auto & point : points_to_insert) {
    auto path_idx = point.slowdown.target_path_idx;
    planning_utils::insertVelocity(*ego_data.path, point.point, point.slowdown.velocity, path_idx);
    if (point.slowdown.velocity == 0.0) {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = point.point.point.pose;
      stop_factor.dist_to_stop_pose = motion_utils::calcSignedArcLength(
        ego_data.path->points, ego_data.pose.position, point.point.point.pose.position);
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }
    velocity_factor_.set(
      path->points, planner_data_->current_odometry->pose, point.point.point.pose,
      VelocityFactor::UNKNOWN);
  }
  const auto insert_slowdown_points_us = stopwatch.toc("insert_slowdown_points");

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total time = %2.2fus\n"
    "\tcalculate_path_footprints = %2.0fus\n"
    "\tcalculate_overlapping_ranges = %2.0fus\n"
    "\tcalculate_decisions = %2.0fus\n"
    "\tcalc_slowdown_points = %2.0fus\n"
    "\tinsert_slowdown_points = %2.0fus\n",
    total_time_us, calculate_path_footprints_us, calculate_overlapping_ranges_us,
    calculate_decisions_us, calc_slowdown_points_us, insert_slowdown_points_us);
  return true;
}

MarkerArray OutOfLaneModule::createDebugMarkerArray()
{
  constexpr auto z = 0.0;
  MarkerArray debug_marker_array;

  debug::add_footprint_markers(debug_marker_array, debug_data_.footprints, z);
  debug::add_current_overlap_marker(
    debug_marker_array, debug_data_.current_footprint, debug_data_.current_overlapped_lanelets, z);
  debug::add_lanelet_markers(
    debug_marker_array, debug_data_.path_lanelets, "path_lanelets",
    tier4_autoware_utils::createMarkerColor(0.1, 0.1, 1.0, 0.5));
  debug::add_lanelet_markers(
    debug_marker_array, debug_data_.ignored_lanelets, "ignored_lanelets",
    tier4_autoware_utils::createMarkerColor(0.7, 0.7, 0.2, 0.5));
  debug::add_lanelet_markers(
    debug_marker_array, debug_data_.other_lanelets, "other_lanelets",
    tier4_autoware_utils::createMarkerColor(0.4, 0.4, 0.7, 0.5));
  return debug_marker_array;
}

motion_utils::VirtualWalls OutOfLaneModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls;
  motion_utils::VirtualWall wall;
  wall.text = "out_of_lane";
  wall.longitudinal_offset = params_.front_offset;
  wall.style = motion_utils::VirtualWallType::slowdown;
  for (const auto & slowdown : debug_data_.slowdowns) {
    wall.pose = slowdown.point.point.pose;
    virtual_walls.push_back(wall);
  }
  return virtual_walls;
}

std::vector<SlowdownToInsert> calculate_slowdown_points(
  const EgoData & ego_data, const std::vector<Slowdown> & decisions, PlannerParam params)
{
  std::vector<SlowdownToInsert> to_insert;
  params.extra_front_offset += params.dist_buffer;
  const auto base_footprint = make_base_footprint(params);

  const auto can_decel = [&](const auto dist_ahead_of_ego, const auto target_vel) {
    const auto acc_to_target_vel =
      (ego_data.velocity * ego_data.velocity - target_vel * target_vel) / (2 * dist_ahead_of_ego);
    return acc_to_target_vel < std::abs(ego_data.max_decel);
  };
  const auto insert_decision = [&](const auto & path_point, const auto & decision) -> bool {
    const auto dist_ahead_of_ego = motion_utils::calcSignedArcLength(
      ego_data.path->points, ego_data.pose.position, path_point.point.pose.position);
    if (!params.skip_if_over_max_decel || can_decel(dist_ahead_of_ego, decision.velocity)) {
      to_insert.push_back({decision, path_point});
      return true;
    }
    return false;
  };
  const auto insert_interpolated_decision =
    [&](const auto & path_point, const auto & decision) -> bool {
    auto interpolated_point = path_point;
    const auto & path_pose = path_point.point.pose;
    const auto & prev_path_pose = ego_data.path->points[decision.target_path_idx - 1].point.pose;
    constexpr auto precision = 0.1;
    for (auto ratio = precision; ratio <= 1.0; ratio += precision) {
      interpolated_point.point.pose =
        tier4_autoware_utils::calcInterpolatedPose(path_pose, prev_path_pose, ratio, false);
      const auto is_overlap = boost::geometry::overlaps(
        project_to_pose(base_footprint, interpolated_point.point.pose),
        decision.lane_to_avoid.polygon2d().basicPolygon());
      if (!is_overlap) {
        return insert_decision(path_point, decision);
      }
    }
    return false;
  };
  for (const auto & decision : decisions) {
    const auto & path_point = ego_data.path->points[decision.target_path_idx];
    const auto decision_is_at_beginning_of_path =
      decision.target_path_idx == ego_data.first_path_idx;
    bool inserted = false;
    if (decision_is_at_beginning_of_path) {
      inserted = insert_decision(path_point, decision);
    } else {
      inserted = insert_interpolated_decision(path_point, decision);
      // if no valid point found, fallback to using the previous index (known to not overlap)
      if (!inserted)
        inserted = insert_decision(ego_data.path->points[decision.target_path_idx], decision);
    }
    // only insert the first (i.e., lowest arc length) decision
    if (inserted) break;
  }
  return to_insert;
}

}  // namespace behavior_velocity_planner::out_of_lane
