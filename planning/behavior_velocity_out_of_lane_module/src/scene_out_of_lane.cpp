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

#include "calculate_slowdown_points.hpp"
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
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

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
  velocity_factor_.init(PlanningBehavior::UNKNOWN);
}

bool OutOfLaneModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_.reset_data();
  *stop_reason = planning_utils::initializeStopReason(StopReason::OUT_OF_LANE);
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();
  EgoData ego_data;
  ego_data.pose = planner_data_->current_odometry->pose;
  ego_data.path.points = path->points;
  ego_data.first_path_idx =
    motion_utils::findNearestSegmentIndex(ego_data.path.points, ego_data.pose.position);
  motion_utils::removeOverlapPoints(ego_data.path.points);
  ego_data.velocity = planner_data_->current_velocity->twist.linear.x;
  ego_data.max_decel = -planner_data_->max_stop_acceleration_threshold;
  stopwatch.tic("calculate_path_footprints");
  const auto current_ego_footprint = calculate_current_ego_footprint(ego_data, params_, true);
  const auto path_footprints = calculate_path_footprints(ego_data, params_);
  const auto calculate_path_footprints_us = stopwatch.toc("calculate_path_footprints");
  // Calculate lanelets to ignore and consider
  const auto path_lanelets = planning_utils::getLaneletsOnPath(
    ego_data.path, planner_data_->route_handler_->getLaneletMapPtr(),
    planner_data_->current_odometry->pose);
  const auto ignored_lanelets =
    calculate_ignored_lanelets(ego_data, path_lanelets, *planner_data_->route_handler_, params_);
  const auto other_lanelets = calculate_other_lanelets(
    ego_data, path_lanelets, ignored_lanelets, *planner_data_->route_handler_, params_);

  debug_data_.footprints = path_footprints;
  debug_data_.path_lanelets = path_lanelets;
  debug_data_.ignored_lanelets = ignored_lanelets;
  debug_data_.other_lanelets = other_lanelets;
  debug_data_.path = ego_data.path;
  debug_data_.first_path_idx = ego_data.first_path_idx;

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
  const auto decisions = calculate_decisions(inputs, params_, logger_);
  const auto calculate_decisions_us = stopwatch.toc("calculate_decisions");
  stopwatch.tic("calc_slowdown_points");
  if (  // reset the previous inserted point if the timer expired
    prev_inserted_point_ &&
    (clock_->now() - prev_inserted_point_time_).seconds() > params_.min_decision_duration)
    prev_inserted_point_.reset();
  auto point_to_insert =
    calculate_slowdown_point(ego_data, decisions, prev_inserted_point_, params_);
  const auto calc_slowdown_points_us = stopwatch.toc("calc_slowdown_points");
  stopwatch.tic("insert_slowdown_points");
  debug_data_.slowdowns.clear();
  if (  // reset the timer if there is no previous inserted point or if we avoid the same lane
    point_to_insert &&
    (!prev_inserted_point_ || prev_inserted_point_->slowdown.lane_to_avoid.id() ==
                                point_to_insert->slowdown.lane_to_avoid.id()))
    prev_inserted_point_time_ = clock_->now();
  // reuse previous stop point if there is no new one or if its velocity is not higher than the new
  // one and its arc length is lower
  const auto should_use_prev_inserted_point = [&]() {
    if (
      point_to_insert && prev_inserted_point_ &&
      prev_inserted_point_->slowdown.velocity <= point_to_insert->slowdown.velocity) {
      const auto arc_length = motion_utils::calcSignedArcLength(
        path->points, 0LU, point_to_insert->point.point.pose.position);
      const auto prev_arc_length = motion_utils::calcSignedArcLength(
        path->points, 0LU, prev_inserted_point_->point.point.pose.position);
      return prev_arc_length < arc_length;
    }
    return !point_to_insert && prev_inserted_point_;
  }();
  if (should_use_prev_inserted_point) {
    // if the path changed the prev point is no longer on the path so we project it
    const auto insert_arc_length = motion_utils::calcSignedArcLength(
      path->points, 0LU, prev_inserted_point_->point.point.pose.position);
    prev_inserted_point_->point.point.pose =
      motion_utils::calcInterpolatedPose(path->points, insert_arc_length);
    point_to_insert = prev_inserted_point_;
  }
  if (point_to_insert) {
    prev_inserted_point_ = point_to_insert;
    RCLCPP_INFO(logger_, "Avoiding lane %lu", point_to_insert->slowdown.lane_to_avoid.id());
    debug_data_.slowdowns = {*point_to_insert};
    auto path_idx = motion_utils::findNearestSegmentIndex(
                      path->points, point_to_insert->point.point.pose.position) +
                    1;
    planning_utils::insertVelocity(
      *path, point_to_insert->point, point_to_insert->slowdown.velocity, path_idx);
    if (point_to_insert->slowdown.velocity == 0.0) {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = point_to_insert->point.point.pose;
      stop_factor.dist_to_stop_pose = motion_utils::calcSignedArcLength(
        path->points, ego_data.pose.position, point_to_insert->point.point.pose.position);
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }
    velocity_factor_.set(
      path->points, planner_data_->current_odometry->pose, point_to_insert->point.point.pose,
      VelocityFactor::UNKNOWN);
  } else if (!decisions.empty()) {
    RCLCPP_WARN(logger_, "Could not insert stop point (would violate max deceleration limits)");
  }
  const auto insert_slowdown_points_us = stopwatch.toc("insert_slowdown_points");
  debug_data_.ranges = inputs.ranges;

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

  debug::add_footprint_markers(
    debug_marker_array, debug_data_.footprints, z, debug_data_.prev_footprints);
  debug::add_current_overlap_marker(
    debug_marker_array, debug_data_.current_footprint, debug_data_.current_overlapped_lanelets, z,
    debug_data_.prev_current_overlapped_lanelets);
  debug::add_lanelet_markers(
    debug_marker_array, debug_data_.path_lanelets, "path_lanelets",
    tier4_autoware_utils::createMarkerColor(0.1, 0.1, 1.0, 0.5), debug_data_.prev_path_lanelets);
  debug::add_lanelet_markers(
    debug_marker_array, debug_data_.ignored_lanelets, "ignored_lanelets",
    tier4_autoware_utils::createMarkerColor(0.7, 0.7, 0.2, 0.5), debug_data_.prev_ignored_lanelets);
  debug::add_lanelet_markers(
    debug_marker_array, debug_data_.other_lanelets, "other_lanelets",
    tier4_autoware_utils::createMarkerColor(0.4, 0.4, 0.7, 0.5), debug_data_.prev_other_lanelets);
  debug::add_range_markers(
    debug_marker_array, debug_data_.ranges, debug_data_.path, debug_data_.first_path_idx, z,
    debug_data_.prev_ranges);
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

}  // namespace behavior_velocity_planner::out_of_lane
