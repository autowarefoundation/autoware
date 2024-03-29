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

#include "scene_dynamic_obstacle_stop.hpp"

#include "collision.hpp"
#include "debug.hpp"
#include "footprint.hpp"
#include "object_filtering.hpp"
#include "types.hpp"

#include <behavior_velocity_planner_common/utilization/debug.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

DynamicObstacleStopModule::DynamicObstacleStopModule(
  const int64_t module_id, PlannerParam planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock), params_(std::move(planner_param))
{
  prev_stop_decision_time_ =
    clock->now() -
    rclcpp::Duration(std::chrono::duration<double>(params_.decision_duration_buffer));
  velocity_factor_.init(PlanningBehavior::ROUTE_OBSTACLE);
}

bool DynamicObstacleStopModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_.reset_data();
  *stop_reason = planning_utils::initializeStopReason(StopReason::OBSTACLE_STOP);
  if (!path || path->points.size() < 2) return true;
  tier4_autoware_utils::StopWatch<std::chrono::microseconds> stopwatch;
  stopwatch.tic();

  stopwatch.tic("preprocessing");
  EgoData ego_data;
  ego_data.pose = planner_data_->current_odometry->pose;
  ego_data.path.points = path->points;
  motion_utils::removeOverlapPoints(ego_data.path.points);
  ego_data.first_path_idx =
    motion_utils::findNearestSegmentIndex(ego_data.path.points, ego_data.pose.position);
  ego_data.longitudinal_offset_to_first_path_idx = motion_utils::calcLongitudinalOffsetToSegment(
    ego_data.path.points, ego_data.first_path_idx, ego_data.pose.position);
  const auto min_stop_distance =
    motion_utils::calcDecelDistWithJerkAndAccConstraints(
      planner_data_->current_velocity->twist.linear.x, 0.0,
      planner_data_->current_acceleration->accel.accel.linear.x,
      planner_data_->max_stop_acceleration_threshold, -planner_data_->max_stop_jerk_threshold,
      planner_data_->max_stop_jerk_threshold)
      .value_or(0.0);
  ego_data.earliest_stop_pose = motion_utils::calcLongitudinalOffsetPose(
    ego_data.path.points, ego_data.pose.position, min_stop_distance);

  make_ego_footprint_rtree(ego_data, params_);
  const auto start_time = clock_->now();
  const auto has_decided_to_stop =
    (start_time - prev_stop_decision_time_).seconds() < params_.decision_duration_buffer;
  if (!has_decided_to_stop) current_stop_pose_.reset();
  double hysteresis = has_decided_to_stop ? params_.hysteresis : 0.0;
  const auto dynamic_obstacles =
    filter_predicted_objects(*planner_data_->predicted_objects, ego_data, params_, hysteresis);

  const auto preprocessing_duration_us = stopwatch.toc("preprocessing");

  stopwatch.tic("footprints");
  const auto obstacle_forward_footprints =
    make_forward_footprints(dynamic_obstacles, params_, hysteresis);
  const auto footprints_duration_us = stopwatch.toc("footprints");
  stopwatch.tic("collisions");
  const auto collision =
    find_earliest_collision(ego_data, dynamic_obstacles, obstacle_forward_footprints, debug_data_);
  const auto collisions_duration_us = stopwatch.toc("collisions");
  if (collision) {
    const auto arc_length_diff =
      motion_utils::calcSignedArcLength(ego_data.path.points, *collision, ego_data.pose.position);
    const auto can_stop_before_limit = arc_length_diff < min_stop_distance -
                                                           params_.ego_longitudinal_offset -
                                                           params_.stop_distance_buffer;
    const auto stop_pose = can_stop_before_limit
                             ? motion_utils::calcLongitudinalOffsetPose(
                                 ego_data.path.points, *collision,
                                 -params_.stop_distance_buffer - params_.ego_longitudinal_offset)
                             : ego_data.earliest_stop_pose;
    if (stop_pose) {
      const auto use_new_stop_pose = !has_decided_to_stop || motion_utils::calcSignedArcLength(
                                                               path->points, stop_pose->position,
                                                               current_stop_pose_->position) > 0.0;
      if (use_new_stop_pose) current_stop_pose_ = *stop_pose;
      prev_stop_decision_time_ = start_time;
    }
  }

  if (current_stop_pose_) {
    motion_utils::insertStopPoint(*current_stop_pose_, 0.0, path->points);
    const auto stop_pose_reached =
      planner_data_->current_velocity->twist.linear.x < 1e-3 &&
      tier4_autoware_utils::calcDistance2d(ego_data.pose, *current_stop_pose_) < 1e-3;
    velocity_factor_.set(
      path->points, ego_data.pose, *current_stop_pose_,
      stop_pose_reached ? VelocityFactor::STOPPED : VelocityFactor::APPROACHING,
      "dynamic_obstacle_stop");
  }

  const auto total_time_us = stopwatch.toc();
  RCLCPP_DEBUG(
    logger_,
    "Total time = %2.2fus\n\tpreprocessing = %2.2fus\n\tfootprints = "
    "%2.2fus\n\tcollisions = %2.2fus\n",
    total_time_us, preprocessing_duration_us, footprints_duration_us, collisions_duration_us);
  debug_data_.ego_footprints = ego_data.path_footprints;
  debug_data_.obstacle_footprints = obstacle_forward_footprints;
  return true;
}

MarkerArray DynamicObstacleStopModule::createDebugMarkerArray()
{
  constexpr auto z = 0.0;
  MarkerArray debug_marker_array;
  // dynamic obstacles footprints
  const auto obstacle_footprint_markers =
    debug::make_polygon_markers(debug_data_.obstacle_footprints, "dynamic_obstacles_footprints", z);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), obstacle_footprint_markers.begin(),
    obstacle_footprint_markers.end());
  const auto delete_footprint_markers = debug::make_delete_markers(
    obstacle_footprint_markers.size(), debug_data_.prev_dynamic_obstacles_nb,
    "dynamic_obstacles_footprints");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_footprint_markers.begin(),
    delete_footprint_markers.end());
  // ego path footprints
  const auto ego_footprint_markers =
    debug::make_polygon_markers(debug_data_.ego_footprints, "ego_footprints", z);
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), ego_footprint_markers.begin(), ego_footprint_markers.end());
  const auto delete_ego_footprint_markers = debug::make_delete_markers(
    ego_footprint_markers.size(), debug_data_.prev_ego_footprints_nb, "ego_footprints");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_ego_footprint_markers.begin(),
    delete_ego_footprint_markers.end());
  // collisions
  auto collision_markers = debug::make_polygon_markers(debug_data_.collisions, "collisions", z);
  for (auto & m : collision_markers) m.color.r = 1.0;
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), collision_markers.begin(), collision_markers.end());
  const auto delete_collision_markers = debug::make_delete_markers(
    collision_markers.size(), debug_data_.prev_collisions_nb, "collisions");
  debug_marker_array.markers.insert(
    debug_marker_array.markers.end(), delete_collision_markers.begin(),
    delete_collision_markers.end());

  debug_data_.prev_dynamic_obstacles_nb = obstacle_footprint_markers.size();
  debug_data_.prev_collisions_nb = collision_markers.size();
  debug_data_.prev_ego_footprints_nb = ego_footprint_markers.size();
  return debug_marker_array;
}

motion_utils::VirtualWalls DynamicObstacleStopModule::createVirtualWalls()
{
  motion_utils::VirtualWalls virtual_walls{};
  if (current_stop_pose_) {
    motion_utils::VirtualWall virtual_wall;
    virtual_wall.text = "dynamic_obstacle_stop";
    virtual_wall.longitudinal_offset = params_.ego_longitudinal_offset;
    virtual_wall.style = motion_utils::VirtualWallType::stop;
    virtual_wall.pose = *current_stop_pose_;
    virtual_walls.push_back(virtual_wall);
  }
  return virtual_walls;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
