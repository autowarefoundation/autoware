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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_

#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <behavior_path_planner_common/data_manager.hpp>
#include <behavior_path_planner_common/parameters.hpp>
#include <freespace_planning_algorithms/abstract_algorithm.hpp>

#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/Forward.h>

#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_path_planner::utils
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

std::vector<double> calcPathArcLengthArray(
  const PathWithLaneId & path, const size_t start = 0,
  const size_t end = std::numeric_limits<size_t>::max(), const double offset = 0.0);

/**
 * @brief resample path by spline with constant interval distance
 * @param [in] path original path to be resampled
 * @param [in] interval constant interval distance
 * @param [in] keep_input_points original points are kept in the resampled points
 * @param [in] target_section target section defined by arclength if you want to resample a part of
 * the path
 * @return resampled path
 */
PathWithLaneId resamplePathWithSpline(
  const PathWithLaneId & path, const double interval, const bool keep_input_points = false,
  const std::pair<double, double> target_section = {0.0, std::numeric_limits<double>::max()});

Path toPath(const PathWithLaneId & input);

size_t getIdxByArclength(
  const PathWithLaneId & path, const size_t target_idx, const double signed_arc);

void clipPathLength(
  PathWithLaneId & path, const size_t target_idx, const double forward, const double backward);

void clipPathLength(
  PathWithLaneId & path, const size_t target_idx, const BehaviorPathPlannerParameters & params);

std::pair<TurnIndicatorsCommand, double> getPathTurnSignal(
  const lanelet::ConstLanelets & current_lanes, const ShiftedPath & path,
  const ShiftLine & shift_line, const Pose & pose, const double & velocity,
  const BehaviorPathPlannerParameters & common_parameter);

PathWithLaneId convertWayPointsToPathWithLaneId(
  const freespace_planning_algorithms::PlannerWaypoints & waypoints, const double velocity,
  const lanelet::ConstLanelets & lanelets);

std::vector<size_t> getReversingIndices(const PathWithLaneId & path);

std::vector<PathWithLaneId> dividePath(
  const PathWithLaneId & path, const std::vector<size_t> indices);

void correctDividedPathVelocity(std::vector<PathWithLaneId> & divided_paths);

bool isCloseToPath(const PathWithLaneId & path, const Pose & pose, const double distance_threshold);

// only two points is supported
std::vector<double> splineTwoPoints(
  std::vector<double> base_s, std::vector<double> base_x, const double begin_diff,
  const double end_diff, std::vector<double> new_s);

std::vector<Pose> interpolatePose(
  const Pose & start_pose, const Pose & end_pose, const double resample_interval);

geometry_msgs::msg::Pose getUnshiftedEgoPose(
  const geometry_msgs::msg::Pose & ego_pose, const ShiftedPath & prev_path);

PathWithLaneId calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data, const Pose & ref_pose,
  const double longest_dist_to_shift_line,
  const std::optional<PathWithLaneId> & prev_module_path = std::nullopt);

PathWithLaneId combinePath(const PathWithLaneId & path1, const PathWithLaneId & path2);

std::optional<Pose> getFirstStopPoseFromPath(const PathWithLaneId & path);

BehaviorModuleOutput getReferencePath(
  const lanelet::ConstLanelet & current_lane,
  const std::shared_ptr<const PlannerData> & planner_data);

BehaviorModuleOutput createGoalAroundPath(const std::shared_ptr<const PlannerData> & planner_data);

}  // namespace behavior_path_planner::utils

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_UTILS_HPP_
