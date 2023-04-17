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

#include "behavior_path_planner/scene_module/lane_change/normal.hpp"

#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
NormalLaneChange::NormalLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters, Direction direction)
: LaneChangeBase(parameters, direction)
{
}

void NormalLaneChange::updateLaneChangeStatus(
  const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & previous_module_path)
{
  status_.current_lanes = utils::getCurrentLanesFromPath(prev_module_reference_path, planner_data_);
  status_.lane_change_lanes = getLaneChangeLanes(status_.current_lanes);

  // Find lane change path
  const auto [found_valid_path, found_safe_path] =
    getSafePath(prev_module_reference_path, previous_module_path, status_.lane_change_path);

  // Update status
  status_.is_valid_path = found_valid_path;
  status_.is_safe = found_safe_path;
  status_.lane_follow_lane_ids = utils::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = utils::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

std::pair<bool, bool> NormalLaneChange::getSafePath(
  const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & prev_module_path,
  LaneChangePath & safe_path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto & common_parameters = planner_data_->parameters;

  const auto current_lanes =
    utils::getCurrentLanesFromPath(prev_module_reference_path, planner_data_);

  if (current_lanes.empty()) {
    return std::make_pair(false, false);
  }

  const auto lane_change_lanes = getLaneChangeLanes(current_lanes);

  if (lane_change_lanes.empty()) {
    return std::make_pair(false, false);
  }

  // find candidate paths
  LaneChangePaths valid_paths;

  const auto found_safe_path = utils::lane_change::getLaneChangePaths(
    prev_module_path, *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
    planner_data_->dynamic_object, common_parameters, *parameters_, check_distance_, direction_,
    &valid_paths, &object_debug_);

  if (valid_paths.empty()) {
    return {false, false};
  }

  if (found_safe_path) {
    safe_path = valid_paths.back();
  } else {
    // force candidate
    safe_path = valid_paths.front();
  }

  return {true, found_safe_path};
}

PathWithLaneId NormalLaneChange::generatePlannedPath(
  const std::vector<DrivableLanes> & prev_drivable_lanes)
{
  auto path = getLaneChangePath().path;
  generateExtendedDrivableArea(prev_drivable_lanes, path);

  if (isAbortState()) {
    return path;
  }

  if (isStopState()) {
    const auto stop_point = utils::insertStopPoint(0.1, path);
  }

  return path;
}

void NormalLaneChange::generateExtendedDrivableArea(
  const std::vector<DrivableLanes> & prev_drivable_lanes, PathWithLaneId & path)
{
  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;
  auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, status_.current_lanes, status_.lane_change_lanes);
  drivable_lanes = utils::lane_change::combineDrivableLanes(prev_drivable_lanes, drivable_lanes);
  const auto shorten_lanes = utils::cutOverlappedLanes(path, drivable_lanes);
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);
  utils::generateDrivableArea(
    path, expanded_lanes, common_parameters.vehicle_length, planner_data_);
}
bool NormalLaneChange::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto & lane_change_path = status_.lane_change_path.path;
  const auto & lane_change_end = status_.lane_change_path.shift_line.end;
  const double dist_to_lane_change_end = motion_utils::calcSignedArcLength(
    lane_change_path.points, current_pose.position, lane_change_end.position);
  return dist_to_lane_change_end + parameters_->lane_change_finish_judge_buffer < 0.0;
}

PathWithLaneId NormalLaneChange::getReferencePath() const
{
  return utils::getCenterLinePathFromRootLanelet(status_.lane_change_lanes.front(), planner_data_);
}

bool NormalLaneChange::isCancelConditionSatisfied()
{
  current_lane_change_state_ = LaneChangeStates::Normal;

  if (!parameters_->enable_cancel_lane_change) {
    return false;
  }

  Pose ego_pose_before_collision;
  const auto is_path_safe = isApprovedPathSafe(ego_pose_before_collision);

  if (!is_path_safe) {
    const auto & common_parameters = planner_data_->parameters;
    const bool is_within_original_lane = utils::lane_change::isEgoWithinOriginalLane(
      status_.current_lanes, getEgoPose(), common_parameters);

    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      current_lane_change_state_ = LaneChangeStates::Stop;
      return false;
    }

    if (is_within_original_lane) {
      current_lane_change_state_ = LaneChangeStates::Cancel;
      return true;
    }

    if (!parameters_->enable_abort_lane_change) {
      return false;
    }

    return isAbortConditionSatisfied(ego_pose_before_collision);
  }

  return false;
}

bool NormalLaneChange::isAbortConditionSatisfied(const Pose & pose)
{
  const auto & common_parameters = planner_data_->parameters;

  const auto found_abort_path = utils::lane_change::getAbortPaths(
    planner_data_, status_.lane_change_path, pose, common_parameters, *parameters_);

  if (!found_abort_path && !is_abort_path_approved_) {
    current_lane_change_state_ = LaneChangeStates::Stop;
    return true;
  }

  current_lane_change_state_ = LaneChangeStates::Abort;

  if (!is_abort_path_approved_) {
    abort_path_ = std::make_shared<LaneChangePath>(*found_abort_path);
  }

  return true;
}

void NormalLaneChange::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;

  object_debug_.clear();
}

TurnSignalInfo NormalLaneChange::updateOutputTurnSignal()
{
  calcTurnSignalInfo();
  TurnSignalInfo turn_signal_info;
  const auto [turn_signal_command, distance_to_vehicle_front] = utils::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_line, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  turn_signal_info.turn_signal.command = turn_signal_command.command;

  turn_signal_info.desired_start_point =
    status_.lane_change_path.turn_signal_info.desired_start_point;
  turn_signal_info.required_start_point =
    status_.lane_change_path.turn_signal_info.required_start_point;
  turn_signal_info.required_end_point =
    status_.lane_change_path.turn_signal_info.required_end_point;
  turn_signal_info.desired_end_point = status_.lane_change_path.turn_signal_info.desired_end_point;

  return turn_signal_info;
}

lanelet::ConstLanelets NormalLaneChange::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes) const
{
  if (current_lanes.empty()) {
    return {};
  }
  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, getEgoPose(), &current_lane);

  const auto minimum_lane_changing_length = planner_data_->parameters.minimum_lane_changing_length;

  const auto lane_change_prepare_length =
    std::max(getEgoVelocity() * parameters_->prepare_duration, minimum_lane_changing_length);

  const auto & route_handler = getRouteHandler();

  const auto current_check_lanes =
    route_handler->getLaneletSequence(current_lane, getEgoPose(), 0.0, lane_change_prepare_length);

  const auto lane_change_lane = route_handler->getLaneChangeTarget(current_check_lanes, direction_);

  if (lane_change_lane) {
    return route_handler->getLaneletSequence(
      lane_change_lane.get(), getEgoPose(), lane_change_lane_length_, lane_change_lane_length_);
  }

  return {};
}

bool NormalLaneChange::isApprovedPathSafe(Pose & ego_pose_before_collision) const
{
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = planner_data_->parameters;
  const auto & lane_change_parameters = *parameters_;
  const auto & route_handler = planner_data_->route_handler;
  const auto & path = status_.lane_change_path;

  // get lanes used for detection
  const auto check_lanes = utils::lane_change::getExtendedTargetLanesForCollisionCheck(
    *route_handler, path.target_lanelets.front(), current_pose, check_distance_);

  CollisionCheckDebugMap debug_data;
  const auto lateral_buffer =
    utils::lane_change::calcLateralBufferForFiltering(common_parameters.vehicle_width);
  const auto dynamic_object_indices = utils::lane_change::filterObjectIndices(
    {path}, *dynamic_objects, check_lanes, current_pose, common_parameters.forward_path_length,
    lane_change_parameters, lateral_buffer);

  return utils::lane_change::isLaneChangePathSafe(
    path, dynamic_objects, dynamic_object_indices, current_pose, current_twist, common_parameters,
    *parameters_, common_parameters.expected_front_deceleration_for_abort,
    common_parameters.expected_rear_deceleration_for_abort, ego_pose_before_collision, debug_data,
    status_.lane_change_path.acceleration);
}

void NormalLaneChange::calcTurnSignalInfo()
{
  const auto get_blinker_pose = [](const PathWithLaneId & path, const double length) {
    double accumulated_length = 0.0;
    for (size_t i = 0; i < path.points.size() - 1; ++i) {
      accumulated_length +=
        tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
      if (accumulated_length > length) {
        return path.points.at(i).point.pose;
      }
    }

    return path.points.front().point.pose;
  };

  const auto & path = status_.lane_change_path;
  TurnSignalInfo turn_signal_info{};

  turn_signal_info.desired_start_point = std::invoke([&]() {
    const auto blinker_start_duration = planner_data_->parameters.turn_signal_search_time;
    const auto prepare_duration = parameters_->prepare_duration;
    const auto diff_time = prepare_duration - blinker_start_duration;
    if (diff_time < 1e-5) {
      return path.path.points.front().point.pose;
    }

    const auto current_twist = getEgoTwist();
    const auto diff_length = std::abs(current_twist.linear.x) * diff_time;
    return get_blinker_pose(path.path, diff_length);
  });
  turn_signal_info.desired_end_point = path.shift_line.end;

  turn_signal_info.required_start_point = path.shift_line.start;
  const auto mid_lane_change_length = path.length.lane_changing / 2;
  const auto & shifted_path = path.shifted_path.path;
  turn_signal_info.required_end_point = get_blinker_pose(shifted_path, mid_lane_change_length);

  status_.lane_change_path.turn_signal_info = turn_signal_info;
}

bool NormalLaneChange::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;

  // check lane departure
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, utils::extendLanes(route_handler, status_.current_lanes),
    utils::extendLanes(route_handler, status_.lane_change_lanes));
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);
  const auto lanelets = utils::transformToLanelets(expanded_lanes);

  // check path points are in any lanelets
  for (const auto & point : path.points) {
    bool is_in_lanelet = false;
    for (const auto & lanelet : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lanelet)) {
        is_in_lanelet = true;
        break;
      }
    }
    if (!is_in_lanelet) {
      return false;
    }
  }

  // check relative angle
  if (!utils::checkPathRelativeAngle(path, M_PI)) {
    return false;
  }

  return true;
}
}  // namespace behavior_path_planner
