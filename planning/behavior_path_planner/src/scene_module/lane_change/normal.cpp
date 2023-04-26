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
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: LaneChangeBase(parameters, type, direction)
{
}

void NormalLaneChange::updateLaneChangeStatus()
{
  const auto [found_valid_path, found_safe_path] = getSafePath(status_.lane_change_path);

  // Update status
  status_.current_lanes = status_.lane_change_path.reference_lanelets;
  status_.lane_change_lanes = status_.lane_change_path.target_lanelets;
  status_.is_valid_path = found_valid_path;
  status_.is_safe = found_safe_path;
  status_.lane_follow_lane_ids = utils::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = utils::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

std::pair<bool, bool> NormalLaneChange::getSafePath(LaneChangePath & safe_path) const
{
  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return std::make_pair(false, false);
  }

  const auto target_lanes = getLaneChangeLanes(current_lanes);

  if (target_lanes.empty()) {
    return std::make_pair(false, false);
  }

  // find candidate paths
  LaneChangePaths valid_paths{};
  const auto found_safe_path = getLaneChangePaths(current_lanes, target_lanes, &valid_paths);

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

BehaviorModuleOutput NormalLaneChange::generateOutput()
{
  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(getLaneChangePath().path);

  extendOutputDrivableArea(output);

  if (isAbortState()) {
    return output;
  }

  if (isStopState()) {
    const auto stop_point = utils::insertStopPoint(0.1, *output.path);
  }

  output.reference_path = std::make_shared<PathWithLaneId>(getReferencePath());
  output.turn_signal_info = updateOutputTurnSignal();

  return output;
}

void NormalLaneChange::extendOutputDrivableArea(BehaviorModuleOutput & output)
{
  const auto & common_parameters = planner_data_->parameters;
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto drivable_lanes = getDrivableLanes();
  const auto shorten_lanes = utils::cutOverlappedLanes(*output.path, drivable_lanes);
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // for new architecture
  output.drivable_area_info.drivable_lanes = expanded_lanes;

  // for old architecture
  utils::generateDrivableArea(
    *output.path, expanded_lanes, common_parameters.vehicle_length, planner_data_);
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

lanelet::ConstLanelets NormalLaneChange::getCurrentLanes() const
{
  return utils::getCurrentLanesFromPath(*prev_module_reference_path_, planner_data_);
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

  const auto minimum_prepare_length = planner_data_->parameters.minimum_prepare_length;

  const auto lane_change_prepare_length = std::max(
    getEgoVelocity() * planner_data_->parameters.lane_change_prepare_duration,
    minimum_prepare_length);

  const auto & route_handler = getRouteHandler();

  const auto current_check_lanes =
    route_handler->getLaneletSequence(current_lane, getEgoPose(), 0.0, lane_change_prepare_length);

  const auto lane_change_lane = utils::lane_change::getLaneChangeTargetLane(
    *getRouteHandler(), current_lanes, type_, direction_);

  const auto lane_change_lane_length = std::max(lane_change_lane_length_, getEgoVelocity() * 10.0);
  if (lane_change_lane) {
    return route_handler->getLaneletSequence(
      lane_change_lane.get(), getEgoPose(), lane_change_lane_length, lane_change_lane_length);
  }

  return {};
}

int NormalLaneChange::getNumToPreferredLane(const lanelet::ConstLanelet & lane) const
{
  const auto get_opposite_direction =
    (direction_ == Direction::RIGHT) ? Direction::LEFT : Direction::RIGHT;
  return std::abs(getRouteHandler()->getNumLaneToPreferredLane(lane, get_opposite_direction));
}

PathWithLaneId NormalLaneChange::getPrepareSegment(
  const lanelet::ConstLanelets & current_lanes,
  [[maybe_unused]] const double arc_length_from_current, const double backward_path_length,
  const double prepare_length, const double prepare_velocity) const
{
  if (current_lanes.empty()) {
    return PathWithLaneId();
  }

  auto prepare_segment = *prev_module_path_;
  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    prepare_segment.points, getEgoPose(), 3.0, 1.0);
  utils::clipPathLength(prepare_segment, current_seg_idx, prepare_length, backward_path_length);

  prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
    prepare_segment.points.back().point.longitudinal_velocity_mps,
    static_cast<float>(prepare_velocity));

  return prepare_segment;
}

bool NormalLaneChange::getLaneChangePaths(
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  LaneChangePaths * candidate_paths) const
{
  object_debug_.clear();
  if (original_lanelets.empty() || target_lanelets.empty()) {
    return false;
  }
  const auto & route_handler = *getRouteHandler();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameter = planner_data_->parameters;

  Pose ego_pose_before_collision{};

  const auto backward_path_length = common_parameter.backward_path_length;
  const auto forward_path_length = common_parameter.forward_path_length;
  const auto prepare_duration = common_parameter.lane_change_prepare_duration;
  const auto minimum_prepare_length = common_parameter.minimum_prepare_length;
  const auto minimum_lane_changing_velocity = common_parameter.minimum_lane_changing_velocity;
  const auto lane_change_sampling_num = parameters_->lane_change_sampling_num;

  // get velocity
  const auto current_velocity = getEgoTwist().linear.x;

  // compute maximum_deceleration
  const auto maximum_deceleration =
    std::invoke([&minimum_lane_changing_velocity, &current_velocity, &common_parameter, this]() {
      const double min_a = (minimum_lane_changing_velocity - current_velocity) /
                           common_parameter.lane_change_prepare_duration;
      return std::clamp(
        min_a, -std::abs(common_parameter.min_acc), -std::numeric_limits<double>::epsilon());
    });

  const auto acceleration_resolution = std::abs(maximum_deceleration) / lane_change_sampling_num;

  const auto target_length =
    utils::getArcLengthToTargetLanelet(original_lanelets, target_lanelets.front(), getEgoPose());

  const auto is_goal_in_route = route_handler.isInGoalRouteSection(target_lanelets.back());

  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(target_lanelets.back());
  const double lane_change_buffer =
    utils::calcMinimumLaneChangeLength(common_parameter, shift_intervals);

  const auto dist_to_end_of_current_lanes =
    utils::getDistanceToEndOfLane(getEgoPose(), original_lanelets) - lane_change_buffer;

  [[maybe_unused]] const auto arc_position_from_current =
    lanelet::utils::getArcCoordinates(original_lanelets, getEgoPose());
  const auto arc_position_from_target =
    lanelet::utils::getArcCoordinates(target_lanelets, getEgoPose());

  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanelets);

  const auto sorted_lane_ids = utils::lane_change::getSortedLaneIds(
    route_handler, original_lanelets, target_lanelets, arc_position_from_target.distance);
  const auto lateral_buffer =
    utils::lane_change::calcLateralBufferForFiltering(common_parameter.vehicle_width, 0.5);

  LaneChangeTargetObjectIndices dynamic_object_indices;

  candidate_paths->reserve(lane_change_sampling_num);
  for (double sampled_acc = 0.0; sampled_acc >= maximum_deceleration;
       sampled_acc -= acceleration_resolution) {
    const auto prepare_velocity =
      std::max(current_velocity + sampled_acc * prepare_duration, minimum_lane_changing_velocity);

    // compute actual acceleration
    const double acceleration = (prepare_velocity - current_velocity) / prepare_duration;

    // get path on original lanes
    const double prepare_length = std::max(
      current_velocity * prepare_duration + 0.5 * acceleration * std::pow(prepare_duration, 2),
      minimum_prepare_length);

    if (prepare_length < target_length) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "prepare length is shorter than distance to target lane!!");
      break;
    }

    const auto prepare_segment = getPrepareSegment(
      original_lanelets, arc_position_from_current.length, backward_path_length, prepare_length,
      prepare_velocity);

    if (prepare_segment.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "prepare segment is empty!!");
      continue;
    }

    // lane changing start getEgoPose() is at the end of prepare segment
    const auto & lane_changing_start_pose = prepare_segment.points.back().point.pose;

    const auto target_length_from_lane_change_start_pose = utils::getArcLengthToTargetLanelet(
      original_lanelets, target_lanelets.front(), lane_changing_start_pose);
    // In new architecture, there is a possibility that the lane change start getEgoPose() is behind
    // of the target lanelet, even if the condition prepare_length > target_length is satisfied. In
    // that case, the lane change shouldn't be executed.
    if (target_length_from_lane_change_start_pose > 0.0) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "[only new arch] lane change start getEgoPose() is behind target lanelet!!");
      break;
    }

    const auto shift_length =
      lanelet::utils::getLateralDistanceToClosestLanelet(target_lanelets, lane_changing_start_pose);

    // we assume constant velocity during lane change
    const auto lane_changing_velocity = prepare_velocity;
    const auto lane_changing_length = utils::lane_change::calcLaneChangingLength(
      lane_changing_velocity, shift_length, common_parameter);

    if (lane_changing_length + prepare_length > dist_to_end_of_current_lanes) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "lane changing path too long");
      continue;
    }

    if (is_goal_in_route) {
      const double s_start =
        lanelet::utils::getArcCoordinates(target_lanelets, lane_changing_start_pose).length;
      const double s_goal =
        lanelet::utils::getArcCoordinates(target_lanelets, route_handler.getGoalPose()).length;
      if (
        s_start + lane_changing_length + parameters_->lane_change_finish_judge_buffer +
          lane_change_buffer >
        s_goal) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
          "length of lane changing path is longer than length to goal!!");
        continue;
      }
    }

    const auto target_segment = utils::lane_change::getTargetSegment(
      route_handler, target_lanelets, forward_path_length, lane_changing_start_pose,
      target_lane_length, lane_changing_length, lane_changing_velocity, lane_change_buffer);

    if (target_segment.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "target segment is empty!! something wrong...");
      continue;
    }

    const auto resample_interval = utils::lane_change::calcLaneChangeResampleInterval(
      lane_changing_length, lane_changing_velocity);

    const auto lc_length = LaneChangePhaseInfo{prepare_length, lane_changing_length};
    const auto target_lane_reference_path = utils::lane_change::getReferencePathFromTargetLane(
      route_handler, target_lanelets, lane_changing_start_pose, target_lane_length,
      lc_length.lane_changing, forward_path_length, resample_interval, is_goal_in_route);

    if (target_lane_reference_path.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "target_lane_reference_path is empty!!");
      continue;
    }

    const auto shift_line = utils::lane_change::getLaneChangingShiftLine(
      prepare_segment, target_segment, target_lane_reference_path, shift_length);

    const auto lc_velocity = LaneChangePhaseInfo{prepare_velocity, lane_changing_velocity};

    const auto candidate_path = utils::lane_change::constructCandidatePath(
      prepare_segment, target_segment, target_lane_reference_path, shift_line, original_lanelets,
      target_lanelets, sorted_lane_ids, acceleration, lc_length, lc_velocity, common_parameter,
      *parameters_);

    if (!candidate_path) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "no candidate path!!");
      continue;
    }

    const auto is_valid = utils::lane_change::hasEnoughLength(
      *candidate_path, original_lanelets, target_lanelets, getEgoPose(), route_handler,
      minimum_lane_changing_velocity, common_parameter, direction_);

    if (!is_valid) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
        "invalid candidate path!!");
      continue;
    }

    if (candidate_paths->empty()) {
      // only compute dynamic object indices once
      const auto backward_lanes = utils::lane_change::getExtendedTargetLanesForCollisionCheck(
        route_handler, target_lanelets.front(), getEgoPose(), check_length_);
      dynamic_object_indices = utils::lane_change::filterObjectIndices(
        {*candidate_path}, *dynamic_objects, backward_lanes, getEgoPose(),
        common_parameter.forward_path_length, *parameters_, lateral_buffer);
    }
    candidate_paths->push_back(*candidate_path);

    const auto is_safe = utils::lane_change::isLaneChangePathSafe(
      *candidate_path, dynamic_objects, dynamic_object_indices, getEgoPose(), getEgoTwist(),
      common_parameter, *parameters_, common_parameter.expected_front_deceleration,
      common_parameter.expected_rear_deceleration, ego_pose_before_collision, object_debug_,
      acceleration);

    if (is_safe) {
      return true;
    }
  }

  return false;
}

std::vector<DrivableLanes> NormalLaneChange::getDrivableLanes() const
{
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), status_.current_lanes, status_.lane_change_lanes);
  return utils::combineDrivableLanes(*prev_drivable_lanes_, drivable_lanes);
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
    *route_handler, path.target_lanelets.front(), current_pose, check_length_);

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
    const auto prepare_duration = planner_data_->parameters.lane_change_prepare_duration;
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
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  // check lane departure
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, utils::extendLanes(route_handler, status_.current_lanes),
    utils::extendLanes(route_handler, status_.lane_change_lanes));
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

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
