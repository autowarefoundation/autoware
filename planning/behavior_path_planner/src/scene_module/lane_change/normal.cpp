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
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
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
    return {false, false};
  }

  const auto target_lanes = getLaneChangeLanes(current_lanes, direction_);

  if (target_lanes.empty()) {
    return {false, false};
  }

  // find candidate paths
  LaneChangePaths valid_paths{};
  const auto found_safe_path =
    getLaneChangePaths(current_lanes, target_lanes, direction_, &valid_paths);

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

LaneChangePath NormalLaneChange::getLaneChangePath() const
{
  return isAbortState() ? *abort_path_ : status_.lane_change_path;
}

BehaviorModuleOutput NormalLaneChange::generateOutput()
{
  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(getLaneChangePath().path);
  extendOutputDrivableArea(output);
  output.reference_path = std::make_shared<PathWithLaneId>(getReferencePath());
  output.turn_signal_info = updateOutputTurnSignal();

  if (isAbortState()) {
    return output;
  }

  if (isStopState()) {
    const auto current_velocity = getEgoVelocity();
    const auto current_dist = motion_utils::calcSignedArcLength(
      output.path->points, output.path->points.front().point.pose.position, getEgoPosition());
    const auto stop_dist =
      -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));
    const auto stop_point = utils::insertStopPoint(stop_dist + current_dist, *output.path);
  }

  output.reference_path = std::make_shared<PathWithLaneId>(getReferencePath());
  output.turn_signal_info = updateOutputTurnSignal();

  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path->points);
  output.turn_signal_info = planner_data_->turn_signal_decider.use_prior_turn_signal(
    *output.path, getEgoPose(), current_seg_idx, prev_turn_signal_info_, output.turn_signal_info,
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

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
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = expanded_lanes;
  output.drivable_area_info =
    utils::combineDrivableAreaInfo(current_drivable_area_info, prev_drivable_area_info_);

  // for old architecture
  utils::generateDrivableArea(
    *output.path, expanded_lanes, false, common_parameters.vehicle_length, planner_data_);
}

PathWithLaneId NormalLaneChange::getReferencePath() const
{
  return utils::getCenterLinePathFromRootLanelet(status_.lane_change_lanes.front(), planner_data_);
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
  return utils::getCurrentLanesFromPath(prev_module_reference_path_, planner_data_);
}

lanelet::ConstLanelets NormalLaneChange::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, Direction direction) const
{
  if (current_lanes.empty()) {
    return {};
  }
  // Get lane change lanes
  const auto & route_handler = getRouteHandler();

  const auto lane_change_lane = utils::lane_change::getLaneChangeTargetLane(
    *getRouteHandler(), current_lanes, type_, direction);

  if (!lane_change_lane) {
    return {};
  }

  const auto front_pose = std::invoke([&lane_change_lane]() {
    const auto & p = lane_change_lane->centerline().front();
    const auto front_point = lanelet::utils::conversion::toGeomMsgPt(p);
    const auto front_yaw = lanelet::utils::getLaneletAngle(*lane_change_lane, front_point);
    geometry_msgs::msg::Pose front_pose;
    front_pose.position = front_point;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, front_yaw);
    front_pose.orientation = tf2::toMsg(quat);
    return front_pose;
  });

  const auto forward_length = std::invoke([&]() {
    const auto signed_distance = utils::getSignedDistance(front_pose, getEgoPose(), current_lanes);
    const auto forward_path_length = planner_data_->parameters.forward_path_length;
    if (signed_distance <= 0.0) {
      return forward_path_length;
    }

    return signed_distance + forward_path_length;
  });
  const auto backward_length = lane_change_parameters_->backward_lane_length;

  return route_handler->getLaneletSequence(
    lane_change_lane.get(), getEgoPose(), backward_length, forward_length);
}

bool NormalLaneChange::isNearEndOfLane() const
{
  const auto & route_handler = getRouteHandler();
  const auto & current_pose = getEgoPose();
  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(status_.current_lanes.back());
  const auto threshold =
    utils::calcMinimumLaneChangeLength(planner_data_->parameters, shift_intervals);

  auto distance_to_end = utils::getDistanceToEndOfLane(current_pose, status_.current_lanes);

  if (route_handler->isInGoalRouteSection(status_.lane_change_lanes.back())) {
    distance_to_end = std::min(
      distance_to_end,
      utils::getSignedDistance(current_pose, route_handler->getGoalPose(), status_.current_lanes));
  }

  return (std::max(0.0, distance_to_end) - threshold) <
         planner_data_->parameters.backward_length_buffer_for_end_of_lane;
}

bool NormalLaneChange::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto & lane_change_path = status_.lane_change_path.path;
  const auto & lane_change_end = status_.lane_change_path.shift_line.end;
  const double dist_to_lane_change_end = motion_utils::calcSignedArcLength(
    lane_change_path.points, current_pose.position, lane_change_end.position);
  if (dist_to_lane_change_end + lane_change_parameters_->lane_change_finish_judge_buffer < 0.0) {
    return true;
  }
  return false;
}

bool NormalLaneChange::isAbleToReturnCurrentLane() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    return false;
  }

  const auto nearest_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const double ego_velocity =
    std::max(getEgoVelocity(), planner_data_->parameters.minimum_lane_changing_velocity);
  const double estimated_travel_dist = ego_velocity * lane_change_parameters_->abort_delta_time;

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += motion_utils::calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > estimated_travel_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      return utils::lane_change::isEgoWithinOriginalLane(
        status_.current_lanes, estimated_pose, planner_data_->parameters);
    }
  }

  return true;
}

bool NormalLaneChange::isEgoOnPreparePhase() const
{
  const auto & start_position = status_.lane_change_path.shift_line.start.position;
  const auto & path_points = status_.lane_change_path.path.points;
  return motion_utils::calcSignedArcLength(path_points, start_position, getEgoPosition()) < 0.0;
}

bool NormalLaneChange::isAbleToStopSafely() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    return false;
  }

  const auto nearest_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const auto current_velocity = getEgoVelocity();
  const auto stop_dist =
    -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += motion_utils::calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > stop_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      return utils::lane_change::isEgoWithinOriginalLane(
        status_.current_lanes, estimated_pose, planner_data_->parameters);
    }
  }
  return true;
}

bool NormalLaneChange::hasFinishedAbort() const
{
  if (!abort_path_) {
    return true;
  }

  const auto distance_to_finish = motion_utils::calcSignedArcLength(
    abort_path_->path.points, getEgoPosition(), abort_path_->shift_line.end.position);

  if (distance_to_finish < 0.0) {
    return true;
  }

  return false;
}

bool NormalLaneChange::isAbortState() const
{
  if (!lane_change_parameters_->enable_abort_lane_change) {
    return false;
  }

  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    return false;
  }

  if (!abort_path_) {
    return false;
  }

  return true;
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

  auto prepare_segment = prev_module_path_;
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
  Direction direction, LaneChangePaths * candidate_paths) const
{
  object_debug_.clear();
  if (original_lanelets.empty() || target_lanelets.empty()) {
    return false;
  }
  const auto & route_handler = *getRouteHandler();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameter = planner_data_->parameters;

  const auto backward_path_length = common_parameter.backward_path_length;
  const auto forward_path_length = common_parameter.forward_path_length;
  const auto prepare_duration = common_parameter.lane_change_prepare_duration;
  const auto minimum_prepare_length = common_parameter.minimum_prepare_length;
  const auto minimum_lane_changing_velocity = common_parameter.minimum_lane_changing_velocity;
  const auto longitudinal_acc_sampling_num = lane_change_parameters_->longitudinal_acc_sampling_num;
  const auto lateral_acc_sampling_num = lane_change_parameters_->lateral_acc_sampling_num;

  // get velocity
  const auto current_velocity = getEgoTwist().linear.x;

  // compute maximum longitudinal deceleration
  const auto maximum_deceleration =
    std::invoke([&minimum_lane_changing_velocity, &current_velocity, &common_parameter, this]() {
      const double min_a = (minimum_lane_changing_velocity - current_velocity) /
                           common_parameter.lane_change_prepare_duration;
      return std::clamp(
        min_a, -std::abs(common_parameter.min_acc), -std::numeric_limits<double>::epsilon());
    });

  const auto longitudinal_acc_resolution =
    std::abs(maximum_deceleration) / longitudinal_acc_sampling_num;

  const auto target_length =
    utils::getArcLengthToTargetLanelet(original_lanelets, target_lanelets.front(), getEgoPose());

  const auto is_goal_in_route = route_handler.isInGoalRouteSection(target_lanelets.back());

  const auto shift_intervals =
    route_handler.getLateralIntervalsToPreferredLane(target_lanelets.back());
  const double next_lane_change_buffer =
    utils::calcMinimumLaneChangeLength(common_parameter, shift_intervals);

  const auto dist_to_end_of_current_lanes =
    utils::getDistanceToEndOfLane(getEgoPose(), original_lanelets);

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

  candidate_paths->reserve(longitudinal_acc_sampling_num * lateral_acc_sampling_num);
  for (double sampled_longitudinal_acc = 0.0; sampled_longitudinal_acc >= maximum_deceleration;
       sampled_longitudinal_acc -= longitudinal_acc_resolution) {
    const auto prepare_velocity = std::max(
      current_velocity + sampled_longitudinal_acc * prepare_duration,
      minimum_lane_changing_velocity);

    // compute actual longitudinal acceleration
    const double longitudinal_acc = (prepare_velocity - current_velocity) / prepare_duration;

    // get path on original lanes
    const double prepare_length = std::max(
      current_velocity * prepare_duration + 0.5 * longitudinal_acc * std::pow(prepare_duration, 2),
      minimum_prepare_length);

    if (prepare_length < target_length) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
        "prepare length is shorter than distance to target lane!!");
      break;
    }

    const auto prepare_segment = getPrepareSegment(
      original_lanelets, arc_position_from_current.length, backward_path_length, prepare_length,
      prepare_velocity);

    if (prepare_segment.points.empty()) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
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
        rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
        "[only new arch] lane change start getEgoPose() is behind target lanelet!!");
      break;
    }

    const auto shift_length =
      lanelet::utils::getLateralDistanceToClosestLanelet(target_lanelets, lane_changing_start_pose);

    // we assume constant velocity during lane change
    const auto lane_changing_velocity = prepare_velocity;

    // get lateral acceleration range
    const auto [min_lateral_acc, max_lateral_acc] =
      common_parameter.lane_change_lat_acc_map.find(lane_changing_velocity);
    const auto lateral_acc_resolution =
      std::abs(max_lateral_acc - min_lateral_acc) / lateral_acc_sampling_num;
    constexpr double lateral_acc_epsilon = 0.01;

    for (double lateral_acc = min_lateral_acc; lateral_acc < max_lateral_acc + lateral_acc_epsilon;
         lateral_acc += lateral_acc_resolution) {
      const auto lane_changing_length = utils::lane_change::calcLaneChangingLength(
        lane_changing_velocity, shift_length, lateral_acc,
        common_parameter.lane_changing_lateral_jerk);

      if (lane_changing_length + prepare_length > dist_to_end_of_current_lanes) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
          "length of lane changing path is longer than length to goal!!");
        continue;
      }

      if (is_goal_in_route) {
        const double s_start =
          lanelet::utils::getArcCoordinates(target_lanelets, lane_changing_start_pose).length;
        const double s_goal =
          lanelet::utils::getArcCoordinates(target_lanelets, route_handler.getGoalPose()).length;
        if (
          s_start + lane_changing_length +
            lane_change_parameters_->lane_change_finish_judge_buffer + next_lane_change_buffer >
          s_goal) {
          RCLCPP_DEBUG(
            rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
            "length of lane changing path is longer than length to goal!!");
          continue;
        }
      }

      const auto target_segment = utils::lane_change::getTargetSegment(
        route_handler, target_lanelets, forward_path_length, lane_changing_start_pose,
        target_lane_length, lane_changing_length, lane_changing_velocity, next_lane_change_buffer);

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
        lc_length.lane_changing, forward_path_length, resample_interval, is_goal_in_route,
        next_lane_change_buffer);

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
        target_lanelets, sorted_lane_ids, longitudinal_acc, lateral_acc, lc_length, lc_velocity,
        common_parameter, *lane_change_parameters_);

      if (!candidate_path) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
          "no candidate path!!");
        continue;
      }

      const auto is_valid = utils::lane_change::hasEnoughLength(
        *candidate_path, original_lanelets, target_lanelets, getEgoPose(), route_handler,
        minimum_lane_changing_velocity, common_parameter, direction);

      if (!is_valid) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("behavior_path_planner").get_child("util").get_child("lane_change"),
          "invalid candidate path!!");
        continue;
      }

      if (candidate_paths->empty()) {
        // only compute dynamic object indices once
        const auto backward_length = lane_change_parameters_->backward_lane_length;
        const auto backward_target_lanes_for_object_filtering =
          utils::lane_change::getBackwardLanelets(
            route_handler, target_lanelets, getEgoPose(), backward_length);
        dynamic_object_indices = utils::lane_change::filterObjectIndices(
          {*candidate_path}, *dynamic_objects, backward_target_lanes_for_object_filtering,
          getEgoPose(), common_parameter.forward_path_length, *lane_change_parameters_,
          lateral_buffer);
      }
      candidate_paths->push_back(*candidate_path);

      const auto [is_safe, is_object_coming_from_rear] = utils::lane_change::isLaneChangePathSafe(
        *candidate_path, dynamic_objects, dynamic_object_indices, getEgoPose(), getEgoTwist(),
        common_parameter, *lane_change_parameters_, common_parameter.expected_front_deceleration,
        common_parameter.expected_rear_deceleration, object_debug_, longitudinal_acc);

      if (is_safe) {
        return true;
      }
    }
  }

  return false;
}

std::vector<DrivableLanes> NormalLaneChange::getDrivableLanes() const
{
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), status_.current_lanes, status_.lane_change_lanes);
  return utils::combineDrivableLanes(prev_drivable_area_info_.drivable_lanes, drivable_lanes);
}

PathSafetyStatus NormalLaneChange::isApprovedPathSafe() const
{
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = getCommonParam();
  const auto & lane_change_parameters = *lane_change_parameters_;
  const auto & route_handler = getRouteHandler();
  const auto & path = status_.lane_change_path;

  // get lanes used for detection
  const auto backward_target_lanes_for_object_filtering = utils::lane_change::getBackwardLanelets(
    *route_handler, path.target_lanelets, current_pose,
    lane_change_parameters.backward_lane_length);

  CollisionCheckDebugMap debug_data;
  const auto lateral_buffer =
    utils::lane_change::calcLateralBufferForFiltering(common_parameters.vehicle_width);
  const auto dynamic_object_indices = utils::lane_change::filterObjectIndices(
    {path}, *dynamic_objects, backward_target_lanes_for_object_filtering, current_pose,
    common_parameters.forward_path_length, lane_change_parameters, lateral_buffer);

  const auto safety_status = utils::lane_change::isLaneChangePathSafe(
    path, dynamic_objects, dynamic_object_indices, current_pose, current_twist, common_parameters,
    *lane_change_parameters_, common_parameters.expected_front_deceleration_for_abort,
    common_parameters.expected_rear_deceleration_for_abort, debug_data,
    status_.lane_change_path.acceleration);

  return safety_status;
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

bool NormalLaneChange::isRequiredStop(const bool is_object_coming_from_rear) const
{
  return isNearEndOfLane() && isAbleToStopSafely() && is_object_coming_from_rear;
}

bool NormalLaneChange::getAbortPath()
{
  const auto & route_handler = getRouteHandler();
  const auto & common_param = getCommonParam();
  const auto current_velocity =
    std::max(common_param.minimum_lane_changing_velocity, getEgoVelocity());
  const auto current_pose = getEgoPose();
  const auto & selected_path = status_.lane_change_path;
  const auto reference_lanelets = selected_path.reference_lanelets;

  const auto ego_nearest_dist_threshold = common_param.ego_nearest_dist_threshold;
  const auto ego_nearest_yaw_threshold = common_param.ego_nearest_yaw_threshold;

  const auto direction = getDirection();
  const auto shift_intervals = route_handler->getLateralIntervalsToPreferredLane(
    selected_path.reference_lanelets.back(), direction);
  const double minimum_lane_change_length =
    utils::calcMinimumLaneChangeLength(common_param, shift_intervals);

  const auto & lane_changing_path = selected_path.path;
  const auto lane_changing_end_pose_idx = std::invoke([&]() {
    constexpr double s_start = 0.0;
    const double s_end = std::max(
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length, 0.0);

    const auto ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    return motion_utils::findFirstNearestIndexWithSoftConstraints(
      lane_changing_path.points, ref.points.back().point.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  });

  const auto ego_pose_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    lane_changing_path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const auto get_abort_idx_and_distance = [&](const double param_time) {
    if (ego_pose_idx > lane_changing_end_pose_idx) {
      return std::make_pair(ego_pose_idx, 0.0);
    }

    const auto desired_distance = current_velocity * param_time;
    const auto & points = lane_changing_path.points;

    for (size_t idx = ego_pose_idx; idx < lane_changing_end_pose_idx; ++idx) {
      const double distance =
        utils::getSignedDistance(current_pose, points.at(idx).point.pose, reference_lanelets);
      if (distance > desired_distance) {
        return std::make_pair(idx, distance);
      }
    }

    return std::make_pair(ego_pose_idx, 0.0);
  };

  const auto [abort_start_idx, abort_start_dist] =
    get_abort_idx_and_distance(lane_change_parameters_->abort_delta_time);
  const auto [abort_return_idx, abort_return_dist] = get_abort_idx_and_distance(
    lane_change_parameters_->abort_delta_time + lane_change_parameters_->aborting_time);

  if (abort_start_idx >= abort_return_idx) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
      "abort start idx and return idx is equal. can't compute abort path.");
    return false;
  }

  if (!utils::lane_change::hasEnoughLengthToLaneChangeAfterAbort(
        *route_handler, reference_lanelets, current_pose, abort_return_dist, common_param,
        direction)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
      "insufficient distance to abort.");
    return false;
  }

  const auto abort_start_pose = lane_changing_path.points.at(abort_start_idx).point.pose;
  const auto abort_return_pose = lane_changing_path.points.at(abort_return_idx).point.pose;
  const auto shift_length =
    lanelet::utils::getArcCoordinates(reference_lanelets, abort_return_pose).distance;

  ShiftLine shift_line;
  shift_line.start = abort_start_pose;
  shift_line.end = abort_return_pose;
  shift_line.end_shift_length = -shift_length;
  shift_line.start_idx = abort_start_idx;
  shift_line.end_idx = abort_return_idx;

  PathShifter path_shifter;
  path_shifter.setPath(lane_changing_path);
  path_shifter.addShiftLine(shift_line);
  const auto lateral_jerk = behavior_path_planner::PathShifter::calcJerkFromLatLonDistance(
    shift_line.end_shift_length, abort_start_dist, current_velocity);
  path_shifter.setVelocity(current_velocity);
  const auto lateral_acc_range = common_param.lane_change_lat_acc_map.find(current_velocity);
  const double & max_lateral_acc = lateral_acc_range.second;
  path_shifter.setLateralAccelerationLimit(max_lateral_acc);

  if (lateral_jerk > lane_change_parameters_->abort_max_lateral_jerk) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
      "Aborting jerk is too strong. lateral_jerk = " << lateral_jerk);
    return false;
  }

  ShiftedPath shifted_path;
  // offset front side
  // bool offset_back = false;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()),
      "failed to generate abort shifted path.");
  }

  const auto arc_position = lanelet::utils::getArcCoordinates(
    reference_lanelets, shifted_path.path.points.at(abort_return_idx).point.pose);
  const PathWithLaneId reference_lane_segment = std::invoke([&]() {
    const double s_start = arc_position.length;
    double s_end = std::max(
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length, s_start);

    if (route_handler->isInGoalRouteSection(selected_path.target_lanelets.back())) {
      const auto goal_arc_coordinates =
        lanelet::utils::getArcCoordinates(reference_lanelets, route_handler->getGoalPose());
      const double forward_length =
        std::max(goal_arc_coordinates.length - minimum_lane_change_length, s_start);
      s_end = std::min(s_end, forward_length);
    }
    PathWithLaneId ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end, true);
    ref.points.back().point.longitudinal_velocity_mps = std::min(
      ref.points.back().point.longitudinal_velocity_mps,
      static_cast<float>(common_param.minimum_lane_changing_velocity));
    return ref;
  });

  PathWithLaneId start_to_abort_return_pose;
  start_to_abort_return_pose.points.insert(
    start_to_abort_return_pose.points.end(), shifted_path.path.points.begin(),
    shifted_path.path.points.begin() + abort_return_idx);
  if (reference_lane_segment.points.size() > 1) {
    start_to_abort_return_pose.points.insert(
      start_to_abort_return_pose.points.end(), (reference_lane_segment.points.begin() + 1),
      reference_lane_segment.points.end());
  }

  auto abort_path = selected_path;
  abort_path.shifted_path = shifted_path;
  abort_path.path = start_to_abort_return_pose;
  abort_path.shift_line = shift_line;
  abort_path_ = std::make_shared<LaneChangePath>(abort_path);
  return true;
}

NormalLaneChangeBT::NormalLaneChangeBT(
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: NormalLaneChange(parameters, type, direction)
{
}

PathWithLaneId NormalLaneChangeBT::getReferencePath() const
{
  PathWithLaneId reference_path;
  if (!utils::lane_change::isEgoWithinOriginalLane(
        status_.current_lanes, getEgoPose(), planner_data_->parameters)) {
    return reference_path;
  }

  const auto & route_handler = getRouteHandler();
  const auto & current_pose = getEgoPose();
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = getRouteHeader();

  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return reference_path;
  }

  reference_path = utils::getCenterLinePath(
    *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length, common_parameters);

  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const double lane_change_buffer =
    utils::calcMinimumLaneChangeLength(common_parameters, shift_intervals);

  reference_path = utils::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, common_parameters.lane_change_prepare_duration,
    lane_change_buffer);

  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto drivable_lanes = utils::generateDrivableLanes(current_lanes);
  const auto shorten_lanes = utils::cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);
  utils::generateDrivableArea(
    reference_path, expanded_lanes, false, common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets NormalLaneChangeBT::getCurrentLanes() const
{
  return utils::getCurrentLanes(planner_data_);
}

int NormalLaneChangeBT::getNumToPreferredLane(const lanelet::ConstLanelet & lane) const
{
  return std::abs(getRouteHandler()->getNumLaneToPreferredLane(lane));
}

PathWithLaneId NormalLaneChangeBT::getPrepareSegment(
  const lanelet::ConstLanelets & current_lanes, const double arc_length_from_current,
  const double backward_path_length, const double prepare_length,
  const double prepare_velocity) const
{
  if (current_lanes.empty()) {
    return PathWithLaneId();
  }

  const double s_start = arc_length_from_current - backward_path_length;
  const double s_end = arc_length_from_current + prepare_length;

  RCLCPP_DEBUG(
    rclcpp::get_logger("lane_change").get_child(getModuleTypeStr()).get_child("getPrepareSegment"),
    "start: %f, end: %f", s_start, s_end);

  PathWithLaneId prepare_segment =
    getRouteHandler()->getCenterLinePath(current_lanes, s_start, s_end);

  prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
    prepare_segment.points.back().point.longitudinal_velocity_mps,
    static_cast<float>(prepare_velocity));

  return prepare_segment;
}

std::vector<DrivableLanes> NormalLaneChangeBT::getDrivableLanes() const
{
  return utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), status_.current_lanes, status_.lane_change_lanes);
}
}  // namespace behavior_path_planner
