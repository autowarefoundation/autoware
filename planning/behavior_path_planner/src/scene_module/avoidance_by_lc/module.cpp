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

#include "behavior_path_planner/scene_module/avoidance_by_lc/module.hpp"

#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utils/avoidance/utils.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

using autoware_auto_perception_msgs::msg::ObjectClassification;
using motion_utils::calcSignedArcLength;
using motion_utils::findNearestIndex;
using motion_utils::findNearestSegmentIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::toHexString;

AvoidanceByLCModule::AvoidanceByLCModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<AvoidanceByLCParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map}, parameters_{parameters}
{
}
BehaviorModuleOutput AvoidanceByLCModule::planWaitingApproval()
{
  prev_approved_path_ = *getPreviousModuleOutput().path;
  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(prev_approved_path_);
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;

  // for new architecture
  const auto current_lanes =
    utils::getCurrentLanesFromPath(*getPreviousModuleOutput().reference_path, planner_data_);
  const auto drivable_lanes = utils::generateDrivableLanes(current_lanes);
  const auto target_drivable_lanes = getNonOverlappingExpandedLanes(*out.path, drivable_lanes);
  out.drivable_area_info.drivable_lanes = utils::combineDrivableLanes(
    getPreviousModuleOutput().drivable_area_info.drivable_lanes, target_drivable_lanes);

  if (!avoidance_data_.target_objects.empty()) {
    const auto to_front_object_distance = avoidance_data_.target_objects.front().longitudinal;
    const double shift_length = status_.lane_change_path.shift_line.end_shift_length -
                                status_.lane_change_path.shift_line.start_shift_length;
    const double lane_change_buffer =
      utils::calcMinimumLaneChangeLength(planner_data_->parameters, {shift_length});

    utils::avoidance::insertDecelPoint(
      getEgoPosition(), to_front_object_distance - lane_change_buffer, 0.0, *out.path, stop_pose_);
  }

  updateLaneChangeStatus();

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  path_reference_ = getPreviousModuleOutput().reference_path;
  updateRTCStatus(candidate);
  waitApproval();
  is_abort_path_approved_ = false;

  return out;
}

void AvoidanceByLCModule::updateLaneChangeStatus()
{
#ifdef USE_OLD_ARCHITECTURE
  status_.current_lanes = utils::getCurrentLanes(planner_data_);
#else
  status_.current_lanes =
    utils::getCurrentLanesFromPath(*getPreviousModuleOutput().reference_path, planner_data_);
#endif
  status_.lane_change_lanes = getLaneChangeLanes(status_.current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(status_.lane_change_lanes, check_distance_, selected_path);

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_change_path = selected_path;
  status_.lane_follow_lane_ids = utils::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = utils::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

PathWithLaneId AvoidanceByLCModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = getRouteHeader();

#ifdef USE_OLD_ARCHITECTURE
  const auto current_lanes = utils::getCurrentLanes(planner_data_);
#else
  const auto current_lanes =
    utils::getCurrentLanesFromPath(*getPreviousModuleOutput().reference_path, planner_data_);
#endif

  if (current_lanes.empty()) {
    return reference_path;
  }

  if (reference_path.points.empty()) {
    reference_path = utils::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters);
  }

  reference_path = utils::getCenterLinePath(
    *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length, common_parameters, 0.0);

  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const double lane_change_buffer =
    utils::calcMinimumLaneChangeLength(common_parameters, shift_intervals);

  reference_path = utils::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, common_parameters.lane_change_prepare_duration,
    lane_change_buffer);

  const auto drivable_lanes = utils::generateDrivableLanes(current_lanes);
  const auto target_drivable_lanes = getNonOverlappingExpandedLanes(reference_path, drivable_lanes);
  utils::generateDrivableArea(
    reference_path, target_drivable_lanes, common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets AvoidanceByLCModule::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto minimum_prepare_length = planner_data_->parameters.minimum_prepare_length;
  const auto prepare_duration = planner_data_->parameters.lane_change_prepare_duration;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  const auto object_num = avoidance_data_.target_objects.size();
  if (object_num < parameters_->execute_object_num) {
    return lane_change_lanes;
  }

  const auto o_front = avoidance_data_.target_objects.front();

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const double lane_change_prepare_length =
    std::max(current_twist.linear.x * prepare_duration, minimum_prepare_length);
  lanelet::ConstLanelets current_check_lanes =
    route_handler->getLaneletSequence(current_lane, current_pose, 0.0, lane_change_prepare_length);
  lanelet::ConstLanelet lane_change_lane;

  if (utils::avoidance::isOnRight(o_front)) {
    for (const auto & lanelet : current_check_lanes) {
      const auto & left_lane = route_handler->getRoutingGraphPtr()->left(lanelet);
      if (left_lane) {
        lane_change_lanes = route_handler->getLaneletSequence(
          left_lane.get(), current_pose, lane_change_lane_length, lane_change_lane_length);
        break;
      }
    }
  } else {
    for (const auto & lanelet : current_check_lanes) {
      const auto & right_lane = route_handler->getRoutingGraphPtr()->right(lanelet);
      if (right_lane) {
        lane_change_lanes = route_handler->getLaneletSequence(
          right_lane.get(), current_pose, lane_change_lane_length, lane_change_lane_length);
        break;
      }
    }
  }

  return lane_change_lanes;
}

std::pair<bool, bool> AvoidanceByLCModule::getSafePath(
  const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & common_parameters = planner_data_->parameters;

#ifdef USE_OLD_ARCHITECTURE
  const auto current_lanes = utils::getCurrentLanes(planner_data_);
#else
  const auto current_lanes =
    utils::getCurrentLanesFromPath(*getPreviousModuleOutput().reference_path, planner_data_);
#endif

  if (lane_change_lanes.empty()) {
    return std::make_pair(false, false);
  }

  if (avoidance_data_.target_objects.empty()) {
    return std::make_pair(false, false);
  }

  // find candidate paths
  LaneChangePaths valid_paths;
#ifdef USE_OLD_ARCHITECTURE
  const auto found_safe_path = utils::lane_change::getLaneChangePaths(
    *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
    planner_data_->dynamic_object, common_parameters, *parameters_->lane_change, check_distance,
    &valid_paths, &object_debug_);
#else
  const auto o_front = avoidance_data_.target_objects.front();
  const auto direction = utils::avoidance::isOnRight(o_front) ? Direction::LEFT : Direction::RIGHT;
  const auto found_safe_path = utils::lane_change::getLaneChangePaths(
    *getPreviousModuleOutput().path, *route_handler, current_lanes, lane_change_lanes, current_pose,
    current_twist, planner_data_->dynamic_object, common_parameters, *parameters_->lane_change,
    check_distance, direction, &valid_paths, &object_debug_);
#endif
  debug_valid_path_ = valid_paths;

  if (parameters_->lane_change->publish_debug_marker) {
    setObjectDebugVisualization();
  } else {
    debug_marker_.markers.clear();
  }

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

bool AvoidanceByLCModule::isSafe() const
{
  return status_.is_safe;
}

bool AvoidanceByLCModule::isValidPath() const
{
  return status_.is_valid_path;
}

bool AvoidanceByLCModule::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;

  // check lane departure
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, utils::extendLanes(route_handler, status_.current_lanes),
    utils::extendLanes(route_handler, status_.lane_change_lanes));
  const auto & dp = planner_data_->drivable_area_expansion_parameters;
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
      RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *clock_, 1000, "path is out of lanes");
      return false;
    }
  }

  // check relative angle
  if (!utils::checkPathRelativeAngle(path, M_PI)) {
    RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *clock_, 1000, "path relative angle is invalid");
    return false;
  }

  return true;
}

bool AvoidanceByLCModule::isNearEndOfLane() const
{
  const auto & current_pose = getEgoPose();
  const auto shift_intervals =
    planner_data_->route_handler->getLateralIntervalsToPreferredLane(status_.current_lanes.back());
  const double threshold =
    utils::calcMinimumLaneChangeLength(planner_data_->parameters, shift_intervals);

  return std::max(0.0, utils::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool AvoidanceByLCModule::isCurrentVelocityLow() const
{
  constexpr double threshold_ms = 10.0 * 1000 / 3600;
  return utils::l2Norm(getEgoTwist().linear) < threshold_ms;
}

bool AvoidanceByLCModule::isAbortConditionSatisfied()
{
  is_abort_condition_satisfied_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;

  if (!parameters_->lane_change->enable_cancel_lane_change) {
    return false;
  }

  if (!is_activated_) {
    return false;
  }

  Pose ego_pose_before_collision;
  const auto is_path_safe = isApprovedPathSafe(ego_pose_before_collision);

  if (!is_path_safe) {
    const auto & common_parameters = planner_data_->parameters;
    const bool is_within_original_lane = utils::lane_change::isEgoWithinOriginalLane(
      status_.current_lanes, getEgoPose(), common_parameters);

    if (is_within_original_lane) {
      current_lane_change_state_ = LaneChangeStates::Cancel;
      return true;
    }

    // check abort enable flag
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger(), *clock_, 1000,
      "DANGER!!! Path is not safe anymore, but it is too late to CANCEL! Please be cautious");

    if (!parameters_->lane_change->enable_abort_lane_change) {
      current_lane_change_state_ = LaneChangeStates::Stop;
      return false;
    }

    const auto found_abort_path = utils::lane_change::getAbortPaths(
      planner_data_, status_.lane_change_path, ego_pose_before_collision, common_parameters,
      *parameters_->lane_change);

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

  return false;
}

bool AvoidanceByLCModule::isAbortState() const
{
  if (!parameters_->lane_change->enable_abort_lane_change) {
    return false;
  }

  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    return false;
  }

  if (!abort_path_) {
    return false;
  }

  RCLCPP_WARN_STREAM_THROTTLE(
    getLogger(), *clock_, 1000,
    "DANGER!!! Lane change transition to ABORT state, return path will be computed!");
  return true;
}

bool AvoidanceByLCModule::isAvoidancePlanRunning() const
{
  constexpr double AVOIDING_SHIFT_THR = 0.1;

  const auto & current_pose = getEgoPose();
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose);

  return std::abs(arclength_current.distance) > AVOIDING_SHIFT_THR;
}

bool AvoidanceByLCModule::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  const double travel_distance = arclength_current.length - status_.start_distance;
  const double finish_distance = status_.lane_change_path.length.sum() +
                                 parameters_->lane_change->lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

void AvoidanceByLCModule::setObjectDebugVisualization() const
{
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  debug_marker_.markers.clear();
  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  add(showObjectInfo(object_debug_, "object_debug_info"));
  add(showLerpedPose(object_debug_, "lerp_pose_before_true"));
  add(showPolygonPose(object_debug_, "expected_pose"));
  add(showPolygon(object_debug_, "lerped_polygon"));
  add(showAllValidLaneChangePath(debug_valid_path_, "lane_change_valid_paths"));
}

std::shared_ptr<LaneChangeDebugMsgArray> AvoidanceByLCModule::get_debug_msg_array() const
{
  LaneChangeDebugMsgArray debug_msg_array;
  debug_msg_array.lane_change_info.reserve(object_debug_.size());
  for (const auto & [uuid, debug_data] : object_debug_) {
    LaneChangeDebugMsg debug_msg;
    debug_msg.object_id = uuid;
    debug_msg.allow_lane_change = debug_data.allow_lane_change;
    debug_msg.is_front = debug_data.is_front;
    debug_msg.relative_distance = debug_data.relative_to_ego;
    debug_msg.failed_reason = debug_data.failed_reason;
    debug_msg.velocity = utils::l2Norm(debug_data.object_twist.linear);
    debug_msg_array.lane_change_info.push_back(debug_msg);
  }
  lane_change_debug_msg_array_ = debug_msg_array;

  lane_change_debug_msg_array_.header.stamp = clock_->now();
  return std::make_shared<LaneChangeDebugMsgArray>(lane_change_debug_msg_array_);
}

void AvoidanceByLCModule::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  const auto turn_signal_info = output.turn_signal_info;
  const auto current_pose = getEgoPose();
  const double start_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status_.lane_change_path.shift_line.start.position);
  const double finish_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status_.lane_change_path.shift_line.end.position);

  const uint16_t steering_factor_direction =
    std::invoke([this, &start_distance, &finish_distance, &turn_signal_info]() {
      if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
        waitApprovalLeft(start_distance, finish_distance);
        return SteeringFactor::LEFT;
      }
      if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
        waitApprovalRight(start_distance, finish_distance);
        return SteeringFactor::RIGHT;
      }
      return SteeringFactor::UNKNOWN;
    });

  // TODO(tkhmy) add handle status TRYING
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.lane_change_path.shift_line.start, status_.lane_change_path.shift_line.end},
    {start_distance, finish_distance}, SteeringFactor::LANE_CHANGE, steering_factor_direction,
    SteeringFactor::TURNING, "");
}

void AvoidanceByLCModule::updateSteeringFactorPtr(
  const CandidateOutput & output, const LaneChangePath & selected_path) const
{
  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return SteeringFactor::LEFT;
    }
    return SteeringFactor::RIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {selected_path.shift_line.start, selected_path.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    SteeringFactor::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");
}

std_msgs::msg::Header AvoidanceByLCModule::getRouteHeader() const
{
  return planner_data_->route_handler->getRouteHeader();
}

void AvoidanceByLCModule::generateExtendedDrivableArea(PathWithLaneId & path)
{
  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, status_.current_lanes, status_.lane_change_lanes);
  const auto target_drivable_lanes = getNonOverlappingExpandedLanes(path, drivable_lanes);
  utils::generateDrivableArea(
    path, target_drivable_lanes, common_parameters.vehicle_length, planner_data_);
}

bool AvoidanceByLCModule::isApprovedPathSafe(Pose & ego_pose_before_collision) const
{
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & common_parameters = planner_data_->parameters;
  const auto & lane_change_parameters = parameters_->lane_change;
  const auto & route_handler = planner_data_->route_handler;
  const auto & path = status_.lane_change_path;

  // get lanes used for detection
  const auto backward_target_lanes_for_object_filtering = utils::lane_change::getBackwardLanelets(
    *route_handler, path.target_lanelets, current_pose, check_distance_);

  std::unordered_map<std::string, CollisionCheckDebug> debug_data;
  const auto lateral_buffer =
    utils::lane_change::calcLateralBufferForFiltering(common_parameters.vehicle_width);
  const auto dynamic_object_indices = utils::lane_change::filterObjectIndices(
    {path}, *dynamic_objects, backward_target_lanes_for_object_filtering, current_pose,
    common_parameters.forward_path_length, *lane_change_parameters, lateral_buffer);

  return utils::lane_change::isLaneChangePathSafe(
    path, dynamic_objects, dynamic_object_indices, current_pose, current_twist, common_parameters,
    *parameters_->lane_change, common_parameters.expected_front_deceleration_for_abort,
    common_parameters.expected_rear_deceleration_for_abort, ego_pose_before_collision, debug_data,
    status_.lane_change_path.acceleration);
}

void AvoidanceByLCModule::updateOutputTurnSignal(BehaviorModuleOutput & output)
{
  const auto turn_signal_info = utils::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_line, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  output.turn_signal_info.turn_signal.command = turn_signal_info.first.command;

  utils::lane_change::get_turn_signal_info(status_.lane_change_path, &output.turn_signal_info);
}

void AvoidanceByLCModule::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;

  object_debug_.clear();
  debug_marker_.markers.clear();
  resetPathCandidate();
  resetPathReference();
}

void AvoidanceByLCModule::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitAvoidanceByLCModule(this);
  }
}

void SceneModuleVisitor::visitAvoidanceByLCModule(
  [[maybe_unused]] const AvoidanceByLCModule * module) const
{
  // lane_change_visitor_ = module->get_debug_msg_array();
}
}  // namespace behavior_path_planner
