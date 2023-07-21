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

#include "behavior_path_planner/scene_module/lane_change/interface.hpp"

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/module_status.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using utils::lane_change::assignToCandidate;

LaneChangeInterface::LaneChangeInterface(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map,
  std::unique_ptr<LaneChangeBase> && module_type)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  parameters_{std::move(parameters)},
  module_type_{std::move(module_type)},
  prev_approved_path_{std::make_unique<PathWithLaneId>()}
{
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, name);
}

void LaneChangeInterface::processOnEntry()
{
  waitApproval();
  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  module_type_->updateLaneChangeStatus();
}

void LaneChangeInterface::processOnExit()
{
  module_type_->resetParameters();
  debug_marker_.markers.clear();
  resetPathCandidate();
}

bool LaneChangeInterface::isExecutionRequested() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  return module_type_->isLaneChangeRequired();
}

bool LaneChangeInterface::isExecutionReady() const
{
  return module_type_->isSafe();
}

ModuleStatus LaneChangeInterface::updateState()
{
  if (module_type_->specialExpiredCheck()) {
    if (isWaitingApproval()) {
      return ModuleStatus::SUCCESS;
    }
  }

  if (!isActivated() || isWaitingApproval()) {
    return ModuleStatus::IDLE;
  }

  if (!module_type_->isValidPath()) {
    return ModuleStatus::RUNNING;
  }

  if (module_type_->isAbortState()) {
    if (module_type_->hasFinishedAbort()) {
      resetLaneChangeModule();
    }
    return ModuleStatus::RUNNING;
  }

  if (module_type_->hasFinishedLaneChange()) {
    return ModuleStatus::SUCCESS;
  }

  const auto [is_safe, is_object_coming_from_rear] = module_type_->isApprovedPathSafe();

  setObjectDebugVisualization();
  if (is_safe) {
    module_type_->toNormalState();
    return ModuleStatus::RUNNING;
  }

  if (!module_type_->isCancelEnabled()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
      "Lane change path is unsafe but cancel was not enabled. Continue lane change.");
    if (module_type_->isRequiredStop(is_object_coming_from_rear)) {
      module_type_->toStopState();
    } else {
      module_type_->toNormalState();
    }
    return ModuleStatus::RUNNING;
  }

  if (!module_type_->isAbleToReturnCurrentLane()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
      "Lane change path is unsafe but cannot return. Continue lane change.");
    if (module_type_->isRequiredStop(is_object_coming_from_rear)) {
      module_type_->toStopState();
    } else {
      module_type_->toNormalState();
    }
    return ModuleStatus::RUNNING;
  }

  const auto & common_parameters = module_type_->getCommonParam();
  const auto threshold = common_parameters.backward_length_buffer_for_end_of_lane;
  const auto status = module_type_->getLaneChangeStatus();
  if (module_type_->isNearEndOfCurrentLanes(status.current_lanes, status.target_lanes, threshold)) {
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
      "Lane change path is unsafe but near end of lane. Continue lane change.");
    if (module_type_->isRequiredStop(is_object_coming_from_rear)) {
      module_type_->toStopState();
    } else {
      module_type_->toNormalState();
    }
    return ModuleStatus::RUNNING;
  }

  if (module_type_->isEgoOnPreparePhase()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
      "Lane change path is unsafe. Cancel lane change.");
    module_type_->toCancelState();
    if (!isWaitingApproval()) {
      resetLaneChangeModule();
    }
    return ModuleStatus::RUNNING;
  }

  if (!module_type_->isAbortEnabled()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
      "Lane change path is unsafe but abort was not enabled. Continue lane change.");
    if (module_type_->isRequiredStop(is_object_coming_from_rear)) {
      module_type_->toStopState();
    } else {
      module_type_->toNormalState();
    }
    return ModuleStatus::RUNNING;
  }

  const auto found_abort_path = module_type_->getAbortPath();
  if (!found_abort_path) {
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
      "Lane change path is unsafe but not found abort path. Continue lane change.");
    if (module_type_->isRequiredStop(is_object_coming_from_rear)) {
      module_type_->toStopState();
    } else {
      module_type_->toNormalState();
    }
    return ModuleStatus::RUNNING;
  }

  RCLCPP_WARN_STREAM_THROTTLE(
    getLogger().get_child(module_type_->getModuleTypeStr()), *clock_, 5000,
    "Lane change path is unsafe. Abort lane change.");
  module_type_->toAbortState();
  return ModuleStatus::RUNNING;
}

void LaneChangeInterface::resetLaneChangeModule()
{
  processOnExit();
  removeRTCStatus();
  processOnEntry();
}

void LaneChangeInterface::updateData()
{
  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  module_type_->updateSpecialData();
}

BehaviorModuleOutput LaneChangeInterface::plan()
{
  resetPathCandidate();
  resetPathReference();

  if (!module_type_->isValidPath()) {
    return {};
  }

  module_type_->setPreviousDrivableAreaInfo(getPreviousModuleOutput().drivable_area_info);
  module_type_->setPreviousTurnSignalInfo(getPreviousModuleOutput().turn_signal_info);
  auto output = module_type_->generateOutput();
  path_reference_ = output.reference_path;
  *prev_approved_path_ = *getPreviousModuleOutput().path;

  updateSteeringFactorPtr(output);
  clearWaitingApproval();

  return output;
}

BehaviorModuleOutput LaneChangeInterface::planWaitingApproval()
{
  *prev_approved_path_ = *getPreviousModuleOutput().path;
  module_type_->insertStopPoint(*prev_approved_path_);

  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(*prev_approved_path_);
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  out.drivable_area_info = getPreviousModuleOutput().drivable_area_info;

  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  module_type_->updateLaneChangeStatus();
  setObjectDebugVisualization();

  // change turn signal when the vehicle reaches at the end of the path for waiting lane change
  out.turn_signal_info = getCurrentTurnSignalInfo(*out.path, out.turn_signal_info);

  path_reference_ = getPreviousModuleOutput().reference_path;

  if (!module_type_->isValidPath()) {
    removeRTCStatus();
    path_candidate_ = std::make_shared<PathWithLaneId>();
    return out;
  }

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  updateRTCStatus(
    candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change);
  is_abort_path_approved_ = false;

  return out;
}

CandidateOutput LaneChangeInterface::planCandidate() const
{
  const auto selected_path = module_type_->getLaneChangePath();

  if (selected_path.path.points.empty()) {
    return {};
  }

  CandidateOutput output = assignToCandidate(selected_path, module_type_->getEgoPosition());

  updateSteeringFactorPtr(output, selected_path);
  return output;
}

void LaneChangeInterface::updateModuleParams(
  const std::shared_ptr<LaneChangeParameters> & parameters)
{
  parameters_ = parameters;
}

void LaneChangeInterface::setData(const std::shared_ptr<const PlannerData> & data)
{
  module_type_->setData(data);
}

void LaneChangeInterface::setObjectDebugVisualization() const
{
  debug_marker_.markers.clear();
  if (!parameters_->publish_debug_marker) {
    return;
  }
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  const auto debug_data = module_type_->getDebugData();
  const auto debug_after_approval = module_type_->getAfterApprovalDebugData();
  const auto debug_valid_path = module_type_->getDebugValidPath();

  debug_marker_.markers.clear();
  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  add(showAllValidLaneChangePath(debug_valid_path, "lane_change_valid_paths"));
  if (!debug_data.empty()) {
    add(showObjectInfo(debug_data, "object_debug_info"));
    add(showLerpedPose(debug_data, "ego_predicted_path"));
    add(showPolygon(debug_data, "ego_and_target_polygon_relation"));
  }

  if (!debug_after_approval.empty()) {
    add(showObjectInfo(debug_after_approval, "object_debug_info_after_approval"));
    add(showLerpedPose(debug_after_approval, "ego_predicted_path_after_approval"));
    add(showPolygon(debug_after_approval, "ego_and_target_polygon_relation_after_approval"));
  }
}

std::shared_ptr<LaneChangeDebugMsgArray> LaneChangeInterface::get_debug_msg_array() const
{
  const auto debug_data = module_type_->getDebugData();
  LaneChangeDebugMsgArray debug_msg_array;
  debug_msg_array.lane_change_info.reserve(debug_data.size());
  for (const auto & [uuid, debug_data] : debug_data) {
    LaneChangeDebugMsg debug_msg;
    debug_msg.object_id = uuid;
    debug_msg.allow_lane_change = debug_data.allow_lane_change;
    debug_msg.is_front = debug_data.is_front;
    debug_msg.relative_distance = debug_data.relative_to_ego;
    debug_msg.failed_reason = debug_data.failed_reason;
    debug_msg.velocity = debug_data.object_twist.linear.x;
    debug_msg_array.lane_change_info.push_back(debug_msg);
  }
  lane_change_debug_msg_array_ = debug_msg_array;

  lane_change_debug_msg_array_.header.stamp = clock_->now();
  return std::make_shared<LaneChangeDebugMsgArray>(lane_change_debug_msg_array_);
}

MarkerArray LaneChangeInterface::getModuleVirtualWall()
{
  using marker_utils::lane_change_markers::createLaneChangingVirtualWallMarker;
  MarkerArray marker;

  if (!parameters_->publish_debug_marker) {
    return marker;
  }

  if (isWaitingApproval() || current_state_ != ModuleStatus::RUNNING) {
    return marker;
  }
  const auto & start_pose = module_type_->getLaneChangePath().info.lane_changing_start;
  const auto start_marker =
    createLaneChangingVirtualWallMarker(start_pose, name(), clock_->now(), "lane_change_start");

  const auto & end_pose = module_type_->getLaneChangePath().info.lane_changing_end;
  const auto end_marker =
    createLaneChangingVirtualWallMarker(end_pose, name(), clock_->now(), "lane_change_end");
  marker.markers.reserve(start_marker.markers.size() + end_marker.markers.size());
  appendMarkerArray(start_marker, &marker);
  appendMarkerArray(end_marker, &marker);
  return marker;
}

void LaneChangeInterface::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  const auto steering_factor_direction = std::invoke([&]() {
    if (module_type_->getDirection() == Direction::LEFT) {
      return SteeringFactor::LEFT;
    }
    if (module_type_->getDirection() == Direction::RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::UNKNOWN;
  });

  const auto current_position = module_type_->getEgoPosition();
  const auto status = module_type_->getLaneChangeStatus();
  const auto start_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_position, status.lane_change_path.info.shift_line.start.position);
  const auto finish_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_position, status.lane_change_path.info.shift_line.end.position);

  // TODO(tkhmy) add handle status TRYING
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status.lane_change_path.info.shift_line.start, status.lane_change_path.info.shift_line.end},
    {start_distance, finish_distance}, SteeringFactor::LANE_CHANGE, steering_factor_direction,
    SteeringFactor::TURNING, "");
}

void LaneChangeInterface::updateSteeringFactorPtr(
  const CandidateOutput & output, const LaneChangePath & selected_path) const
{
  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return SteeringFactor::LEFT;
    }
    return SteeringFactor::RIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {selected_path.info.shift_line.start, selected_path.info.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    SteeringFactor::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");
}
void LaneChangeInterface::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitLaneChangeInterface(this);
  }
}

TurnSignalInfo LaneChangeInterface::getCurrentTurnSignalInfo(
  const PathWithLaneId & path, const TurnSignalInfo & original_turn_signal_info)
{
  const auto & current_lanes = module_type_->getLaneChangeStatus().current_lanes;
  const auto & is_valid = module_type_->getLaneChangeStatus().is_valid_path;
  const auto & lane_change_path = module_type_->getLaneChangeStatus().lane_change_path;
  const auto & lane_change_param = module_type_->getLaneChangeParam();

  if (
    module_type_->getModuleType() != LaneChangeModuleType::NORMAL || current_lanes.empty() ||
    !is_valid) {
    return original_turn_signal_info;
  }

  // check direction
  TurnSignalInfo current_turn_signal_info;
  const auto & current_pose = module_type_->getEgoPose();
  const auto direction = module_type_->getDirection();
  if (direction == Direction::LEFT) {
    current_turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  } else if (direction == Direction::RIGHT) {
    current_turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  }

  if (path.points.empty()) {
    current_turn_signal_info.desired_start_point = current_pose;
    current_turn_signal_info.required_start_point = current_pose;
    current_turn_signal_info.desired_end_point = lane_change_path.info.lane_changing_end;
    current_turn_signal_info.required_end_point = lane_change_path.info.lane_changing_end;
    return current_turn_signal_info;
  }

  const auto & min_length_for_turn_signal_activation =
    lane_change_param.min_length_for_turn_signal_activation;
  const auto & route_handler = module_type_->getRouteHandler();
  const auto & common_parameter = module_type_->getCommonParam();
  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const double next_lane_change_buffer =
    utils::calcMinimumLaneChangeLength(common_parameter, shift_intervals);
  const double & nearest_dist_threshold = common_parameter.ego_nearest_dist_threshold;
  const double & nearest_yaw_threshold = common_parameter.ego_nearest_yaw_threshold;
  const double & base_to_front = common_parameter.base_link2front;

  const double buffer =
    next_lane_change_buffer + min_length_for_turn_signal_activation + base_to_front;
  const double path_length = motion_utils::calcArcLength(path.points);
  const auto & front_point = path.points.front().point.pose.position;
  const size_t & current_nearest_seg_idx =
    motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double length_front_to_ego = motion_utils::calcSignedArcLength(
    path.points, front_point, static_cast<size_t>(0), current_pose.position,
    current_nearest_seg_idx);
  const auto start_pose =
    motion_utils::calcLongitudinalOffsetPose(path.points, 0, std::max(path_length - buffer, 0.0));
  if (path_length - length_front_to_ego < buffer && start_pose) {
    // modify turn signal
    current_turn_signal_info.desired_start_point = *start_pose;
    current_turn_signal_info.desired_end_point = lane_change_path.info.lane_changing_end;
    current_turn_signal_info.required_start_point = current_turn_signal_info.desired_start_point;
    current_turn_signal_info.required_end_point = current_turn_signal_info.desired_end_point;

    const auto & original_command = original_turn_signal_info.turn_signal.command;
    if (
      original_command == TurnIndicatorsCommand::DISABLE ||
      original_command == TurnIndicatorsCommand::NO_COMMAND) {
      return current_turn_signal_info;
    }

    // check the priority of turn signals
    return module_type_->getTurnSignalDecider().use_prior_turn_signal(
      path, current_pose, current_nearest_seg_idx, original_turn_signal_info,
      current_turn_signal_info, nearest_dist_threshold, nearest_yaw_threshold);
  }

  // not in the vicinity of the end of the path. return original
  return original_turn_signal_info;
}

void SceneModuleVisitor::visitLaneChangeInterface(const LaneChangeInterface * interface) const
{
  lane_change_visitor_ = interface->get_debug_msg_array();
}

AvoidanceByLaneChangeInterface::AvoidanceByLaneChangeInterface(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters,
  const std::shared_ptr<AvoidanceByLCParameters> & avoidance_by_lane_change_parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map)
: LaneChangeInterface{
    name, node, parameters, rtc_interface_ptr_map,
    std::make_unique<AvoidanceByLaneChange>(parameters, avoidance_by_lane_change_parameters)}
{
}

bool AvoidanceByLaneChangeInterface::isExecutionRequested() const
{
  return module_type_->specialRequiredCheck() && module_type_->isLaneChangeRequired();
}

void AvoidanceByLaneChangeInterface::updateRTCStatus(
  const double start_distance, const double finish_distance)
{
  const auto direction = std::invoke([&]() -> std::string {
    const auto dir = module_type_->getDirection();
    return (dir == Direction::LEFT) ? "left" : "right";
  });

  rtc_interface_ptr_map_.at(direction)->updateCooperateStatus(
    uuid_map_.at(direction), isExecutionReady(), start_distance, finish_distance, clock_->now());
}
}  // namespace behavior_path_planner
