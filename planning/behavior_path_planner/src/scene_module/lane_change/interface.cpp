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

#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

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

  LaneChangePath selected_path;
  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  const auto [found_valid_path, found_safe_path] = module_type_->getSafePath(selected_path);

  return found_valid_path;
}

bool LaneChangeInterface::isExecutionReady() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  LaneChangePath selected_path;
  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  const auto [found_valid_path, found_safe_path] = module_type_->getSafePath(selected_path);

  return found_safe_path;
}

ModuleStatus LaneChangeInterface::updateState()
{
  if (!module_type_->isValidPath()) {
    current_state_ = ModuleStatus::FAILURE;
    return current_state_;
  }

  if (module_type_->isAbortState()) {
    current_state_ = ModuleStatus::RUNNING;
    return current_state_;
  }

  if (module_type_->isCancelConditionSatisfied()) {
    current_state_ = ModuleStatus::FAILURE;
    return current_state_;
  }

  if (module_type_->hasFinishedLaneChange()) {
    current_state_ = ModuleStatus::SUCCESS;
    return current_state_;
  }

  current_state_ = ModuleStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput LaneChangeInterface::plan()
{
  resetPathCandidate();
  resetPathReference();

  if (!module_type_->isValidPath()) {
    return {};
  }

  if (module_type_->isAbortState()) {
    resetPathIfAbort();
  }

  module_type_->setPreviousDrivableLanes(
    getPreviousModuleOutput().drivable_area_info.drivable_lanes);
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

  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(*getPreviousModuleOutput().path);
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;

  out.drivable_area_info.drivable_lanes =
    getPreviousModuleOutput().drivable_area_info.drivable_lanes;

  module_type_->setPreviousModulePaths(
    getPreviousModuleOutput().reference_path, getPreviousModuleOutput().path);
  module_type_->updateLaneChangeStatus();

  if (!module_type_->isValidPath()) {
    path_candidate_ = std::make_shared<PathWithLaneId>();
    return out;
  }

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  path_reference_ = getPreviousModuleOutput().reference_path;
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

void LaneChangeInterface::resetPathIfAbort()
{
  if (!is_abort_approval_requested_) {
    RCLCPP_DEBUG(getLogger(), "%s - resetPathIfAbort: removeRTCStatus", name().c_str());
    removeRTCStatus();
    is_abort_approval_requested_ = true;
    clearAbortApproval();
    return;
  }

  if (isActivated()) {
    RCLCPP_DEBUG(getLogger(), "%s - resetPathIfAbort: isActivated = false.", name().c_str());
    is_abort_path_approved_ = true;
    clearWaitingApproval();
  } else {
    RCLCPP_DEBUG(getLogger(), "%s - resetPathIfAbort: isActivated = true.", name().c_str());
    clearAbortApproval();
    waitApproval();
  }
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
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  const auto debug_data = module_type_->getDebugData();
  const auto debug_valid_path = module_type_->getDebugValidPath();

  debug_marker_.markers.clear();
  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  add(showObjectInfo(debug_data, "object_debug_info"));
  add(showLerpedPose(debug_data, "lerp_pose_before_true"));
  add(showPolygonPose(debug_data, "expected_pose"));
  add(showPolygon(debug_data, "lerped_polygon"));
  add(showAllValidLaneChangePath(debug_valid_path, "lane_change_valid_paths"));
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
    output.path->points, current_position, status.lane_change_path.shift_line.start.position);
  const auto finish_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_position, status.lane_change_path.shift_line.end.position);

  // TODO(tkhmy) add handle status TRYING
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status.lane_change_path.shift_line.start, status.lane_change_path.shift_line.end},
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
    {selected_path.shift_line.start, selected_path.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    SteeringFactor::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");
}
void LaneChangeInterface::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitLaneChangeInterface(this);
  }
}

void SceneModuleVisitor::visitLaneChangeInterface(const LaneChangeInterface * interface) const
{
  lane_change_visitor_ = interface->get_debug_msg_array();
}

LaneChangeBTInterface::LaneChangeBTInterface(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map,
  std::unique_ptr<LaneChangeBase> && module_type)
: LaneChangeInterface{name, node, parameters, rtc_interface_ptr_map, std::move(module_type)}
{
}

void LaneChangeBTInterface::processOnEntry()
{
  module_type_->updateLaneChangeStatus();
}

BehaviorModuleOutput LaneChangeBTInterface::plan()
{
  resetPathCandidate();
  resetPathReference();
  is_activated_ = isActivated();

  if (!module_type_->isValidPath()) {
    return {};
  }

  if (module_type_->isAbortState()) {
    resetPathIfAbort();
  }

  auto output = module_type_->generateOutput();
  path_reference_ = getPreviousModuleOutput().reference_path;
  *prev_approved_path_ = *output.path;

  updateSteeringFactorPtr(output);
  clearWaitingApproval();

  return output;
}

BehaviorModuleOutput LaneChangeBTInterface::planWaitingApproval()
{
  const auto path = module_type_->getReferencePath();
  if (!path.points.empty()) {
    *prev_approved_path_ = module_type_->getReferencePath();
  }

  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(*prev_approved_path_);
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  path_reference_ = getPreviousModuleOutput().reference_path;
  updateRTCStatus(
    candidate.start_distance_to_path_change, candidate.finish_distance_to_path_change);
  is_abort_path_approved_ = false;

  return out;
}

CandidateOutput LaneChangeBTInterface::planCandidate() const
{
  LaneChangePath selected_path;

  if (module_type_->isAbortState()) {
    selected_path = module_type_->getLaneChangePath();
  } else {
    [[maybe_unused]] const auto [found_valid_path, found_safe_path] =
      module_type_->getSafePath(selected_path);
  }

  selected_path.path.header = module_type_->getRouteHeader();

  if (selected_path.path.points.empty()) {
    return {};
  }

  CandidateOutput output = assignToCandidate(selected_path, module_type_->getEgoPosition());

  updateSteeringFactorPtr(output, selected_path);
  return output;
}

void LaneChangeBTInterface::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitLaneChangeBTInterface(this);
  }
}

void SceneModuleVisitor::visitLaneChangeBTInterface(const LaneChangeBTInterface * module) const
{
  external_request_lane_change_bt_visitor_ = module->get_debug_msg_array();
}

ExternalRequestLaneChangeLeftBTModule::ExternalRequestLaneChangeLeftBTModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters)
: LaneChangeBTInterface{
    name, node, parameters, createRTCInterfaceMap(node, name, {""}),
    std::make_unique<ExternalRequestLaneChangeBT>(parameters, Direction::LEFT)}
{
}

ExternalRequestLaneChangeRightBTModule::ExternalRequestLaneChangeRightBTModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<LaneChangeParameters> & parameters)
: LaneChangeBTInterface{
    name, node, parameters, createRTCInterfaceMap(node, name, {""}),
    std::make_unique<ExternalRequestLaneChangeBT>(parameters, Direction::RIGHT)}
{
}
}  // namespace behavior_path_planner
