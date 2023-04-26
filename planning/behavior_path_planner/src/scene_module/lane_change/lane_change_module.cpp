// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"

#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

LaneChangeModule::LaneChangeModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters)
: SceneModuleInterface{name, node, createRTCInterfaceMap(node, name, {"left", "right"})},
  module_type_{
    std::make_unique<NormalLaneChangeBT>(parameters, LaneChangeModuleType::NORMAL, Direction::NONE)}
{
}

void LaneChangeModule::processOnEntry()
{
  module_type_->updateLaneChangeStatus();
}

void LaneChangeModule::processOnExit()
{
  module_type_->resetParameters();
  debug_marker_.markers.clear();
  resetPathCandidate();
}

bool LaneChangeModule::isExecutionRequested() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] = module_type_->getSafePath(selected_path);

  return found_valid_path;
}

bool LaneChangeModule::isExecutionReady() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] = module_type_->getSafePath(selected_path);

  return found_safe_path;
}

ModuleStatus LaneChangeModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "%s updateState", name().c_str());

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

BehaviorModuleOutput LaneChangeModule::plan()
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
  prev_approved_path_ = *output.path;

  updateSteeringFactorPtr(output);
  clearWaitingApproval();

  return output;
}

BehaviorModuleOutput LaneChangeModule::planWaitingApproval()
{
  const auto path = module_type_->getReferencePath();
  if (!path.points.empty()) {
    prev_approved_path_ = module_type_->getReferencePath();
  }

  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(prev_approved_path_);
  out.reference_path = getPreviousModuleOutput().reference_path;
  out.turn_signal_info = getPreviousModuleOutput().turn_signal_info;

  module_type_->extendOutputDrivableArea(out);

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);

  path_reference_ = getPreviousModuleOutput().reference_path;
  updateRTCStatus(candidate);
  waitApproval();
  is_abort_path_approved_ = false;

  return out;
}

CandidateOutput LaneChangeModule::planCandidate() const
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

  CandidateOutput output;
  output.path_candidate = selected_path.path;
  output.lateral_shift = utils::lane_change::getLateralShift(selected_path);
  output.start_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, module_type_->getEgoPosition(),
    selected_path.shift_line.start.position);
  output.finish_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, module_type_->getEgoPosition(),
    selected_path.shift_line.end.position);

  updateSteeringFactorPtr(output, selected_path);
  return output;
}

void LaneChangeModule::resetPathIfAbort()
{
  if (!module_type_->isAbortState()) {
    return;
  }

  if (!is_abort_approval_requested_) {
    removeRTCStatus();
    RCLCPP_DEBUG(getLogger(), "[abort] uuid is reset to request abort approval.");
    is_abort_approval_requested_ = true;
    is_abort_path_approved_ = false;
    return;
  }

  if (isActivated()) {
    RCLCPP_DEBUG(getLogger(), "[abort] isActivated() is true. set is_abort_path_approved to true.");
    is_abort_path_approved_ = true;
    clearWaitingApproval();
  } else {
    RCLCPP_DEBUG(getLogger(), "[abort] isActivated() is False.");
    is_abort_path_approved_ = false;
    waitApproval();
  }
}

void LaneChangeModule::setData(const std::shared_ptr<const PlannerData> & data)
{
  module_type_->setData(data);
}

void LaneChangeModule::setObjectDebugVisualization() const
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

std::shared_ptr<LaneChangeDebugMsgArray> LaneChangeModule::get_debug_msg_array() const
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

void LaneChangeModule::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  const auto current_pose = module_type_->getEgoPose();
  const auto status = module_type_->getLaneChangeStatus();
  const double start_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status.lane_change_path.shift_line.start.position);
  const double finish_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status.lane_change_path.shift_line.end.position);

  const auto turn_signal_info = output.turn_signal_info;
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
    {status.lane_change_path.shift_line.start, status.lane_change_path.shift_line.end},
    {start_distance, finish_distance}, SteeringFactor::LANE_CHANGE, steering_factor_direction,
    SteeringFactor::TURNING, "");
}

void LaneChangeModule::updateSteeringFactorPtr(
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

void LaneChangeModule::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitLaneChangeModule(this);
  }
}

void SceneModuleVisitor::visitLaneChangeModule(const LaneChangeModule * module) const
{
  lane_change_visitor_ = module->get_debug_msg_array();
}
}  // namespace behavior_path_planner
