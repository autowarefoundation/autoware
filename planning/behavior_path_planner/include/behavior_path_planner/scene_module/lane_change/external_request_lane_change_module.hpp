// Copyright 2022 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__EXTERNAL_REQUEST_LANE_CHANGE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__EXTERNAL_REQUEST_LANE_CHANGE_MODULE_HPP_

#include "behavior_path_planner/scene_module/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class ExternalRequestLaneChangeModule : public SceneModuleInterface
{
public:
  enum class Direction {
    RIGHT = 0,
    LEFT,
  };

  ExternalRequestLaneChangeModule(
    const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters,
    const Direction & direction);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BT::NodeStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void onEntry() override;
  void onExit() override;

  std::shared_ptr<LaneChangeDebugMsgArray> get_debug_msg_array() const;
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

protected:
  std::shared_ptr<LaneChangeParameters> parameters_;
  LaneChangeStatus status_;
  PathShifter path_shifter_;
  mutable LaneChangeDebugMsgArray lane_change_debug_msg_array_;
  LaneChangeStates current_lane_change_state_;
  std::shared_ptr<LaneChangePath> abort_path_;
  PathWithLaneId prev_approved_path_;

  Direction direction_;

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};
  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_abort_condition_satisfied_{false};
  bool is_activated_ = false;

  void resetParameters();
  void resetPathIfAbort();

  void waitApproval(const double start_distance, const double finish_distance)
  {
    updateRTCStatus(start_distance, finish_distance);
    is_waiting_approval_ = true;
  }

  lanelet::ConstLanelets get_original_lanes() const;
  PathWithLaneId getReferencePath() const;
  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const;
  std::pair<bool, bool> getSafePath(
    const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
    LaneChangePath & safe_path) const;

  void updateLaneChangeStatus();
  void generateExtendedDrivableArea(PathWithLaneId & path);
  void updateOutputTurnSignal(BehaviorModuleOutput & output);
  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);
  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;
  void calcTurnSignalInfo();

  bool isSafe() const;
  bool isValidPath(const PathWithLaneId & path) const;
  bool isApprovedPathSafe(Pose & ego_pose_before_collision) const;
  bool isNearEndOfLane() const;
  bool isCurrentSpeedLow() const;
  bool isAbortConditionSatisfied();
  bool hasFinishedLaneChange() const;
  bool isAbortState() const;

  // getter
  Pose getEgoPose() const;
  Twist getEgoTwist() const;
  std_msgs::msg::Header getRouteHeader() const;

  // debug
  mutable std::unordered_map<std::string, CollisionCheckDebug> object_debug_;
  mutable std::vector<LaneChangePath> debug_valid_path_;

  void setObjectDebugVisualization() const;
};

class ExternalRequestLaneChangeRightModule : public ExternalRequestLaneChangeModule
{
public:
  ExternalRequestLaneChangeRightModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<LaneChangeParameters> parameters);
};

class ExternalRequestLaneChangeLeftModule : public ExternalRequestLaneChangeModule
{
public:
  ExternalRequestLaneChangeLeftModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<LaneChangeParameters> parameters);
};

}  // namespace behavior_path_planner
// clang-format off
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__EXTERNAL_REQUEST_LANE_CHANGE_MODULE_HPP_  // NOLINT
// clang-format on
