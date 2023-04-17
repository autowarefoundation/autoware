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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"

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
using route_handler::Direction;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class LaneChangeModule : public SceneModuleInterface
{
public:
#ifdef USE_OLD_ARCHITECTURE
  LaneChangeModule(
    const std::string & name, rclcpp::Node & node,
    std::shared_ptr<LaneChangeParameters> parameters);
#else
  LaneChangeModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<LaneChangeParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map,
    Direction direction, LaneChangeModuleType type);
#endif
  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  ModuleStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  std::shared_ptr<LaneChangeDebugMsgArray> get_debug_msg_array() const;
  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

#ifndef USE_OLD_ARCHITECTURE
  void updateModuleParams(const std::shared_ptr<LaneChangeParameters> & parameters)
  {
    parameters_ = parameters;
  }
#endif

private:
  std::shared_ptr<LaneChangeParameters> parameters_;
  LaneChangeStatus status_;
  PathShifter path_shifter_;
  mutable LaneChangeDebugMsgArray lane_change_debug_msg_array_;
  LaneChangeStates current_lane_change_state_;
  std::shared_ptr<LaneChangePath> abort_path_;
  PathWithLaneId prev_approved_path_;

  double lane_change_lane_length_{200.0};
  double check_distance_{100.0};
  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_abort_condition_satisfied_{false};
  bool is_activated_{false};

#ifdef USE_OLD_ARCHITECTURE
  UUID candidate_uuid_;
#else
  Direction direction_{Direction::NONE};
  LaneChangeModuleType type_{LaneChangeModuleType::NORMAL};
#endif

  void resetParameters();

#ifdef USE_OLD_ARCHITECTURE
  void waitApprovalLeft(const double start_distance, const double finish_distance)
  {
    rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
      uuid_map_.at("left"), isExecutionReady(), start_distance, finish_distance, clock_->now());
    is_waiting_approval_ = true;
  }

  void waitApprovalRight(const double start_distance, const double finish_distance)
  {
    rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
      uuid_map_.at("right"), isExecutionReady(), start_distance, finish_distance, clock_->now());
    is_waiting_approval_ = true;
  }

  void updateRTCStatus(const CandidateOutput & candidate)
  {
    if (candidate.lateral_shift > 0.0) {
      rtc_interface_ptr_map_.at("left")->updateCooperateStatus(
        uuid_map_.at("left"), isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_map_.at("left");
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        uuid_map_.at("right"), isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      candidate_uuid_ = uuid_map_.at("right");
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(),
      "Direction is UNKNOWN, start_distance = " << candidate.start_distance_to_path_change);
  }

  void removePreviousRTCStatusLeft()
  {
    if (rtc_interface_ptr_map_.at("left")->isRegistered(uuid_map_.at("left"))) {
      rtc_interface_ptr_map_.at("left")->removeCooperateStatus(uuid_map_.at("left"));
    }
  }

  void removePreviousRTCStatusRight()
  {
    if (rtc_interface_ptr_map_.at("right")->isRegistered(uuid_map_.at("right"))) {
      rtc_interface_ptr_map_.at("right")->removeCooperateStatus(uuid_map_.at("right"));
    }
  }
#endif

  PathWithLaneId getReferencePath() const;
#ifdef USE_OLD_ARCHITECTURE
  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const;
#endif
  std::pair<bool, bool> getSafePath(
    const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
    LaneChangePath & safe_path) const;
  PathWithLaneId extendBackwardLength(const PathWithLaneId & original_path) const;

  void updateLaneChangeStatus();
  void generateExtendedDrivableArea(PathWithLaneId & path);
  void updateOutputTurnSignal(BehaviorModuleOutput & output);
  void updateSteeringFactorPtr(const BehaviorModuleOutput & output);
  bool isApprovedPathSafe(Pose & ego_pose_before_collision) const;
  void calcTurnSignalInfo();

  void updateSteeringFactorPtr(
    const CandidateOutput & output, const LaneChangePath & selected_path) const;
  bool isSafe() const;
  bool isValidPath() const;
  bool isValidPath(const PathWithLaneId & path) const;
  bool isNearEndOfLane() const;
  bool isCurrentVelocityLow() const;
  bool isAbortConditionSatisfied();
  bool hasFinishedLaneChange() const;
  bool isAbortState() const;

  // getter
  std_msgs::msg::Header getRouteHeader() const;
  void resetPathIfAbort();

  // debug
  void setObjectDebugVisualization() const;
  mutable std::unordered_map<std::string, CollisionCheckDebug> object_debug_;
  mutable std::vector<LaneChangePath> debug_valid_path_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__LANE_CHANGE_MODULE_HPP_
