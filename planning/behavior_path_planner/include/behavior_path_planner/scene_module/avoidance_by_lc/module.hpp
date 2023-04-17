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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__MODULE_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__MODULE_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/utils/avoidance_by_lc/module_data.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_path.hpp"

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

class AvoidanceByLCModule : public SceneModuleInterface
{
public:
  AvoidanceByLCModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<AvoidanceByLCParameters> parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  ModuleStatus updateState() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void updateData() override;

  std::shared_ptr<LaneChangeDebugMsgArray> get_debug_msg_array() const;
  void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const override;

#ifndef USE_OLD_ARCHITECTURE
  void updateModuleParams(const std::shared_ptr<AvoidanceByLCParameters> & parameters)
  {
    parameters_ = parameters;
  }
#endif

private:
  std::shared_ptr<AvoidanceByLCParameters> parameters_;
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
  bool is_activated_ = false;

  void resetParameters();

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
      return;
    }
    if (candidate.lateral_shift < 0.0) {
      rtc_interface_ptr_map_.at("right")->updateCooperateStatus(
        uuid_map_.at("right"), isExecutionReady(), candidate.start_distance_to_path_change,
        candidate.finish_distance_to_path_change, clock_->now());
      return;
    }

    RCLCPP_WARN_STREAM(
      getLogger(),
      "Direction is UNKNOWN, start_distance = " << candidate.start_distance_to_path_change);
  }

  lanelet::ConstLanelets getCurrentLanes(const PathWithLaneId & path) const
  {
    const auto & route_handler = planner_data_->route_handler;
    const auto & common_parameters = planner_data_->parameters;

    lanelet::ConstLanelets reference_lanes{};
    for (const auto & p : path.points) {
      reference_lanes.push_back(
        planner_data_->route_handler->getLaneletsFromId(p.lane_ids.front()));
    }

    lanelet::ConstLanelet current_lane;
    lanelet::utils::query::getClosestLanelet(reference_lanes, getEgoPose(), &current_lane);

    return route_handler->getLaneletSequence(
      current_lane, getEgoPose(), common_parameters.backward_path_length,
      common_parameters.forward_path_length);
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

  AvoidancePlanningData calcAvoidancePlanningData(DebugData & debug) const;
  AvoidancePlanningData avoidance_data_;
  mutable DebugData debug_data_;

  ObjectDataArray registered_objects_;
  mutable ObjectDataArray stopped_objects_;

  ObjectData createObjectData(
    const AvoidancePlanningData & data, const PredictedObject & object) const;

  void fillAvoidanceTargetObjects(AvoidancePlanningData & data, DebugData & debug) const;

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
  bool isAvoidancePlanRunning() const;
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

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE_BY_LC__MODULE_HPP_
