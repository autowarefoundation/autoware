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
#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BASE_CLASS_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BASE_CLASS_HPP_

#include "behavior_path_lane_change_module/utils/data_structs.hpp"
#include "behavior_path_lane_change_module/utils/path.hpp"
#include "behavior_path_lane_change_module/utils/utils.hpp"
#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/turn_signal_decider.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>
#include <utility>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using data::lane_change::PathSafetyStatus;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using route_handler::Direction;
using tier4_autoware_utils::StopWatch;

class LaneChangeBase
{
public:
  LaneChangeBase(
    std::shared_ptr<LaneChangeParameters> parameters, LaneChangeModuleType type,
    Direction direction)
  : lane_change_parameters_{std::move(parameters)}, direction_{direction}, type_{type}
  {
  }

  LaneChangeBase(const LaneChangeBase &) = delete;
  LaneChangeBase(LaneChangeBase &&) = delete;
  LaneChangeBase & operator=(const LaneChangeBase &) = delete;
  LaneChangeBase & operator=(LaneChangeBase &&) = delete;
  virtual ~LaneChangeBase() = default;

  virtual void updateLaneChangeStatus() = 0;

  virtual std::pair<bool, bool> getSafePath(LaneChangePath & safe_path) const = 0;

  virtual BehaviorModuleOutput generateOutput() = 0;

  virtual void extendOutputDrivableArea(BehaviorModuleOutput & output) = 0;

  virtual PathWithLaneId getReferencePath() const = 0;

  virtual std::optional<PathWithLaneId> extendPath() = 0;

  virtual void resetParameters() = 0;

  virtual TurnSignalInfo updateOutputTurnSignal() = 0;

  virtual bool hasFinishedLaneChange() const = 0;

  virtual bool hasFinishedAbort() const = 0;

  virtual bool isLaneChangeRequired() const = 0;

  virtual bool isAbortState() const = 0;

  virtual bool isAbleToReturnCurrentLane() const = 0;

  virtual LaneChangePath getLaneChangePath() const = 0;

  virtual bool isEgoOnPreparePhase() const = 0;

  virtual bool isRequiredStop(const bool is_object_coming_from_rear) = 0;

  virtual PathSafetyStatus isApprovedPathSafe() const = 0;

  virtual bool isNearEndOfCurrentLanes(
    const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
    const double threshold) const = 0;

  virtual bool isStoppedAtRedTrafficLight() const = 0;

  virtual bool calcAbortPath() = 0;

  virtual bool specialRequiredCheck() const { return false; }

  virtual bool specialExpiredCheck() const { return false; }

  virtual void setPreviousModulePaths(
    const PathWithLaneId & prev_module_reference_path, const PathWithLaneId & prev_module_path)
  {
    if (!prev_module_reference_path.points.empty()) {
      prev_module_reference_path_ = prev_module_reference_path;
    }
    if (!prev_module_path.points.empty()) {
      prev_module_path_ = prev_module_path;
    }
  };

  virtual void setPreviousDrivableAreaInfo(const DrivableAreaInfo & prev_drivable_area_info)
  {
    prev_drivable_area_info_ = prev_drivable_area_info;
  }

  virtual void setPreviousTurnSignalInfo(const TurnSignalInfo & prev_turn_signal_info)
  {
    prev_turn_signal_info_ = prev_turn_signal_info;
  }

  virtual void updateSpecialData() {}

  virtual void insertStopPoint(
    [[maybe_unused]] const lanelet::ConstLanelets & lanelets,
    [[maybe_unused]] PathWithLaneId & path)
  {
  }

  const LaneChangeStatus & getLaneChangeStatus() const { return status_; }

  const LaneChangePaths & getDebugValidPath() const { return debug_valid_path_; }

  const CollisionCheckDebugMap & getDebugData() const { return object_debug_; }

  const CollisionCheckDebugMap & getAfterApprovalDebugData() const
  {
    return object_debug_after_approval_;
  }

  const LaneChangeTargetObjects & getDebugFilteredObjects() const
  {
    return debug_filtered_objects_;
  }

  const Pose & getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  const Point & getEgoPosition() const { return getEgoPose().position; }

  const Twist & getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }

  const BehaviorPathPlannerParameters & getCommonParam() const { return planner_data_->parameters; }

  LaneChangeParameters getLaneChangeParam() const { return *lane_change_parameters_; }

  bool isCancelEnabled() const { return lane_change_parameters_->cancel.enable_on_prepare_phase; }

  bool isAbortEnabled() const
  {
    return lane_change_parameters_->cancel.enable_on_lane_changing_phase;
  }

  bool isSafe() const { return status_.is_safe; }

  bool isStopState() const { return current_lane_change_state_ == LaneChangeStates::Stop; }

  bool isValidPath() const { return status_.is_valid_path; }

  void setData(const std::shared_ptr<const PlannerData> & data) { planner_data_ = data; }

  void toNormalState() { current_lane_change_state_ = LaneChangeStates::Normal; }

  void toStopState() { current_lane_change_state_ = LaneChangeStates::Stop; }

  void toCancelState() { current_lane_change_state_ = LaneChangeStates::Cancel; }

  void toAbortState() { current_lane_change_state_ = LaneChangeStates::Abort; }

  double getEgoVelocity() const { return getEgoTwist().linear.x; }

  std::shared_ptr<RouteHandler> getRouteHandler() const { return planner_data_->route_handler; }

  std_msgs::msg::Header getRouteHeader() const { return getRouteHandler()->getRouteHeader(); }

  std::string getModuleTypeStr() const { return std::string{magic_enum::enum_name(type_)}; }

  LaneChangeModuleType getModuleType() const { return type_; }

  TurnSignalDecider getTurnSignalDecider() { return planner_data_->turn_signal_decider; }

  Direction getDirection() const
  {
    if (direction_ == Direction::NONE && !status_.lane_change_path.path.points.empty()) {
      const auto lateral_shift = utils::lane_change::getLateralShift(status_.lane_change_path);
      return lateral_shift > 0.0 ? Direction::LEFT : Direction::RIGHT;
    }

    return direction_;
  }

  std::optional<Pose> getStopPose() const { return lane_change_stop_pose_; }

  void resetStopPose() { lane_change_stop_pose_ = std::nullopt; }

protected:
  virtual lanelet::ConstLanelets getCurrentLanes() const = 0;

  virtual int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const = 0;

  virtual PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double backward_path_length,
    const double prepare_length) const = 0;

  virtual bool getLaneChangePaths(
    const lanelet::ConstLanelets & original_lanelets,
    const lanelet::ConstLanelets & target_lanelets, Direction direction,
    LaneChangePaths * candidate_paths, const utils::path_safety_checker::RSSparams rss_params,
    const bool is_stuck, const bool check_safety) const = 0;

  virtual TurnSignalInfo calcTurnSignalInfo() = 0;

  virtual bool isValidPath(const PathWithLaneId & path) const = 0;

  virtual bool isAbleToStopSafely() const = 0;

  virtual lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, Direction direction) const = 0;

  LaneChangeStatus status_{};
  PathShifter path_shifter_{};

  LaneChangeStates current_lane_change_state_{};

  std::shared_ptr<LaneChangeParameters> lane_change_parameters_{};
  std::shared_ptr<LaneChangePath> abort_path_{};
  std::shared_ptr<const PlannerData> planner_data_{};
  PathWithLaneId prev_module_reference_path_{};
  PathWithLaneId prev_module_path_{};
  DrivableAreaInfo prev_drivable_area_info_{};
  TurnSignalInfo prev_turn_signal_info_{};
  std::optional<Pose> lane_change_stop_pose_{std::nullopt};

  PathWithLaneId prev_approved_path_{};

  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_activated_{false};

  Direction direction_{Direction::NONE};
  LaneChangeModuleType type_{LaneChangeModuleType::NORMAL};

  mutable LaneChangePaths debug_valid_path_{};
  mutable CollisionCheckDebugMap object_debug_{};
  mutable CollisionCheckDebugMap object_debug_after_approval_{};
  mutable LaneChangeTargetObjects debug_filtered_objects_{};
  mutable double object_debug_lifetime_{0.0};
  mutable StopWatch<std::chrono::milliseconds> stop_watch_;

  rclcpp::Logger logger_ = utils::lane_change::getLogger(getModuleTypeStr());
  mutable rclcpp::Clock clock_{RCL_ROS_TIME};
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BASE_CLASS_HPP_
