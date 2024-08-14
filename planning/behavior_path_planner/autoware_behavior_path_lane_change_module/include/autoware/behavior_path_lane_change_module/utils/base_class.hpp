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
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BASE_CLASS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BASE_CLASS_HPP_

#include "autoware/behavior_path_lane_change_module/utils/data_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/debug_structs.hpp"
#include "autoware/behavior_path_lane_change_module/utils/path.hpp"
#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/turn_signal_decider.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/universe_utils/system/stop_watch.hpp"

#include <autoware/universe_utils/system/time_keeper.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <string>
#include <utility>

namespace autoware::behavior_path_planner
{
using autoware::route_handler::Direction;
using autoware::universe_utils::StopWatch;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using lane_change::PathSafetyStatus;
using tier4_planning_msgs::msg::PathWithLaneId;

class LaneChangeBase
{
public:
  LaneChangeBase(
    std::shared_ptr<LaneChangeParameters> parameters, LaneChangeModuleType type,
    Direction direction)
  : lane_change_parameters_{std::move(parameters)},
    common_data_ptr_{std::make_shared<lane_change::CommonData>()},
    direction_{direction},
    type_{type},
    time_keeper_(std::make_shared<universe_utils::TimeKeeper>())
  {
  }

  LaneChangeBase(const LaneChangeBase &) = delete;
  LaneChangeBase(LaneChangeBase &&) = delete;
  LaneChangeBase & operator=(const LaneChangeBase &) = delete;
  LaneChangeBase & operator=(LaneChangeBase &&) = delete;
  virtual ~LaneChangeBase() = default;

  virtual void update_lanes(const bool is_approved) = 0;

  virtual void updateLaneChangeStatus() = 0;

  virtual std::pair<bool, bool> getSafePath(LaneChangePath & safe_path) const = 0;

  virtual BehaviorModuleOutput generateOutput() = 0;

  virtual void extendOutputDrivableArea(BehaviorModuleOutput & output) const = 0;

  virtual PathWithLaneId getReferencePath() const = 0;

  virtual std::optional<PathWithLaneId> extendPath() = 0;

  virtual void resetParameters() = 0;

  virtual TurnSignalInfo updateOutputTurnSignal() const = 0;

  virtual bool hasFinishedLaneChange() const = 0;

  virtual bool hasFinishedAbort() const = 0;

  virtual bool isLaneChangeRequired() = 0;

  virtual bool isAbortState() const = 0;

  virtual bool isAbleToReturnCurrentLane() const = 0;

  virtual LaneChangePath getLaneChangePath() const = 0;

  virtual BehaviorModuleOutput getTerminalLaneChangePath() const = 0;

  virtual bool isEgoOnPreparePhase() const = 0;

  virtual bool isRequiredStop(const bool is_trailing_object) = 0;

  virtual PathSafetyStatus isApprovedPathSafe() const = 0;

  virtual PathSafetyStatus evaluateApprovedPathWithUnsafeHysteresis(
    PathSafetyStatus approve_path_safety_status) = 0;

  virtual bool isNearEndOfCurrentLanes(
    const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
    const double threshold) const = 0;

  virtual bool isStoppedAtRedTrafficLight() const = 0;

  virtual bool calcAbortPath() = 0;

  virtual bool specialRequiredCheck() const { return false; }

  virtual bool specialExpiredCheck() const { return false; }

  void setPreviousModuleOutput(const BehaviorModuleOutput & prev_module_output)
  {
    prev_module_output_ = prev_module_output;
  }

  virtual void updateSpecialData() {}

  virtual void insertStopPoint(
    [[maybe_unused]] const lanelet::ConstLanelets & lanelets,
    [[maybe_unused]] PathWithLaneId & path)
  {
  }

  const LaneChangeStatus & getLaneChangeStatus() const { return status_; }

  const lane_change::Debug & getDebugData() const { return lane_change_debug_; }

  const Pose & getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  const Point & getEgoPosition() const { return getEgoPose().position; }

  const Twist & getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }

  const lanelet::ConstLanelets & get_current_lanes() const
  {
    return common_data_ptr_->lanes_ptr->current;
  }

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

  void setData(const std::shared_ptr<const PlannerData> & data)
  {
    planner_data_ = data;
    if (!common_data_ptr_->bpp_param_ptr) {
      common_data_ptr_->bpp_param_ptr =
        std::make_shared<BehaviorPathPlannerParameters>(data->parameters);
    }

    if (!common_data_ptr_->lanes_ptr) {
      common_data_ptr_->lanes_ptr = std::make_shared<lane_change::Lanes>();
    }

    if (!common_data_ptr_->lanes_polygon_ptr) {
      common_data_ptr_->lanes_polygon_ptr = std::make_shared<lane_change::LanesPolygon>();
    }

    common_data_ptr_->self_odometry_ptr = data->self_odometry;
    common_data_ptr_->route_handler_ptr = data->route_handler;
    common_data_ptr_->lc_param_ptr = lane_change_parameters_;
    common_data_ptr_->lc_type = type_;
    common_data_ptr_->direction = direction_;
  }

  void setTimeKeeper(const std::shared_ptr<universe_utils::TimeKeeper> & time_keeper)
  {
    time_keeper_ = time_keeper;
  }

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

  virtual TurnSignalInfo get_current_turn_signal_info() = 0;

protected:
  virtual int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const = 0;

  virtual PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double backward_path_length,
    const double prepare_length) const = 0;

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
  lane_change::CommonDataPtr common_data_ptr_{};
  BehaviorModuleOutput prev_module_output_{};
  std::optional<Pose> lane_change_stop_pose_{std::nullopt};

  PathWithLaneId prev_approved_path_{};

  int unsafe_hysteresis_count_{0};
  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_activated_{false};

  Direction direction_{Direction::NONE};
  LaneChangeModuleType type_{LaneChangeModuleType::NORMAL};

  mutable StopWatch<std::chrono::milliseconds> stop_watch_;
  mutable lane_change::Debug lane_change_debug_;

  rclcpp::Logger logger_ = utils::lane_change::getLogger(getModuleTypeStr());
  mutable rclcpp::Clock clock_{RCL_ROS_TIME};

  mutable std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__BASE_CLASS_HPP_
