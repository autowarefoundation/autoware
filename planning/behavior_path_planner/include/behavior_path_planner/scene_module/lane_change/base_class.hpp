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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BASE_CLASS_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BASE_CLASS_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"

#include <rclcpp/rclcpp.hpp>

#include "tier4_planning_msgs/msg/lane_change_debug_msg.hpp"
#include "tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp"
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebugMap;
using route_handler::Direction;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class LaneChangeBase
{
public:
  LaneChangeBase(
    const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
    Direction direction)
  : parameters_{parameters}, direction_{direction}, type_{type}
  {
    prev_module_reference_path_ = std::make_shared<PathWithLaneId>();
    prev_module_path_ = std::make_shared<PathWithLaneId>();
    prev_drivable_lanes_ = std::make_shared<std::vector<DrivableLanes>>();
  }

  virtual ~LaneChangeBase() = default;

  virtual void updateLaneChangeStatus() = 0;

  virtual std::pair<bool, bool> getSafePath(LaneChangePath & safe_path) const = 0;

  virtual BehaviorModuleOutput generateOutput() = 0;

  virtual void extendOutputDrivableArea(BehaviorModuleOutput & output) = 0;

  virtual bool hasFinishedLaneChange() const = 0;

  virtual PathWithLaneId getReferencePath() const = 0;

  virtual bool isCancelConditionSatisfied() = 0;

  virtual bool isAbortConditionSatisfied(const Pose & pose) = 0;

  virtual void resetParameters() = 0;

  virtual TurnSignalInfo updateOutputTurnSignal() = 0;

  virtual void setPreviousModulePaths(
    const std::shared_ptr<PathWithLaneId> & prev_module_reference_path,
    const std::shared_ptr<PathWithLaneId> & prev_module_path)
  {
    if (prev_module_reference_path) {
      *prev_module_reference_path_ = *prev_module_reference_path;
    }
    if (prev_module_path) {
      *prev_module_path_ = *prev_module_path;
    }
  };

  virtual void setPreviousDrivableLanes(const std::vector<DrivableLanes> & prev_drivable_lanes)
  {
    if (prev_drivable_lanes_) {
      *prev_drivable_lanes_ = prev_drivable_lanes;
    }
  }

  const LaneChangeStatus & getLaneChangeStatus() const { return status_; }

  LaneChangePath getLaneChangePath() const
  {
    return isAbortState() ? *abort_path_ : status_.lane_change_path;
  }

  const LaneChangePaths & getDebugValidPath() const { return debug_valid_path_; }

  const CollisionCheckDebugMap & getDebugData() const { return object_debug_; }

  bool isAbortState() const
  {
    if (!parameters_->enable_abort_lane_change) {
      return false;
    }

    const auto is_within_current_lane = utils::lane_change::isEgoWithinOriginalLane(
      status_.current_lanes, getEgoPose(), planner_data_->parameters);

    if (!is_within_current_lane) {
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

  bool isSafe() const { return status_.is_safe; }

  bool isStopState() const { return current_lane_change_state_ == LaneChangeStates::Stop; }

  bool isValidPath() const { return status_.is_valid_path; }

  std_msgs::msg::Header getRouteHeader() const
  {
    return planner_data_->route_handler->getRouteHeader();
  }

  void setData(const std::shared_ptr<const PlannerData> & data) { planner_data_ = data; }

  const Pose & getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  const Point & getEgoPosition() const { return getEgoPose().position; }

  const Twist & getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }

  std::shared_ptr<RouteHandler> getRouteHandler() const { return planner_data_->route_handler; }

  double getEgoVelocity() const { return getEgoTwist().linear.x; }

  Direction getDirection() const
  {
    if (direction_ == Direction::NONE) {
      const auto lateral_shift = utils::lane_change::getLateralShift(status_.lane_change_path);
      return lateral_shift > 0.0 ? Direction::LEFT : Direction::RIGHT;
    }

    return direction_;
  }

protected:
  virtual lanelet::ConstLanelets getCurrentLanes() const = 0;

  virtual int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const = 0;

  virtual PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double arc_length_from_current,
    const double backward_path_length, const double prepare_length,
    const double prepare_velocity) const = 0;

  virtual bool getLaneChangePaths(
    const lanelet::ConstLanelets & original_lanelets,
    const lanelet::ConstLanelets & target_lanelets, LaneChangePaths * candidate_paths) const = 0;

  virtual std::vector<DrivableLanes> getDrivableLanes() const = 0;

  virtual bool isApprovedPathSafe(Pose & ego_pose_before_collision) const = 0;

  virtual void calcTurnSignalInfo() = 0;

  virtual bool isValidPath(const PathWithLaneId & path) const = 0;

  lanelet::ConstLanelets getLaneChangeLanes(const lanelet::ConstLanelets & current_lanes) const;

  bool isNearEndOfLane() const
  {
    const auto & current_pose = getEgoPose();
    const auto shift_intervals = planner_data_->route_handler->getLateralIntervalsToPreferredLane(
      status_.current_lanes.back());
    const double threshold =
      utils::calcMinimumLaneChangeLength(planner_data_->parameters, shift_intervals);

    return (std::max(0.0, utils::getDistanceToEndOfLane(current_pose, status_.current_lanes)) -
            threshold) < planner_data_->parameters.backward_length_buffer_for_end_of_lane;
  }

  bool isCurrentSpeedLow() const
  {
    constexpr double threshold_ms = 10.0 * 1000 / 3600;
    return getEgoTwist().linear.x < threshold_ms;
  }

  LaneChangeStatus status_{};
  PathShifter path_shifter_{};

  LaneChangeStates current_lane_change_state_{};

  std::shared_ptr<LaneChangeParameters> parameters_{};
  std::shared_ptr<LaneChangePath> abort_path_{};
  std::shared_ptr<const PlannerData> planner_data_{};
  std::shared_ptr<PathWithLaneId> prev_module_reference_path_{};
  std::shared_ptr<PathWithLaneId> prev_module_path_{};
  std::shared_ptr<std::vector<DrivableLanes>> prev_drivable_lanes_{};

  PathWithLaneId prev_approved_path_{};

  double lane_change_lane_length_{200.0};
  double check_length_{100.0};

  bool is_abort_path_approved_{false};
  bool is_abort_approval_requested_{false};
  bool is_activated_{false};

  Direction direction_{Direction::NONE};
  LaneChangeModuleType type_{LaneChangeModuleType::NORMAL};

  mutable CollisionCheckDebugMap object_debug_{};
  mutable LaneChangePaths debug_valid_path_{};
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__BASE_CLASS_HPP_
