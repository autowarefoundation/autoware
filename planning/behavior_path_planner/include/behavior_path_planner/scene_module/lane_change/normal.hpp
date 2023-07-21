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
#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__NORMAL_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__NORMAL_HPP_

#include "behavior_path_planner/marker_util/debug_utilities.hpp"
#include "behavior_path_planner/scene_module/lane_change/base_class.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using marker_utils::CollisionCheckDebugMap;
using route_handler::Direction;
using tier4_planning_msgs::msg::LaneChangeDebugMsg;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;

class NormalLaneChange : public LaneChangeBase
{
public:
  NormalLaneChange(
    const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
    Direction direction);

  NormalLaneChange(const NormalLaneChange &) = delete;
  NormalLaneChange(NormalLaneChange &&) = delete;
  NormalLaneChange & operator=(const NormalLaneChange &) = delete;
  NormalLaneChange & operator=(NormalLaneChange &&) = delete;
  ~NormalLaneChange() override = default;

  void updateLaneChangeStatus() override;

  std::pair<bool, bool> getSafePath(LaneChangePath & safe_path) const override;

  LaneChangePath getLaneChangePath() const override;

  BehaviorModuleOutput generateOutput() override;

  void extendOutputDrivableArea(BehaviorModuleOutput & output) override;

  void insertStopPoint(PathWithLaneId & path) override;

  PathWithLaneId getReferencePath() const override;

  std::optional<PathWithLaneId> extendPath() override;

  void resetParameters() override;

  TurnSignalInfo updateOutputTurnSignal() override;

  bool getAbortPath() override;

  PathSafetyStatus isApprovedPathSafe() const override;

  bool isRequiredStop(const bool is_object_coming_from_rear) const override;

  bool isNearEndOfCurrentLanes(
    const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
    const double threshold) const override;

  bool hasFinishedLaneChange() const override;

  bool isAbleToReturnCurrentLane() const override;

  bool isEgoOnPreparePhase() const override;

  bool isAbleToStopSafely() const override;

  bool hasFinishedAbort() const override;

  bool isAbortState() const override;

  bool isLaneChangeRequired() const override;

protected:
  lanelet::ConstLanelets getCurrentLanes() const override;

  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, Direction direction) const override;

  int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const override;

  double calcPrepareDuration(
    const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes) const;

  LaneChangeTargetObjects getTargetObjects(
    const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes) const;

  PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double backward_path_length,
    const double prepare_length) const override;

  PathWithLaneId getTargetSegment(
    const lanelet::ConstLanelets & target_lanes, const Pose & lane_changing_start_pose,
    const double target_lane_length, const double lane_changing_length,
    const double lane_changing_velocity, const double buffer_for_next_lane_change) const;

  bool hasEnoughLength(
    const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes, const Direction direction = Direction::NONE) const;

  bool getLaneChangePaths(
    const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
    Direction direction, LaneChangePaths * candidate_paths,
    const bool check_safety = true) const override;

  TurnSignalInfo calcTurnSignalInfo() override;

  bool isValidPath(const PathWithLaneId & path) const override;

  PathSafetyStatus isLaneChangePathSafe(
    const LaneChangePath & lane_change_path, const LaneChangeTargetObjects & target_objects,
    const double front_decel, const double rear_decel,
    std::unordered_map<std::string, CollisionCheckDebug> & debug_data) const;

  rclcpp::Logger logger_ = rclcpp::get_logger("lane_change").get_child(getModuleTypeStr());
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__NORMAL_HPP_
