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

#include "behavior_path_planner/scene_module/lane_change/base_class.hpp"

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

  PathWithLaneId getReferencePath() const override;

  void resetParameters() override;

  TurnSignalInfo updateOutputTurnSignal() override;

  bool getAbortPath() override;

  PathSafetyStatus isApprovedPathSafe() const override;

  bool isRequiredStop(const bool is_object_coming_from_rear) const override;

  bool isNearEndOfLane() const override;

  bool hasFinishedLaneChange() const override;

  bool isAbleToReturnCurrentLane() const override;

  bool isEgoOnPreparePhase() const override;

  bool isAbleToStopSafely() const override;

  bool hasFinishedAbort() const override;

  bool isAbortState() const override;

protected:
  lanelet::ConstLanelets getCurrentLanes() const override;

  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, Direction direction) const override;

  int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const override;

  PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double arc_length_from_current,
    const double backward_path_length, const double prepare_length) const override;

  bool getLaneChangePaths(
    const lanelet::ConstLanelets & original_lanelets,
    const lanelet::ConstLanelets & target_lanelets, Direction direction,
    LaneChangePaths * candidate_paths) const override;

  std::vector<DrivableLanes> getDrivableLanes() const override;

  void calcTurnSignalInfo() override;

  bool isValidPath(const PathWithLaneId & path) const override;
};

class NormalLaneChangeBT : public NormalLaneChange
{
public:
  NormalLaneChangeBT(
    const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
    Direction direction);

  NormalLaneChangeBT(const NormalLaneChangeBT &) = delete;
  NormalLaneChangeBT(NormalLaneChangeBT &&) = delete;
  NormalLaneChangeBT & operator=(const NormalLaneChangeBT &) = delete;
  NormalLaneChangeBT & operator=(NormalLaneChangeBT &&) = delete;
  ~NormalLaneChangeBT() override = default;

  PathWithLaneId getReferencePath() const override;

protected:
  lanelet::ConstLanelets getCurrentLanes() const override;

  int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const override;

  PathWithLaneId getPrepareSegment(
    const lanelet::ConstLanelets & current_lanes, const double arc_length_from_current,
    const double backward_path_length, const double prepare_length) const override;

  std::vector<DrivableLanes> getDrivableLanes() const override;
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__LANE_CHANGE__NORMAL_HPP_
