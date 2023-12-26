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
#ifndef BEHAVIOR_PATH_LANE_CHANGE_MODULE__SCENE_HPP_
#define BEHAVIOR_PATH_LANE_CHANGE_MODULE__SCENE_HPP_

#include "behavior_path_lane_change_module/utils/base_class.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using behavior_path_planner::utils::path_safety_checker::CollisionCheckDebug;
using behavior_path_planner::utils::path_safety_checker::CollisionCheckDebugMap;
using behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityAndPolygonStamped;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using behavior_path_planner::utils::path_safety_checker::PredictedPathWithPolygon;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using route_handler::Direction;

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

  void insertStopPoint(const lanelet::ConstLanelets & lanelets, PathWithLaneId & path) override;

  PathWithLaneId getReferencePath() const override;

  std::optional<PathWithLaneId> extendPath() override;

  void resetParameters() override;

  TurnSignalInfo updateOutputTurnSignal() override;

  bool calcAbortPath() override;

  PathSafetyStatus isApprovedPathSafe() const override;

  bool isRequiredStop(const bool is_object_coming_from_rear) override;

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

  bool isStoppedAtRedTrafficLight() const override;

protected:
  lanelet::ConstLanelets getCurrentLanes() const override;

  lanelet::ConstLanelets getLaneChangeLanes(
    const lanelet::ConstLanelets & current_lanes, Direction direction) const override;

  int getNumToPreferredLane(const lanelet::ConstLanelet & lane) const override;

  std::vector<double> sampleLongitudinalAccValues(
    const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes) const;

  std::vector<double> calcPrepareDuration(
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

  bool hasEnoughLengthToCrosswalk(
    const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const;

  bool hasEnoughLengthToIntersection(
    const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const;

  bool hasEnoughLengthToTrafficLight(
    const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const;

  bool getLaneChangePaths(
    const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
    Direction direction, LaneChangePaths * candidate_paths,
    const utils::path_safety_checker::RSSparams rss_params, const bool is_stuck,
    const bool check_safety = true) const override;

  TurnSignalInfo calcTurnSignalInfo() override;

  bool isValidPath(const PathWithLaneId & path) const override;

  PathSafetyStatus isLaneChangePathSafe(
    const LaneChangePath & lane_change_path, const LaneChangeTargetObjects & target_objects,
    const utils::path_safety_checker::RSSparams & rss_params, const bool is_stuck,
    CollisionCheckDebugMap & debug_data) const;

  LaneChangeTargetObjectIndices filterObject(
    const PredictedObjects & objects, const lanelet::ConstLanelets & current_lanes,
    const lanelet::ConstLanelets & target_lanes,
    const lanelet::ConstLanelets & target_backward_lanes) const;

  std::vector<ExtendedPredictedObject> filterObjectsInTargetLane(
    const LaneChangeTargetObjects & objects, const lanelet::ConstLanelets & target_lanes) const;

  //! @brief Check if the ego vehicle is in stuck by a stationary obstacle.
  //! @param obstacle_check_distance Distance to check ahead for any objects that might be
  //! obstructing ego path. It makes sense to use values like the maximum lane change distance.
  bool isVehicleStuck(
    const lanelet::ConstLanelets & current_lanes, const double obstacle_check_distance) const;

  bool isVehicleStuck(const lanelet::ConstLanelets & current_lanes) const;

  double calcMaximumLaneChangeLength(
    const lanelet::ConstLanelet & current_terminal_lanelet, const double max_acc) const;

  std::pair<double, double> calcCurrentMinMaxAcceleration() const;

  void setStopPose(const Pose & stop_pose);

  void updateStopTime();

  double getStopTime() const { return stop_time_; }

  double stop_time_{0.0};
};
}  // namespace behavior_path_planner
#endif  // BEHAVIOR_PATH_LANE_CHANGE_MODULE__SCENE_HPP_
