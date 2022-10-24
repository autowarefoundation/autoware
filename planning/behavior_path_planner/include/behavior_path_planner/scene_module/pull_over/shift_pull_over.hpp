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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__SHIFT_PULL_OVER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__SHIFT_PULL_OVER_HPP_

#include "behavior_path_planner/scene_module/pull_over/pull_over_planner_base.hpp"
#include "behavior_path_planner/scene_module/utils/occupancy_grid_based_collision_detector.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using lane_departure_checker::LaneDepartureChecker;

class ShiftPullOver : public PullOverPlannerBase
{
public:
  ShiftPullOver(
    rclcpp::Node & node, const PullOverParameters & parameters,
    const LaneDepartureChecker & lane_departure_checker,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map);

  PullOverPlannerType getPlannerType() const override { return PullOverPlannerType::SHIFT; };
  boost::optional<PullOverPath> plan(const Pose & goal_pose) override;

protected:
  PathWithLaneId generateRoadLaneReferencePath(
    const lanelet::ConstLanelets & road_lanes, const Pose & shift_end_pose,
    const double pull_over_distance) const;
  PathWithLaneId generateShoulderLaneReferencePath(
    const lanelet::ConstLanelets & shoulder_lanes, const Pose & shift_start_pose,
    const Pose & goal_pose, const double shoulder_center_to_goal_distance) const;
  boost::optional<PullOverPath> generatePullOverPath(
    const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
    const Pose & shift_end_pose, const Pose & goal_pose, const double lateral_jerk,
    const double road_center_to_goal_distance, const double shoulder_center_to_goal_distance,
    const double shoulder_left_bound_to_goal_distance) const;
  bool hasEnoughDistance(
    const PathWithLaneId & path, const lanelet::ConstLanelets & road_lanes, const Pose & start_pose,
    const Pose & goal_pose, const double pull_over_distance) const;
  bool isSafePath(const PathWithLaneId & path) const;

  LaneDepartureChecker lane_departure_checker_{};
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_{};

  static constexpr double resample_interval_{1.0};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__SHIFT_PULL_OVER_HPP_
