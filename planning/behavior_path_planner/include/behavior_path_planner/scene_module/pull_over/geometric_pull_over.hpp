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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GEOMETRIC_PULL_OVER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GEOMETRIC_PULL_OVER_HPP_

#include "behavior_path_planner/scene_module/pull_over/pull_over_planner_base.hpp"
#include "behavior_path_planner/scene_module/utils/geometric_parallel_parking.hpp"
#include "behavior_path_planner/scene_module/utils/occupancy_grid_based_collision_detector.hpp"

#include <lane_departure_checker/lane_departure_checker.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using lane_departure_checker::LaneDepartureChecker;
class GeometricPullOver : public PullOverPlannerBase
{
public:
  GeometricPullOver(
    rclcpp::Node & node, const PullOverParameters & parameters,
    const ParallelParkingParameters & parallel_parking_parameters,
    const LaneDepartureChecker & lane_departure_checker,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map,
    const bool is_forward);

  PullOverPlannerType getPlannerType() const override
  {
    return is_forward_ ? PullOverPlannerType::ARC_FORWARD : PullOverPlannerType::ARC_BACKWARD;
  }
  Pose getCr() const { return planner_.getCr(); }
  Pose getCl() const { return planner_.getCl(); }

  boost::optional<PullOverPath> plan(const Pose & goal_pose) override;

  std::vector<PullOverPath> generatePullOverPaths(
    const lanelet::ConstLanelets & road_lanes, const lanelet::ConstLanelets & shoulder_lanes,
    const Pose & goal_pose) const;
  std::vector<PullOverPath> selectValidPaths(
    const std::vector<PullOverPath> & paths, const lanelet::ConstLanelets & road_lanes,
    const lanelet::ConstLanelets & shoulder_lanes, const bool is_in_goal_route_section,
    const Pose & goal_pose) const;
  bool hasEnoughDistance(
    const PullOverPath & path, const lanelet::ConstLanelets & road_lanes,
    const bool is_in_goal_route_section, const Pose & goal_pose) const;

protected:
  ParallelParkingParameters parallel_parking_parameters_;
  LaneDepartureChecker lane_departure_checker_{};
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_;
  bool is_forward_{true};

  GeometricParallelParking planner_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GEOMETRIC_PULL_OVER_HPP_
