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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GOAL_SEARCHER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GOAL_SEARCHER_HPP_

#include "behavior_path_planner/scene_module/pull_over/goal_searcher_base.hpp"
#include "behavior_path_planner/scene_module/utils/occupancy_grid_based_collision_detector.hpp"

#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <memory>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using tier4_autoware_utils::LinearRing2d;
using BasicPolygons2d = std::vector<lanelet::BasicPolygon2d>;

class GoalSearcher : public GoalSearcherBase
{
public:
  GoalSearcher(
    const PullOverParameters & parameters, const LinearRing2d & vehicle_footprint,
    const std::shared_ptr<OccupancyGridBasedCollisionDetector> & occupancy_grid_map);

  GoalCandidates search(const Pose & original_goal_pose) override;
  void update(GoalCandidates & goal_candidates) const override;

private:
  void createAreaPolygons(std::vector<Pose> original_search_poses);
  bool checkCollision(const Pose & pose) const;
  bool checkCollisionWithLongitudinalDistance(
    const Pose & ego_pose, const PredictedObjects & dynamic_objects) const;
  BasicPolygons2d getNoStoppingAreaPolygons(const lanelet::ConstLanelets & lanes) const;
  bool isInAreas(const LinearRing2d & footprint, const BasicPolygons2d & areas) const;

  LinearRing2d vehicle_footprint_{};
  std::shared_ptr<OccupancyGridBasedCollisionDetector> occupancy_grid_map_{};
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__PULL_OVER__GOAL_SEARCHER_HPP_
