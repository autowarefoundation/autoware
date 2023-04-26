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

#include "behavior_path_planner/utils/goal_planner/default_fixed_goal_planner.hpp"

#include "behavior_path_planner/utils/goal_planner/util.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>
#include <vector>

namespace behavior_path_planner
{

BehaviorModuleOutput DefaultFixedGoalPlanner::plan(
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  auto output = getReferencePath(planner_data);
  if (!output) {
    return {};
  }
  const PathWithLaneId smoothed_path =
    modifyPathForSmoothGoalConnection(*(output->path), planner_data);

  output->path = std::make_shared<PathWithLaneId>(smoothed_path);
  output->reference_path = std::make_shared<PathWithLaneId>(smoothed_path);
  return *output;
}

boost::optional<BehaviorModuleOutput> DefaultFixedGoalPlanner::getReferencePath(
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto & route_handler = planner_data->route_handler;
  const auto current_pose = planner_data->self_odometry->pose.pose;
  const double dist_threshold = planner_data->parameters.ego_nearest_dist_threshold;
  const double yaw_threshold = planner_data->parameters.ego_nearest_yaw_threshold;

  lanelet::ConstLanelet current_lane{};
  if (!route_handler->getClosestLaneletWithConstrainsWithinRoute(
        current_pose, &current_lane, dist_threshold, yaw_threshold)) {
    return {};  // TODO(Horibe)
  }

  return utils::getReferencePath(current_lane, planner_data);
}

PathWithLaneId DefaultFixedGoalPlanner::modifyPathForSmoothGoalConnection(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto goal = planner_data->route_handler->getGoalPose();
  const auto goal_lane_id = planner_data->route_handler->getGoalLaneId();

  Pose refined_goal{};
  {
    lanelet::ConstLanelet goal_lanelet;
    if (planner_data->route_handler->getGoalLanelet(&goal_lanelet)) {
      refined_goal = utils::refineGoal(goal, goal_lanelet);
    } else {
      refined_goal = goal;
    }
  }

  auto refined_path = utils::refinePathForGoal(
    planner_data->parameters.refine_goal_search_radius_range, M_PI * 0.5, path, refined_goal,
    goal_lane_id);

  return refined_path;
}

}  // namespace behavior_path_planner
