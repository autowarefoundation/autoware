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

#include "behavior_path_goal_planner_module/default_fixed_goal_planner.hpp"

#include "behavior_path_goal_planner_module/util.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <memory>

namespace behavior_path_planner
{
using Point2d = tier4_autoware_utils::Point2d;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
BehaviorModuleOutput DefaultFixedGoalPlanner::plan(
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  BehaviorModuleOutput output =
    // use planner previous module reference path
    getPreviousModuleOutput();
  const PathWithLaneId smoothed_path = modifyPathForSmoothGoalConnection(output.path, planner_data);
  output.path = smoothed_path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  return output;
}

lanelet::ConstLanelets DefaultFixedGoalPlanner::extractLaneletsFromPath(
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto & rh = planner_data->route_handler;
  lanelet::ConstLanelets refined_path_lanelets;
  for (size_t i = 0; i < refined_path.points.size(); ++i) {
    const auto & path_point = refined_path.points.at(i);
    int64_t lane_id = path_point.lane_ids.at(0);
    lanelet::ConstLanelet lanelet = rh->getLaneletsFromId(lane_id);
    bool is_unique =
      std::find(refined_path_lanelets.begin(), refined_path_lanelets.end(), lanelet) ==
      refined_path_lanelets.end();
    if (is_unique) {
      refined_path_lanelets.push_back(lanelet);
    }
  }
  return refined_path_lanelets;
}

bool DefaultFixedGoalPlanner::isPathValid(
  const PathWithLaneId & refined_path,
  const std::shared_ptr<const PlannerData> & planner_data) const
{
  const auto lanelets = extractLaneletsFromPath(refined_path, planner_data);
  // std::any_of detects whether any point lies outside lanelets
  bool has_points_outside_lanelet = std::any_of(
    refined_path.points.begin(), refined_path.points.end(),
    [&lanelets](const auto & refined_path_point) {
      return !utils::isInLanelets(refined_path_point.point.pose, lanelets);
    });
  return !has_points_outside_lanelet;
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
  double goal_search_radius{planner_data->parameters.refine_goal_search_radius_range};
  // TODO(shen): define in the parameter
  constexpr double range_reduce_by{1.0};  // set a reasonable value, 10% - 20% of the
                                          // refine_goal_search_radius_range is recommended
  bool is_valid_path{false};
  autoware_auto_planning_msgs::msg::PathWithLaneId refined_path;
  while (goal_search_radius >= 0 && !is_valid_path) {
    refined_path =
      utils::refinePathForGoal(goal_search_radius, M_PI * 0.5, path, refined_goal, goal_lane_id);
    if (isPathValid(refined_path, planner_data)) {
      is_valid_path = true;
    }
    goal_search_radius -= range_reduce_by;
  }
  return refined_path;
}

}  // namespace behavior_path_planner
