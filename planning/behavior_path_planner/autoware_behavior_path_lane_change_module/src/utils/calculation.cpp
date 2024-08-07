// Copyright 2024 TIER IV, Inc.
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

#include <autoware/behavior_path_lane_change_module/utils/calculation.hpp>
#include <autoware/behavior_path_planner_common/utils/utils.hpp>

namespace autoware::behavior_path_planner::utils::lane_change::calculation
{
double calc_ego_dist_to_terminal_end(const CommonDataPtr & common_data_ptr)
{
  const auto & lanes_ptr = common_data_ptr->lanes_ptr;
  const auto & current_lanes = lanes_ptr->current;
  const auto & current_pose = common_data_ptr->get_ego_pose();

  return calc_dist_from_pose_to_terminal_end(common_data_ptr, current_lanes, current_pose);
}

double calc_dist_from_pose_to_terminal_end(
  const CommonDataPtr & common_data_ptr, const lanelet::ConstLanelets & lanes,
  const Pose & src_pose)
{
  if (lanes.empty()) {
    return 0.0;
  }

  const auto & lanes_ptr = common_data_ptr->lanes_ptr;
  const auto & goal_pose = common_data_ptr->route_handler_ptr->getGoalPose();

  if (lanes_ptr->current_lane_in_goal_section) {
    return utils::getSignedDistance(src_pose, goal_pose, lanes);
  }
  return utils::getDistanceToEndOfLane(src_pose, lanes);
}
}  // namespace autoware::behavior_path_planner::utils::lane_change::calculation
