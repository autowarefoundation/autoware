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

#include "behavior_path_planner/utils/start_goal_planner_common/utils.hpp"

namespace behavior_path_planner::utils::start_goal_planner_common
{

using motion_utils::calcDecelDistWithJerkAndAccConstraints;

std::optional<double> calcFeasibleDecelDistance(
  std::shared_ptr<const PlannerData> planner_data, const double acc_lim, const double jerk_lim,
  const double target_velocity)
{
  const auto v_now = planner_data->self_odometry->twist.twist.linear.x;
  const auto a_now = planner_data->self_acceleration->accel.accel.linear.x;

  if (v_now < target_velocity) {
    return 0.0;
  }

  auto min_stop_distance = calcDecelDistWithJerkAndAccConstraints(
    v_now, target_velocity, a_now, -acc_lim, jerk_lim, -1.0 * jerk_lim);

  if (!min_stop_distance) {
    return {};
  }

  min_stop_distance = std::max(*min_stop_distance, 0.0);

  return min_stop_distance;
}

void modifyVelocityByDirection(
  std::vector<PathWithLaneId> & paths,
  std::vector<std::pair<double, double>> & terminal_vel_acc_pairs, const double target_velocity,
  const double acceleration)
{
  assert(paths.size() == terminal_vel_acc_pairs.size());

  auto path_itr = std::begin(paths);
  auto pair_itr = std::begin(terminal_vel_acc_pairs);

  for (; path_itr != std::end(paths); ++path_itr, ++pair_itr) {
    const auto is_driving_forward = motion_utils::isDrivingForward(path_itr->points);

    // If the number of points in the path is less than 2, don't insert stop velocity and
    // set pairs_terminal_velocity_and_accel to 0
    if (!is_driving_forward) {
      *pair_itr = std::make_pair(0.0, 0.0);
      continue;
    }

    if (*is_driving_forward) {
      for (auto & point : path_itr->points) {
        // TODO(Sugahara): velocity calculation can be improved by considering the acceleration
        point.point.longitudinal_velocity_mps = std::abs(point.point.longitudinal_velocity_mps);
      }
      // TODO(Sugahara): Consider the calculation of the target velocity and acceleration for ego's
      // predicted path when ego will stop at the end of the path
      *pair_itr = std::make_pair(target_velocity, acceleration);
    } else {
      for (auto & point : path_itr->points) {
        point.point.longitudinal_velocity_mps = -std::abs(point.point.longitudinal_velocity_mps);
      }
      *pair_itr = std::make_pair(-target_velocity, -acceleration);
    }
    path_itr->points.back().point.longitudinal_velocity_mps = 0.0;
  }
}

void updateEgoPredictedPathParams(
  std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params)
{
  ego_predicted_path_params =
    std::make_shared<EgoPredictedPathParams>(start_planner_params->ego_predicted_path_params);
}
void updateEgoPredictedPathParams(
  std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::shared_ptr<GoalPlannerParameters> & goal_planner_params)
{
  ego_predicted_path_params =
    std::make_shared<EgoPredictedPathParams>(goal_planner_params->ego_predicted_path_params);
}

void updateSafetyCheckParams(
  std::shared_ptr<SafetyCheckParams> & safety_check_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params)
{
  safety_check_params =
    std::make_shared<SafetyCheckParams>(start_planner_params->safety_check_params);
}

void updateSafetyCheckParams(
  std::shared_ptr<SafetyCheckParams> & safety_check_params,
  const std::shared_ptr<GoalPlannerParameters> & goal_planner_params)
{
  safety_check_params =
    std::make_shared<SafetyCheckParams>(goal_planner_params->safety_check_params);
}

void updateObjectsFilteringParams(
  std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params)
{
  objects_filtering_params =
    std::make_shared<ObjectsFilteringParams>(start_planner_params->objects_filtering_params);
}

void updateObjectsFilteringParams(
  std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
  const std::shared_ptr<GoalPlannerParameters> & goal_planner_params)
{
  objects_filtering_params =
    std::make_shared<ObjectsFilteringParams>(goal_planner_params->objects_filtering_params);
}

void updatePathProperty(
  std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::pair<double, double> & pairs_terminal_velocity_and_accel)
{
  // If acceleration is close to 0, the ego predicted path will be too short, so a minimum value is
  // necessary to ensure a reasonable path length.
  const double min_accel_for_ego_predicted_path = ego_predicted_path_params->min_acceleration;
  const double acceleration =
    std::max(pairs_terminal_velocity_and_accel.second, min_accel_for_ego_predicted_path);

  ego_predicted_path_params->max_velocity = pairs_terminal_velocity_and_accel.first;
  ego_predicted_path_params->acceleration = acceleration;
}

void initializeCollisionCheckDebugMap(CollisionCheckDebugMap & collision_check_debug_map)
{
  collision_check_debug_map.clear();
}

void updateSafetyCheckTargetObjectsData(
  StartGoalPlannerData & data, const PredictedObjects & filtered_objects,
  const TargetObjectsOnLane & target_objects_on_lane,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path)
{
  data.filtered_objects = filtered_objects;
  data.target_objects_on_lane = target_objects_on_lane;
  data.ego_predicted_path = ego_predicted_path;
}

std::pair<double, double> getPairsTerminalVelocityAndAccel(
  const std::vector<std::pair<double, double>> & pairs_terminal_velocity_and_accel,
  const size_t current_path_idx)
{
  if (pairs_terminal_velocity_and_accel.size() <= current_path_idx) {
    return std::make_pair(0.0, 0.0);
  }
  return pairs_terminal_velocity_and_accel.at(current_path_idx);
}

std::optional<PathWithLaneId> generateFeasibleStopPath(
  PathWithLaneId & current_path, std::shared_ptr<const PlannerData> planner_data,
  geometry_msgs::msg::Pose & stop_pose, const double maximum_deceleration,
  const double maximum_jerk)
{
  if (current_path.points.empty()) {
    return {};
  }

  // try to insert stop point in current_path after approval
  // but if can't stop with constraints(maximum deceleration, maximum jerk), don't insert stop point
  const auto min_stop_distance =
    behavior_path_planner::utils::start_goal_planner_common::calcFeasibleDecelDistance(
      planner_data, maximum_deceleration, maximum_jerk, 0.0);

  if (!min_stop_distance) {
    return {};
  }

  // set stop point
  const auto stop_idx = motion_utils::insertStopPoint(
    planner_data->self_odometry->pose.pose, *min_stop_distance, current_path.points);

  if (!stop_idx) {
    return {};
  }

  stop_pose = current_path.points.at(*stop_idx).point.pose;

  return current_path;
}

}  // namespace behavior_path_planner::utils::start_goal_planner_common
