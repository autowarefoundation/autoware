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

#include "obstacle_cruise_planner/planner_interface.hpp"

namespace
{
StopSpeedExceeded createStopSpeedExceededMsg(
  const rclcpp::Time & current_time, const bool stop_flag)
{
  StopSpeedExceeded msg{};
  msg.stamp = current_time;
  msg.stop_speed_exceeded = stop_flag;
  return msg;
}

tier4_planning_msgs::msg::StopReasonArray makeStopReasonArray(
  const rclcpp::Time & current_time, const geometry_msgs::msg::Pose & stop_pose,
  const TargetObstacle & stop_obstacle)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = current_time;

  // create stop factor
  tier4_planning_msgs::msg::StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose;
  stop_factor.stop_factor_points.emplace_back(stop_obstacle.collision_point);

  // create stop reason stamped
  tier4_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = tier4_planning_msgs::msg::StopReason::OBSTACLE_STOP;
  stop_reason_msg.stop_factors.emplace_back(stop_factor);

  // create stop reason array
  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

double calcMinimumDistanceToStop(const double initial_vel, const double min_acc)
{
  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

boost::optional<TargetObstacle> getClosestStopObstacle(
  const Trajectory & traj, const std::vector<TargetObstacle> & target_obstacles)
{
  if (target_obstacles.empty()) {
    return boost::none;
  }

  boost::optional<TargetObstacle> closest_stop_obstacle = boost::none;
  double dist_to_closest_stop_obstacle = std::numeric_limits<double>::max();
  for (const auto & obstacle : target_obstacles) {
    // Ignore obstacle that has not stopped
    if (!obstacle.has_stopped) {
      continue;
    }

    const double dist_to_stop_obstacle =
      tier4_autoware_utils::calcSignedArcLength(traj.points, 0, obstacle.collision_point);
    if (dist_to_stop_obstacle < dist_to_closest_stop_obstacle) {
      dist_to_closest_stop_obstacle = dist_to_stop_obstacle;
      closest_stop_obstacle = obstacle;
    }
  }
  return closest_stop_obstacle;
}
}  // namespace

Trajectory PlannerInterface::generateStopTrajectory(
  const ObstacleCruisePlannerData & planner_data, DebugData & debug_data)
{
  if (planner_data.target_obstacles.empty()) {
    return planner_data.traj;
  }

  // Get Closest Stop Obstacle
  const auto closest_stop_obstacle =
    getClosestStopObstacle(planner_data.traj, planner_data.target_obstacles);
  if (!closest_stop_obstacle) {
    return planner_data.traj;
  }

  // Get Closest Obstacle Stop Distance
  const double closest_obstacle_dist = tier4_autoware_utils::calcSignedArcLength(
    planner_data.traj.points, 0, closest_stop_obstacle->collision_point);

  const auto negative_dist_to_ego = tier4_autoware_utils::calcSignedArcLength(
    planner_data.traj.points, planner_data.current_pose, 0, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!negative_dist_to_ego) {
    return planner_data.traj;
  }
  const double dist_to_ego = -negative_dist_to_ego.get();

  // If behavior stop point is ahead of the closest_obstacle_stop point within a certain margin
  // we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const double margin_from_obstacle = [&]() {
    const auto closest_behavior_stop_dist_from_ego =
      tier4_autoware_utils::calcDistanceToForwardStopPoint(
        planner_data.traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
        nearest_yaw_deviation_threshold_);

    if (closest_behavior_stop_dist_from_ego) {
      const double closest_obstacle_stop_dist_from_ego = closest_obstacle_dist - dist_to_ego -
                                                         longitudinal_info_.safe_distance_margin -
                                                         vehicle_info_.max_longitudinal_offset_m;

      const double stop_dist_diff =
        *closest_behavior_stop_dist_from_ego - closest_obstacle_stop_dist_from_ego;
      if (0.0 < stop_dist_diff && stop_dist_diff < longitudinal_info_.safe_distance_margin) {
        // Use shorter margin (min_behavior_stop_margin) for obstacle stop
        return min_behavior_stop_margin_;
      }
    }
    return longitudinal_info_.safe_distance_margin;
  }();

  // Calculate feasible stop margin (Check the feasibility)
  const double feasible_stop_dist =
    calcMinimumDistanceToStop(planner_data.current_vel, longitudinal_info_.limit_min_accel) +
    dist_to_ego;
  const double closest_obstacle_stop_dist =
    closest_obstacle_dist - margin_from_obstacle - vehicle_info_.max_longitudinal_offset_m;

  bool will_collide_with_obstacle = false;
  double feasible_margin_from_obstacle = margin_from_obstacle;
  if (closest_obstacle_stop_dist < feasible_stop_dist) {
    feasible_margin_from_obstacle =
      margin_from_obstacle - (feasible_stop_dist - closest_obstacle_stop_dist);
    will_collide_with_obstacle = true;
  }

  const size_t collision_idx = tier4_autoware_utils::findNearestIndex(
    planner_data.traj.points, closest_stop_obstacle->collision_point);
  const size_t zero_vel_idx = obstacle_cruise_utils::getIndexWithLongitudinalOffset(
    planner_data.traj.points,
    -vehicle_info_.max_longitudinal_offset_m - feasible_margin_from_obstacle, collision_idx);
  const size_t wall_idx = obstacle_cruise_utils::getIndexWithLongitudinalOffset(
    planner_data.traj.points, -feasible_margin_from_obstacle, collision_idx);

  // TODO(shimizu) insert stop point with interpolation
  // Generate Output Trajectory
  auto output_traj = planner_data.traj;
  for (size_t o_idx = zero_vel_idx; o_idx < output_traj.points.size(); ++o_idx) {
    output_traj.points.at(o_idx).longitudinal_velocity_mps = 0.0;
  }

  // virtual wall marker for stop obstacle
  const auto marker_pose = planner_data.traj.points.at(wall_idx).pose;
  const auto markers = tier4_autoware_utils::createStopVirtualWallMarker(
    marker_pose, "obstacle stop", planner_data.current_time, 0);
  tier4_autoware_utils::appendMarkerArray(markers, &debug_data.stop_wall_marker);
  debug_data.obstacles_to_stop.push_back(*closest_stop_obstacle);

  // Publish Stop Reason
  const auto stop_pose = output_traj.points.at(zero_vel_idx).pose;
  const auto stop_reasons_msg =
    makeStopReasonArray(planner_data.current_time, stop_pose, *closest_stop_obstacle);
  stop_reasons_pub_->publish(stop_reasons_msg);

  // Publish if ego vehicle collides with the obstacle with a limit acceleration
  const auto stop_speed_exceeded_msg =
    createStopSpeedExceededMsg(planner_data.current_time, will_collide_with_obstacle);
  stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);

  return output_traj;
}
