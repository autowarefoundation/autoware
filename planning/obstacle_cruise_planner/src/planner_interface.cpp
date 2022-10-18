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
  geometry_msgs::msg::Point stop_factor_point = stop_obstacle.collision_points.front().point;
  stop_factor_point.z = stop_pose.position.z;
  stop_factor.stop_factor_points.emplace_back(stop_factor_point);

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

tier4_planning_msgs::msg::StopReasonArray makeEmptyStopReasonArray(
  const rclcpp::Time & current_time)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = current_time;

  // create stop reason stamped
  tier4_planning_msgs::msg::StopReason stop_reason_msg;
  stop_reason_msg.reason = tier4_planning_msgs::msg::StopReason::OBSTACLE_STOP;

  // create stop reason array
  tier4_planning_msgs::msg::StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

double calcMinimumDistanceToStop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }

  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}
}  // namespace

Trajectory PlannerInterface::generateStopTrajectory(
  const ObstacleCruisePlannerData & planner_data, DebugData & debug_data)
{
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  if (planner_data.target_obstacles.empty()) {
    stop_reasons_pub_->publish(makeEmptyStopReasonArray(planner_data.current_time));

    // delete marker
    const auto markers =
      motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data.stop_wall_marker);

    return planner_data.traj;
  }

  // Get Closest Stop Obstacle
  const auto closest_stop_obstacle =
    obstacle_cruise_utils::getClosestStopObstacle(planner_data.traj, planner_data.target_obstacles);
  if (!closest_stop_obstacle || closest_stop_obstacle->collision_points.empty()) {
    // delete marker
    const auto markers =
      motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data.stop_wall_marker);

    return planner_data.traj;
  }

  // Get Closest Obstacle Stop Distance
  const double closest_obstacle_dist = motion_utils::calcSignedArcLength(
    planner_data.traj.points, 0, closest_stop_obstacle->collision_points.front().point);

  const auto negative_dist_to_ego = motion_utils::calcSignedArcLength(
    planner_data.traj.points, planner_data.current_pose, 0, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!negative_dist_to_ego) {
    // delete marker
    const auto markers =
      motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data.stop_wall_marker);

    return planner_data.traj;
  }
  const double dist_to_ego = -negative_dist_to_ego.get();

  // If behavior stop point is ahead of the closest_obstacle_stop point within a certain margin
  // we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const double margin_from_obstacle = [&]() {
    const size_t nearest_segment_idx =
      motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
        planner_data.traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
        nearest_yaw_deviation_threshold_);
    const auto closest_behavior_stop_idx =
      motion_utils::searchZeroVelocityIndex(planner_data.traj.points, nearest_segment_idx + 1);

    if (!closest_behavior_stop_idx) {
      return longitudinal_info_.safe_distance_margin;
    }

    const double closest_behavior_stop_dist_from_ego = motion_utils::calcSignedArcLength(
      planner_data.traj.points, planner_data.current_pose.position, nearest_segment_idx,
      *closest_behavior_stop_idx);

    if (*closest_behavior_stop_idx == planner_data.traj.points.size() - 1) {
      // Closest behavior stop point is the end point
      const double closest_obstacle_stop_dist_from_ego =
        closest_obstacle_dist - dist_to_ego - longitudinal_info_.terminal_safe_distance_margin -
        abs_ego_offset;
      const double stop_dist_diff =
        closest_behavior_stop_dist_from_ego - closest_obstacle_stop_dist_from_ego;
      if (stop_dist_diff < longitudinal_info_.safe_distance_margin) {
        // Use terminal margin (terminal_safe_distance_margin) for obstacle stop
        return longitudinal_info_.terminal_safe_distance_margin;
      }
    } else {
      const double closest_obstacle_stop_dist_from_ego = closest_obstacle_dist - dist_to_ego -
                                                         longitudinal_info_.safe_distance_margin -
                                                         abs_ego_offset;
      const double stop_dist_diff =
        closest_behavior_stop_dist_from_ego - closest_obstacle_stop_dist_from_ego;
      if (0.0 < stop_dist_diff && stop_dist_diff < longitudinal_info_.safe_distance_margin) {
        // Use shorter margin (min_behavior_stop_margin) for obstacle stop
        return min_behavior_stop_margin_;
      }
    }
    return longitudinal_info_.safe_distance_margin;
  }();

  // Calculate feasible stop margin (Check the feasibility)
  const double feasible_stop_dist = calcMinimumDistanceToStop(
                                      planner_data.current_vel, longitudinal_info_.limit_max_accel,
                                      longitudinal_info_.limit_min_accel) +
                                    dist_to_ego;
  const double closest_obstacle_stop_dist =
    closest_obstacle_dist - margin_from_obstacle - abs_ego_offset;

  bool will_collide_with_obstacle = false;
  double feasible_margin_from_obstacle = margin_from_obstacle;
  if (closest_obstacle_stop_dist < feasible_stop_dist) {
    feasible_margin_from_obstacle =
      margin_from_obstacle - (feasible_stop_dist - closest_obstacle_stop_dist);
    will_collide_with_obstacle = true;
  }

  // Generate Output Trajectory
  auto output_traj = planner_data.traj;
  const double zero_vel_dist =
    std::max(0.0, closest_obstacle_dist - abs_ego_offset - feasible_margin_from_obstacle);
  const auto zero_vel_idx = motion_utils::insertStopPoint(0, zero_vel_dist, output_traj.points);
  if (zero_vel_idx) {
    // virtual wall marker for stop obstacle
    const auto markers = motion_utils::createStopVirtualWallMarker(
      output_traj.points.at(*zero_vel_idx).pose, "obstacle stop", planner_data.current_time, 0,
      abs_ego_offset);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data.stop_wall_marker);
    debug_data.obstacles_to_stop.push_back(*closest_stop_obstacle);

    // Publish Stop Reason
    const auto stop_pose = output_traj.points.at(*zero_vel_idx).pose;
    const auto stop_reasons_msg =
      makeStopReasonArray(planner_data.current_time, stop_pose, *closest_stop_obstacle);
    stop_reasons_pub_->publish(stop_reasons_msg);

    // Publish if ego vehicle collides with the obstacle with a limit acceleration
    const auto stop_speed_exceeded_msg =
      createStopSpeedExceededMsg(planner_data.current_time, will_collide_with_obstacle);
    stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);
  }

  return output_traj;
}

double PlannerInterface::calcDistanceToCollisionPoint(
  const ObstacleCruisePlannerData & planner_data, const geometry_msgs::msg::Point & collision_point)
{
  const double offset = planner_data.is_driving_forward
                          ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                          : std::abs(vehicle_info_.min_longitudinal_offset_m);

  const auto dist_to_collision_point = motion_utils::calcSignedArcLength(
    planner_data.traj.points, planner_data.current_pose, collision_point,
    nearest_dist_deviation_threshold_, nearest_yaw_deviation_threshold_);

  if (dist_to_collision_point) {
    return dist_to_collision_point.get() - offset;
  }

  return motion_utils::calcSignedArcLength(
           planner_data.traj.points, planner_data.current_pose.position, collision_point) -
         offset;
}
