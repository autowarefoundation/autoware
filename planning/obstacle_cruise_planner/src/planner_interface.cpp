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
  const StopObstacle & stop_obstacle)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = current_time;

  // create stop factor
  StopFactor stop_factor;
  stop_factor.stop_pose = stop_pose;
  geometry_msgs::msg::Point stop_factor_point = stop_obstacle.collision_point;
  stop_factor_point.z = stop_pose.position.z;
  stop_factor.stop_factor_points.emplace_back(stop_factor_point);

  // create stop reason stamped
  StopReason stop_reason_msg;
  stop_reason_msg.reason = StopReason::OBSTACLE_STOP;
  stop_reason_msg.stop_factors.emplace_back(stop_factor);

  // create stop reason array
  StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

StopReasonArray makeEmptyStopReasonArray(const rclcpp::Time & current_time)
{
  // create header
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp = current_time;

  // create stop reason stamped
  StopReason stop_reason_msg;
  stop_reason_msg.reason = StopReason::OBSTACLE_STOP;

  // create stop reason array
  StopReasonArray stop_reason_array;
  stop_reason_array.header = header;
  stop_reason_array.stop_reasons.emplace_back(stop_reason_msg);
  return stop_reason_array;
}

VelocityFactorArray makeVelocityFactorArray(
  const rclcpp::Time & time, const std::optional<geometry_msgs::msg::Pose> pose = std::nullopt)
{
  VelocityFactorArray velocity_factor_array;
  velocity_factor_array.header.frame_id = "map";
  velocity_factor_array.header.stamp = time;

  if (pose) {
    using distance_type = VelocityFactor::_distance_type;
    VelocityFactor velocity_factor;
    velocity_factor.type = VelocityFactor::ROUTE_OBSTACLE;
    velocity_factor.pose = pose.value();
    velocity_factor.distance = std::numeric_limits<distance_type>::quiet_NaN();
    velocity_factor.status = VelocityFactor::UNKNOWN;
    velocity_factor.detail = std::string();
    velocity_factor_array.factors.push_back(velocity_factor);
  }
  return velocity_factor_array;
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

std::vector<TrajectoryPoint> PlannerInterface::generateStopTrajectory(
  const PlannerData & planner_data, const std::vector<StopObstacle> & stop_obstacles)
{
  stop_watch_.tic(__func__);

  stop_planning_debug_info_.reset();
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data.ego_vel);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::EGO_VELOCITY, planner_data.ego_acc);

  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  if (stop_obstacles.empty()) {
    stop_reasons_pub_->publish(makeEmptyStopReasonArray(planner_data.current_time));
    velocity_factors_pub_->publish(makeVelocityFactorArray(planner_data.current_time));

    // delete marker
    const auto markers =
      motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

    return planner_data.traj_points;
  }

  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::PIDBasedPlanner"), enable_debug_info_,
    "stop planning");

  // Get Closest Stop Obstacle
  const auto closest_stop_obstacle =
    obstacle_cruise_utils::getClosestStopObstacle(planner_data.traj_points, stop_obstacles);
  if (!closest_stop_obstacle) {
    // delete marker
    const auto markers =
      motion_utils::createDeletedStopVirtualWallMarker(planner_data.current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);

    return planner_data.traj_points;
  }

  // Get Closest Obstacle Stop Distance
  const double closest_obstacle_dist = motion_utils::calcSignedArcLength(
    planner_data.traj_points, 0, closest_stop_obstacle->collision_point);

  const auto ego_segment_idx =
    ego_nearest_param_.findSegmentIndex(planner_data.traj_points, planner_data.ego_pose);
  const auto negative_dist_to_ego = motion_utils::calcSignedArcLength(
    planner_data.traj_points, planner_data.ego_pose.position, ego_segment_idx, 0);
  const double dist_to_ego = -negative_dist_to_ego;

  // If behavior stop point is ahead of the closest_obstacle_stop point within a certain margin
  // we set closest_obstacle_stop_distance to closest_behavior_stop_distance
  const double margin_from_obstacle = [&]() {
    const size_t nearest_segment_idx =
      findEgoSegmentIndex(planner_data.traj_points, planner_data.ego_pose);
    const auto closest_behavior_stop_idx =
      motion_utils::searchZeroVelocityIndex(planner_data.traj_points, nearest_segment_idx + 1);

    if (!closest_behavior_stop_idx) {
      return longitudinal_info_.safe_distance_margin;
    }

    const double closest_behavior_stop_dist_from_ego = motion_utils::calcSignedArcLength(
      planner_data.traj_points, planner_data.ego_pose.position, nearest_segment_idx,
      *closest_behavior_stop_idx);

    if (*closest_behavior_stop_idx == planner_data.traj_points.size() - 1) {
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
                                      planner_data.ego_vel, longitudinal_info_.limit_max_accel,
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
  auto output_traj_points = planner_data.traj_points;
  const double zero_vel_dist =
    std::max(0.0, closest_obstacle_dist - abs_ego_offset - feasible_margin_from_obstacle);
  const auto zero_vel_idx = motion_utils::insertStopPoint(0, zero_vel_dist, output_traj_points);
  if (zero_vel_idx) {
    // virtual wall marker for stop obstacle
    const auto markers = motion_utils::createStopVirtualWallMarker(
      output_traj_points.at(*zero_vel_idx).pose, "obstacle stop", planner_data.current_time, 0,
      abs_ego_offset);
    tier4_autoware_utils::appendMarkerArray(markers, &debug_data_ptr_->stop_wall_marker);
    debug_data_ptr_->obstacles_to_stop.push_back(*closest_stop_obstacle);

    // Publish Stop Reason
    const auto stop_pose = output_traj_points.at(*zero_vel_idx).pose;
    const auto stop_reasons_msg =
      makeStopReasonArray(planner_data.current_time, stop_pose, *closest_stop_obstacle);
    stop_reasons_pub_->publish(stop_reasons_msg);
    velocity_factors_pub_->publish(makeVelocityFactorArray(planner_data.current_time, stop_pose));

    // Publish if ego vehicle collides with the obstacle with a limit acceleration
    const auto stop_speed_exceeded_msg =
      createStopSpeedExceededMsg(planner_data.current_time, will_collide_with_obstacle);
    stop_speed_exceeded_pub_->publish(stop_speed_exceeded_msg);
  }

  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_DISTANCE,
    closest_obstacle_dist - abs_ego_offset);  // TODO(murooka)
  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_CURRENT_OBSTACLE_VELOCITY, closest_stop_obstacle->velocity);

  stop_planning_debug_info_.set(
    StopPlanningDebugInfo::TYPE::STOP_TARGET_OBSTACLE_DISTANCE, feasible_margin_from_obstacle);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_VELOCITY, 0.0);
  stop_planning_debug_info_.set(StopPlanningDebugInfo::TYPE::STOP_TARGET_ACCELERATION, 0.0);

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::SlowDownPlanner"), enable_calculation_time_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return output_traj_points;
}

double PlannerInterface::calcDistanceToCollisionPoint(
  const PlannerData & planner_data, const geometry_msgs::msg::Point & collision_point)
{
  const double offset = planner_data.is_driving_forward
                          ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                          : std::abs(vehicle_info_.min_longitudinal_offset_m);

  const size_t ego_segment_idx =
    ego_nearest_param_.findSegmentIndex(planner_data.traj_points, planner_data.ego_pose);

  const size_t collision_segment_idx =
    motion_utils::findNearestSegmentIndex(planner_data.traj_points, collision_point);

  const auto dist_to_collision_point = motion_utils::calcSignedArcLength(
    planner_data.traj_points, planner_data.ego_pose.position, ego_segment_idx, collision_point,
    collision_segment_idx);

  return dist_to_collision_point - offset;
}

std::vector<TrajectoryPoint> PlannerInterface::generateSlowDownTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & cruise_traj_points,
  const std::vector<SlowDownObstacle> & obstacles,
  [[maybe_unused]] std::optional<VelocityLimit> & vel_limit)
{
  stop_watch_.tic(__func__);
  auto slow_down_traj_points = cruise_traj_points;
  slow_down_debug_multi_array_ = Float32MultiArrayStamped();

  const double dist_to_ego = [&]() {
    const size_t ego_seg_idx =
      ego_nearest_param_.findSegmentIndex(slow_down_traj_points, planner_data.ego_pose);
    return motion_utils::calcSignedArcLength(
      slow_down_traj_points, 0, planner_data.ego_pose.position, ego_seg_idx);
  }();
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  // define function to insert slow down velocity to trajectory
  const auto insert_slow_down_to_trajectory =
    [&](const double lon_dist, const double slow_down_vel) -> std::optional<size_t> {
    const auto inserted_idx = motion_utils::insertTargetPoint(0, lon_dist, slow_down_traj_points);
    if (inserted_idx) {
      slow_down_traj_points.at(inserted_idx.get()).longitudinal_velocity_mps = slow_down_vel;
      return inserted_idx.get();
    }
    return std::nullopt;
  };
  const auto add_slow_down_marker =
    [&](const size_t obstacle_idx, const auto slow_down_traj_idx, const bool is_start) {
      if (!slow_down_traj_idx) return;

      const int id = obstacle_idx * 2 + (is_start ? 0 : 1);
      const auto text = is_start ? "obstacle slow down start" : "obstacle slow down end";
      const auto pose = slow_down_traj_points.at(*slow_down_traj_idx).pose;
      const auto markers = motion_utils::createSlowDownVirtualWallMarker(
        pose, text, planner_data.current_time, id, abs_ego_offset);
      tier4_autoware_utils::appendMarkerArray(markers, &debug_data_ptr_->slow_down_wall_marker);
    };

  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obstacle = obstacles.at(i);

    // calculate slow down velocity
    const double slow_down_vel = calculateSlowDownVelocity(obstacle);

    // calculate slow down start distance, and insert slow down velocity
    const double dist_to_slow_down_start = calculateDistanceToSlowDownWithAccConstraint(
      planner_data, slow_down_traj_points, obstacle, dist_to_ego, slow_down_vel);
    const auto slow_down_start_idx =
      insert_slow_down_to_trajectory(dist_to_slow_down_start, slow_down_vel);

    // calculate slow down end distance, and insert slow down velocity
    const double dist_to_slow_down_end =
      motion_utils::calcSignedArcLength(slow_down_traj_points, 0, obstacle.back_collision_point) -
      abs_ego_offset;
    // NOTE: slow_down_start_idx will not be wrong since inserted back point is after inserted
    // front point.
    const auto slow_down_end_idx =
      dist_to_slow_down_start < dist_to_slow_down_end
        ? insert_slow_down_to_trajectory(dist_to_slow_down_end, slow_down_vel)
        : std::nullopt;
    if (!slow_down_end_idx) {
      continue;
    }

    // insert slow down velocity between slow start and end
    for (size_t i = (slow_down_start_idx ? *slow_down_start_idx : 0); i <= *slow_down_end_idx;
         ++i) {
      slow_down_traj_points.at(i).longitudinal_velocity_mps = slow_down_vel;
    }

    // add debug data and virtual wall
    slow_down_debug_multi_array_.data.push_back(obstacle.precise_lat_dist);
    slow_down_debug_multi_array_.data.push_back(slow_down_vel);
    if (slow_down_start_idx) {
      slow_down_debug_multi_array_.data.push_back(
        slow_down_start_idx ? *slow_down_start_idx : -1.0);
      add_slow_down_marker(i, slow_down_start_idx, true);
    }
    if (slow_down_end_idx) {
      slow_down_debug_multi_array_.data.push_back(slow_down_end_idx ? *slow_down_end_idx : -1.0);
      add_slow_down_marker(i, slow_down_end_idx, false);
    }

    debug_data_ptr_->obstacles_to_slow_down.push_back(obstacle);
  }

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::SlowDownPlanner"), enable_calculation_time_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return slow_down_traj_points;
}

double PlannerInterface::calculateSlowDownVelocity(const SlowDownObstacle & obstacle) const
{
  const auto & p = slow_down_param_;

  const double ratio = std::clamp(
    (std::abs(obstacle.precise_lat_dist) - p.min_lat_margin) /
      (p.max_lat_margin - p.min_lat_margin),
    0.0, 1.0);
  const double slow_down_vel =
    p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);

  return slow_down_vel;
}

double PlannerInterface::calculateDistanceToSlowDownWithAccConstraint(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points,
  const SlowDownObstacle & obstacle, const double dist_to_ego, const double slow_down_vel) const
{
  const auto & p = slow_down_param_;
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  // calculate distance between start of path and slow down point
  const double dist_to_slow_down =
    motion_utils::calcSignedArcLength(traj_points, 0, obstacle.front_collision_point) -
    abs_ego_offset - slow_down_vel * p.time_margin_on_target_velocity;
  if (dist_to_slow_down < dist_to_ego) {
    return dist_to_slow_down;
  }
  if (std::abs(planner_data.ego_vel) < std::abs(slow_down_vel)) {
    return dist_to_slow_down;
  }

  // calculate deceleration
  const double dist_to_slow_down_from_ego = dist_to_slow_down - dist_to_ego;
  const double limited_slow_down_acc = [&]() {
    if (dist_to_slow_down_from_ego <= 0.0) {
      return p.max_deceleration;
    }
    const double slow_down_acc = -(std::pow(planner_data.ego_vel, 2) - std::pow(slow_down_vel, 2)) /
                                 2.0 / dist_to_slow_down_from_ego;
    return std::max(slow_down_acc, p.max_deceleration);
  }();

  // calculate lon_dist backwards from limited decleration
  const double limited_dist_to_slow_down_from_ego =
    -(std::pow(planner_data.ego_vel, 2) - std::pow(slow_down_vel, 2)) / 2.0 / limited_slow_down_acc;

  return limited_dist_to_slow_down_from_ego + dist_to_ego;
}
