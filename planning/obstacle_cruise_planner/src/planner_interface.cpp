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

#include "motion_utils/distance/distance.hpp"
#include "signal_processing/lowpass_filter_1d.hpp"

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

template <typename T>
std::optional<T> getObjectFromUuid(const std::vector<T> & objects, const std::string & target_uuid)
{
  const auto itr = std::find_if(objects.begin(), objects.end(), [&](const auto & object) {
    return object.uuid == target_uuid;
  });

  if (itr == objects.end()) {
    return std::nullopt;
  }
  return *itr;
}

// TODO(murooka) following two functions are copied from behavior_velocity_planner.
// These should be refactored.
double findReachTime(
  const double jerk, const double accel, const double velocity, const double distance,
  const double t_min, const double t_max)
{
  const double j = jerk;
  const double a = accel;
  const double v = velocity;
  const double d = distance;
  const double min = t_min;
  const double max = t_max;
  auto f = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  if (f(min, j, a, v, d) > 0 || f(max, j, a, v, d) < 0) {
    std::logic_error("[obstacle_cruise_planner](findReachTime): search range is invalid");
  }
  const double eps = 1e-5;
  const int warn_iter = 100;
  double lower = min;
  double upper = max;
  double t;
  int iter = 0;
  for (int i = 0;; i++) {
    t = 0.5 * (lower + upper);
    const double fx = f(t, j, a, v, d);
    // std::cout<<"fx: "<<fx<<" up: "<<upper<<" lo: "<<lower<<" t: "<<t<<std::endl;
    if (std::abs(fx) < eps) {
      break;
    } else if (fx > 0.0) {
      upper = t;
    } else {
      lower = t;
    }
    iter++;
    if (iter > warn_iter)
      std::cerr << "[obstacle_cruise_planner](findReachTime): current iter is over warning"
                << std::endl;
  }
  return t;
}

double calcDecelerationVelocityFromDistanceToTarget(
  const double max_slowdown_jerk, const double max_slowdown_accel, const double current_accel,
  const double current_velocity, const double distance_to_target)
{
  if (max_slowdown_jerk > 0 || max_slowdown_accel > 0) {
    std::logic_error("max_slowdown_jerk and max_slowdown_accel should be negative");
  }
  // case0: distance to target is behind ego
  if (distance_to_target <= 0) return current_velocity;
  auto ft = [](const double t, const double j, const double a, const double v, const double d) {
    return j * t * t * t / 6.0 + a * t * t / 2.0 + v * t - d;
  };
  auto vt = [](const double t, const double j, const double a, const double v) {
    return j * t * t / 2.0 + a * t + v;
  };
  const double j_max = max_slowdown_jerk;
  const double a0 = current_accel;
  const double a_max = max_slowdown_accel;
  const double v0 = current_velocity;
  const double l = distance_to_target;
  const double t_const_jerk = (a_max - a0) / j_max;
  const double d_const_jerk_stop = ft(t_const_jerk, j_max, a0, v0, 0.0);
  const double d_const_acc_stop = l - d_const_jerk_stop;

  if (d_const_acc_stop < 0) {
    // case0: distance to target is within constant jerk deceleration
    // use binary search instead of solving cubic equation
    const double t_jerk = findReachTime(j_max, a0, v0, l, 0, t_const_jerk);
    const double velocity = vt(t_jerk, j_max, a0, v0);
    return velocity;
  } else {
    const double v1 = vt(t_const_jerk, j_max, a0, v0);
    const double discriminant_of_stop = 2.0 * a_max * d_const_acc_stop + v1 * v1;
    // case3: distance to target is farther than distance to stop
    if (discriminant_of_stop <= 0) {
      return 0.0;
    }
    // case2: distance to target is within constant accel deceleration
    // solve d = 0.5*a^2+v*t by t
    const double t_acc = (-v1 + std::sqrt(discriminant_of_stop)) / a_max;
    return vt(t_acc, 0.0, a_max, v1);
  }
  return current_velocity;
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
  const auto insert_point_in_trajectory = [&](const double lon_dist) -> std::optional<size_t> {
    const auto inserted_idx = motion_utils::insertTargetPoint(0, lon_dist, slow_down_traj_points);
    if (inserted_idx) {
      if (inserted_idx.get() + 1 <= slow_down_traj_points.size() - 1) {
        // zero-order hold for velocity interpolation
        slow_down_traj_points.at(inserted_idx.get()).longitudinal_velocity_mps =
          slow_down_traj_points.at(inserted_idx.get() + 1).longitudinal_velocity_mps;
      }
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

  std::vector<SlowDownOutput> new_prev_slow_down_output;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto & obstacle = obstacles.at(i);
    const auto prev_output = getObjectFromUuid(prev_slow_down_output_, obstacle.uuid);

    // calculate slow down start distance, and insert slow down velocity
    const auto [dist_to_slow_down_start, dist_to_slow_down_end, feasible_slow_down_vel] =
      calculateDistanceToSlowDownWithConstraints(
        planner_data, slow_down_traj_points, obstacle, prev_output, dist_to_ego);

    // calculate slow down end distance, and insert slow down velocity
    // NOTE: slow_down_start_idx will not be wrong since inserted back point is after inserted
    // front point.
    const auto slow_down_start_idx = insert_point_in_trajectory(dist_to_slow_down_start);
    const auto slow_down_end_idx = dist_to_slow_down_start < dist_to_slow_down_end
                                     ? insert_point_in_trajectory(dist_to_slow_down_end)
                                     : std::nullopt;
    if (!slow_down_end_idx) {
      continue;
    }

    // calculate slow down velocity
    const double stable_slow_down_vel = [&]() {
      if (prev_output) {
        return signal_processing::lowpassFilter(
          feasible_slow_down_vel, prev_output->target_vel, slow_down_param_.lpf_gain_slow_down_vel);
      }
      return feasible_slow_down_vel;
    }();

    // insert slow down velocity between slow start and end
    for (size_t i = (slow_down_start_idx ? *slow_down_start_idx : 0); i <= *slow_down_end_idx;
         ++i) {
      auto & traj_point = slow_down_traj_points.at(i);
      traj_point.longitudinal_velocity_mps =
        std::min(traj_point.longitudinal_velocity_mps, static_cast<float>(stable_slow_down_vel));
    }

    // add debug data and virtual wall
    slow_down_debug_multi_array_.data.push_back(obstacle.precise_lat_dist);
    slow_down_debug_multi_array_.data.push_back(stable_slow_down_vel);
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

    // update prev_slow_down_output_
    new_prev_slow_down_output.push_back(SlowDownOutput{
      obstacle.uuid, slow_down_traj_points, slow_down_start_idx, slow_down_end_idx,
      stable_slow_down_vel, obstacle.precise_lat_dist});
  }

  // update prev_slow_down_output_
  prev_slow_down_output_ = new_prev_slow_down_output;

  const double calculation_time = stop_watch_.toc(__func__);
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("ObstacleCruisePlanner::SlowDownPlanner"), enable_calculation_time_info_,
    "  %s := %f [ms]", __func__, calculation_time);

  return slow_down_traj_points;
}

double PlannerInterface::calculateSlowDownVelocity(
  const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output) const
{
  const auto & p = slow_down_param_;

  const double stable_precise_lat_dist = [&]() {
    if (prev_output) {
      return signal_processing::lowpassFilter(
        obstacle.precise_lat_dist, prev_output->precise_lat_dist,
        slow_down_param_.lpf_gain_lat_dist);
    }
    return obstacle.precise_lat_dist;
  }();

  const double ratio = std::clamp(
    (std::abs(stable_precise_lat_dist) - p.min_lat_margin) / (p.max_lat_margin - p.min_lat_margin),
    0.0, 1.0);
  const double slow_down_vel =
    p.min_ego_velocity + ratio * (p.max_ego_velocity - p.min_ego_velocity);

  return slow_down_vel;
}

std::tuple<double, double, double> PlannerInterface::calculateDistanceToSlowDownWithConstraints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & traj_points,
  const SlowDownObstacle & obstacle, const std::optional<SlowDownOutput> & prev_output,
  const double dist_to_ego) const
{
  const auto & p = slow_down_param_;
  const double abs_ego_offset = planner_data.is_driving_forward
                                  ? std::abs(vehicle_info_.max_longitudinal_offset_m)
                                  : std::abs(vehicle_info_.min_longitudinal_offset_m);

  // calculate slow down velocity
  const double slow_down_vel = calculateSlowDownVelocity(obstacle, prev_output);

  // calculate distance to collision points
  const double dist_to_front_collision =
    motion_utils::calcSignedArcLength(traj_points, 0, obstacle.front_collision_point);
  const double dist_to_back_collision =
    motion_utils::calcSignedArcLength(traj_points, 0, obstacle.back_collision_point);

  // calculate distance during deceleration, slow down preparation, and slow down
  const double slow_down_prepare_dist = slow_down_vel * p.time_margin_on_target_velocity;
  const double deceleration_dist =
    dist_to_front_collision - abs_ego_offset - dist_to_ego - slow_down_prepare_dist;
  const double slow_down_dist =
    dist_to_back_collision - dist_to_front_collision + slow_down_prepare_dist;

  // calculate distance to start/end slow down
  const double dist_to_slow_down_start = dist_to_ego + deceleration_dist;
  const double dist_to_slow_down_end = dist_to_ego + deceleration_dist + slow_down_dist;

  // apply low-pass filter to distance to start/end slow down
  const auto apply_lowpass_filter = [&](const double dist_to_slow_down, const auto prev_point) {
    if (prev_output && prev_point) {
      const size_t seg_idx =
        motion_utils::findNearestSegmentIndex(traj_points, prev_point->position);
      const double prev_dist_to_slow_down =
        motion_utils::calcSignedArcLength(traj_points, 0, prev_point->position, seg_idx);
      return signal_processing::lowpassFilter(
        dist_to_slow_down, prev_dist_to_slow_down, slow_down_param_.lpf_gain_dist_to_slow_down);
    }
    return dist_to_slow_down;
  };
  const double filtered_dist_to_slow_down_start =
    apply_lowpass_filter(dist_to_slow_down_start, prev_output->start_point);
  const double filtered_dist_to_slow_down_end =
    apply_lowpass_filter(dist_to_slow_down_end, prev_output->end_point);

  // calculate velocity considering constraints
  const double feasible_slow_down_vel = [&]() {
    if (deceleration_dist < 0) {
      if (prev_output) {
        return prev_output->target_vel;
      }
      return slow_down_vel;
    }
    if (planner_data.ego_vel < slow_down_vel) {
      return slow_down_vel;
    }
    if (planner_data.ego_acc < longitudinal_info_.min_accel) {
      const double squared_vel =
        std::pow(planner_data.ego_vel, 2) + 2 * deceleration_dist * longitudinal_info_.min_accel;
      if (squared_vel < 0) {
        return slow_down_vel;
      }
      return std::max(std::sqrt(squared_vel), slow_down_vel);
    }
    // TODO(murooka) Calculate more precisely. Final acceleration should be zero.
    const double min_feasible_slow_down_vel = calcDecelerationVelocityFromDistanceToTarget(
      longitudinal_info_.min_jerk, longitudinal_info_.min_accel, planner_data.ego_acc,
      planner_data.ego_vel, deceleration_dist);
    return std::max(min_feasible_slow_down_vel, slow_down_vel);
  }();

  return std::make_tuple(
    filtered_dist_to_slow_down_start, filtered_dist_to_slow_down_end, feasible_slow_down_vel);
}
