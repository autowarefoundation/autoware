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

#include "obstacle_cruise_planner/optimization_based_planner/optimization_based_planner.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "interpolation/spline_interpolation.hpp"
#include "interpolation/zero_order_hold.hpp"
#include "motion_utils/marker/marker_helper.hpp"
#include "motion_utils/resample/resample.hpp"
#include "motion_utils/trajectory/interpolation.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/ros/marker_helper.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2/utils.h>

constexpr double ZERO_VEL_THRESHOLD = 0.01;
constexpr double CLOSE_S_DIST_THRESHOLD = 1e-3;

OptimizationBasedPlanner::OptimizationBasedPlanner(
  rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
  const vehicle_info_util::VehicleInfo & vehicle_info, const EgoNearestParam & ego_nearest_param,
  const std::shared_ptr<DebugData> debug_data_ptr)
: PlannerInterface(node, longitudinal_info, vehicle_info, ego_nearest_param, debug_data_ptr)
{
  smoothed_traj_sub_ = node.create_subscription<Trajectory>(
    "/planning/scenario_planning/trajectory", rclcpp::QoS{1},
    [this](const Trajectory::ConstSharedPtr msg) { smoothed_trajectory_ptr_ = msg; });

  // parameter
  dense_resampling_time_interval_ = node.declare_parameter<double>(
    "cruise.optimization_based_planner.dense_resampling_time_interval");
  sparse_resampling_time_interval_ = node.declare_parameter<double>(
    "cruise.optimization_based_planner.sparse_resampling_time_interval");
  dense_time_horizon_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.dense_time_horizon");
  max_time_horizon_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.max_time_horizon");

  t_dangerous_ = node.declare_parameter<double>("cruise.optimization_based_planner.t_dangerous");
  velocity_margin_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.velocity_margin");

  replan_vel_deviation_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.replan_vel_deviation");
  engage_velocity_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.engage_velocity");
  engage_acceleration_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.engage_acceleration");
  engage_exit_ratio_ =
    node.declare_parameter<double>("cruise.optimization_based_planner.engage_exit_ratio");
  stop_dist_to_prohibit_engage_ = node.declare_parameter<double>(
    "cruise.optimization_based_planner.stop_dist_to_prohibit_engage");

  const double max_s_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.max_s_weight");
  const double max_v_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.max_v_weight");
  const double over_s_safety_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.over_s_safety_weight");
  const double over_s_ideal_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.over_s_ideal_weight");
  const double over_v_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.over_v_weight");
  const double over_a_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.over_a_weight");
  const double over_j_weight =
    node.declare_parameter<double>("cruise.optimization_based_planner.over_j_weight");

  // velocity optimizer
  velocity_optimizer_ptr_ = std::make_shared<VelocityOptimizer>(
    max_s_weight, max_v_weight, over_s_safety_weight, over_s_ideal_weight, over_v_weight,
    over_a_weight, over_j_weight);

  // publisher
  optimized_sv_pub_ = node.create_publisher<Trajectory>("~/optimized_sv_trajectory", 1);
  optimized_st_graph_pub_ = node.create_publisher<Trajectory>("~/optimized_st_graph", 1);
  boundary_pub_ = node.create_publisher<Trajectory>("~/boundary", 1);
  debug_wall_marker_pub_ = node.create_publisher<MarkerArray>("~/debug/wall_marker", 1);
}

std::vector<TrajectoryPoint> OptimizationBasedPlanner::generateCruiseTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
  const std::vector<CruiseObstacle> & obstacles,
  [[maybe_unused]] std::optional<VelocityLimit> & vel_limit)
{
  // Create Time Vector defined by resampling time interval
  const std::vector<double> time_vec = createTimeVector();
  if (time_vec.size() < 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Resolution size is not enough");
    prev_output_ = stop_traj_points;
    return stop_traj_points;
  }

  // Get the nearest point on the trajectory
  const size_t closest_idx = findEgoSegmentIndex(stop_traj_points, planner_data.ego_pose);

  // Compute maximum velocity
  double v_max = 0.0;
  for (const auto & point : stop_traj_points) {
    v_max = std::max(v_max, static_cast<double>(point.longitudinal_velocity_mps));
  }

  // Get Current Velocity
  double v0;
  double a0;
  std::tie(v0, a0) = calcInitialMotion(planner_data, stop_traj_points, prev_output_);
  a0 = std::min(longitudinal_info_.max_accel, std::max(a0, longitudinal_info_.min_accel));

  // Check trajectory size
  if (stop_traj_points.size() - closest_idx <= 2) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "The number of points on the trajectory is too small or failed to calculate front offset");
    prev_output_ = stop_traj_points;
    return stop_traj_points;
  }

  // Check if reached goal
  if (checkHasReachedGoal(planner_data, stop_traj_points)) {
    prev_output_ = stop_traj_points;
    return stop_traj_points;
  }

  // Get S Boundaries from the obstacle
  const auto s_boundaries = getSBoundaries(planner_data, stop_traj_points, obstacles, time_vec);
  if (!s_boundaries) {
    prev_output_ = stop_traj_points;
    return stop_traj_points;
  }

  // Optimization
  VelocityOptimizer::OptimizationData data;
  data.time_vec = time_vec;
  data.s0 = 0.0;
  data.a0 = a0;
  data.v_max = v_max;
  data.a_max = longitudinal_info_.max_accel;
  data.a_min = longitudinal_info_.min_accel;
  data.j_max = longitudinal_info_.max_jerk;
  data.j_min = longitudinal_info_.min_jerk;
  data.limit_a_max = longitudinal_info_.limit_max_accel;
  data.limit_a_min = longitudinal_info_.limit_min_accel;
  data.limit_j_max = longitudinal_info_.limit_max_jerk;
  data.limit_j_min = longitudinal_info_.limit_min_jerk;
  data.t_dangerous = t_dangerous_;
  data.idling_time = longitudinal_info_.idling_time;
  data.s_boundary = *s_boundaries;
  data.v0 = v0;
  RCLCPP_DEBUG(rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"), "v0 %f", v0);

  const auto optimized_result = velocity_optimizer_ptr_->optimize(data);

  // Publish Debug trajectories
  const double traj_front_to_vehicle_offset =
    motion_utils::calcSignedArcLength(stop_traj_points, 0, closest_idx);
  publishDebugTrajectory(
    planner_data, traj_front_to_vehicle_offset, time_vec, *s_boundaries, optimized_result);

  // Transformation from t to s
  const auto processed_result =
    processOptimizedResult(data.v0, optimized_result, traj_front_to_vehicle_offset);
  if (!processed_result) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Processed Result is empty");
    prev_output_ = stop_traj_points;
    return stop_traj_points;
  }
  const auto & opt_position = processed_result->s;
  const auto & opt_velocity = processed_result->v;

  // Check Size
  if (opt_position.size() == 1 && opt_velocity.front() < ZERO_VEL_THRESHOLD) {
    auto output = stop_traj_points;
    output.at(closest_idx).longitudinal_velocity_mps = data.v0;
    for (size_t i = closest_idx + 1; i < output.size(); ++i) {
      output.at(i).longitudinal_velocity_mps = 0.0;
    }
    prev_output_ = output;
    return output;
  } else if (opt_position.size() == 1) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Optimized Trajectory is too small");
    prev_output_ = stop_traj_points;
    return stop_traj_points;
  }

  // Get Zero Velocity Position
  double closest_stop_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < opt_velocity.size(); ++i) {
    if (opt_velocity.at(i) < ZERO_VEL_THRESHOLD) {
      closest_stop_dist = std::min(closest_stop_dist, opt_position.at(i));
      break;
    }
  }
  const auto traj_stop_dist =
    motion_utils::calcDistanceToForwardStopPoint(stop_traj_points, closest_idx);
  if (traj_stop_dist) {
    closest_stop_dist = std::min(*traj_stop_dist + traj_front_to_vehicle_offset, closest_stop_dist);
  }

  // Resample Optimum Velocity
  size_t break_id = stop_traj_points.size();
  std::vector<double> resampled_opt_position;
  for (size_t i = closest_idx; i < stop_traj_points.size(); ++i) {
    const double query_s =
      std::max(motion_utils::calcSignedArcLength(stop_traj_points, 0, i), opt_position.front());
    if (query_s > opt_position.back()) {
      break_id = i;
      break;
    }
    resampled_opt_position.push_back(query_s);
  }
  // resample optimum velocity for original each position
  auto resampled_opt_velocity =
    interpolation::lerp(opt_position, opt_velocity, resampled_opt_position);
  for (size_t i = break_id; i < stop_traj_points.size(); ++i) {
    resampled_opt_velocity.push_back(stop_traj_points.at(i).longitudinal_velocity_mps);
  }

  // Create Output Data
  std::vector<TrajectoryPoint> output = stop_traj_points;
  for (size_t i = 0; i < closest_idx; ++i) {
    output.at(i).longitudinal_velocity_mps = data.v0;
  }
  for (size_t i = closest_idx; i < output.size(); ++i) {
    output.at(i).longitudinal_velocity_mps =
      resampled_opt_velocity.at(i - closest_idx) + velocity_margin_;
  }
  output.back().longitudinal_velocity_mps = 0.0;  // terminal velocity is zero

  // Insert Closest Stop Point
  motion_utils::insertStopPoint(0, closest_stop_dist, output);

  prev_output_ = output;
  return output;
}

std::vector<double> OptimizationBasedPlanner::createTimeVector()
{
  std::vector<double> time_vec;
  for (double t = 0.0; t < dense_time_horizon_; t += dense_resampling_time_interval_) {
    time_vec.push_back(t);
  }
  if (dense_time_horizon_ - time_vec.back() < 1e-3) {
    time_vec.back() = dense_time_horizon_;
  } else {
    time_vec.push_back(dense_time_horizon_);
  }

  for (double t = dense_time_horizon_ + sparse_resampling_time_interval_; t < max_time_horizon_;
       t += sparse_resampling_time_interval_) {
    time_vec.push_back(t);
  }
  if (max_time_horizon_ - time_vec.back() < 1e-3) {
    time_vec.back() = max_time_horizon_;
  } else {
    time_vec.push_back(max_time_horizon_);
  }

  return time_vec;
}

// v0, a0
std::tuple<double, double> OptimizationBasedPlanner::calcInitialMotion(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
  const std::vector<TrajectoryPoint> & prev_traj_points)
{
  const auto & ego_vel = planner_data.ego_vel;
  const auto & ego_pose = planner_data.ego_pose;
  const auto & input_traj_points = stop_traj_points;
  const double vehicle_speed{std::abs(ego_vel)};
  const auto current_closest_point =
    ego_nearest_param_.calcInterpolatedPoint(input_traj_points, planner_data.ego_pose);
  const double target_vel{std::abs(current_closest_point.longitudinal_velocity_mps)};

  double initial_vel{};
  double initial_acc{};

  // first time
  if (prev_traj_points.empty()) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;
    return std::make_tuple(initial_vel, initial_acc);
  }

  TrajectoryPoint prev_output_closest_point;
  if (smoothed_trajectory_ptr_) {
    prev_output_closest_point = motion_utils::calcInterpolatedPoint(
      *smoothed_trajectory_ptr_, ego_pose, ego_nearest_param_.dist_threshold,
      ego_nearest_param_.yaw_threshold);
  } else {
    prev_output_closest_point =
      ego_nearest_param_.calcInterpolatedPoint(prev_traj_points, ego_pose);
  }

  // when velocity tracking deviation is large
  const double desired_vel{prev_output_closest_point.longitudinal_velocity_mps};
  const double vel_error{vehicle_speed - std::abs(desired_vel)};
  if (std::abs(vel_error) > replan_vel_deviation_) {
    initial_vel = vehicle_speed;  // use current vehicle speed
    initial_acc = prev_output_closest_point.acceleration_mps2;
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "calcInitialMotion : Large deviation error for speed control. Use current speed for "
      "initial value, desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
      desired_vel, vehicle_speed, vel_error, replan_vel_deviation_);
    return std::make_tuple(initial_vel, initial_acc);
  }

  // if current vehicle velocity is low && base_desired speed is high,
  // use engage_velocity for engage vehicle
  const double engage_vel_thr = engage_velocity_ * engage_exit_ratio_;
  if (vehicle_speed < engage_vel_thr) {
    if (target_vel >= engage_velocity_) {
      const auto stop_dist = motion_utils::calcDistanceToForwardStopPoint(
        input_traj_points, ego_pose, ego_nearest_param_.dist_threshold,
        ego_nearest_param_.yaw_threshold);
      if ((stop_dist && *stop_dist > stop_dist_to_prohibit_engage_) || !stop_dist) {
        initial_vel = engage_velocity_;
        initial_acc = engage_acceleration_;
        RCLCPP_DEBUG(
          rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
          "calcInitialMotion : vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, engage_velocity_, engage_vel_thr, stop_dist.value());
        return std::make_tuple(initial_vel, initial_acc);
      } else if (stop_dist) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
          "calcInitialMotion : stop point is close (%.3f[m]). no engage.", stop_dist.value());
      }
    } else if (target_vel > 0.0) {
      auto clock{rclcpp::Clock{RCL_ROS_TIME}};
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"), clock, 3000,
        "calcInitialMotion : target velocity(%.3f[m/s]) is lower than engage velocity(%.3f[m/s]). ",
        target_vel, engage_velocity_);
    }
  }

  // normal update: use closest in prev_output
  initial_vel = prev_output_closest_point.longitudinal_velocity_mps;
  initial_acc = prev_output_closest_point.acceleration_mps2;
  return std::make_tuple(initial_vel, initial_acc);
}

bool OptimizationBasedPlanner::checkHasReachedGoal(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points)
{
  // If goal is close and current velocity is low, we don't optimize trajectory
  const auto closest_stop_dist = motion_utils::calcDistanceToForwardStopPoint(
    stop_traj_points, planner_data.ego_pose, ego_nearest_param_.dist_threshold,
    ego_nearest_param_.yaw_threshold);
  if (closest_stop_dist && *closest_stop_dist < 0.5 && planner_data.ego_vel < 0.6) {
    return true;
  }

  return false;
}

std::optional<SBoundaries> OptimizationBasedPlanner::getSBoundaries(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
  const std::vector<CruiseObstacle> & obstacles, const std::vector<double> & time_vec)
{
  if (stop_traj_points.empty()) {
    return std::nullopt;
  }

  const auto traj_length =
    calcTrajectoryLengthFromCurrentPose(stop_traj_points, planner_data.ego_pose);
  if (!traj_length) {
    return {};
  }

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = *traj_length;
  }

  double min_slow_down_point_length = std::numeric_limits<double>::max();
  std::optional<size_t> min_slow_down_idx = {};
  for (size_t o_idx = 0; o_idx < obstacles.size(); ++o_idx) {
    const auto & obj = obstacles.at(o_idx);
    if (obj.collision_points.empty()) {
      continue;
    }

    // Step1 Get S boundary from the obstacle
    const auto obj_s_boundaries =
      getSBoundaries(planner_data, stop_traj_points, obj, time_vec, *traj_length);
    if (!obj_s_boundaries) {
      continue;
    }

    // Step2 update s boundaries
    for (size_t i = 0; i < obj_s_boundaries->size(); ++i) {
      if (obj_s_boundaries->at(i).max_s < s_boundaries.at(i).max_s) {
        s_boundaries.at(i) = obj_s_boundaries->at(i);
      }
    }

    // Step3 search nearest obstacle to follow for rviz marker
    const double obj_vel = std::abs(obj.velocity);
    const double rss_dist = calcRSSDistance(planner_data.ego_vel, obj_vel);

    const auto & safe_distance_margin = longitudinal_info_.safe_distance_margin;
    const double ego_obj_length = motion_utils::calcSignedArcLength(
      stop_traj_points, planner_data.ego_pose.position, obj.collision_points.front().point);
    const double slow_down_point_length = ego_obj_length - (rss_dist + safe_distance_margin);

    if (slow_down_point_length < min_slow_down_point_length) {
      min_slow_down_point_length = slow_down_point_length;
      min_slow_down_idx = o_idx;
    }
  }

  // Publish wall marker for slowing down or stopping
  if (min_slow_down_idx) {
    const auto & current_time = planner_data.current_time;

    const auto marker_pose = motion_utils::calcLongitudinalOffsetPose(
      stop_traj_points, planner_data.ego_pose.position, min_slow_down_point_length);

    if (marker_pose) {
      MarkerArray wall_msg;

      const auto markers = motion_utils::createSlowDownVirtualWallMarker(
        marker_pose.value(), "obstacle to follow", current_time, 0);
      tier4_autoware_utils::appendMarkerArray(markers, &wall_msg);

      // publish rviz marker
      debug_wall_marker_pub_->publish(wall_msg);
    }
  }

  return s_boundaries;
}

std::optional<SBoundaries> OptimizationBasedPlanner::getSBoundaries(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
  const CruiseObstacle & object, const std::vector<double> & time_vec, const double traj_length)
{
  if (object.collision_points.empty()) {
    return {};
  }

  const bool onEgoTrajectory =
    checkOnTrajectory(planner_data, stop_traj_points, object.collision_points.front());
  const auto & safe_distance_margin = longitudinal_info_.safe_distance_margin;

  // If the object is on the current ego trajectory,
  // we assume the object travels along ego trajectory
  if (onEgoTrajectory) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "On Trajectory Object");

    return getSBoundariesForOnTrajectoryObject(
      planner_data, time_vec, safe_distance_margin, object, traj_length);
  }

  // Ignore low velocity objects that are not on the trajectory
  return getSBoundariesForOffTrajectoryObject(
    planner_data, time_vec, safe_distance_margin, object, traj_length);
}

std::optional<SBoundaries> OptimizationBasedPlanner::getSBoundariesForOnTrajectoryObject(
  const PlannerData & planner_data, const std::vector<double> & time_vec,
  const double safety_distance, const CruiseObstacle & object, const double traj_length)
{
  const double & min_object_accel_for_rss = longitudinal_info_.min_object_accel_for_rss;

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = traj_length;
  }

  const double v_obj = std::abs(object.velocity);

  const double dist_to_collision_point =
    calcDistanceToCollisionPoint(planner_data, object.collision_points.front().point);

  double current_s_obj = std::max(dist_to_collision_point - safety_distance, 0.0);
  const double current_v_obj = v_obj;
  const double initial_s_upper_bound =
    current_s_obj + (current_v_obj * current_v_obj) / (2 * std::fabs(min_object_accel_for_rss));
  s_boundaries.front().max_s = std::clamp(initial_s_upper_bound, 0.0, s_boundaries.front().max_s);
  s_boundaries.front().is_object = true;
  for (size_t i = 1; i < time_vec.size(); ++i) {
    const double dt = time_vec.at(i) - time_vec.at(i - 1);
    current_s_obj = current_s_obj + current_v_obj * dt;

    const double s_upper_bound =
      current_s_obj + (current_v_obj * current_v_obj) / (2 * std::fabs(min_object_accel_for_rss));
    s_boundaries.at(i).max_s = std::clamp(s_upper_bound, 0.0, s_boundaries.at(i).max_s);
    s_boundaries.at(i).is_object = true;
  }

  return s_boundaries;
}

std::optional<SBoundaries> OptimizationBasedPlanner::getSBoundariesForOffTrajectoryObject(
  const PlannerData & planner_data, const std::vector<double> & time_vec,
  const double safety_distance, const CruiseObstacle & object, const double traj_length)
{
  const auto & current_time = planner_data.current_time;
  const double & min_object_accel_for_rss = longitudinal_info_.min_object_accel_for_rss;

  const double v_obj = std::abs(object.velocity);

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = traj_length;
  }

  for (const auto & collision_point : object.collision_points) {
    const double object_time = (rclcpp::Time(collision_point.stamp) - current_time).seconds();
    if (object_time < 0) {
      // Ignore Past Positions
      continue;
    }

    const double dist_to_collision_point =
      calcDistanceToCollisionPoint(planner_data, collision_point.point);
    if (!dist_to_collision_point) {
      continue;
    }

    const double current_s_obj = std::max(dist_to_collision_point - safety_distance, 0.0);
    const double s_upper_bound =
      current_s_obj + (v_obj * v_obj) / (2 * std::fabs(min_object_accel_for_rss));

    size_t object_time_segment_idx = 0;
    for (size_t i = 0; i < time_vec.size() - 1; ++i) {
      if (time_vec.at(i) < object_time && object_time < time_vec.at(i + 1)) {
        object_time_segment_idx = i;
        break;
      }
    }

    for (size_t i = 0; i <= object_time_segment_idx + 1; ++i) {
      if (time_vec.at(i) < object_time && s_upper_bound < s_boundaries.at(i).max_s) {
        s_boundaries.at(i).max_s = std::max(0.0, s_upper_bound);
        s_boundaries.at(i).is_object = true;
      }
    }
  }

  return s_boundaries;
}

bool OptimizationBasedPlanner::checkOnTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & stop_traj_points,
  const PointWithStamp & point)
{
  // If the collision point is in the future, we return false
  if ((rclcpp::Time(point.stamp) - planner_data.current_time).seconds() > 0.1) {
    return false;
  }

  const double lateral_offset =
    std::fabs(motion_utils::calcLateralOffset(stop_traj_points, point.point));

  if (lateral_offset < vehicle_info_.max_lateral_offset_m + 0.1) {
    return true;
  }

  return false;
}

std::optional<double> OptimizationBasedPlanner::calcTrajectoryLengthFromCurrentPose(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose)
{
  const size_t ego_segment_idx = ego_nearest_param_.findSegmentIndex(traj_points, ego_pose);

  const double traj_length = motion_utils::calcSignedArcLength(
    traj_points, ego_pose.position, ego_segment_idx, traj_points.size() - 1);

  const auto dist_to_closest_stop_point = motion_utils::calcDistanceToForwardStopPoint(
    traj_points, ego_pose, ego_nearest_param_.dist_threshold, ego_nearest_param_.yaw_threshold);
  if (dist_to_closest_stop_point) {
    return std::min(traj_length, dist_to_closest_stop_point.value());
  }

  return traj_length;
}

geometry_msgs::msg::Pose OptimizationBasedPlanner::transformBaseLink2Center(
  const geometry_msgs::msg::Pose & pose_base_link)
{
  tf2::Transform tf_map2base;
  tf2::fromMsg(pose_base_link, tf_map2base);

  // set vertices at map coordinate
  tf2::Vector3 base2center;
  base2center.setX(std::abs(vehicle_info_.vehicle_length_m / 2.0 - vehicle_info_.rear_overhang_m));
  base2center.setY(0.0);
  base2center.setZ(0.0);
  base2center.setW(1.0);

  // transform vertices from map coordinate to object coordinate
  const auto map2center = tf_map2base * base2center;

  geometry_msgs::msg::Pose center_pose;
  center_pose.position =
    tier4_autoware_utils::createPoint(map2center.x(), map2center.y(), map2center.z());
  center_pose.orientation = pose_base_link.orientation;

  return center_pose;
}

std::optional<VelocityOptimizer::OptimizationResult>
OptimizationBasedPlanner::processOptimizedResult(
  const double v0, const VelocityOptimizer::OptimizationResult & opt_result, const double offset)
{
  if (
    opt_result.t.empty() || opt_result.s.empty() || opt_result.v.empty() || opt_result.a.empty() ||
    opt_result.j.empty()) {
    return std::nullopt;
  }

  size_t break_id = opt_result.s.size();
  VelocityOptimizer::OptimizationResult processed_result;
  processed_result.t.push_back(0.0);
  processed_result.s.push_back(offset);
  processed_result.v.push_back(v0);
  processed_result.a.push_back(opt_result.a.front());
  processed_result.j.push_back(opt_result.j.front());

  for (size_t i = 1; i < opt_result.s.size(); ++i) {
    const double prev_s = processed_result.s.back();
    const double current_s = std::max(opt_result.s.at(i), 0.0) + offset;
    const double current_v = opt_result.v.at(i);
    if (prev_s >= current_s) {
      processed_result.v.back() = 0.0;
      break_id = i;
      break;
    } else if (current_v < ZERO_VEL_THRESHOLD) {
      break_id = i;
      break;
    } else if (std::fabs(current_s - prev_s) < CLOSE_S_DIST_THRESHOLD) {
      processed_result.v.back() = current_v;
      processed_result.s.back() = current_s;
      continue;
    }

    processed_result.t.push_back(opt_result.t.at(i));
    processed_result.s.push_back(current_s);
    processed_result.v.push_back(current_v);
    processed_result.a.push_back(opt_result.a.at(i));
    processed_result.j.push_back(opt_result.j.at(i));
  }

  // Padding Zero Velocity after break id
  for (size_t i = break_id; i < opt_result.s.size(); ++i) {
    const double prev_s = processed_result.s.back();
    const double current_s = std::max(opt_result.s.at(i), 0.0) + offset;
    if (prev_s >= current_s) {
      processed_result.v.back() = 0.0;
      continue;
    } else if (std::fabs(current_s - prev_s) < CLOSE_S_DIST_THRESHOLD) {
      processed_result.v.back() = 0.0;
      processed_result.s.back() = current_s;
      continue;
    }
    processed_result.t.push_back(opt_result.t.at(i));
    processed_result.s.push_back(current_s);
    processed_result.v.push_back(0.0);
    processed_result.a.push_back(0.0);
    processed_result.j.push_back(0.0);
  }

  return processed_result;
}

void OptimizationBasedPlanner::publishDebugTrajectory(
  const PlannerData & planner_data, const double offset, const std::vector<double> & time_vec,
  const SBoundaries & s_boundaries, const VelocityOptimizer::OptimizationResult & opt_result)
{
  const auto & current_time = planner_data.current_time;
  const std::vector<double> time = opt_result.t;
  // Publish optimized result and boundary
  Trajectory boundary_traj;
  boundary_traj.header.stamp = current_time;
  boundary_traj.points.resize(s_boundaries.size());
  double boundary_s_max = 0.0;
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    const double bound_s = s_boundaries.at(i).max_s;
    const double bound_t = time_vec.at(i);
    boundary_traj.points.at(i).pose.position.x = bound_s;
    boundary_traj.points.at(i).pose.position.y = bound_t;
    boundary_s_max = std::max(bound_s, boundary_s_max);
  }
  boundary_pub_->publish(boundary_traj);

  Trajectory optimized_sv_traj;
  optimized_sv_traj.header.stamp = current_time;
  optimized_sv_traj.points.resize(opt_result.s.size());
  for (size_t i = 0; i < opt_result.s.size(); ++i) {
    const double s = opt_result.s.at(i);
    const double v = opt_result.v.at(i);
    optimized_sv_traj.points.at(i).pose.position.x = s + offset;
    optimized_sv_traj.points.at(i).pose.position.y = v;
  }
  optimized_sv_pub_->publish(optimized_sv_traj);

  Trajectory optimized_st_graph;
  optimized_st_graph.header.stamp = current_time;
  optimized_st_graph.points.resize(opt_result.s.size());
  for (size_t i = 0; i < opt_result.s.size(); ++i) {
    const double bound_s = opt_result.s.at(i);
    const double bound_t = opt_result.t.at(i);
    optimized_st_graph.points.at(i).pose.position.x = bound_s;
    optimized_st_graph.points.at(i).pose.position.y = bound_t;
  }
  optimized_st_graph_pub_->publish(optimized_st_graph);
}
