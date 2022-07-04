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
#include "obstacle_cruise_planner/optimization_based_planner/resample.hpp"
#include "obstacle_cruise_planner/utils.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2/utils.h>

constexpr double ZERO_VEL_THRESHOLD = 0.01;
constexpr double CLOSE_S_DIST_THRESHOLD = 1e-3;

namespace
{
inline void convertEulerAngleToMonotonic(std::vector<double> & a)
{
  for (unsigned int i = 1; i < a.size(); ++i) {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + tier4_autoware_utils::normalizeRadian(da);
  }
}

inline tf2::Vector3 getTransVector3(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to)
{
  double dx = to.position.x - from.position.x;
  double dy = to.position.y - from.position.y;
  double dz = to.position.z - from.position.z;
  return tf2::Vector3(dx, dy, dz);
}

}  // namespace

OptimizationBasedPlanner::OptimizationBasedPlanner(
  rclcpp::Node & node, const LongitudinalInfo & longitudinal_info,
  const vehicle_info_util::VehicleInfo & vehicle_info)
: PlannerInterface(node, longitudinal_info, vehicle_info)
{
  // parameter
  resampling_s_interval_ =
    node.declare_parameter<double>("optimization_based_planner.resampling_s_interval");
  max_trajectory_length_ =
    node.declare_parameter<double>("optimization_based_planner.max_trajectory_length");
  dense_resampling_time_interval_ =
    node.declare_parameter<double>("optimization_based_planner.dense_resampling_time_interval");
  sparse_resampling_time_interval_ =
    node.declare_parameter<double>("optimization_based_planner.sparse_resampling_time_interval");
  dense_time_horizon_ =
    node.declare_parameter<double>("optimization_based_planner.dense_time_horizon");
  max_time_horizon_ = node.declare_parameter<double>("optimization_based_planner.max_time_horizon");

  t_dangerous_ = node.declare_parameter<double>("optimization_based_planner.t_dangerous");
  velocity_margin_ = node.declare_parameter<double>("optimization_based_planner.velocity_margin");

  replan_vel_deviation_ =
    node.declare_parameter<double>("optimization_based_planner.replan_vel_deviation");
  engage_velocity_ = node.declare_parameter<double>("optimization_based_planner.engage_velocity");
  engage_acceleration_ =
    node.declare_parameter<double>("optimization_based_planner.engage_acceleration");
  engage_exit_ratio_ =
    node.declare_parameter<double>("optimization_based_planner.engage_exit_ratio");
  stop_dist_to_prohibit_engage_ =
    node.declare_parameter<double>("optimization_based_planner.stop_dist_to_prohibit_engage");

  const double max_s_weight =
    node.declare_parameter<double>("optimization_based_planner.max_s_weight");
  const double max_v_weight =
    node.declare_parameter<double>("optimization_based_planner.max_v_weight");
  const double over_s_safety_weight =
    node.declare_parameter<double>("optimization_based_planner.over_s_safety_weight");
  const double over_s_ideal_weight =
    node.declare_parameter<double>("optimization_based_planner.over_s_ideal_weight");
  const double over_v_weight =
    node.declare_parameter<double>("optimization_based_planner.over_v_weight");
  const double over_a_weight =
    node.declare_parameter<double>("optimization_based_planner.over_a_weight");
  const double over_j_weight =
    node.declare_parameter<double>("optimization_based_planner.over_j_weight");

  // velocity optimizer
  velocity_optimizer_ptr_ = std::make_shared<VelocityOptimizer>(
    max_s_weight, max_v_weight, over_s_safety_weight, over_s_ideal_weight, over_v_weight,
    over_a_weight, over_j_weight);

  // publisher
  optimized_sv_pub_ = node.create_publisher<Trajectory>("~/optimized_sv_trajectory", 1);
  optimized_st_graph_pub_ = node.create_publisher<Trajectory>("~/optimized_st_graph", 1);
  boundary_pub_ = node.create_publisher<Trajectory>("~/boundary", 1);
  debug_calculation_time_ = node.create_publisher<Float32Stamped>("~/calculation_time", 1);
  debug_wall_marker_pub_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/wall_marker", 1);
}

Trajectory OptimizationBasedPlanner::generateCruiseTrajectory(
  const ObstacleCruisePlannerData & planner_data,
  [[maybe_unused]] boost::optional<VelocityLimit> & vel_limit,
  [[maybe_unused]] DebugData & debug_data)
{
  // Create Time Vector defined by resampling time interval
  const std::vector<double> time_vec = createTimeVector();
  if (time_vec.size() < 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Resolution size is not enough");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Get the nearest point on the trajectory
  const auto closest_idx = tier4_autoware_utils::findNearestIndex(
    planner_data.traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (!closest_idx) {  // Check validity of the closest index
    RCLCPP_ERROR(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Closest Index is Invalid");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Transform original trajectory to TrajectoryData
  const auto base_traj_data = getTrajectoryData(planner_data.traj, planner_data.current_pose);
  if (base_traj_data.traj.points.size() < 2) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "The number of points on the trajectory data is too small");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Compute maximum velocity
  double v_max = 0.0;
  for (const auto & point : planner_data.traj.points) {
    v_max = std::max(v_max, static_cast<double>(point.longitudinal_velocity_mps));
  }

  // Get Current Velocity
  double v0;
  double a0;
  std::tie(v0, a0) = calcInitialMotion(planner_data, *closest_idx, prev_output_);
  a0 = std::min(longitudinal_info_.max_accel, std::max(a0, longitudinal_info_.min_accel));

  // Check trajectory size
  if (planner_data.traj.points.size() - *closest_idx <= 2) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "The number of points on the trajectory is too small");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Check if reached goal
  if (checkHasReachedGoal(planner_data)) {
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Resample base trajectory data by time
  const auto resampled_traj_data =
    resampleTrajectoryData(base_traj_data, resampling_s_interval_, max_trajectory_length_);
  if (resampled_traj_data.traj.points.size() < 2) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "The number of points on the resampled trajectory data is too small");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Get S Boundaries from the obstacle
  const auto s_boundaries = getSBoundaries(planner_data, resampled_traj_data, time_vec);
  if (!s_boundaries) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "No Dangerous Objects around the ego vehicle");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Optimization
  VelocityOptimizer::OptimizationData data;
  data.time_vec = time_vec;
  data.s0 = resampled_traj_data.s.front();
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
  data.v_margin = velocity_margin_;
  data.s_boundary = *s_boundaries;
  data.v0 = v0;
  RCLCPP_DEBUG(rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"), "v0 %f", v0);

  stop_watch_.tic();

  const auto optimized_result = velocity_optimizer_ptr_->optimize(data);

  Float32Stamped calculation_time_data{};
  calculation_time_data.stamp = planner_data.current_time;
  calculation_time_data.data = stop_watch_.toc();
  debug_calculation_time_->publish(calculation_time_data);
  RCLCPP_DEBUG(
    rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
    "Optimization Time; %f[ms]", calculation_time_data.data);

  // Publish Debug trajectories
  publishDebugTrajectory(
    planner_data.current_time, planner_data.traj, *closest_idx, time_vec, *s_boundaries,
    optimized_result);

  // Transformation from t to s
  const auto processed_result = processOptimizedResult(data.v0, optimized_result);
  if (!processed_result) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Processed Result is empty");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }
  const auto & opt_position = processed_result->s;
  const auto & opt_velocity = processed_result->v;

  // Check Size
  if (opt_position.size() == 1 && opt_velocity.front() < ZERO_VEL_THRESHOLD) {
    auto output = planner_data.traj;
    output.points.at(*closest_idx).longitudinal_velocity_mps = data.v0;
    for (size_t i = *closest_idx + 1; i < output.points.size(); ++i) {
      output.points.at(i).longitudinal_velocity_mps = 0.0;
    }
    prev_output_ = output;
    return output;
  } else if (opt_position.size() == 1) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Optimized Trajectory is too small");
    prev_output_ = planner_data.traj;
    return planner_data.traj;
  }

  // Get Zero Velocity Position
  double closest_stop_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < opt_velocity.size(); ++i) {
    if (opt_velocity.at(i) < ZERO_VEL_THRESHOLD) {
      const double zero_vel_s = opt_position.at(i);
      closest_stop_dist = std::min(closest_stop_dist, zero_vel_s);
      break;
    }
  }
  const auto traj_stop_dist = tier4_autoware_utils::calcDistanceToForwardStopPoint(
    planner_data.traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (traj_stop_dist) {
    closest_stop_dist = std::min(*traj_stop_dist, closest_stop_dist);
  }

  size_t break_id = base_traj_data.s.size();
  bool has_insert_stop_point = false;
  std::vector<double> resampled_opt_position = {base_traj_data.s.front()};
  for (size_t i = 1; i < base_traj_data.s.size(); ++i) {
    const double query_s = base_traj_data.s.at(i);
    if (
      !has_insert_stop_point && query_s > closest_stop_dist &&
      closest_stop_dist < opt_position.back()) {
      const double prev_query_s = resampled_opt_position.back();
      if (closest_stop_dist - prev_query_s > CLOSE_S_DIST_THRESHOLD) {
        resampled_opt_position.push_back(closest_stop_dist);
      } else {
        resampled_opt_position.back() = closest_stop_dist;
      }
      has_insert_stop_point = true;
    }

    if (query_s > opt_position.back()) {
      break_id = i;
      break;
    }

    const double prev_query_s = resampled_opt_position.back();
    if (query_s - prev_query_s > 1e-3) {
      resampled_opt_position.push_back(query_s);
    }
  }
  const auto resampled_opt_velocity =
    interpolation::lerp(opt_position, opt_velocity, resampled_opt_position);

  // Push positions after the last position of the opt position
  for (size_t i = break_id; i < base_traj_data.s.size(); ++i) {
    const double query_s = base_traj_data.s.at(i);
    const double prev_query_s = resampled_opt_position.back();
    if (query_s - prev_query_s > CLOSE_S_DIST_THRESHOLD) {
      resampled_opt_position.push_back(query_s);
    }
  }

  auto resampled_traj = resampling::applyLinearInterpolation(
    base_traj_data.s, base_traj_data.traj, resampled_opt_position);
  for (size_t i = 0; i < resampled_opt_velocity.size(); ++i) {
    resampled_traj.points.at(i).longitudinal_velocity_mps = std::min(
      resampled_opt_velocity.at(i),
      static_cast<double>(resampled_traj.points.at(i).longitudinal_velocity_mps));
  }
  for (size_t i = 0; i < resampled_opt_position.size(); ++i) {
    if (resampled_opt_position.at(i) >= closest_stop_dist) {
      resampled_traj.points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }

  Trajectory output;
  output.header = planner_data.traj.header;
  for (size_t i = 0; i < *closest_idx; ++i) {
    auto point = planner_data.traj.points.at(i);
    point.longitudinal_velocity_mps = data.v0;
    output.points.push_back(point);
  }
  for (const auto & resampled_point : resampled_traj.points) {
    if (output.points.empty()) {
      output.points.push_back(resampled_point);
    } else {
      const auto prev_point = output.points.back();
      const double dist = tier4_autoware_utils::calcDistance2d(
        prev_point.pose.position, resampled_point.pose.position);
      if (dist > 1e-3) {
        output.points.push_back(resampled_point);
      } else {
        output.points.back() = resampled_point;
      }
    }
  }
  output.points.back().longitudinal_velocity_mps = 0.0;  // terminal velocity is zero

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
  const ObstacleCruisePlannerData & planner_data, const size_t input_closest,
  const Trajectory & prev_traj)
{
  const auto & current_vel = planner_data.current_vel;
  const auto & input_traj = planner_data.traj;
  const double vehicle_speed{std::abs(current_vel)};
  const double target_vel{std::abs(input_traj.points.at(input_closest).longitudinal_velocity_mps)};

  double initial_vel{};
  double initial_acc{};

  // first time
  if (prev_traj.points.empty()) {
    initial_vel = vehicle_speed;
    initial_acc = 0.0;
    return std::make_tuple(initial_vel, initial_acc);
  }

  TrajectoryPoint prev_output_closest_point;
  if (smoothed_trajectory_ptr_) {
    prev_output_closest_point =
      calcInterpolatedTrajectoryPoint(*smoothed_trajectory_ptr_, planner_data.current_pose);
  } else {
    prev_output_closest_point =
      calcInterpolatedTrajectoryPoint(prev_traj, planner_data.current_pose);
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
      const auto stop_dist = tier4_autoware_utils::calcDistanceToForwardStopPoint(
        input_traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
        nearest_yaw_deviation_threshold_);
      if ((stop_dist && *stop_dist > stop_dist_to_prohibit_engage_) || !stop_dist) {
        initial_vel = engage_velocity_;
        initial_acc = engage_acceleration_;
        RCLCPP_DEBUG(
          rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
          "calcInitialMotion : vehicle speed is low (%.3f), and desired speed is high (%.3f). Use "
          "engage speed (%.3f) until vehicle speed reaches engage_vel_thr (%.3f). stop_dist = %.3f",
          vehicle_speed, target_vel, engage_velocity_, engage_vel_thr, stop_dist.get());
        return std::make_tuple(initial_vel, initial_acc);
      } else if (stop_dist) {
        RCLCPP_DEBUG(
          rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
          "calcInitialMotion : stop point is close (%.3f[m]). no engage.", stop_dist.get());
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

TrajectoryPoint OptimizationBasedPlanner::calcInterpolatedTrajectoryPoint(
  const Trajectory & trajectory, const geometry_msgs::msg::Pose & target_pose)
{
  TrajectoryPoint traj_p{};
  traj_p.pose = target_pose;

  if (trajectory.points.empty()) {
    traj_p.longitudinal_velocity_mps = 0.0;
    traj_p.acceleration_mps2 = 0.0;
    return traj_p;
  }

  if (trajectory.points.size() == 1) {
    traj_p.longitudinal_velocity_mps = trajectory.points.at(0).longitudinal_velocity_mps;
    traj_p.acceleration_mps2 = trajectory.points.at(0).acceleration_mps2;
    return traj_p;
  }

  const size_t segment_idx =
    tier4_autoware_utils::findNearestSegmentIndex(trajectory.points, target_pose.position);

  const auto v1 = getTransVector3(
    trajectory.points.at(segment_idx).pose, trajectory.points.at(segment_idx + 1).pose);
  const auto v2 = getTransVector3(trajectory.points.at(segment_idx).pose, target_pose);

  // Calc interpolation ratio
  const double prop{std::clamp(v1.dot(v2) / v1.length2(), 0.0, 1.0)};

  {
    const auto & curr_pt = trajectory.points.at(segment_idx);
    const auto & next_pt = trajectory.points.at(segment_idx + 1);
    traj_p.pose = tier4_autoware_utils::calcInterpolatedPose(curr_pt, next_pt, prop);
    traj_p.longitudinal_velocity_mps = interpolation::lerp(
      curr_pt.longitudinal_velocity_mps, next_pt.longitudinal_velocity_mps, prop);
    traj_p.acceleration_mps2 =
      interpolation::lerp(curr_pt.acceleration_mps2, next_pt.acceleration_mps2, prop);
  }

  return traj_p;
}

bool OptimizationBasedPlanner::checkHasReachedGoal(const ObstacleCruisePlannerData & planner_data)
{
  // If goal is close and current velocity is low, we don't optimize trajectory
  const auto closest_stop_dist = tier4_autoware_utils::calcDistanceToForwardStopPoint(
    planner_data.traj.points, planner_data.current_pose, nearest_dist_deviation_threshold_,
    nearest_yaw_deviation_threshold_);
  if (closest_stop_dist && *closest_stop_dist < 0.5 && planner_data.current_vel < 0.6) {
    return true;
  }

  return false;
}

OptimizationBasedPlanner::TrajectoryData OptimizationBasedPlanner::getTrajectoryData(
  const Trajectory & traj, const geometry_msgs::msg::Pose & current_pose)
{
  TrajectoryData base_traj;
  const auto nearest_segment_idx =
    tier4_autoware_utils::findNearestSegmentIndex(traj.points, current_pose.position);
  const auto interpolated_point = calcInterpolatedTrajectoryPoint(traj, current_pose);
  const auto dist = tier4_autoware_utils::calcDistance2d(
    interpolated_point.pose.position, traj.points.at(nearest_segment_idx).pose.position);
  const auto current_point =
    dist > CLOSE_S_DIST_THRESHOLD ? interpolated_point : traj.points.at(nearest_segment_idx);
  base_traj.traj.points.push_back(current_point);
  base_traj.s.push_back(0.0);

  for (size_t id = nearest_segment_idx + 1; id < traj.points.size(); ++id) {
    const auto prev_point = base_traj.traj.points.back();
    const double ds = tier4_autoware_utils::calcDistance2d(
      prev_point.pose.position, traj.points.at(id).pose.position);
    if (ds < CLOSE_S_DIST_THRESHOLD) {
      continue;
    }
    const double current_s = base_traj.s.back() + ds;

    base_traj.traj.points.push_back(traj.points.at(id));
    base_traj.s.push_back(current_s);
  }

  return base_traj;
}

OptimizationBasedPlanner::TrajectoryData OptimizationBasedPlanner::resampleTrajectoryData(
  const TrajectoryData & base_traj_data, const double resampling_s_interval,
  const double max_traj_length)
{
  // Create Base Keys
  std::vector<double> base_s(base_traj_data.s.size());
  for (size_t i = 0; i < base_s.size(); ++i) {
    base_s.at(i) = base_traj_data.s.at(i);
  }

  // Obtain trajectory length until the velocity is zero or stop dist
  const auto closest_stop_id =
    tier4_autoware_utils::searchZeroVelocityIndex(base_traj_data.traj.points);
  const double closest_stop_dist = closest_stop_id ? base_s.at(*closest_stop_id) : base_s.back();
  const double traj_length = std::min(closest_stop_dist, std::min(base_s.back(), max_traj_length));

  // Create Query Keys
  std::vector<double> resampled_s;
  for (double s = 0.0; s < traj_length; s += resampling_s_interval) {
    resampled_s.push_back(s);
  }

  if (!resampled_s.empty() && traj_length - resampled_s.back() < CLOSE_S_DIST_THRESHOLD) {
    resampled_s.back() = traj_length;
  } else {
    resampled_s.push_back(traj_length);
  }

  if (resampled_s.empty()) {
    return TrajectoryData{};
  }

  // Resample trajectory
  const auto resampled_traj = resampleTrajectory(base_s, base_traj_data.traj, resampled_s);

  // Store Data
  TrajectoryData resampled_traj_data;
  resampled_traj_data.traj = resampled_traj;
  resampled_traj_data.s = resampled_s;

  return resampled_traj_data;
}

// TODO(shimizu) what is the difference with apply linear interpolation
Trajectory OptimizationBasedPlanner::resampleTrajectory(
  const std::vector<double> & base_index, const Trajectory & base_trajectory,
  const std::vector<double> & query_index, const bool use_spline_for_pose)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx;
  for (const auto & p : base_trajectory.points) {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.longitudinal_velocity_mps);
    taz.push_back(p.heading_rate_rps);
    alx.push_back(p.acceleration_mps2);
  }

  convertEulerAngleToMonotonic(pyaw);

  std::vector<double> px_p, py_p, pz_p, pyaw_p;
  if (use_spline_for_pose) {
    px_p = interpolation::slerp(base_index, px, query_index);
    py_p = interpolation::slerp(base_index, py, query_index);
    pz_p = interpolation::slerp(base_index, pz, query_index);
    pyaw_p = interpolation::slerp(base_index, pyaw, query_index);
  } else {
    px_p = interpolation::lerp(base_index, px, query_index);
    py_p = interpolation::lerp(base_index, py, query_index);
    pz_p = interpolation::lerp(base_index, pz, query_index);
    pyaw_p = interpolation::lerp(base_index, pyaw, query_index);
  }
  const auto tlx_p = interpolation::lerp(base_index, tlx, query_index);
  const auto taz_p = interpolation::lerp(base_index, taz, query_index);
  const auto alx_p = interpolation::lerp(base_index, alx, query_index);

  Trajectory resampled_trajectory;
  resampled_trajectory.header = base_trajectory.header;
  resampled_trajectory.points.resize(query_index.size());

  for (size_t i = 0; i < query_index.size(); ++i) {
    TrajectoryPoint point;
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(pyaw_p.at(i));

    point.longitudinal_velocity_mps = tlx_p.at(i);
    point.heading_rate_rps = taz_p.at(i);
    point.acceleration_mps2 = alx_p.at(i);
    resampled_trajectory.points.at(i) = point;
  }
  return resampled_trajectory;
}

boost::optional<SBoundaries> OptimizationBasedPlanner::getSBoundaries(
  const ObstacleCruisePlannerData & planner_data, const TrajectoryData & ego_traj_data,
  const std::vector<double> & time_vec)
{
  if (ego_traj_data.traj.points.empty()) {
    return boost::none;
  }

  const auto & current_time = planner_data.current_time;

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = ego_traj_data.s.back();
  }

  double min_slow_down_point_length = std::numeric_limits<double>::max();
  boost::optional<size_t> min_slow_down_idx = {};
  for (size_t o_idx = 0; o_idx < planner_data.target_obstacles.size(); ++o_idx) {
    const auto & obj = planner_data.target_obstacles.at(o_idx);
    const auto obj_base_time = planner_data.target_obstacles.at(o_idx).time_stamp;
    // Only see cruise obstacles
    if (obj.has_stopped) {
      continue;
    }

    // Step1 Get S boundary from the obstacle
    const auto obj_s_boundaries =
      getSBoundaries(planner_data.current_time, ego_traj_data, obj, obj_base_time, time_vec);
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
    const double object_offset = obj.shape.dimensions.x / 2.0;

    const auto current_object_pose = obstacle_cruise_utils::getCurrentObjectPoseFromPredictedPaths(
      obj.predicted_paths, obj_base_time, current_time);

    const double obj_vel = std::abs(obj.velocity);
    const double rss_dist = calcRSSDistance(planner_data.current_vel, obj_vel);

    const auto & safe_distance_margin = longitudinal_info_.safe_distance_margin;
    const double ego_obj_length = tier4_autoware_utils::calcSignedArcLength(
      ego_traj_data.traj.points, planner_data.current_pose.position,
      current_object_pose.get().position);
    const double slow_down_point_length =
      ego_obj_length - (rss_dist + object_offset + safe_distance_margin);

    if (slow_down_point_length < min_slow_down_point_length) {
      min_slow_down_point_length = slow_down_point_length;
      min_slow_down_idx = o_idx;
    }
  }

  // Publish wall marker for slowing down or stopping
  if (min_slow_down_idx) {
    const auto & obj = planner_data.target_obstacles.at(min_slow_down_idx.get());

    const auto current_object_pose = obstacle_cruise_utils::getCurrentObjectPoseFromPredictedPaths(
      obj.predicted_paths, obj.time_stamp, current_time);

    const auto marker_pose =
      obstacle_cruise_utils::calcForwardPose(ego_traj_data.traj, 0, min_slow_down_point_length);

    if (marker_pose) {
      visualization_msgs::msg::MarkerArray wall_msg;

      if (obj.has_stopped) {
        const auto markers = tier4_autoware_utils::createStopVirtualWallMarker(
          marker_pose.get(), "obstacle to follow", current_time, 0);
        tier4_autoware_utils::appendMarkerArray(markers, &wall_msg);
      } else {
        const auto markers = tier4_autoware_utils::createSlowDownVirtualWallMarker(
          marker_pose.get(), "obstacle to follow", current_time, 0);
        tier4_autoware_utils::appendMarkerArray(markers, &wall_msg);
      }

      // publish rviz marker
      debug_wall_marker_pub_->publish(wall_msg);
    }
  }

  return s_boundaries;
}

boost::optional<SBoundaries> OptimizationBasedPlanner::getSBoundaries(
  const rclcpp::Time & current_time, const TrajectoryData & ego_traj_data,
  const TargetObstacle & object, const rclcpp::Time & obj_base_time,
  const std::vector<double> & time_vec)
{
  // Get the predicted path, which has the most high confidence
  const double max_horizon = time_vec.back();
  const auto predicted_path =
    resamplePredictedPath(object, obj_base_time, current_time, time_vec, max_horizon);
  if (!predicted_path) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Closest Obstacle does not have a predicted path");
    return boost::none;
  }

  // Get current pose from object's predicted path
  const auto current_object_pose = obstacle_cruise_utils::getCurrentObjectPoseFromPredictedPath(
    *predicted_path, obj_base_time, current_time);
  if (!current_object_pose) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "Failed to get current pose from the predicted path");
    return boost::none;
  }

  // Check current object.kinematics
  ObjectData obj_data;
  obj_data.pose = current_object_pose.get();
  obj_data.length = object.shape.dimensions.x;
  obj_data.width = object.shape.dimensions.y;
  obj_data.time = std::max((obj_base_time - current_time).seconds(), 0.0);
  const auto current_collision_dist = getDistanceToCollisionPoint(ego_traj_data, obj_data);

  // Calculate Safety Distance
  const auto & safe_distance_margin = longitudinal_info_.safe_distance_margin;
  const double ego_vehicle_offset = vehicle_info_.wheel_base_m + vehicle_info_.front_overhang_m;
  const double object_offset = obj_data.length / 2.0;
  const double safety_distance = ego_vehicle_offset + object_offset + safe_distance_margin;

  // If the object is on the current ego trajectory,
  // we assume the object travels along ego trajectory
  if (current_collision_dist) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ObstacleCruisePlanner::OptimizationBasedPlanner"),
      "On Trajectory Object");

    return getSBoundaries(
      ego_traj_data, time_vec, safety_distance, object, *current_collision_dist);
  }

  // Ignore low velocity objects that are not on the trajectory
  return getSBoundaries(
    current_time, ego_traj_data, time_vec, safety_distance, object, obj_base_time, *predicted_path);
}

boost::optional<SBoundaries> OptimizationBasedPlanner::getSBoundaries(
  const TrajectoryData & ego_traj_data, const std::vector<double> & time_vec,
  const double safety_distance, const TargetObstacle & object, const double dist_to_collision_point)
{
  const double & min_object_accel_for_rss = longitudinal_info_.min_object_accel_for_rss;

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = ego_traj_data.s.back();
  }

  const double v_obj = std::abs(object.velocity);

  double current_s_obj = std::max(dist_to_collision_point - safety_distance, 0.0);
  const double current_v_obj = object.has_stopped ? 0.0 : v_obj;
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

boost::optional<SBoundaries> OptimizationBasedPlanner::getSBoundaries(
  const rclcpp::Time & current_time, const TrajectoryData & ego_traj_data,
  const std::vector<double> & time_vec, const double safety_distance, const TargetObstacle & object,
  const rclcpp::Time & obj_base_time, const PredictedPath & predicted_path)
{
  const double & min_object_accel_for_rss = longitudinal_info_.min_object_accel_for_rss;

  const double abs_obj_vel = std::abs(object.velocity);
  const double v_obj = object.has_stopped ? 0.0 : abs_obj_vel;

  SBoundaries s_boundaries(time_vec.size());
  for (size_t i = 0; i < s_boundaries.size(); ++i) {
    s_boundaries.at(i).max_s = ego_traj_data.s.back();
  }

  for (size_t predicted_path_id = 0; predicted_path_id < predicted_path.path.size();
       ++predicted_path_id) {
    const auto predicted_pose = predicted_path.path.at(predicted_path_id);
    const double object_time = (obj_base_time - current_time).seconds();
    if (object_time < 0) {
      // Ignore Past Positions
      continue;
    }

    ObjectData obj_data;
    obj_data.pose = predicted_pose;
    obj_data.length = object.shape.dimensions.x;
    obj_data.width = object.shape.dimensions.y;
    obj_data.time = object_time;

    const auto dist_to_collision_point = getDistanceToCollisionPoint(ego_traj_data, obj_data);
    if (!dist_to_collision_point) {
      continue;
    }

    const double current_s_obj = std::max(*dist_to_collision_point - safety_distance, 0.0);
    const double s_upper_bound =
      current_s_obj + (v_obj * v_obj) / (2 * std::fabs(min_object_accel_for_rss));
    for (size_t i = 0; i < predicted_path_id; ++i) {
      if (s_upper_bound < s_boundaries.at(i).max_s) {
        s_boundaries.at(i).max_s = std::max(0.0, s_upper_bound);
        s_boundaries.at(i).is_object = true;
      }
    }
  }

  return s_boundaries;
}

boost::optional<double> OptimizationBasedPlanner::getDistanceToCollisionPoint(
  const TrajectoryData & ego_traj_data, const ObjectData & obj_data)
{
  const auto obj_pose = obj_data.pose;
  const auto obj_length = obj_data.length;
  const auto obj_width = obj_data.width;
  const auto object_box = Box2d(obj_pose, obj_length, obj_width);
  const auto object_points = object_box.getAllCorners();

  // Get nearest segment index for each point
  size_t min_nearest_idx = ego_traj_data.s.size();
  size_t max_nearest_idx = 0;
  for (const auto & obj_p : object_points) {
    size_t nearest_idx =
      tier4_autoware_utils::findNearestSegmentIndex(ego_traj_data.traj.points, obj_p);
    min_nearest_idx = std::min(nearest_idx, min_nearest_idx);
    max_nearest_idx = std::max(nearest_idx, max_nearest_idx);
  }

  double min_len = 0.0;
  size_t start_idx = 0;
  for (size_t i = min_nearest_idx; i > 0; --i) {
    min_len += (ego_traj_data.s.at(i) - ego_traj_data.s.at(i - 1));
    if (min_len > 5.0) {
      start_idx = i - 1;
      break;
    }
  }

  double max_len = 0.0;
  size_t end_idx = ego_traj_data.s.size() - 1;
  for (size_t i = max_nearest_idx; i < ego_traj_data.s.size() - 1; ++i) {
    max_len += (ego_traj_data.s.at(i + 1) - ego_traj_data.s.at(i));
    if (max_len > 5.0) {
      end_idx = i + 1;
      break;
    }
  }

  // Check collision
  const auto collision_idx = getCollisionIdx(ego_traj_data, object_box, start_idx, end_idx);

  if (collision_idx) {
    // TODO(shimizu) Consider the time difference between ego vehicle and objects
    return tier4_autoware_utils::calcSignedArcLength(
      ego_traj_data.traj.points, ego_traj_data.traj.points.front().pose.position,
      obj_pose.position);
  }

  return boost::none;
}

boost::optional<size_t> OptimizationBasedPlanner::getCollisionIdx(
  const TrajectoryData & ego_traj, const Box2d & obj_box, const size_t start_idx,
  const size_t end_idx)
{
  for (size_t ego_idx = start_idx; ego_idx <= end_idx; ++ego_idx) {
    const auto ego_center_pose = transformBaseLink2Center(ego_traj.traj.points.at(ego_idx).pose);
    const auto ego_box =
      Box2d(ego_center_pose, vehicle_info_.vehicle_length_m, vehicle_info_.vehicle_width_m);
    if (ego_box.hasOverlap(obj_box)) {
      return ego_idx;
    }
  }

  return boost::none;
}

boost::optional<PredictedPath> OptimizationBasedPlanner::resamplePredictedPath(
  const TargetObstacle & object, const rclcpp::Time & obj_base_time,
  const rclcpp::Time & current_time, const std::vector<double> & resolutions, const double horizon)
{
  if (object.predicted_paths.empty()) {
    return boost::none;
  }

  // Get the most reliable path
  const auto reliable_path = std::max_element(
    object.predicted_paths.begin(), object.predicted_paths.end(),
    [](const PredictedPath & a, const PredictedPath & b) { return a.confidence < b.confidence; });

  // Resample Predicted Path
  const double duration = std::min(
    std::max(
      (obj_base_time +
       rclcpp::Duration(reliable_path->time_step) *
         (static_cast<double>(reliable_path->path.size()) - 1) -
       current_time)
        .seconds(),
      0.0),
    horizon);

  // Calculate relative valid time vector
  // rel_valid_time_vec is relative to obj_base_time.
  const auto rel_valid_time_vec = resampling::resampledValidRelativeTimeVector(
    current_time, obj_base_time, resolutions, duration);

  return resampling::resamplePredictedPath(*reliable_path, rel_valid_time_vec);
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

boost::optional<VelocityOptimizer::OptimizationResult>
OptimizationBasedPlanner::processOptimizedResult(
  const double v0, const VelocityOptimizer::OptimizationResult & opt_result)
{
  if (
    opt_result.t.empty() || opt_result.s.empty() || opt_result.v.empty() || opt_result.a.empty() ||
    opt_result.j.empty()) {
    return boost::none;
  }

  size_t break_id = opt_result.s.size();
  VelocityOptimizer::OptimizationResult processed_result;
  processed_result.t.push_back(0.0);
  processed_result.s.push_back(0.0);
  processed_result.v.push_back(v0);
  processed_result.a.push_back(opt_result.a.front());
  processed_result.j.push_back(opt_result.j.front());

  for (size_t i = 1; i < opt_result.s.size(); ++i) {
    const double prev_s = processed_result.s.back();
    const double current_s = std::max(opt_result.s.at(i), 0.0);
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
      processed_result.s.back() = prev_s > 0.0 ? current_s : prev_s;
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
    const double current_s = std::max(opt_result.s.at(i), 0.0);
    if (prev_s >= current_s) {
      processed_result.v.back() = 0.0;
      continue;
    } else if (std::fabs(current_s - prev_s) < CLOSE_S_DIST_THRESHOLD) {
      processed_result.v.back() = 0.0;
      processed_result.s.back() = prev_s > 0.0 ? current_s : prev_s;
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
  const rclcpp::Time & current_time, const Trajectory & traj, const size_t closest_idx,
  const std::vector<double> & time_vec, const SBoundaries & s_boundaries,
  const VelocityOptimizer::OptimizationResult & opt_result)
{
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

  const double s_before = tier4_autoware_utils::calcSignedArcLength(traj.points, 0, closest_idx);
  Trajectory optimized_sv_traj;
  optimized_sv_traj.header.stamp = current_time;
  optimized_sv_traj.points.resize(opt_result.s.size());
  for (size_t i = 0; i < opt_result.s.size(); ++i) {
    const double s = opt_result.s.at(i);
    const double v = opt_result.v.at(i);
    optimized_sv_traj.points.at(i).pose.position.x = s + s_before;
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
