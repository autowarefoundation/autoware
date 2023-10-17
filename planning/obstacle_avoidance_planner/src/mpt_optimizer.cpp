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

#include "obstacle_avoidance_planner/mpt_optimizer.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "obstacle_avoidance_planner/utils/geometry_utils.hpp"
#include "obstacle_avoidance_planner/utils/trajectory_utils.hpp"
#include "tf2/utils.h"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/normalization.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <optional>
#include <tuple>

namespace obstacle_avoidance_planner
{
namespace
{
std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByUniformCircle(
  const vehicle_info_util::VehicleInfo & vehicle_info, const size_t circle_num,
  const double radius_ratio)
{
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const double radius = std::hypot(
                          vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
                          vehicle_info.vehicle_width_m / 2.0 + lateral_offset) *
                        radius_ratio;
  const std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(
      unit_lon_length / 2.0 + unit_lon_length * i - vehicle_info.rear_overhang_m);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByBicycleModel(
  const vehicle_info_util::VehicleInfo & vehicle_info, const size_t circle_num,
  const double rear_radius_ratio, const double front_radius_ratio)
{
  if (circle_num < 2) {
    throw std::invalid_argument("circle_num is less than 2.");
  }
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  // 1st circle (rear wheel)
  const double rear_radius =
    vehicle_info.vehicle_width_m / 2.0 + lateral_offset * rear_radius_ratio;
  const double rear_lon_offset = 0.0;

  // 2nd circle (front wheel)
  const double front_radius =
    std::hypot(
      vehicle_info.vehicle_length_m / static_cast<double>(circle_num) / 2.0,
      vehicle_info.vehicle_width_m / 2.0 + lateral_offset) *
    front_radius_ratio;

  const double unit_lon_length = vehicle_info.vehicle_length_m / static_cast<double>(circle_num);
  const double front_lon_offset =
    unit_lon_length / 2.0 + unit_lon_length * (circle_num - 1) - vehicle_info.rear_overhang_m;

  return {{rear_radius, front_radius}, {rear_lon_offset, front_lon_offset}};
}

std::tuple<std::vector<double>, std::vector<double>> calcVehicleCirclesByFittingUniformCircle(
  const vehicle_info_util::VehicleInfo & vehicle_info, const size_t circle_num)
{
  if (circle_num < 2) {
    throw std::invalid_argument("circle_num is less than 2.");
  }
  const double lateral_offset =
    abs(vehicle_info.right_overhang_m - vehicle_info.left_overhang_m) / 2.0;
  const double radius = vehicle_info.vehicle_width_m / 2.0 + lateral_offset;
  std::vector<double> radiuses(circle_num, radius);

  const double unit_lon_length =
    vehicle_info.vehicle_length_m / static_cast<double>(circle_num - 1);
  std::vector<double> longitudinal_offsets;
  for (size_t i = 0; i < circle_num; ++i) {
    longitudinal_offsets.push_back(unit_lon_length * i - vehicle_info.rear_overhang_m);
    radiuses.push_back(radius);
  }

  return {radiuses, longitudinal_offsets};
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd> extractBounds(
  const std::vector<ReferencePoint> & ref_points, const size_t l_idx, const double offset)
{
  Eigen::VectorXd ub_vec(ref_points.size());
  Eigen::VectorXd lb_vec(ref_points.size());
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ub_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).upper_bound + offset;
    lb_vec(i) = ref_points.at(i).bounds_on_constraints.at(l_idx).lower_bound - offset;
  }
  return {ub_vec, lb_vec};
}

std::vector<double> toStdVector(const Eigen::VectorXd & eigen_vec)
{
  return {eigen_vec.data(), eigen_vec.data() + eigen_vec.rows()};
}

bool isLeft(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Point & target_pos)
{
  const double base_theta = tf2::getYaw(pose.orientation);
  const double target_theta = tier4_autoware_utils::calcAzimuthAngle(pose.position, target_pos);
  const double diff_theta = tier4_autoware_utils::normalizeRadian(target_theta - base_theta);
  return diff_theta > 0;
}

// NOTE: Regarding boundary's sign, left is positive, and right is negative
double calcLateralDistToBounds(
  const geometry_msgs::msg::Pose & pose, const std::vector<geometry_msgs::msg::Point> & bound,
  const double additional_offset, const bool is_left_bound = true)
{
  constexpr double max_lat_offset_for_left = 5.0;
  constexpr double min_lat_offset_for_left = -5.0;

  const double max_lat_offset = is_left_bound ? max_lat_offset_for_left : -max_lat_offset_for_left;
  const double min_lat_offset = is_left_bound ? min_lat_offset_for_left : -min_lat_offset_for_left;
  const auto max_lat_offset_point =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, max_lat_offset, 0.0).position;
  const auto min_lat_offset_point =
    tier4_autoware_utils::calcOffsetPose(pose, 0.0, min_lat_offset, 0.0).position;

  double closest_dist_to_bound = max_lat_offset;
  for (size_t i = 0; i < bound.size() - 1; ++i) {
    const auto intersect_point = tier4_autoware_utils::intersect(
      min_lat_offset_point, max_lat_offset_point, bound.at(i), bound.at(i + 1));
    if (intersect_point) {
      const bool is_point_left = isLeft(pose, *intersect_point);
      const double dist_to_bound =
        tier4_autoware_utils::calcDistance2d(pose.position, *intersect_point) *
        (is_point_left ? 1.0 : -1.0);

      // the bound which is closest to the centerline will be chosen
      const double tmp_dist_to_bound =
        is_left_bound ? dist_to_bound - additional_offset : dist_to_bound + additional_offset;
      if (std::abs(tmp_dist_to_bound) < std::abs(closest_dist_to_bound)) {
        closest_dist_to_bound = tmp_dist_to_bound;
      }
    }
  }

  return closest_dist_to_bound;
}
}  // namespace

MPTOptimizer::MPTParam::MPTParam(
  rclcpp::Node * node, const vehicle_info_util::VehicleInfo & vehicle_info)
{
  {  // option
    steer_limit_constraint = node->declare_parameter<bool>("mpt.option.steer_limit_constraint");
    enable_warm_start = node->declare_parameter<bool>("mpt.option.enable_warm_start");
    enable_manual_warm_start = node->declare_parameter<bool>("mpt.option.enable_manual_warm_start");
    enable_optimization_validation =
      node->declare_parameter<bool>("mpt.option.enable_optimization_validation");
    mpt_visualize_sampling_num = node->declare_parameter<int>("mpt.option.visualize_sampling_num");
  }

  {  // common
    num_points = node->declare_parameter<int>("mpt.common.num_points");
    delta_arc_length = node->declare_parameter<double>("mpt.common.delta_arc_length");
  }

  // kinematics
  max_steer_rad = vehicle_info.max_steer_angle_rad;

  // NOTE: By default, optimization_center_offset will be vehicle_info.wheel_base * 0.8
  //       The 0.8 scale is adopted as it performed the best.
  constexpr double default_wheel_base_ratio = 0.8;
  optimization_center_offset = node->declare_parameter<double>(
    "mpt.kinematics.optimization_center_offset",
    vehicle_info.wheel_base_m * default_wheel_base_ratio);

  {  // clearance
    hard_clearance_from_road =
      node->declare_parameter<double>("mpt.clearance.hard_clearance_from_road");
    soft_clearance_from_road =
      node->declare_parameter<double>("mpt.clearance.soft_clearance_from_road");
  }

  {  // weight
    soft_collision_free_weight =
      node->declare_parameter<double>("mpt.weight.soft_collision_free_weight");

    lat_error_weight = node->declare_parameter<double>("mpt.weight.lat_error_weight");
    yaw_error_weight = node->declare_parameter<double>("mpt.weight.yaw_error_weight");
    yaw_error_rate_weight = node->declare_parameter<double>("mpt.weight.yaw_error_rate_weight");
    steer_input_weight = node->declare_parameter<double>("mpt.weight.steer_input_weight");
    steer_rate_weight = node->declare_parameter<double>("mpt.weight.steer_rate_weight");

    terminal_lat_error_weight =
      node->declare_parameter<double>("mpt.weight.terminal_lat_error_weight");
    terminal_yaw_error_weight =
      node->declare_parameter<double>("mpt.weight.terminal_yaw_error_weight");
    goal_lat_error_weight = node->declare_parameter<double>("mpt.weight.goal_lat_error_weight");
    goal_yaw_error_weight = node->declare_parameter<double>("mpt.weight.goal_yaw_error_weight");
  }

  {  // avoidance
    max_longitudinal_margin_for_bound_violation =
      node->declare_parameter<double>("mpt.avoidance.max_longitudinal_margin_for_bound_violation");
    max_bound_fixing_time = node->declare_parameter<double>("mpt.avoidance.max_bound_fixing_time");
    max_avoidance_cost = node->declare_parameter<double>("mpt.avoidance.max_avoidance_cost");
    avoidance_cost_margin = node->declare_parameter<double>("mpt.avoidance.avoidance_cost_margin");
    avoidance_cost_band_length =
      node->declare_parameter<double>("mpt.avoidance.avoidance_cost_band_length");
    avoidance_cost_decrease_rate =
      node->declare_parameter<double>("mpt.avoidance.avoidance_cost_decrease_rate");
    min_drivable_width = node->declare_parameter<double>("mpt.avoidance.min_drivable_width");

    avoidance_lat_error_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.lat_error_weight");
    avoidance_yaw_error_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.yaw_error_weight");
    avoidance_steer_input_weight =
      node->declare_parameter<double>("mpt.avoidance.weight.steer_input_weight");
  }

  {  // collision free constraints
    l_inf_norm = node->declare_parameter<bool>("mpt.collision_free_constraints.option.l_inf_norm");
    soft_constraint =
      node->declare_parameter<bool>("mpt.collision_free_constraints.option.soft_constraint");
    hard_constraint =
      node->declare_parameter<bool>("mpt.collision_free_constraints.option.hard_constraint");
  }

  {  // vehicle_circles
    // NOTE: Vehicle shape for collision free constraints is considered as a set of circles
    vehicle_circles_method =
      node->declare_parameter<std::string>("mpt.collision_free_constraints.vehicle_circles.method");

    // uniform circles
    vehicle_circles_uniform_circle_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.uniform_circle.num");
    vehicle_circles_uniform_circle_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio");

    // bicycle model
    vehicle_circles_bicycle_model_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_"
      "calculation");
    vehicle_circles_bicycle_model_rear_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles."
      "bicycle_model.rear_radius_ratio");
    vehicle_circles_bicycle_model_front_radius_ratio = node->declare_parameter<double>(
      "mpt.collision_free_constraints.vehicle_circles."
      "bicycle_model.front_radius_ratio");

    // fitting uniform circles
    vehicle_circles_fitting_uniform_circle_num = node->declare_parameter<int>(
      "mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num");
  }

  {  // validation
    max_validation_lat_error = node->declare_parameter<double>("mpt.validation.max_lat_error");
    max_validation_yaw_error = node->declare_parameter<double>("mpt.validation.max_yaw_error");
  }
}

void MPTOptimizer::MPTParam::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  {  // option
    updateParam<bool>(parameters, "mpt.option.steer_limit_constraint", steer_limit_constraint);
    updateParam<bool>(parameters, "mpt.option.enable_warm_start", enable_warm_start);
    updateParam<bool>(parameters, "mpt.option.enable_manual_warm_start", enable_manual_warm_start);
    updateParam<bool>(
      parameters, "mpt.option.enable_optimization_validation", enable_optimization_validation);
    updateParam<int>(parameters, "mpt.option.visualize_sampling_num", mpt_visualize_sampling_num);
  }

  // common
  updateParam<int>(parameters, "mpt.common.num_points", num_points);
  updateParam<double>(parameters, "mpt.common.delta_arc_length", delta_arc_length);

  // kinematics
  updateParam<double>(
    parameters, "mpt.kinematics.optimization_center_offset", optimization_center_offset);

  // collision_free_constraints
  updateParam<bool>(parameters, "mpt.collision_free_constraints.option.l_inf_norm", l_inf_norm);
  updateParam<bool>(
    parameters, "mpt.collision_free_constraints.option.soft_constraint", soft_constraint);
  updateParam<bool>(
    parameters, "mpt.collision_free_constraints.option.hard_constraint", hard_constraint);

  {  // vehicle_circles
    updateParam<std::string>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.method", vehicle_circles_method);

    // uniform circles
    updateParam<int>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.uniform_circle.num",
      vehicle_circles_uniform_circle_num);
    updateParam<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.uniform_circle.radius_ratio",
      vehicle_circles_uniform_circle_radius_ratio);

    // bicycle model
    updateParam<int>(
      parameters,
      "mpt.collision_free_constraints.vehicle_circles.bicycle_model.num_for_calculation",
      vehicle_circles_bicycle_model_num);
    updateParam<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.bicycle_model.rear_radius_ratio",
      vehicle_circles_bicycle_model_rear_radius_ratio);
    updateParam<double>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.bicycle_model.front_radius_ratio",
      vehicle_circles_bicycle_model_front_radius_ratio);

    // fitting uniform circles
    updateParam<int>(
      parameters, "mpt.collision_free_constraints.vehicle_circles.fitting_uniform_circle.num",
      vehicle_circles_fitting_uniform_circle_num);
  }

  {  // clearance
    updateParam<double>(
      parameters, "mpt.clearance.hard_clearance_from_road", hard_clearance_from_road);
    updateParam<double>(
      parameters, "mpt.clearance.soft_clearance_from_road", soft_clearance_from_road);
  }

  {  // weight
    updateParam<double>(
      parameters, "mpt.weight.soft_collision_free_weight", soft_collision_free_weight);

    updateParam<double>(parameters, "mpt.weight.lat_error_weight", lat_error_weight);
    updateParam<double>(parameters, "mpt.weight.yaw_error_weight", yaw_error_weight);
    updateParam<double>(parameters, "mpt.weight.yaw_error_rate_weight", yaw_error_rate_weight);
    updateParam<double>(parameters, "mpt.weight.steer_input_weight", steer_input_weight);
    updateParam<double>(parameters, "mpt.weight.steer_rate_weight", steer_rate_weight);

    updateParam<double>(
      parameters, "mpt.weight.terminal_lat_error_weight", terminal_lat_error_weight);
    updateParam<double>(
      parameters, "mpt.weight.terminal_yaw_error_weight", terminal_yaw_error_weight);
    updateParam<double>(parameters, "mpt.weight.goal_lat_error_weight", goal_lat_error_weight);
    updateParam<double>(parameters, "mpt.weight.goal_yaw_error_weight", goal_yaw_error_weight);
  }

  {  // avoidance
    updateParam<double>(
      parameters, "mpt.avoidance.max_longitudinal_margin_for_bound_violation",
      max_longitudinal_margin_for_bound_violation);
    updateParam<double>(parameters, "mpt.avoidance.max_bound_fixing_time", max_bound_fixing_time);
    updateParam<double>(parameters, "mpt.avoidance.min_drivable_width", min_drivable_width);
    updateParam<double>(parameters, "mpt.avoidance.max_avoidance_cost", max_avoidance_cost);
    updateParam<double>(parameters, "mpt.avoidance.avoidance_cost_margin", avoidance_cost_margin);
    updateParam<double>(
      parameters, "mpt.avoidance.avoidance_cost_band_length", avoidance_cost_band_length);
    updateParam<double>(
      parameters, "mpt.avoidance.avoidance_cost_decrease_rate", avoidance_cost_decrease_rate);

    updateParam<double>(
      parameters, "mpt.avoidance.weight.lat_error_weight", avoidance_lat_error_weight);
    updateParam<double>(
      parameters, "mpt.avoidance.weight.yaw_error_weight", avoidance_yaw_error_weight);
    updateParam<double>(
      parameters, "mpt.avoidance.weight.steer_input_weight", avoidance_steer_input_weight);
  }

  {  // validation
    updateParam<double>(parameters, "mpt.validation.max_lat_error", max_validation_lat_error);
    updateParam<double>(parameters, "mpt.validation.max_yaw_error", max_validation_yaw_error);
  }
}

MPTOptimizer::MPTOptimizer(
  rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
  const vehicle_info_util::VehicleInfo & vehicle_info, const TrajectoryParam & traj_param,
  const std::shared_ptr<DebugData> debug_data_ptr,
  const std::shared_ptr<TimeKeeper> time_keeper_ptr)
: enable_debug_info_(enable_debug_info),
  ego_nearest_param_(ego_nearest_param),
  vehicle_info_(vehicle_info),
  traj_param_(traj_param),
  debug_data_ptr_(debug_data_ptr),
  time_keeper_ptr_(time_keeper_ptr),
  logger_(node->get_logger().get_child("mpt_optimizer"))
{
  // initialize mpt param
  mpt_param_ = MPTParam(node, vehicle_info);
  updateVehicleCircles();
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_param_.mpt_visualize_sampling_num;

  // state equation generator
  state_equation_generator_ =
    StateEquationGenerator(vehicle_info_.wheel_base_m, mpt_param_.max_steer_rad, time_keeper_ptr_);

  // osqp solver
  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(osqp_epsilon_);

  // publisher
  debug_fixed_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_fixed_traj", 1);
  debug_ref_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_ref_traj", 1);
  debug_mpt_traj_pub_ = node->create_publisher<Trajectory>("~/debug/mpt_traj", 1);
}

void MPTOptimizer::updateVehicleCircles()
{
  const auto & p = mpt_param_;

  if (p.vehicle_circles_method == "uniform_circle") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByUniformCircle(
        vehicle_info_, p.vehicle_circles_uniform_circle_num,
        p.vehicle_circles_uniform_circle_radius_ratio);
  } else if (p.vehicle_circles_method == "bicycle_model") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByBicycleModel(
        vehicle_info_, p.vehicle_circles_bicycle_model_num,
        p.vehicle_circles_bicycle_model_rear_radius_ratio,
        p.vehicle_circles_bicycle_model_front_radius_ratio);
  } else if (p.vehicle_circles_method == "fitting_uniform_circle") {
    std::tie(vehicle_circle_radiuses_, vehicle_circle_longitudinal_offsets_) =
      calcVehicleCirclesByFittingUniformCircle(
        vehicle_info_, p.vehicle_circles_fitting_uniform_circle_num);
  } else {
    throw std::invalid_argument("mpt_param_.vehicle_circles_method is invalid.");
  }

  debug_data_ptr_->vehicle_circle_radiuses = vehicle_circle_radiuses_;
  debug_data_ptr_->vehicle_circle_longitudinal_offsets = vehicle_circle_longitudinal_offsets_;
}

void MPTOptimizer::initialize(const bool enable_debug_info, const TrajectoryParam & traj_param)
{
  enable_debug_info_ = enable_debug_info;
  traj_param_ = traj_param;
}

void MPTOptimizer::resetPreviousData()
{
  prev_ref_points_ptr_ = nullptr;
  prev_optimized_traj_points_ptr_ = nullptr;
}

void MPTOptimizer::onParam(const std::vector<rclcpp::Parameter> & parameters)
{
  mpt_param_.onParam(parameters);
  updateVehicleCircles();
  debug_data_ptr_->mpt_visualize_sampling_num = mpt_param_.mpt_visualize_sampling_num;
}

std::vector<TrajectoryPoint> MPTOptimizer::optimizeTrajectory(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points)
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = planner_data;
  const auto & traj_points = p.traj_points;

  const auto get_prev_optimized_traj_points = [&]() {
    if (prev_optimized_traj_points_ptr_) {
      return *prev_optimized_traj_points_ptr_;
    }
    return smoothed_points;
  };

  // 1. calculate reference points
  auto ref_points = calcReferencePoints(planner_data, smoothed_points);
  if (ref_points.size() < 2) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since ref_points size is less than 2.");
    return get_prev_optimized_traj_points();
  }

  // 2. calculate B and W matrices where x = B u + W
  const auto mpt_mat = state_equation_generator_.calcMatrix(ref_points);

  // 3. calculate Q and R matrices where J(x, u) = x^t Q x + u^t R u
  const auto val_mat = calcValueMatrix(ref_points, traj_points);

  // 4. get objective matrix
  const auto obj_mat = calcObjectiveMatrix(mpt_mat, val_mat, ref_points);

  // 5. get constraints matrix
  const auto const_mat = calcConstraintMatrix(mpt_mat, ref_points);

  // 6. optimize steer angles
  const auto optimized_variables = calcOptimizedSteerAngles(ref_points, obj_mat, const_mat);
  if (!optimized_variables) {
    RCLCPP_INFO_EXPRESSION(
      logger_, enable_debug_info_, "return std::nullopt since could not solve qp");
    return get_prev_optimized_traj_points();
  }

  // 7. convert to points with validation
  const auto mpt_traj_points = calcMPTPoints(ref_points, *optimized_variables, mpt_mat);
  if (!mpt_traj_points) {
    RCLCPP_WARN(logger_, "return std::nullopt since lateral or yaw error is too large.");
    return get_prev_optimized_traj_points();
  }

  // 8. publish trajectories for debug
  publishDebugTrajectories(p.header, ref_points, *mpt_traj_points);

  time_keeper_ptr_->toc(__func__, "      ");

  debug_data_ptr_->ref_points = ref_points;
  prev_ref_points_ptr_ = std::make_shared<std::vector<ReferencePoint>>(ref_points);
  prev_optimized_traj_points_ptr_ =
    std::make_shared<std::vector<TrajectoryPoint>>(*mpt_traj_points);

  return *mpt_traj_points;
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::getPrevOptimizedTrajectoryPoints() const
{
  if (prev_optimized_traj_points_ptr_) {
    return *prev_optimized_traj_points_ptr_;
  }
  return std::nullopt;
}

std::vector<ReferencePoint> MPTOptimizer::calcReferencePoints(
  const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const
{
  time_keeper_ptr_->tic(__func__);

  const auto & p = planner_data;

  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;

  // 1. resample and convert smoothed points type from trajectory points to reference points
  time_keeper_ptr_->tic("resampleReferencePoints");
  auto ref_points = [&]() {
    const auto resampled_smoothed_points =
      trajectory_utils::resampleTrajectoryPointsWithoutStopPoint(
        smoothed_points, mpt_param_.delta_arc_length);
    return trajectory_utils::convertToReferencePoints(resampled_smoothed_points);
  }();
  time_keeper_ptr_->toc("resampleReferencePoints", "          ");

  // 2. crop forward and backward with margin, and calculate spline interpolation
  // NOTE: Margin is added to calculate orientation, curvature, etc precisely.
  //       Start point may change. Spline calculation is required.
  constexpr double tmp_margin = 10.0;
  size_t ego_seg_idx =
    trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);
  ref_points = motion_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    backward_traj_length + tmp_margin);
  SplineInterpolationPoints2d ref_points_spline(ref_points);
  ego_seg_idx = trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);

  // 3. calculate orientation and curvature
  updateOrientation(ref_points, ref_points_spline);
  updateCurvature(ref_points, ref_points_spline);

  // 4. crop backward
  // NOTE: Start point may change. Spline calculation is required.
  ref_points = motion_utils::cropPoints(
    ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length + tmp_margin,
    backward_traj_length);
  ref_points_spline = SplineInterpolationPoints2d(ref_points);
  ego_seg_idx = trajectory_utils::findEgoSegmentIndex(ref_points, p.ego_pose, ego_nearest_param_);

  // 5. update fixed points, and resample
  // NOTE: This must be after backward cropping.
  //       New start point may be added and resampled. Spline calculation is required.
  updateFixedPoint(ref_points);
  ref_points_spline = SplineInterpolationPoints2d(ref_points);

  // 6. update bounds
  // NOTE: After this, resample must not be called since bounds are not interpolated.
  updateBounds(ref_points, p.left_bound, p.right_bound, p.ego_pose, p.ego_vel);
  updateVehicleBounds(ref_points, ref_points_spline);

  // 7. update delta arc length
  updateDeltaArcLength(ref_points);

  // 8. update extra information (alpha and beta)
  // NOTE: This must be after calculation of bounds and delta arc length
  updateExtraPoints(ref_points);

  // 9. crop forward
  // ref_points = motion_utils::cropForwardPoints(
  //   ref_points, p.ego_pose.position, ego_seg_idx, forward_traj_length);
  if (static_cast<size_t>(mpt_param_.num_points) < ref_points.size()) {
    ref_points.resize(mpt_param_.num_points);
  }

  time_keeper_ptr_->toc(__func__, "        ");

  return ref_points;
}

void MPTOptimizer::updateOrientation(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto yaw_vec = ref_points_spline.getSplineInterpolatedYaws();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).pose.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_vec.at(i));
  }
}

void MPTOptimizer::updateCurvature(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  const auto curvature_vec = ref_points_spline.getSplineInterpolatedCurvatures();
  for (size_t i = 0; i < ref_points.size(); ++i) {
    ref_points.at(i).curvature = curvature_vec.at(i);
  }
}

void MPTOptimizer::updateFixedPoint(std::vector<ReferencePoint> & ref_points) const
{
  time_keeper_ptr_->tic(__func__);

  if (!prev_ref_points_ptr_) {
    // no fixed point
    return;
  }

  // replace the front pose and curvature with previous reference points
  const auto idx = trajectory_utils::updateFrontPointForFix(
    ref_points, *prev_ref_points_ptr_, mpt_param_.delta_arc_length, ego_nearest_param_);

  // NOTE: memorize front point to be fixed before resampling
  const auto front_point = ref_points.front();

  if (idx && *idx != 0) {
    // In order to fix the front "orientation" defined by two front points, insert the previous
    // fixed point.
    ref_points.insert(ref_points.begin(), prev_ref_points_ptr_->at(static_cast<int>(*idx) - 1));

    // resample to make ref_points' interval constant.
    // NOTE: Only pose, velocity and curvature will be interpolated.
    ref_points = trajectory_utils::resampleReferencePoints(ref_points, mpt_param_.delta_arc_length);

    // update pose which is previous one, and fixed kinematic state
    // NOTE: There may be a lateral error between the previous and input points.
    //       Therefore, the pose for fix should not be resampled.
    const auto & prev_ref_front_point = prev_ref_points_ptr_->at(*idx);
    const auto & prev_ref_prev_front_point = prev_ref_points_ptr_->at(static_cast<int>(*idx) - 1);

    ref_points.front().pose = prev_ref_prev_front_point.pose;
    ref_points.front().fixed_kinematic_state = prev_ref_prev_front_point.optimized_kinematic_state;
    ref_points.at(1).pose = prev_ref_front_point.pose;
    ref_points.at(1).fixed_kinematic_state = prev_ref_front_point.optimized_kinematic_state;
  } else {
    // resample to make ref_points' interval constant.
    // NOTE: Only pose, velocity and curvature will be interpolated.
    ref_points = trajectory_utils::resampleReferencePoints(ref_points, mpt_param_.delta_arc_length);

    ref_points.front().pose = front_point.pose;
    ref_points.front().curvature = front_point.curvature;
    ref_points.front().fixed_kinematic_state = front_point.optimized_kinematic_state;
  }

  time_keeper_ptr_->toc(__func__, "          ");
}

void MPTOptimizer::updateDeltaArcLength(std::vector<ReferencePoint> & ref_points) const
{
  for (size_t i = 0; i < ref_points.size(); i++) {
    ref_points.at(i).delta_arc_length =
      (i == ref_points.size() - 1)
        ? 0.0
        : tier4_autoware_utils::calcDistance2d(ref_points.at(i + 1), ref_points.at(i));
  }
}

void MPTOptimizer::updateExtraPoints(std::vector<ReferencePoint> & ref_points) const
{
  time_keeper_ptr_->tic(__func__);

  // alpha
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto front_wheel_pos =
      trajectory_utils::getNearestPosition(ref_points, i, vehicle_info_.wheel_base_m);

    const bool are_too_close_points =
      tier4_autoware_utils::calcDistance2d(front_wheel_pos, ref_points.at(i).pose.position) < 1e-03;
    const auto front_wheel_yaw =
      are_too_close_points
        ? ref_points.at(i).getYaw()
        : tier4_autoware_utils::calcAzimuthAngle(ref_points.at(i).pose.position, front_wheel_pos);
    ref_points.at(i).alpha =
      tier4_autoware_utils::normalizeRadian(front_wheel_yaw - ref_points.at(i).getYaw());
  }

  {  // avoidance
    // calculate one-step avoidance const
    for (size_t i = 0; i < ref_points.size(); ++i) {
      const auto normalized_avoidance_cost = calcNormalizedAvoidanceCost(ref_points.at(i));
      if (normalized_avoidance_cost) {
        const int max_length_idx =
          std::floor(mpt_param_.avoidance_cost_band_length / mpt_param_.delta_arc_length);
        for (int j = -max_length_idx; j <= max_length_idx; ++j) {
          const int k = i + j;
          if (0 <= k && k < static_cast<int>(ref_points.size())) {
            ref_points.at(k).normalized_avoidance_cost = *normalized_avoidance_cost;
          }
        }
      }
    }

    /*
    // update avoidance cost between longitudinally close obstacles
    constexpr double max_longitudinal_length_to_fill_drivable_area = 50;
    const int edge_fill_index = std::ceil(max_longitudinal_length_to_fill_drivable_area /
    mpt_param_.delta_arc_length / 2); const auto copied_ref_points = ref_points; for (size_t i = 0;
    i < ref_points.size(); ++i) { const double base_normalized_avoidance_cost =
    ref_points.at(i).normalized_avoidance_cost; for (int j = -edge_fill_index; j <= edge_fill_index;
    ++j) { const int k = i + j; if (k < 0 || ref_points.size() - 1 <= k) { continue;
        }
        ref_points.at(i).normalized_avoidance_cost =
    std::max(ref_points.at(i).normalized_avoidance_cost,
    copied_ref_points.at(k).normalized_avoidance_cost);
      }
    }
    */

    // update spread avoidance cost
    for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
      const double base_normalized_avoidance_cost = ref_points.at(i).normalized_avoidance_cost;
      if (0 < base_normalized_avoidance_cost) {
        const int edge_decrease_idx = std::floor(
          ref_points.at(i).normalized_avoidance_cost / mpt_param_.avoidance_cost_decrease_rate);
        for (int j = -edge_decrease_idx; j <= edge_decrease_idx; ++j) {
          const int k = i + j;
          if (0 <= k && k < static_cast<int>(ref_points.size())) {
            const double normalized_avoidance_cost = std::max(
              base_normalized_avoidance_cost -
                std::abs(j) * mpt_param_.avoidance_cost_decrease_rate,
              ref_points.at(k).normalized_avoidance_cost);
            ref_points.at(k).normalized_avoidance_cost =
              std::clamp(normalized_avoidance_cost, 0.0, 1.0);
          }
        }
      }
    }

    // take over previous avoidance cost
    const double max_dist_threshold = mpt_param_.delta_arc_length / 2.0;
    if (prev_ref_points_ptr_ && !prev_ref_points_ptr_->empty()) {
      for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
        const size_t prev_idx = trajectory_utils::findEgoIndex(
          *prev_ref_points_ptr_, tier4_autoware_utils::getPose(ref_points.at(i)),
          ego_nearest_param_);

        const double dist_to_prev = tier4_autoware_utils::calcDistance2d(
          ref_points.at(i), prev_ref_points_ptr_->at(prev_idx));
        if (max_dist_threshold < dist_to_prev) {
          continue;
        }

        ref_points.at(i).normalized_avoidance_cost = std::max(
          prev_ref_points_ptr_->at(prev_idx).normalized_avoidance_cost,
          ref_points.at(i).normalized_avoidance_cost);
      }
    }
  }

  time_keeper_ptr_->toc(__func__, "          ");
}

void MPTOptimizer::updateBounds(
  std::vector<ReferencePoint> & ref_points,
  const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound,
  const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const
{
  time_keeper_ptr_->tic(__func__);

  const double soft_road_clearance =
    mpt_param_.soft_clearance_from_road + vehicle_info_.vehicle_width_m / 2.0;

  // calculate distance to left/right bound on each reference point
  // NOTE: Reference points is sometimes not fully covered by the drivable area.
  //       In some edge cases like U-turn, the wrong bound may be found. To avoid finding the wrong
  //       bound, some beginning bounds are copied from the following one.
  const size_t min_ref_point_index = std::min(
    static_cast<size_t>(std::ceil(1.0 / mpt_param_.delta_arc_length)), ref_points.size() - 1);
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto ref_point_for_bound_search = ref_points.at(std::max(min_ref_point_index, i));
    const double dist_to_left_bound = calcLateralDistToBounds(
      ref_point_for_bound_search.pose, left_bound, soft_road_clearance, true);
    const double dist_to_right_bound = calcLateralDistToBounds(
      ref_point_for_bound_search.pose, right_bound, soft_road_clearance, false);
    ref_points.at(i).bounds = Bounds{dist_to_right_bound, dist_to_left_bound};
  }

  // keep vehicle width + margin
  // NOTE: The drivable area's width is sometimes narrower than the vehicle width which means
  // infeasible to run especially when obstacles are extracted from the drivable area.
  //       In this case, the drivable area's width is forced to be wider.
  keepMinimumBoundsWidth(ref_points);

  // extend violated bounds, where the input path is outside the drivable area
  ref_points = extendViolatedBounds(ref_points);

  // keep previous boundary's width around ego to avoid sudden steering
  avoidSuddenSteering(ref_points, ego_pose, ego_vel);

  /*
  // TODO(murooka) deal with filling data between obstacles
  // fill between obstacles
  constexpr double max_longitudinal_length_to_fill_drivable_area = 20;
  const int edge_fill_index = std::ceil(max_longitudinal_length_to_fill_drivable_area /
  mpt_param_.delta_arc_length / 2); for (int i = 0; i < ref_points.size(); ++i) { for (int j =
  -edge_fill_index; j <= edge_fill_index; ++j) { const int k = i + j; if (k < 0 || ref_points.size()
  - 1 <= k) { continue;
      }

      const auto normalized_avoidance_cost = calcNormalizedAvoidanceCost(ref_points.at(k));
      if (normalized_avoidance_cost) {
      }
    }
  }
  */

  time_keeper_ptr_->toc(__func__, "          ");
  return;
}

void MPTOptimizer::keepMinimumBoundsWidth(std::vector<ReferencePoint> & ref_points) const
{
  // calculate drivable area width considering the curvature
  std::vector<double> min_dynamic_drivable_width_vec;
  for (int i = 0; i < static_cast<int>(ref_points.size()); ++i) {
    double curvature = std::abs(ref_points.at(i).curvature);
    if (i != static_cast<int>(ref_points.size()) - 1) {
      curvature = std::max(curvature, std::abs(ref_points.at(i + 1).curvature));
    }
    if (i != 0) {
      curvature = std::max(curvature, std::abs(ref_points.at(i - 1).curvature));
    }

    const double max_longitudinal_length = std::max(
      std::abs(vehicle_info_.max_longitudinal_offset_m),
      std::abs(vehicle_info_.min_longitudinal_offset_m));
    const double turning_radius = 1.0 / curvature;
    const double additional_drivable_width_by_curvature =
      std::hypot(max_longitudinal_length, turning_radius + vehicle_info_.vehicle_width_m / 2.0) -
      turning_radius - vehicle_info_.vehicle_width_m / 2.0;
    min_dynamic_drivable_width_vec.push_back(
      mpt_param_.min_drivable_width + additional_drivable_width_by_curvature);
  }

  // 1. calculate start and end sections which are out of bounds
  std::vector<std::pair<size_t, size_t>> out_of_upper_bound_sections;
  std::vector<std::pair<size_t, size_t>> out_of_lower_bound_sections;
  std::optional<size_t> out_of_upper_bound_start_idx = std::nullopt;
  std::optional<size_t> out_of_lower_bound_start_idx = std::nullopt;
  for (size_t i = 0; i < ref_points.size(); ++i) {
    const auto & b = ref_points.at(i).bounds;

    // const double drivable_width = b.upper_bound - b.lower_bound;
    // const bool is_infeasible_to_drive = drivable_width < min_dynamic_drivable_width

    // NOTE: The following condition should be uncommented to see obstacles outside the path.
    //       However, on a narrow road, the ego may go outside the road border with this condition.
    //       Currently, we cannot distinguish obstacles and road border
    if (/*is_infeasible_to_drive ||*/ b.upper_bound < 0.0) {  // out of upper bound
      if (!out_of_upper_bound_start_idx) {
        out_of_upper_bound_start_idx = i;
      }
    } else {
      if (out_of_upper_bound_start_idx) {
        out_of_upper_bound_sections.push_back({*out_of_upper_bound_start_idx, i - 1});
        out_of_upper_bound_start_idx = std::nullopt;
      }
    }
    if (/*is_infeasible_to_drive ||*/ 0.0 < b.lower_bound) {  // out of lower bound
      if (!out_of_lower_bound_start_idx) {
        out_of_lower_bound_start_idx = i;
      }
    } else {
      if (out_of_lower_bound_start_idx) {
        out_of_lower_bound_sections.push_back({*out_of_lower_bound_start_idx, i - 1});
        out_of_lower_bound_start_idx = std::nullopt;
      }
    }
  }
  if (out_of_upper_bound_start_idx) {
    out_of_upper_bound_sections.push_back({*out_of_upper_bound_start_idx, ref_points.size() - 1});
  }
  if (out_of_lower_bound_start_idx) {
    out_of_lower_bound_sections.push_back({*out_of_lower_bound_start_idx, ref_points.size() - 1});
  }

  auto original_ref_points = ref_points;
  const auto is_inside_sections = [&](const size_t target_idx, const auto & sections) {
    for (const auto & section : sections) {
      if (section.first <= target_idx && target_idx <= section.second) {
        return true;
      }
    }
    return false;
  };

  // lower bound
  for (const auto & out_of_lower_bound_section : out_of_lower_bound_sections) {
    std::optional<size_t> upper_bound_start_idx = std::nullopt;
    std::optional<size_t> upper_bound_end_idx = std::nullopt;
    for (size_t p_idx = out_of_lower_bound_section.first;
         p_idx <= out_of_lower_bound_section.second; ++p_idx) {
      const bool is_out_of_upper_bound = is_inside_sections(p_idx, out_of_upper_bound_sections);

      const auto & original_b = original_ref_points.at(p_idx).bounds;
      auto & b = ref_points.at(p_idx).bounds;
      if (is_out_of_upper_bound) {
        if (!upper_bound_start_idx) {
          upper_bound_start_idx = p_idx;
        }
        upper_bound_end_idx = p_idx;

        // It seems both bounds are cut out. Widen the bounds towards the both side.
        const double center_dist_to_bounds =
          (original_b.upper_bound + original_b.lower_bound) / 2.0;
        b.upper_bound = std::max(
          b.upper_bound, center_dist_to_bounds + min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        b.lower_bound = std::min(
          b.lower_bound, center_dist_to_bounds - min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        continue;
      }
      // Only the Lower bound is cut out. Widen the bounds towards the lower bound since cut out too
      // much.
      b.lower_bound =
        std::min(b.lower_bound, original_b.upper_bound - min_dynamic_drivable_width_vec.at(p_idx));
      continue;
    }
    // extend longitudinal if it overlaps out_of_upper_bound_sections
    if (upper_bound_start_idx) {
      for (size_t p_idx = out_of_lower_bound_section.first; p_idx < *upper_bound_start_idx;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.lower_bound =
          std::min(b.lower_bound, ref_points.at(*upper_bound_start_idx).bounds.lower_bound);
      }
    }
    if (upper_bound_end_idx) {
      for (size_t p_idx = *upper_bound_end_idx + 1; p_idx <= out_of_lower_bound_section.second;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.lower_bound =
          std::min(b.lower_bound, ref_points.at(*upper_bound_end_idx).bounds.lower_bound);
      }
    }
  }

  // upper bound
  for (const auto & out_of_upper_bound_section : out_of_upper_bound_sections) {
    std::optional<size_t> lower_bound_start_idx = std::nullopt;
    std::optional<size_t> lower_bound_end_idx = std::nullopt;
    for (size_t p_idx = out_of_upper_bound_section.first;
         p_idx <= out_of_upper_bound_section.second; ++p_idx) {
      const bool is_out_of_lower_bound = is_inside_sections(p_idx, out_of_lower_bound_sections);

      const auto & original_b = original_ref_points.at(p_idx).bounds;
      auto & b = ref_points.at(p_idx).bounds;
      if (is_out_of_lower_bound) {
        if (!lower_bound_start_idx) {
          lower_bound_start_idx = p_idx;
        }
        lower_bound_end_idx = p_idx;

        // It seems both bounds are cut out. Widen the bounds towards the both side.
        const double center_dist_to_bounds =
          (original_b.upper_bound + original_b.lower_bound) / 2.0;
        b.upper_bound = std::max(
          b.upper_bound, center_dist_to_bounds + min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        b.lower_bound = std::min(
          b.lower_bound, center_dist_to_bounds - min_dynamic_drivable_width_vec.at(p_idx) / 2.0);
        continue;
      }
      // Only the Upper bound is cut out. Widen the bounds towards the upper bound since cut out too
      // much.
      b.upper_bound =
        std::max(b.upper_bound, original_b.lower_bound + min_dynamic_drivable_width_vec.at(p_idx));
      continue;
    }
    // extend longitudinal if it overlaps out_of_lower_bound_sections
    if (lower_bound_start_idx) {
      for (size_t p_idx = out_of_upper_bound_section.first; p_idx < *lower_bound_start_idx;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.upper_bound =
          std::max(b.upper_bound, ref_points.at(*lower_bound_start_idx).bounds.upper_bound);
      }
    }
    if (lower_bound_end_idx) {
      for (size_t p_idx = *lower_bound_end_idx + 1; p_idx <= out_of_upper_bound_section.second;
           ++p_idx) {
        auto & b = ref_points.at(p_idx).bounds;
        b.upper_bound =
          std::max(b.upper_bound, ref_points.at(*lower_bound_end_idx).bounds.upper_bound);
      }
    }
  }
}

std::vector<ReferencePoint> MPTOptimizer::extendViolatedBounds(
  const std::vector<ReferencePoint> & ref_points) const
{
  auto extended_ref_points = ref_points;
  const int max_length_idx = std::floor(
    mpt_param_.max_longitudinal_margin_for_bound_violation / mpt_param_.delta_arc_length);
  for (int i = 0; i < static_cast<int>(ref_points.size()) - 1; ++i) {
    // before violation
    if (
      ref_points.at(i).bounds.lower_bound <= 0.0 &&
      0.0 <= ref_points.at(i + 1).bounds.lower_bound) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i - j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.lower_bound = ref_points.at(i + 1).bounds.lower_bound;
      }
    }

    if (
      0.0 <= ref_points.at(i).bounds.upper_bound &&
      ref_points.at(i + 1).bounds.upper_bound <= 0.0) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i - j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.upper_bound = ref_points.at(i + 1).bounds.upper_bound;
      }
    }

    // after violation
    if (0 <= ref_points.at(i).bounds.lower_bound && ref_points.at(i + 1).bounds.lower_bound <= 0) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i + j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.lower_bound = ref_points.at(i).bounds.lower_bound;
      }
    }

    if (
      ref_points.at(i).bounds.upper_bound <= 0.0 &&
      0.0 <= ref_points.at(i + 1).bounds.upper_bound) {
      for (int j = 0; j <= max_length_idx; ++j) {
        const int k = std::clamp(i + j, 0, static_cast<int>(ref_points.size()) - 1);
        extended_ref_points.at(k).bounds.upper_bound = ref_points.at(i).bounds.upper_bound;
      }
    }
  }

  return extended_ref_points;
}

void MPTOptimizer::avoidSuddenSteering(
  std::vector<ReferencePoint> & ref_points, const geometry_msgs::msg::Pose & ego_pose,
  const double ego_vel) const
{
  if (!prev_ref_points_ptr_) {
    return;
  }
  const size_t prev_ego_idx = trajectory_utils::findEgoIndex(
    *prev_ref_points_ptr_, tier4_autoware_utils::getPose(ref_points.front()), ego_nearest_param_);

  const double max_bound_fixing_length = ego_vel * mpt_param_.max_bound_fixing_time;
  const int max_bound_fixing_idx =
    std::floor(max_bound_fixing_length / mpt_param_.delta_arc_length);

  const size_t ego_idx = trajectory_utils::findEgoIndex(ref_points, ego_pose, ego_nearest_param_);
  const size_t max_fixed_bound_idx =
    std::min(ego_idx + static_cast<size_t>(max_bound_fixing_idx), ref_points.size());

  for (size_t i = 0; i < max_fixed_bound_idx; ++i) {
    const size_t prev_idx = std::min(
      prev_ego_idx + i, static_cast<size_t>(static_cast<int>(prev_ref_points_ptr_->size()) - 1));
    const auto & prev_bounds = prev_ref_points_ptr_->at(prev_idx).bounds;

    ref_points.at(i).bounds.upper_bound = prev_bounds.upper_bound;
    ref_points.at(i).bounds.lower_bound = prev_bounds.lower_bound;
  }
}

void MPTOptimizer::updateVehicleBounds(
  std::vector<ReferencePoint> & ref_points,
  const SplineInterpolationPoints2d & ref_points_spline) const
{
  time_keeper_ptr_->tic(__func__);

  for (size_t p_idx = 0; p_idx < ref_points.size(); ++p_idx) {
    const auto & ref_point = ref_points.at(p_idx);
    // NOTE: This clear is required.
    // It seems they sometimes already have previous values.
    ref_points.at(p_idx).bounds_on_constraints.clear();
    ref_points.at(p_idx).beta.clear();

    for (const double lon_offset : vehicle_circle_longitudinal_offsets_) {
      const auto collision_check_pose =
        ref_points_spline.getSplineInterpolatedPose(p_idx, lon_offset);
      const double collision_check_yaw = tf2::getYaw(collision_check_pose.orientation);

      // calculate beta
      const double beta = ref_point.getYaw() - collision_check_yaw;
      ref_points.at(p_idx).beta.push_back(beta);

      // calculate vehicle_bounds_pose
      const double tmp_yaw = std::atan2(
        collision_check_pose.position.y - ref_point.pose.position.y,
        collision_check_pose.position.x - ref_point.pose.position.x);
      const double offset_y =
        -tier4_autoware_utils::calcDistance2d(ref_point, collision_check_pose) *
        std::sin(tmp_yaw - collision_check_yaw);

      const auto vehicle_bounds_pose =
        tier4_autoware_utils::calcOffsetPose(collision_check_pose, 0.0, offset_y, 0.0);

      // interpolate bounds
      const auto bounds = [&]() {
        const double collision_check_s = ref_points_spline.getAccumulatedLength(p_idx) + lon_offset;
        const size_t collision_check_idx = ref_points_spline.getOffsetIndex(p_idx, lon_offset);

        const size_t prev_idx = std::clamp(
          collision_check_idx - 1, static_cast<size_t>(0),
          static_cast<size_t>(ref_points_spline.getSize() - 2));
        const size_t next_idx = prev_idx + 1;

        const auto & prev_bounds = ref_points.at(prev_idx).bounds;
        const auto & next_bounds = ref_points.at(next_idx).bounds;

        const double prev_s = ref_points_spline.getAccumulatedLength(prev_idx);
        const double next_s = ref_points_spline.getAccumulatedLength(next_idx);

        const double ratio = std::clamp((collision_check_s - prev_s) / (next_s - prev_s), 0.0, 1.0);

        auto bounds = Bounds::lerp(prev_bounds, next_bounds, ratio);
        bounds.translate(offset_y);
        return bounds;
      }();

      ref_points.at(p_idx).bounds_on_constraints.push_back(bounds);
      ref_points.at(p_idx).pose_on_constraints.push_back(vehicle_bounds_pose);
    }
  }

  time_keeper_ptr_->toc(__func__, "          ");
}

// cost function: J = x' Q x + u' R u
MPTOptimizer::ValueMatrix MPTOptimizer::calcValueMatrix(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  const bool is_goal_contained = geometry_utils::isSamePoint(ref_points.back(), traj_points.back());

  // update Q
  std::vector<Eigen::Triplet<double>> Q_triplet_vec;
  for (size_t i = 0; i < N_ref; ++i) {
    const auto adaptive_error_weight = [&]() -> std::array<double, 2> {
      // for terminal point
      if (i == N_ref - 1) {
        if (is_goal_contained) {
          return {mpt_param_.goal_lat_error_weight, mpt_param_.goal_yaw_error_weight};
        }
        return {mpt_param_.terminal_lat_error_weight, mpt_param_.terminal_yaw_error_weight};
      }
      // for avoidance
      if (0 < ref_points.at(i).normalized_avoidance_cost) {
        const double lat_error_weight = interpolation::lerp(
          mpt_param_.lat_error_weight, mpt_param_.avoidance_lat_error_weight,
          ref_points.at(i).normalized_avoidance_cost);
        const double yaw_error_weight = interpolation::lerp(
          mpt_param_.yaw_error_weight, mpt_param_.avoidance_yaw_error_weight,
          ref_points.at(i).normalized_avoidance_cost);
        return {lat_error_weight, yaw_error_weight};
      }
      // normal case
      return {mpt_param_.lat_error_weight, mpt_param_.yaw_error_weight};
    }();

    const double adaptive_lat_error_weight = adaptive_error_weight.at(0);
    const double adaptive_yaw_error_weight = adaptive_error_weight.at(1);

    Q_triplet_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, adaptive_lat_error_weight));
    Q_triplet_vec.push_back(
      Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, adaptive_yaw_error_weight));
  }
  Eigen::SparseMatrix<double> Q_sparse_mat(N_x, N_x);
  Q_sparse_mat.setFromTriplets(Q_triplet_vec.begin(), Q_triplet_vec.end());

  // update R
  std::vector<Eigen::Triplet<double>> R_triplet_vec;
  for (size_t i = 0; i < N_ref - 1; ++i) {
    const double adaptive_steer_weight = interpolation::lerp(
      mpt_param_.steer_input_weight, mpt_param_.avoidance_steer_input_weight,
      ref_points.at(i).normalized_avoidance_cost);
    R_triplet_vec.push_back(Eigen::Triplet<double>(D_u * i, D_u * i, adaptive_steer_weight));
  }
  Eigen::SparseMatrix<double> R_sparse_mat(N_u, N_u);
  addSteerWeightR(R_triplet_vec, ref_points);

  R_sparse_mat.setFromTriplets(R_triplet_vec.begin(), R_triplet_vec.end());

  time_keeper_ptr_->toc(__func__, "        ");
  return ValueMatrix{Q_sparse_mat, R_sparse_mat};
}

MPTOptimizer::ObjectiveMatrix MPTOptimizer::calcObjectiveMatrix(
  [[maybe_unused]] const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & val_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  time_keeper_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_slack = getNumberOfSlackVariables();

  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_s = N_ref * N_slack;

  const size_t N_v = N_x + N_u + N_s;

  // generate T matrix and vector to shift optimization center
  // NOTE: Z is defined as time-series vector of shifted deviation
  //       error where Z = sparse_T_mat * X + T_vec
  std::vector<Eigen::Triplet<double>> triplet_T_vec;
  Eigen::VectorXd T_vec = Eigen::VectorXd::Zero(N_x);
  const double offset = mpt_param_.optimization_center_offset;
  for (size_t i = 0; i < N_ref; ++i) {
    const double alpha = ref_points.at(i).alpha;

    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x, std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x, i * D_x + 1, offset * std::cos(alpha)));
    triplet_T_vec.push_back(Eigen::Triplet<double>(i * D_x + 1, i * D_x + 1, 1.0));

    T_vec(i * D_x) = -offset * std::sin(alpha);
  }
  Eigen::SparseMatrix<double> sparse_T_mat(N_x, N_x);
  sparse_T_mat.setFromTriplets(triplet_T_vec.begin(), triplet_T_vec.end());

  // NOTE: min J(v) = min (v'Hv + v'g)
  Eigen::MatrixXd H_x = Eigen::MatrixXd::Zero(N_x, N_x);
  H_x.triangularView<Eigen::Upper>() =
    Eigen::MatrixXd(sparse_T_mat.transpose() * val_mat.Q * sparse_T_mat);
  H_x.triangularView<Eigen::Lower>() = H_x.transpose();

  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(N_v, N_v);
  H.block(0, 0, N_x, N_x) = H_x;
  H.block(N_x, N_x, N_u, N_u) = val_mat.R;

  Eigen::VectorXd g = Eigen::VectorXd::Zero(N_v);
  g.segment(0, N_x) = T_vec.transpose() * val_mat.Q * sparse_T_mat;
  g.segment(N_x + N_u, N_s) = mpt_param_.soft_collision_free_weight * Eigen::VectorXd::Ones(N_s);

  ObjectiveMatrix obj_matrix;
  obj_matrix.hessian = H;
  obj_matrix.gradient = g;

  time_keeper_ptr_->toc(__func__, "        ");
  return obj_matrix;
}

// Constraint: lb <= A u <= ub
// decision variable
// u := [initial state, steer angles, soft variables]
MPTOptimizer::ConstraintMatrix MPTOptimizer::calcConstraintMatrix(
  const StateEquationGenerator::Matrix & mpt_mat,
  const std::vector<ReferencePoint> & ref_points) const
{
  time_keeper_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;

  // NOTE: The number of one-step slack variables.
  //       The number of all slack variables will be N_ref * N_slack.
  const size_t N_slack = getNumberOfSlackVariables();

  const size_t N_v = N_x + N_u + (mpt_param_.soft_constraint ? N_ref * N_slack : 0);

  const size_t N_collision_check = vehicle_circle_longitudinal_offsets_.size();

  // calculate indices of fixed points
  std::vector<size_t> fixed_points_indices;
  for (size_t i = 0; i < N_ref; ++i) {
    if (ref_points.at(i).fixed_kinematic_state) {
      fixed_points_indices.push_back(i);
    }
  }

  // calculate rows and cols of A
  size_t A_rows = 0;
  A_rows += N_x;
  if (mpt_param_.soft_constraint) {
    // NOTE: 3 means expecting slack variable constraints to be larger than lower bound,
    //       smaller than upper bound, and positive.
    A_rows += 3 * N_ref * N_collision_check;
  }
  if (mpt_param_.hard_constraint) {
    A_rows += N_ref * N_collision_check;
  }
  A_rows += fixed_points_indices.size() * D_x;
  if (mpt_param_.steer_limit_constraint) {
    A_rows += N_u;
  }

  // NOTE: The following takes 1 [ms]
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(A_rows, N_v);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(A_rows, -autoware::common::osqp::INF);
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(A_rows, autoware::common::osqp::INF);
  size_t A_rows_end = 0;

  // 1. State equation
  A.block(0, 0, N_x, N_x) = Eigen::MatrixXd::Identity(N_x, N_x) - mpt_mat.A;
  A.block(0, N_x, N_x, N_u) = -mpt_mat.B;
  lb.segment(0, N_x) = mpt_mat.W;
  ub.segment(0, N_x) = mpt_mat.W;
  A_rows_end += N_x;

  // 2. collision free
  // CX = C(Bv + w) + C \in R^{N_ref, N_ref * D_x}
  for (size_t l_idx = 0; l_idx < N_collision_check; ++l_idx) {
    // create C := [cos(beta) | l cos(beta)]
    Eigen::SparseMatrix<double> C_sparse_mat(N_ref, N_x);
    std::vector<Eigen::Triplet<double>> C_triplet_vec;
    Eigen::VectorXd C_vec = Eigen::VectorXd::Zero(N_ref);

    // calculate C mat and vec
    for (size_t i = 0; i < N_ref; ++i) {
      const double beta = *ref_points.at(i).beta.at(l_idx);
      const double lon_offset = vehicle_circle_longitudinal_offsets_.at(l_idx);

      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x, 1.0 * std::cos(beta)));
      C_triplet_vec.push_back(Eigen::Triplet<double>(i, i * D_x + 1, lon_offset * std::cos(beta)));
      C_vec(i) = lon_offset * std::sin(beta);
    }
    C_sparse_mat.setFromTriplets(C_triplet_vec.begin(), C_triplet_vec.end());

    // calculate bounds
    const double bounds_offset =
      vehicle_info_.vehicle_width_m / 2.0 - vehicle_circle_radiuses_.at(l_idx);
    const auto & [part_ub, part_lb] = extractBounds(ref_points, l_idx, bounds_offset);

    // soft constraints
    if (mpt_param_.soft_constraint) {
      const size_t A_blk_rows = 3 * N_ref;

      // A := [C | O | ... | O | I | O | ...
      //      -C | O | ... | O | I | O | ...
      //          O    | O | ... | O | I | O | ... ]
      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, N_v);
      A_blk.block(0, 0, N_ref, N_x) = C_sparse_mat;
      A_blk.block(N_ref, 0, N_ref, N_x) = -C_sparse_mat;

      const size_t local_A_offset_cols = N_x + N_u + (!mpt_param_.l_inf_norm ? N_ref * l_idx : 0);
      A_blk.block(0, local_A_offset_cols, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);
      A_blk.block(2 * N_ref, local_A_offset_cols, N_ref, N_ref) =
        Eigen::MatrixXd::Identity(N_ref, N_ref);

      // lb := [lower_bound - C
      //        C - upper_bound
      //               O        ]
      Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(A_blk_rows);
      lb_blk.segment(0, N_ref) = -C_vec + part_lb;
      lb_blk.segment(N_ref, N_ref) = C_vec - part_ub;

      A.block(A_rows_end, 0, A_blk_rows, N_v) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = lb_blk;

      A_rows_end += A_blk_rows;
    }

    // hard constraints
    if (mpt_param_.hard_constraint) {
      const size_t A_blk_rows = N_ref;

      Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(A_blk_rows, N_v);
      A_blk.block(0, 0, N_ref, N_ref) = C_sparse_mat;

      A.block(A_rows_end, 0, A_blk_rows, N_v) = A_blk;
      lb.segment(A_rows_end, A_blk_rows) = part_lb - C_vec;
      ub.segment(A_rows_end, A_blk_rows) = part_ub - C_vec;

      A_rows_end += A_blk_rows;
    }
  }

  // 3. fixed points constraint
  // X = B v + w where point is fixed
  for (const size_t i : fixed_points_indices) {
    A.block(A_rows_end, D_x * i, D_x, D_x) = Eigen::MatrixXd::Identity(D_x, D_x);

    lb.segment(A_rows_end, D_x) = ref_points.at(i).fixed_kinematic_state->toEigenVector();
    ub.segment(A_rows_end, D_x) = ref_points.at(i).fixed_kinematic_state->toEigenVector();

    A_rows_end += D_x;
  }

  // 4. steer angle limit
  if (mpt_param_.steer_limit_constraint) {
    A.block(A_rows_end, N_x, N_u, N_u) = Eigen::MatrixXd::Identity(N_u, N_u);

    // TODO(murooka) use curvature by stabling optimization
    // Currently, when using curvature, the optimization result is weird with sample_map.
    // lb.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, -mpt_param_.max_steer_rad);
    // ub.segment(A_rows_end, N_u) = Eigen::MatrixXd::Constant(N_u, 1, mpt_param_.max_steer_rad);

    for (size_t i = 0; i < N_u; ++i) {
      const double ref_steer_angle =
        std::atan2(vehicle_info_.wheel_base_m * ref_points.at(i).curvature, 1.0);
      lb(A_rows_end + i) = ref_steer_angle - mpt_param_.max_steer_rad;
      ub(A_rows_end + i) = ref_steer_angle + mpt_param_.max_steer_rad;
    }

    A_rows_end += N_u;
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return ConstraintMatrix{A, lb, ub};
}

void MPTOptimizer::addSteerWeightR(
  std::vector<Eigen::Triplet<double>> & R_triplet_vec,
  const std::vector<ReferencePoint> & ref_points) const
{
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;

  // add steering rate : weight for (u(i) - u(i-1))^2
  for (size_t i = 0; i < N_u - 1; ++i) {
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i, mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i, i + 1, -mpt_param_.steer_rate_weight));
    R_triplet_vec.push_back(Eigen::Triplet<double>(i + 1, i + 1, mpt_param_.steer_rate_weight));
  }
}

std::optional<Eigen::VectorXd> MPTOptimizer::calcOptimizedSteerAngles(
  const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
  const ConstraintMatrix & const_mat)
{
  time_keeper_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();
  const size_t N_v = N_x + N_u + N_ref * N_slack;

  // for manual warm start, calculate initial solution
  const auto u0 = [&]() -> std::optional<Eigen::VectorXd> {
    if (mpt_param_.enable_manual_warm_start) {
      if (prev_ref_points_ptr_ && 1 < prev_ref_points_ptr_->size()) {
        return calcInitialSolutionForManualWarmStart(ref_points, *prev_ref_points_ptr_);
      }
    }
    return std::nullopt;
  }();

  // for manual start, update objective and constraint matrix
  const auto [updated_obj_mat, updated_const_mat] =
    updateMatrixForManualWarmStart(obj_mat, const_mat, u0);

  // calculate matrices for qp
  const Eigen::MatrixXd & H = updated_obj_mat.hessian;
  const Eigen::MatrixXd & A = updated_const_mat.linear;
  const auto f = toStdVector(updated_obj_mat.gradient);
  const auto upper_bound = toStdVector(updated_const_mat.upper_bound);
  const auto lower_bound = toStdVector(updated_const_mat.lower_bound);

  // initialize or update solver according to warm start
  time_keeper_ptr_->tic("initOsqp");

  const autoware::common::osqp::CSC_Matrix P_csc =
    autoware::common::osqp::calCSCMatrixTrapezoidal(H);
  const autoware::common::osqp::CSC_Matrix A_csc = autoware::common::osqp::calCSCMatrix(A);
  if (
    prev_solution_status_ == 1 && mpt_param_.enable_warm_start && prev_mat_n_ == H.rows() &&
    prev_mat_m_ == A.rows()) {
    RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "warm start");
    osqp_solver_ptr_->updateCscP(P_csc);
    osqp_solver_ptr_->updateQ(f);
    osqp_solver_ptr_->updateCscA(A_csc);
    osqp_solver_ptr_->updateL(lower_bound);
    osqp_solver_ptr_->updateU(upper_bound);
  } else {
    RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "no warm start");
    osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
      P_csc, A_csc, f, lower_bound, upper_bound, osqp_epsilon_);
  }
  prev_mat_n_ = H.rows();
  prev_mat_m_ = A.rows();

  time_keeper_ptr_->toc("initOsqp", "          ");

  // solve qp
  time_keeper_ptr_->tic("solveOsqp");
  const auto result = osqp_solver_ptr_->optimize();
  time_keeper_ptr_->toc("solveOsqp", "          ");

  // check solution status
  const int solution_status = std::get<3>(result);
  prev_solution_status_ = solution_status;
  if (solution_status != 1) {
    osqp_solver_ptr_->logUnsolvedStatus("[MPT]");
    return std::nullopt;
  }

  // print iteration
  const int iteration_status = std::get<4>(result);
  RCLCPP_INFO_EXPRESSION(logger_, enable_debug_info_, "iteration: %d", iteration_status);

  // get optimization result
  auto optimization_result =
    std::get<0>(result);  // NOTE: const cannot be added due to the next operation.
  const auto has_nan = std::any_of(
    optimization_result.begin(), optimization_result.end(),
    [](const auto v) { return std::isnan(v); });
  if (has_nan) {
    RCLCPP_WARN(logger_, "optimization failed: result contains NaN values");
    return std::nullopt;
  }
  const Eigen::VectorXd optimized_variables =
    Eigen::Map<Eigen::VectorXd>(&optimization_result[0], N_v);

  time_keeper_ptr_->toc(__func__, "        ");

  if (u0) {  // manual warm start
    return static_cast<Eigen::VectorXd>(optimized_variables + *u0);
  }
  return optimized_variables;
}

Eigen::VectorXd MPTOptimizer::calcInitialSolutionForManualWarmStart(
  const std::vector<ReferencePoint> & ref_points,
  const std::vector<ReferencePoint> & prev_ref_points) const
{
  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();
  const size_t N_ref = ref_points.size();
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t D_v = D_x + N_u;
  const size_t N_slack = getNumberOfSlackVariables();
  const size_t D_un = D_v + N_ref * N_slack;

  Eigen::VectorXd u0 = Eigen::VectorXd::Zero(D_un);

  const size_t nearest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    prev_ref_points, ref_points.front().pose, ego_nearest_param_.dist_threshold,
    ego_nearest_param_.yaw_threshold);

  // set previous lateral and yaw deviation
  u0(0) = prev_ref_points.at(nearest_idx).optimized_kinematic_state.lat;
  u0(1) = prev_ref_points.at(nearest_idx).optimized_kinematic_state.yaw;

  // set previous steer angles
  for (size_t i = 0; i < N_u; ++i) {
    const size_t prev_target_idx = std::min(nearest_idx + i, prev_ref_points.size() - 1);
    u0(D_x + i) = prev_ref_points.at(prev_target_idx).optimized_input;
  }

  // set previous slack variables
  for (size_t i = 0; i < N_ref; ++i) {
    const auto & slack_variables = ref_points.at(i).slack_variables;
    if (slack_variables) {
      for (size_t j = 0; j < slack_variables->size(); ++j) {
        u0(D_v + i * N_slack + j) = slack_variables->at(j);
      }
    }
  }

  return u0;
}

std::pair<MPTOptimizer::ObjectiveMatrix, MPTOptimizer::ConstraintMatrix>
MPTOptimizer::updateMatrixForManualWarmStart(
  const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
  const std::optional<Eigen::VectorXd> & u0) const
{
  if (!u0) {
    // not manual warm start
    return {obj_mat, const_mat};
  }

  const Eigen::MatrixXd & H = obj_mat.hessian;
  const Eigen::MatrixXd & A = const_mat.linear;

  auto updated_obj_mat = obj_mat;
  auto updated_const_mat = const_mat;

  Eigen::VectorXd & f = updated_obj_mat.gradient;
  Eigen::VectorXd & ub = updated_const_mat.upper_bound;
  Eigen::VectorXd & lb = updated_const_mat.lower_bound;

  // update gradient
  f += H * *u0;

  // update upper_bound and lower_bound
  const Eigen::VectorXd A_times_u0 = A * *u0;
  ub -= A_times_u0;
  lb -= A_times_u0;

  return {updated_obj_mat, updated_const_mat};
}

std::optional<std::vector<TrajectoryPoint>> MPTOptimizer::calcMPTPoints(
  std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & optimized_variables,
  [[maybe_unused]] const StateEquationGenerator::Matrix & mpt_mat) const
{
  time_keeper_ptr_->tic(__func__);

  const size_t D_x = state_equation_generator_.getDimX();
  const size_t D_u = state_equation_generator_.getDimU();

  const size_t N_ref = ref_points.size();
  const size_t N_x = N_ref * D_x;
  const size_t N_u = (N_ref - 1) * D_u;
  const size_t N_slack = getNumberOfSlackVariables();

  const Eigen::VectorXd states = optimized_variables.segment(0, N_x);
  const Eigen::VectorXd steer_angles = optimized_variables.segment(N_x, N_u);
  const Eigen::VectorXd slack_variables = optimized_variables.segment(N_x + N_u, N_ref * N_slack);

  // calculate trajectory points from optimization result
  std::vector<TrajectoryPoint> traj_points;
  for (size_t i = 0; i < N_ref; ++i) {
    auto & ref_point = ref_points.at(i);

    const double lat_error = states(i * D_x);
    const double yaw_error = states(i * D_x + 1);

    // validate optimization result
    if (mpt_param_.enable_optimization_validation) {
      if (
        mpt_param_.max_validation_lat_error < std::abs(lat_error) ||
        mpt_param_.max_validation_yaw_error < std::abs(yaw_error)) {
        return std::nullopt;
      }
    }

    // memorize optimization result (optimized_kinematic_state and optimized_input)
    ref_point.optimized_kinematic_state = KinematicState{lat_error, yaw_error};
    if (i == N_ref - 1) {
      ref_point.optimized_input = 0.0;
    } else {
      ref_point.optimized_input = steer_angles(i * D_u);
    }

    std::vector<double> tmp_slack_variables;
    for (size_t j = 0; j < N_slack; ++j) {
      tmp_slack_variables.push_back(slack_variables(i * N_slack + j));
    }
    ref_point.slack_variables = tmp_slack_variables;

    // update pose and velocity
    TrajectoryPoint traj_point;
    traj_point.pose = ref_point.offsetDeviation(lat_error, yaw_error);
    traj_point.longitudinal_velocity_mps = ref_point.longitudinal_velocity_mps;

    traj_points.push_back(traj_point);
  }

  time_keeper_ptr_->toc(__func__, "        ");
  return traj_points;
}

void MPTOptimizer::publishDebugTrajectories(
  const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
  const std::vector<TrajectoryPoint> & mpt_traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  // reference points
  const auto ref_traj = trajectory_utils::createTrajectory(
    header, trajectory_utils::convertToTrajectoryPoints(ref_points));
  debug_ref_traj_pub_->publish(ref_traj);

  // fixed reference points
  const auto fixed_traj_points = extractFixedPoints(ref_points);
  const auto fixed_traj = trajectory_utils::createTrajectory(header, fixed_traj_points);
  debug_fixed_traj_pub_->publish(fixed_traj);

  // mpt points
  const auto mpt_traj = trajectory_utils::createTrajectory(header, mpt_traj_points);
  debug_mpt_traj_pub_->publish(mpt_traj);

  time_keeper_ptr_->toc(__func__, "        ");
}

std::vector<TrajectoryPoint> MPTOptimizer::extractFixedPoints(
  const std::vector<ReferencePoint> & ref_points) const
{
  std::vector<TrajectoryPoint> fixed_traj_points;
  for (const auto & ref_point : ref_points) {
    if (ref_point.fixed_kinematic_state) {
      TrajectoryPoint fixed_traj_point;
      fixed_traj_point.pose = ref_point.offsetDeviation(
        ref_point.fixed_kinematic_state->lat, ref_point.fixed_kinematic_state->yaw);
      fixed_traj_points.push_back(fixed_traj_point);
    }
  }

  return fixed_traj_points;
}

double MPTOptimizer::getTrajectoryLength() const
{
  const double forward_traj_length = mpt_param_.num_points * mpt_param_.delta_arc_length;
  const double backward_traj_length = traj_param_.output_backward_traj_length;
  return forward_traj_length + backward_traj_length;
}

double MPTOptimizer::getDeltaArcLength() const
{
  return mpt_param_.delta_arc_length;
}

int MPTOptimizer::getNumberOfPoints() const
{
  return mpt_param_.num_points;
}

size_t MPTOptimizer::getNumberOfSlackVariables() const
{
  if (mpt_param_.soft_constraint) {
    if (mpt_param_.l_inf_norm) {
      return 1;
    }
    return vehicle_circle_longitudinal_offsets_.size();
  }
  return 0;
}

std::optional<double> MPTOptimizer::calcNormalizedAvoidanceCost(
  const ReferencePoint & ref_point) const
{
  const double negative_avoidance_cost = std::min(
    -ref_point.bounds.lower_bound - mpt_param_.avoidance_cost_margin,
    ref_point.bounds.upper_bound - mpt_param_.avoidance_cost_margin);
  if (0 <= negative_avoidance_cost) {
    return {};
  }
  return std::clamp(-negative_avoidance_cost / mpt_param_.max_avoidance_cost, 0.0, 1.0);
}
}  // namespace obstacle_avoidance_planner
