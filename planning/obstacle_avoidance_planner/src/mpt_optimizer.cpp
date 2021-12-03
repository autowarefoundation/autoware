// Copyright 2020 Tier IV, Inc.
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

#include "obstacle_avoidance_planner/eb_path_optimizer.hpp"
#include "obstacle_avoidance_planner/process_cv.hpp"
#include "obstacle_avoidance_planner/util.hpp"
#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include <opencv2/core.hpp>
#include <osqp_interface/osqp_interface.hpp>

#include <nav_msgs/msg/map_meta_data.hpp>

#include <boost/optional.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

MPTOptimizer::MPTOptimizer(
  const bool is_showing_debug_info, const QPParam & qp_param, const TrajectoryParam & traj_param,
  const ConstrainParam & constraint_param, const VehicleParam & vehicle_param,
  const MPTParam & mpt_param)
: is_showing_debug_info_(is_showing_debug_info)
{
  qp_param_ptr_ = std::make_unique<QPParam>(qp_param);
  traj_param_ptr_ = std::make_unique<TrajectoryParam>(traj_param);
  constraint_param_ptr_ = std::make_unique<ConstrainParam>(constraint_param);
  vehicle_param_ptr_ = std::make_unique<VehicleParam>(vehicle_param);
  mpt_param_ptr_ = std::make_unique<MPTParam>(mpt_param);

  vehicle_model_ptr_ = std::make_unique<KinematicsBicycleModel>(
    vehicle_param_ptr_->wheelbase, vehicle_param_ptr_->max_steer_rad,
    vehicle_param_ptr_->steer_tau);
}

boost::optional<MPTTrajs> MPTOptimizer::getModelPredictiveTrajectory(
  const bool enable_avoidance,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & smoothed_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, const CVMaps & maps,
  const geometry_msgs::msg::Pose & ego_pose, DebugData * debug_data)
{
  auto t_start1 = std::chrono::high_resolution_clock::now();
  if (smoothed_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_,
      "return boost::none since smoothed_points is empty");
    return boost::none;
  }

  geometry_msgs::msg::Pose origin_pose = smoothed_points.front().pose;
  if (prev_trajs) {
    const int prev_nearest_idx = util::getNearestIdx(
      prev_trajs->model_predictive_trajectory, smoothed_points.front().pose.position);
    origin_pose = prev_trajs->model_predictive_trajectory.at(prev_nearest_idx).pose;
  }
  std::vector<ReferencePoint> ref_points = getReferencePoints(
    origin_pose, ego_pose, smoothed_points, path_points, prev_trajs, maps, debug_data);
  if (ref_points.empty()) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_,
      "return boost::none since ref_points is empty");
    return boost::none;
  }

  const auto mpt_matrix = generateMPTMatrix(ref_points, path_points, prev_trajs);
  if (!mpt_matrix) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_,
      "return boost::none since matrix has nan");
    return boost::none;
  }

  const auto optimized_control_variables = executeOptimization(
    enable_avoidance, mpt_matrix.get(), ref_points, path_points, maps, debug_data);
  if (!optimized_control_variables) {
    RCLCPP_INFO_EXPRESSION(
      rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_,
      "return boost::none since could not solve qp");
    return boost::none;
  }

  const auto mpt_points =
    getMPTPoints(ref_points, optimized_control_variables.get(), mpt_matrix.get(), smoothed_points);

  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_, "MPT time: = %f [ms]", elapsed_ms1);
  MPTTrajs mpt_trajs;
  mpt_trajs.mpt = mpt_points;
  mpt_trajs.ref_points = ref_points;
  return mpt_trajs;
}

std::vector<ReferencePoint> MPTOptimizer::getReferencePoints(
  const geometry_msgs::msg::Pose & origin_pose, const geometry_msgs::msg::Pose & ego_pose,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs, const CVMaps & maps,
  DebugData * debug_data) const
{
  const auto ref_points =
    convertToReferencePoints(points, path_points, prev_trajs, ego_pose, maps, debug_data);

  const int begin_idx = util::getNearestPointIdx(ref_points, origin_pose.position);
  const auto first_it = ref_points.begin() + begin_idx;
  int num_points = std::min(
    static_cast<int>(ref_points.size()) - 1 - begin_idx, traj_param_ptr_->num_sampling_points);
  num_points = std::max(num_points, 0);
  std::vector<ReferencePoint> truncated_points(first_it, first_it + num_points);
  calcInitialState(&truncated_points, origin_pose);
  return truncated_points;
}

std::vector<ReferencePoint> MPTOptimizer::convertToReferencePoints(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs,
  [[maybe_unused]] const geometry_msgs::msg::Pose & ego_pose, [[maybe_unused]] const CVMaps & maps,
  DebugData * debug_data) const
{
  const auto interpolated_points =
    util::getInterpolatedPoints(points, traj_param_ptr_->delta_arc_length_for_mpt_points);
  if (interpolated_points.empty()) {
    return std::vector<ReferencePoint>{};
  }

  auto reference_points = getBaseReferencePoints(interpolated_points, prev_trajs, debug_data);

  calcOrientation(&reference_points);
  calcVelocity(&reference_points, points);
  calcCurvature(&reference_points);
  calcArcLength(&reference_points);
  calcExtraPoints(&reference_points);
  return reference_points;
}

void MPTOptimizer::calcOrientation(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }
  for (std::size_t i = 0; i < ref_points->size(); i++) {
    if (ref_points->at(i).fix_state) {
      continue;
    }

    if (i > 0) {
      ref_points->at(i).q =
        util::getQuaternionFromPoints(ref_points->at(i).p, ref_points->at(i - 1).p);
    } else if (i == 0 && ref_points->size() > 1) {
      ref_points->at(i).q =
        util::getQuaternionFromPoints(ref_points->at(i + 1).p, ref_points->at(i).p);
    }
    ref_points->at(i).yaw = tf2::getYaw(ref_points->at(i).q);
  }
}

void MPTOptimizer::calcVelocity(
  std::vector<ReferencePoint> * ref_points,
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points) const
{
  if (!ref_points) {
    return;
  }
  for (std::size_t i = 0; i < ref_points->size(); i++) {
    ref_points->at(i).v =
      points[util::getNearestIdx(points, ref_points->at(i).p)].longitudinal_velocity_mps;
  }
}

void MPTOptimizer::calcCurvature(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }

  int num_points = static_cast<int>(ref_points->size());

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::msg::Point p1, p2, p3;
  int max_smoothing_num = static_cast<int>(std::floor(0.5 * (num_points - 1)));
  int L = std::min(mpt_param_ptr_->num_curvature_sampling_points, max_smoothing_num);
  for (int i = L; i < num_points - L; ++i) {
    p1 = ref_points->at(i - L).p;
    p2 = ref_points->at(i).p;
    p3 = ref_points->at(i + L).p;
    double den = std::max(
      util::calculate2DDistance(p1, p2) * util::calculate2DDistance(p2, p3) *
        util::calculate2DDistance(p3, p1),
      0.0001);
    const double curvature =
      2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    if (!ref_points->at(i).fix_state) {
      ref_points->at(i).k = curvature;
    }
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < std::min(L, num_points); ++i) {
    if (!ref_points->at(i).fix_state) {
      ref_points->at(i).k = ref_points->at(std::min(L, num_points - 1)).k;
    }
    if (!ref_points->at(num_points - i - 1).fix_state) {
      ref_points->at(num_points - i - 1).k = ref_points->at(std::max(num_points - L - 1, 0)).k;
    }
  }
}

void MPTOptimizer::calcArcLength(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }
  for (std::size_t i = 0; i < ref_points->size(); i++) {
    if (i > 0) {
      geometry_msgs::msg::Point a, b;
      a = ref_points->at(i).p;
      b = ref_points->at(i - 1).p;
      ref_points->at(i).s = ref_points->at(i - 1).s + util::calculate2DDistance(a, b);
    }
  }
}

void MPTOptimizer::calcExtraPoints(std::vector<ReferencePoint> * ref_points) const
{
  if (!ref_points) {
    return;
  }
  for (std::size_t i = 0; i < ref_points->size(); i++) {
    const double p1_target_s = ref_points->at(i).s + mpt_param_ptr_->top_point_dist_from_base_link;
    const int nearest_p1_idx = util::getNearestIdx(*ref_points, p1_target_s, i);
    ref_points->at(i).top_pose.position = ref_points->at(nearest_p1_idx).p;

    const double p2_target_s = ref_points->at(i).s + mpt_param_ptr_->mid_point_dist_from_base_link;
    const int nearest_p2_idx = util::getNearestIdx(*ref_points, p2_target_s, i);
    ref_points->at(i).mid_pose.position = ref_points->at(nearest_p2_idx).p;

    ref_points->at(i).top_pose.orientation =
      util::getQuaternionFromPoints(ref_points->at(i).top_pose.position, ref_points->at(i).p);
    if (static_cast<int>(i) == nearest_p1_idx && i > 0) {
      ref_points->at(i).top_pose.orientation =
        util::getQuaternionFromPoints(ref_points->at(i).p, ref_points->at(i - 1).p);
    } else if (static_cast<int>(i) == nearest_p1_idx) {
      ref_points->at(i).top_pose.orientation = ref_points->at(i).q;
    }
    ref_points->at(i).mid_pose.orientation = ref_points->at(i).top_pose.orientation;

    const double delta_yaw =
      tf2::getYaw(ref_points->at(i).top_pose.orientation) - ref_points->at(i).yaw;
    const double norm_delta_yaw = util::normalizeRadian(delta_yaw);
    ref_points->at(i).delta_yaw_from_p1 = norm_delta_yaw;
    ref_points->at(i).delta_yaw_from_p2 = ref_points->at(i).delta_yaw_from_p1;
  }
}

void MPTOptimizer::calcInitialState(
  std::vector<ReferencePoint> * ref_points, const geometry_msgs::msg::Pose & origin_pose) const
{
  if (!ref_points) {
    return;
  }

  if (ref_points->empty()) {
    return;
  }

  boost::optional<int> begin_idx = boost::none;
  double accum_s = 0;

  for (std::size_t i = 0; i < ref_points->size(); i++) {
    double ds = 0.0;
    if (i < ref_points->size() - 1) {
      ds = ref_points->at(i + 1).s - ref_points->at(i).s;
    } else if (i == ref_points->size() - 1 && ref_points->size() > 1) {
      ds = ref_points->at(i).s - ref_points->at(i - 1).s;
    }
    accum_s += ds;
    constexpr double max_s_for_prev_point = 3;
    if (accum_s < max_s_for_prev_point && ref_points->at(i).fix_state) {
      begin_idx = i;
      break;
    }
  }

  Eigen::VectorXd initial_state;
  if (begin_idx) {
    *ref_points =
      std::vector<ReferencePoint>{ref_points->begin() + begin_idx.get(), ref_points->end()};
    ref_points->front().optimized_state = ref_points->front().fix_state.get();
  } else {
    ref_points->front().optimized_state = getState(origin_pose, ref_points->front());
  }
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
boost::optional<MPTMatrix> MPTOptimizer::generateMPTMatrix(
  const std::vector<ReferencePoint> & reference_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  const std::unique_ptr<Trajectories> & prev_trajs) const
{
  const int N = reference_points.size();
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_X);      // state transition
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_U * N);  // control input
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * N, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  Eigen::MatrixXd R1ex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd R2ex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd Uref_ex = Eigen::MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpt_param_ptr_->lat_error_weight;
  if (!prev_trajs) {
    const double initial_lat_error_weight = mpt_param_ptr_->lat_error_weight * 1000;
    Q(0, 0) = initial_lat_error_weight;
  }
  Q(1, 1) = mpt_param_ptr_->yaw_error_weight;
  R(0, 0) = mpt_param_ptr_->steer_input_weight;

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  geometry_msgs::msg::Pose last_ref_pose;
  last_ref_pose.position = reference_points.back().p;
  last_ref_pose.orientation = reference_points.back().q;
  const auto last_extended_point = util::getLastExtendedPoint(
    path_points.back(), last_ref_pose, traj_param_ptr_->delta_yaw_threshold_for_closest_point,
    traj_param_ptr_->max_dist_for_extending_end_point);

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    const double ref_k = reference_points[i].k;
    double ds = 0.0;
    if (i < N - 1) {
      ds = reference_points[i + 1].s - reference_points[i].s;
    } else if (i == N - 1 && N > 1) {
      ds = reference_points[i].s - reference_points[i - 1].s;
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(&Ad, &Bd, &Cd, &Wd, ds);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1 && last_extended_point) {
      Q_adaptive(0, 0) = mpt_param_ptr_->terminal_path_lat_error_weight;
      Q_adaptive(1, 1) = mpt_param_ptr_->terminal_path_yaw_error_weight;
    } else if (i == N - 1) {
      Q_adaptive(0, 0) = mpt_param_ptr_->terminal_lat_error_weight;
      Q_adaptive(1, 1) = mpt_param_ptr_->terminal_yaw_error_weight;
    }

    /* update mpt matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->calculateReferenceInput(&Uref);
    if (std::fabs(Uref(0, 0)) < mpt_param_ptr_->zero_ff_steer_angle * M_PI / 180.0) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    Uref_ex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  addSteerWeightR(&R1ex, reference_points);

  MPTMatrix m;
  m.Aex = Aex;
  m.Bex = Bex;
  m.Wex = Wex;
  m.Cex = Cex;
  m.Qex = Qex;
  m.R1ex = R1ex;
  m.R2ex = R2ex;
  m.Uref_ex = Uref_ex;
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Uref_ex.array().isNaN().any()) {
    RCLCPP_WARN(rclcpp::get_logger("MPTOptimizer"), "[Avoidance] MPT matrix includes NaN.");
    return boost::none;
  }
  return m;
}

void MPTOptimizer::addSteerWeightR(
  Eigen::MatrixXd * R, const std::vector<ReferencePoint> & ref_points) const
{
  const int N = ref_points.size();
  constexpr double DT = 0.1;
  constexpr double ctrl_period = 0.03;

  /* add steering rate : weight for (u(i) - u(i-1) / dt )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double steer_rate_r = mpt_param_ptr_->steer_rate_weight / (DT * DT);
    (*R)(i + 0, i + 0) += steer_rate_r;
    (*R)(i + 1, i + 0) -= steer_rate_r;
    (*R)(i + 0, i + 1) -= steer_rate_r;
    (*R)(i + 1, i + 1) += steer_rate_r;
  }
  if (N > 1) {
    // steer rate i = 0
    (*R)(0, 0) += mpt_param_ptr_->steer_rate_weight / (ctrl_period * ctrl_period);
  }

  /* add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2 */
  const double steer_acc_r = mpt_param_ptr_->steer_acc_weight / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 3) * ctrl_period);
  const double steer_acc_r_cp2 =
    mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 2) * std::pow(ctrl_period, 2));
  const double steer_acc_r_cp4 = mpt_param_ptr_->steer_acc_weight / std::pow(ctrl_period, 4);
  for (int i = 1; i < N - 1; ++i) {
    (*R)(i - 1, i - 1) += (steer_acc_r);
    (*R)(i - 1, i + 0) += (steer_acc_r * -2.0);
    (*R)(i - 1, i + 1) += (steer_acc_r);
    (*R)(i + 0, i - 1) += (steer_acc_r * -2.0);
    (*R)(i + 0, i + 0) += (steer_acc_r * 4.0);
    (*R)(i + 0, i + 1) += (steer_acc_r * -2.0);
    (*R)(i + 1, i - 1) += (steer_acc_r);
    (*R)(i + 1, i + 0) += (steer_acc_r * -2.0);
    (*R)(i + 1, i + 1) += (steer_acc_r);
  }
  if (N > 1) {
    // steer acc i = 1
    (*R)(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
    (*R)(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
    (*R)(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
    (*R)(1, 1) += steer_acc_r * 1.0;
    // steer acc i = 0
    (*R)(0, 0) += steer_acc_r_cp4 * 1.0;
  }
}

void MPTOptimizer::addSteerWeightF(Eigen::VectorXd * f) const
{
  constexpr double DT = 0.1;
  constexpr double ctrl_period = 0.03;
  constexpr double raw_steer_cmd_prev = 0;
  constexpr double raw_steer_cmd_pprev = 0;

  if (f->rows() < 2) {
    return;
  }

  // steer rate for i = 0
  (*f)(0) += -2.0 * mpt_param_ptr_->steer_rate_weight / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = mpt_param_.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 3) * ctrl_period);
  const double steer_acc_r_cp2 =
    mpt_param_ptr_->steer_acc_weight / (std::pow(DT, 2) * std::pow(ctrl_period, 2));
  const double steer_acc_r_cp4 = mpt_param_ptr_->steer_acc_weight / std::pow(ctrl_period, 4);

  // steer acc  i = 0
  (*f)(0) += ((-2.0 * raw_steer_cmd_prev + raw_steer_cmd_pprev) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  (*f)(0) += (-2.0 * raw_steer_cmd_prev * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  (*f)(1) += (2.0 * raw_steer_cmd_prev * steer_acc_r_cp1) * 0.5;
}

boost::optional<Eigen::VectorXd> MPTOptimizer::executeOptimization(
  const bool enable_avoidance, const MPTMatrix & m, const std::vector<ReferencePoint> & ref_points,
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points, const CVMaps & maps,
  DebugData * debug_data)
{
  auto t_start1 = std::chrono::high_resolution_clock::now();

  const auto x0 = ref_points.front().optimized_state;
  ObjectiveMatrix obj_m = getObjectiveMatrix(x0, m);
  ConstraintMatrix const_m =
    getConstraintMatrix(enable_avoidance, x0, m, maps, ref_points, path_points, debug_data);

  osqp_solver_ptr_ = std::make_unique<autoware::common::osqp::OSQPInterface>(
    obj_m.hessian, const_m.linear, obj_m.gradient, const_m.lower_bound, const_m.upper_bound,
    1.0e-3);
  osqp_solver_ptr_->updateEpsRel(1.0e-3);
  const auto result = osqp_solver_ptr_->optimize();

  int solution_status = std::get<3>(result);
  if (solution_status != 1) {
    util::logOSQPSolutionStatus(solution_status);
    return boost::none;
  }

  std::vector<double> result_vec = std::get<0>(result);
  const Eigen::VectorXd optimized_control_variables =
    Eigen::Map<Eigen::VectorXd>(&result_vec[0], ref_points.size());

  auto t_end1 = std::chrono::high_resolution_clock::now();
  float elapsed_ms1 = std::chrono::duration<float, std::milli>(t_end1 - t_start1).count();
  RCLCPP_INFO_EXPRESSION(
    rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_, "mpt opt time: = %f [ms]",
    elapsed_ms1);
  return optimized_control_variables;
}

double MPTOptimizer::calcLateralError(
  const geometry_msgs::msg::Point & target_point, const ReferencePoint & ref_point) const
{
  const double err_x = target_point.x - ref_point.p.x;
  const double err_y = target_point.y - ref_point.p.y;
  const double ref_yaw = tf2::getYaw(ref_point.q);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

Eigen::VectorXd MPTOptimizer::getState(
  const geometry_msgs::msg::Pose & target_pose, const ReferencePoint & nearest_ref_point) const
{
  const double lat_error = calcLateralError(target_pose.position, nearest_ref_point);
  const double yaw_error =
    util::normalizeRadian(tf2::getYaw(target_pose.orientation) - nearest_ref_point.yaw);
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(3);
  x0 << lat_error, yaw_error, std::atan(vehicle_param_ptr_->wheelbase * nearest_ref_point.k);
  return x0;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> MPTOptimizer::getMPTPoints(
  std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & Uex,
  const MPTMatrix & mpt_matrix,
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &
    optimized_points) const
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const auto x0 = ref_points.front().optimized_state;
  Eigen::VectorXd Xex = mpt_matrix.Aex * x0 + mpt_matrix.Bex * Uex + mpt_matrix.Wex;

  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> traj_points;
  {
    const double lat_error = x0(0);
    autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose.position.x = ref_points[0].p.x - std::sin(ref_points[0].yaw) * lat_error;
    traj_point.pose.position.y = ref_points[0].p.y + std::cos(ref_points[0].yaw) * lat_error;
    traj_point.longitudinal_velocity_mps = ref_points.front().v;
    traj_points.push_back(traj_point);
  }

  for (std::size_t i = 1; i < ref_points.size(); ++i) {
    const double lat_error = Xex((i - 1) * DIM_X);
    Eigen::Vector3d state = Xex.segment((i - 1) * DIM_X, DIM_X);
    setOptimizedState(&ref_points[i], state);
    autoware_auto_planning_msgs::msg::TrajectoryPoint traj_point;
    traj_point.pose.position.x = ref_points[i].p.x - std::sin(ref_points[i].yaw) * lat_error;
    traj_point.pose.position.y = ref_points[i].p.y + std::cos(ref_points[i].yaw) * lat_error;
    traj_point.longitudinal_velocity_mps = ref_points[i].v;

    traj_points.push_back(traj_point);
  }
  for (std::size_t i = 0; i < traj_points.size(); ++i) {
    if (i > 0 && traj_points.size() > 1) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i].pose.position, traj_points[i - 1].pose.position);
    } else if (traj_points.size() > 1) {
      traj_points[i].pose.orientation = util::getQuaternionFromPoints(
        traj_points[i + 1].pose.position, traj_points[i].pose.position);
    } else {
      traj_points[i].pose.orientation = ref_points[i].q;
    }
  }
  return traj_points;
}

void MPTOptimizer::setOptimizedState(
  ReferencePoint * ref_point, const Eigen::Vector3d & optimized_state) const
{
  ref_point->optimized_state = optimized_state;
}

std::vector<Bounds> MPTOptimizer::getReferenceBounds(
  const bool enable_avoidance, const std::vector<ReferencePoint> & ref_points, const CVMaps & maps,
  DebugData * debug_data) const
{
  std::vector<Bounds> ref_bounds;
  std::vector<geometry_msgs::msg::Pose> debug_bounds_candidate_for_base_points;
  std::vector<geometry_msgs::msg::Pose> debug_bounds_candidate_for_top_points;
  std::vector<geometry_msgs::msg::Pose> debug_bounds_candidate_for_mid_points;
  int cnt = 0;
  for (const auto & point : ref_points) {
    ReferencePoint ref_base_point;
    ref_base_point.p = point.p;
    ref_base_point.yaw = point.yaw;

    ReferencePoint ref_top_point;
    ref_top_point.p = point.top_pose.position;
    ref_top_point.yaw = tf2::getYaw(point.top_pose.orientation);

    ReferencePoint ref_mid_point;
    ref_mid_point.p = point.mid_pose.position;
    ref_mid_point.yaw = tf2::getYaw(point.mid_pose.orientation);

    geometry_msgs::msg::Pose debug_for_base_point;
    debug_for_base_point.position = ref_base_point.p;
    debug_for_base_point.orientation = point.q;
    debug_bounds_candidate_for_base_points.push_back(debug_for_base_point);

    geometry_msgs::msg::Pose debug_for_top_point;
    debug_for_top_point.position = ref_top_point.p;
    debug_for_top_point.orientation = point.top_pose.orientation;
    debug_bounds_candidate_for_top_points.push_back(debug_for_top_point);

    geometry_msgs::msg::Pose debug_for_mid_point;
    debug_for_mid_point.position = ref_mid_point.p;
    debug_for_mid_point.orientation = point.top_pose.orientation;
    debug_bounds_candidate_for_mid_points.push_back(debug_for_mid_point);

    if (
      !util::transformMapToOptionalImage(ref_base_point.p, maps.map_info) ||
      !util::transformMapToOptionalImage(ref_mid_point.p, maps.map_info) ||
      !util::transformMapToOptionalImage(ref_top_point.p, maps.map_info)) {
      Bounds bounds;
      bounds.c0 = {1, -1};
      bounds.c1 = {1, -1};
      bounds.c2 = {1, -1};
      ref_bounds.emplace_back(bounds);
      continue;
    }

    // Calculate boundaries.
    auto lat_bounds_0 = getBound(enable_avoidance, ref_base_point, maps);
    auto lat_bounds_1 = getBound(enable_avoidance, ref_top_point, maps);
    auto lat_bounds_2 = getBound(enable_avoidance, ref_mid_point, maps);
    if (!lat_bounds_0 || !lat_bounds_1 || !lat_bounds_2) {
      auto clock = rclcpp::Clock(RCL_ROS_TIME);
      RCLCPP_INFO_EXPRESSION(
        rclcpp::get_logger("MPTOptimizer"), is_showing_debug_info_, "Path is blocked at %i ", cnt);
      Bounds bounds;
      bounds.c0 = {1, -1};
      bounds.c1 = {1, -1};
      bounds.c2 = {1, -1};
      ref_bounds.emplace_back(bounds);
      cnt++;
      continue;
    }
    Bounds bounds;
    bounds.c0 = lat_bounds_0.get();
    bounds.c1 = lat_bounds_1.get();
    bounds.c2 = lat_bounds_2.get();
    ref_bounds.emplace_back(bounds);
    cnt++;
  }
  debug_data->bounds = ref_bounds;
  debug_data->bounds_candidate_for_base_points = debug_bounds_candidate_for_base_points;
  debug_data->bounds_candidate_for_top_points = debug_bounds_candidate_for_top_points;
  debug_data->bounds_candidate_for_mid_points = debug_bounds_candidate_for_mid_points;
  return ref_bounds;
}

boost::optional<std::vector<double>> MPTOptimizer::getBound(
  const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps) const
{
  const auto rough_road_bound = getRoughBound(enable_avoidance, ref_point, maps);

  if (!rough_road_bound) {
    return boost::none;
  }

  std::vector<double> broad_bound;
  // initialize right bound with rough left bound
  double rel_right_bound = rough_road_bound.get()[0];
  auto t_start = std::chrono::high_resolution_clock::now();
  float elapsed_ms = 0.0;
  while (true) {
    const auto bound_candidate = getBoundCandidate(
      enable_avoidance, ref_point, maps, mpt_param_ptr_->clearance_from_road,
      mpt_param_ptr_->clearance_from_object, rel_right_bound, rough_road_bound.get());

    if (!bound_candidate) {
      break;
    }

    if (broad_bound.empty()) {
      broad_bound = bound_candidate.get();
    } else {
      const double bound_candidate_diff =
        std::fabs(bound_candidate.get()[0] - bound_candidate.get()[1]);
      const double broad_bound_diff = std::fabs(broad_bound[0] - broad_bound[1]);
      if (bound_candidate_diff > broad_bound_diff) {
        broad_bound = bound_candidate.get();
      }
    }
    rel_right_bound = bound_candidate.get()[1];
    auto t_end = std::chrono::high_resolution_clock::now();
    elapsed_ms = std::chrono::duration<float, std::milli>(t_end - t_start).count();
    constexpr float max_ms = 10;
    if (elapsed_ms > max_ms) {
      // ROS_WARN_THROTTLE(1.0, "take too much time for calculating bound");
      return boost::none;
    }
  }

  if (broad_bound.empty()) {
    return boost::none;
  } else {
    return broad_bound;
  }
}

boost::optional<std::vector<double>> MPTOptimizer::getBoundCandidate(
  const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps,
  const double min_road_clearance, const double min_obj_clearance, const double rel_initial_lat,
  const std::vector<double> & rough_road_bound) const
{
  const bool search_expanding_side = true;
  const auto left_bound = getValidLatError(
    enable_avoidance, ref_point, rel_initial_lat, maps, min_road_clearance, min_obj_clearance,
    rough_road_bound, search_expanding_side);
  if (!left_bound) {
    return boost::none;
  }
  constexpr double min_valid_lat_error = 0.1;
  const double initial_right_bound = left_bound.get() - min_valid_lat_error;
  const auto right_bound = getValidLatError(
    enable_avoidance, ref_point, initial_right_bound, maps, min_road_clearance, min_obj_clearance,
    rough_road_bound);
  if (!right_bound) {
    return boost::none;
  }
  if (std::fabs(left_bound.get() - right_bound.get()) < 1e-5) {
    return boost::none;
  }
  std::vector<double> bound{left_bound.get(), right_bound.get()};
  return bound;
}

boost::optional<std::vector<double>> MPTOptimizer::getRoughBound(
  const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps) const
{
  double left_bound = 0;
  double right_bound = 0;
  const double left_angle = util::normalizeRadian(ref_point.yaw + M_PI_2);
  const double right_angle = util::normalizeRadian(ref_point.yaw - M_PI_2);

  geometry_msgs::msg::Point new_position;
  new_position.x = ref_point.p.x;
  new_position.y = ref_point.p.y;
  auto original_clearance = getClearance(maps.clearance_map, new_position, maps.map_info);
  auto original_object_clearance =
    getClearance(maps.only_objects_clearance_map, new_position, maps.map_info);
  if (!original_clearance || !original_object_clearance) {
    return boost::none;
  }
  if (!enable_avoidance) {
    original_object_clearance.emplace(std::numeric_limits<double>::max());
  }
  constexpr double min_road_clearance = 0.1;
  constexpr double min_obj_clearance = 0.1;
  if (
    original_clearance.get() > min_road_clearance &&
    original_object_clearance.get() > min_obj_clearance) {
    const double initial_dist = 0;
    right_bound = -1 * getTraversedDistance(
                         enable_avoidance, ref_point, right_angle, initial_dist, maps,
                         min_road_clearance, min_obj_clearance);
    left_bound = getTraversedDistance(
      enable_avoidance, ref_point, left_angle, initial_dist, maps, min_road_clearance,
      min_obj_clearance);
  } else {
    const double initial_dist = 0;
    const bool search_expanding_side = true;
    const double right_s = getTraversedDistance(
      enable_avoidance, ref_point, right_angle, initial_dist, maps, min_road_clearance,
      min_obj_clearance, search_expanding_side);
    const double left_s = getTraversedDistance(
      enable_avoidance, ref_point, left_angle, initial_dist, maps, min_road_clearance,
      min_obj_clearance, search_expanding_side);
    if (left_s < right_s) {
      // Pick left side:
      right_bound = left_s;
      left_bound = getTraversedDistance(
        enable_avoidance, ref_point, left_angle, left_s, maps, min_road_clearance,
        min_obj_clearance);
    } else {
      // Pick right side:
      left_bound = -right_s;
      right_bound = -getTraversedDistance(
        enable_avoidance, ref_point, right_angle, right_s, maps, min_road_clearance,
        min_obj_clearance);
    }
  }
  if (std::fabs(left_bound - right_bound) < 1e-6) {
    return boost::none;
  }
  std::vector<double> bound{left_bound, right_bound};
  return bound;
}

boost::optional<double> MPTOptimizer::getClearance(
  const cv::Mat & clearance_map, const geometry_msgs::msg::Point & map_point,
  const nav_msgs::msg::MapMetaData & map_info) const
{
  const auto image_point = util::transformMapToOptionalImage(map_point, map_info);
  if (!image_point) {
    return boost::none;
  }
  const float clearance = clearance_map.ptr<float>(static_cast<int>(
                            image_point.get().y))[static_cast<int>(image_point.get().x)] *
                          map_info.resolution;
  return clearance;
}

ObjectiveMatrix MPTOptimizer::getObjectiveMatrix(
  const Eigen::VectorXd & x0, const MPTMatrix & m) const
{
  const int DIM_U_N = m.Uref_ex.rows();
  const Eigen::MatrixXd CB = m.Cex * m.Bex;
  const Eigen::MatrixXd QCB = m.Qex * CB;
  // Eigen::MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex;
  // This calculation is heavy. looking for a good way.
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  Eigen::VectorXd f =
    (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Uref_ex.transpose() * m.R1ex;
  addSteerWeightF(&f);

  constexpr int num_lat_constraint = 3;
  const int num_objective_variables = DIM_U_N * (1 + num_lat_constraint);
  Eigen::MatrixXd extend_h = Eigen::MatrixXd::Zero(DIM_U_N, DIM_U_N);
  Eigen::VectorXd extend_f = Eigen::VectorXd::Ones(DIM_U_N);
  Eigen::MatrixXd concat_h =
    Eigen::MatrixXd::Zero(num_objective_variables, num_objective_variables);
  Eigen::VectorXd concat_f = Eigen::VectorXd::Zero(num_objective_variables);
  concat_h.block(0, 0, DIM_U_N, DIM_U_N) = H;
  concat_h.block(DIM_U_N, DIM_U_N, DIM_U_N, DIM_U_N) = extend_h;
  concat_h.block(DIM_U_N * 2, DIM_U_N * 2, DIM_U_N, DIM_U_N) = extend_h;
  concat_h.block(DIM_U_N * 3, DIM_U_N * 3, DIM_U_N, DIM_U_N) = extend_h;
  concat_f << f, mpt_param_ptr_->base_point_weight * extend_f,
    mpt_param_ptr_->top_point_weight * extend_f, mpt_param_ptr_->mid_point_weight * extend_f;
  ObjectiveMatrix obj_matrix;
  obj_matrix.hessian = concat_h;
  obj_matrix.gradient = {concat_f.data(), concat_f.data() + concat_f.rows()};

  return obj_matrix;
}

// Set constraint: lb <= Ax <= ub
// decision variable
// x := [u0, ..., uN-1 | z00, ..., z0N-1 | z10, ..., z1N-1 | z20, ..., z2N-1]
//   \in \mathbb{R}^{N * (N_point + 1)}
ConstraintMatrix MPTOptimizer::getConstraintMatrix(
  const bool enable_avoidance, const Eigen::VectorXd & x0, const MPTMatrix & m, const CVMaps & maps,
  const std::vector<ReferencePoint> & ref_points,
  [[maybe_unused]] const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
  DebugData * debug_data) const
{
  std::vector<double> dist_vec{
    mpt_param_ptr_->base_point_dist_from_base_link, mpt_param_ptr_->top_point_dist_from_base_link,
    mpt_param_ptr_->mid_point_dist_from_base_link};

  const size_t N_ref = m.Uref_ex.rows();
  const size_t N_state = vehicle_model_ptr_->getDimX();
  const size_t N_point = dist_vec.size();
  const size_t N_dec = N_ref * (N_point + 1);

  const auto bounds = getReferenceBounds(enable_avoidance, ref_points, maps, debug_data);

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * N_ref * N_point + N_ref, N_dec);
  Eigen::VectorXd lb =
    Eigen::VectorXd::Constant(3 * N_ref * N_point + N_ref, -autoware::common::osqp::INF);
  Eigen::VectorXd ub =
    Eigen::VectorXd::Constant(3 * N_ref * N_point + N_ref, autoware::common::osqp::INF);

  // Define constraint matrices and vectors
  // Gap from reference point around vehicle base_link
  {
    // C := [I | O | O]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      C(i, N_state * i) = 1;
    }
    // bias := Cast * (Aex * x0 + Wex)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);
    // A_blk := [C * Bex | I | O | O
    //          -C * Bex | I | O | O
    //               O   | I | O | O]
    Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(3 * N_ref, N_dec);
    A_blk.block(0, 0, N_ref, N_ref) = C * m.Bex;
    A_blk.block(0, N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(N_ref, 0, N_ref, N_ref) = -C * m.Bex;
    A_blk.block(N_ref, N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(2 * N_ref, N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    // lb_blk := [-bias + bounds.lb
    //             bias - bounds.ub
    //             0]
    Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(3 * N_ref);
    for (size_t i = 0; i < N_ref; ++i) {
      if (i == N_ref - 1) {
        lb_blk(i) = -bias(i) + bounds[i].c0.lb;
        lb_blk(N_ref + i) = bias(i) - bounds[i].c0.ub;
      } else {
        lb_blk(i) = -bias(i) + bounds[i + 1].c0.lb;
        lb_blk(N_ref + i) = bias(i) - bounds[i + 1].c0.ub;
      }
    }
    // Assign
    A.block(0, 0, 3 * N_ref, N_dec) = A_blk;
    lb.segment(0, 3 * N_ref) = lb_blk;
  }

  // Gap from reference point around vehicle top
  {
    // C := diag([cos(alpha1) | l1*cos(alpha1) | 0])
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      Eigen::MatrixXd Cast = Eigen::MatrixXd::Zero(1, N_state);
      if (i == N_ref - 1) {
        Cast(0, 0) = std::cos(ref_points[i].delta_yaw_from_p1);
        Cast(0, 1) = dist_vec[1] * std::cos(ref_points[i].delta_yaw_from_p1);
      } else {
        Cast(0, 0) = std::cos(ref_points[i + 1].delta_yaw_from_p1);
        Cast(0, 1) = dist_vec[1] * std::cos(ref_points[i + 1].delta_yaw_from_p1);
      }
      C.block(i, N_state * i, 1, N_state) = Cast;
    }
    // bias := Cast * (Aex * x0 + Wex) - l1 * sin(alpha1)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);
    for (size_t i = 0; i < N_ref; ++i) {
      if (i == N_ref - 1) {
        bias(i) -= dist_vec[1] * std::sin(ref_points[i].delta_yaw_from_p1);
      } else {
        bias(i) -= dist_vec[1] * std::sin(ref_points[i + 1].delta_yaw_from_p1);
      }
    }
    // A_blk := [C * Bex | O | I | O
    //          -C * Bex | O | I | O
    //               O   | O | I | O]
    Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(3 * N_ref, N_dec);
    A_blk.block(0, 0, N_ref, N_ref) = C * m.Bex;
    A_blk.block(0, 2 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(N_ref, 0, N_ref, N_ref) = -C * m.Bex;
    A_blk.block(N_ref, 2 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(2 * N_ref, 2 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    // lb_blk := [-bias + bounds.lb
    //             bias - bounds.ub
    //             0]
    Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(3 * N_ref);
    for (size_t i = 0; i < N_ref; ++i) {
      if (i == N_ref - 1) {
        lb_blk(i) = -bias(i) + bounds[i].c1.lb;
        lb_blk(N_ref + i) = bias(i) - bounds[i].c1.ub;
      } else {
        lb_blk(i) = -bias(i) + bounds[i + 1].c1.lb;
        lb_blk(N_ref + i) = bias(i) - bounds[i + 1].c1.ub;
      }
    }
    // Assign
    A.block(3 * N_ref, 0, 3 * N_ref, N_dec) = A_blk;
    lb.segment(3 * N_ref, 3 * N_ref) = lb_blk;
  }
  // Gap from reference point around vehicle middle
  {
    // C := [diag(cos(alpha2)) | diag(l2*cos(alpha2)) | O]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      Eigen::MatrixXd Cast = Eigen::MatrixXd::Zero(1, N_state);
      if (i == N_ref - 1) {
        Cast(0, 0) = std::cos(ref_points[i].delta_yaw_from_p2);
        Cast(0, 1) = dist_vec[2] * std::cos(ref_points[i].delta_yaw_from_p2);
      } else {
        Cast(0, 0) = std::cos(ref_points[i + 1].delta_yaw_from_p2);
        Cast(0, 1) = dist_vec[2] * std::cos(ref_points[i + 1].delta_yaw_from_p2);
      }
      C.block(i, N_state * i, 1, N_state) = Cast;
    }
    // bias := Cast * (Aex * x0 + Wex) - l2 * sin(alpha2)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);
    for (size_t i = 0; i < N_ref; ++i) {
      if (i == N_ref - 1) {
        bias(i) -= dist_vec[2] * std::sin(ref_points[i].delta_yaw_from_p2);
      } else {
        bias(i) -= dist_vec[2] * std::sin(ref_points[i + 1].delta_yaw_from_p2);
      }
    }
    // A_blk := [C * Bex | O | O | I
    //          -C * Bex | O | O | I
    //               O   | O | O | I]
    Eigen::MatrixXd A_blk = Eigen::MatrixXd::Zero(3 * N_ref, N_dec);
    A_blk.block(0, 0, N_ref, N_ref) = C * m.Bex;
    A_blk.block(0, 3 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(N_ref, 0, N_ref, N_ref) = -C * m.Bex;
    A_blk.block(N_ref, 3 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    A_blk.block(2 * N_ref, 3 * N_ref, N_ref, N_ref) = Eigen::MatrixXd::Identity(N_ref, N_ref);
    // lb_blk := [-bias + bounds.lb
    //             bias - bounds.ub
    //             0]
    Eigen::VectorXd lb_blk = Eigen::VectorXd::Zero(3 * N_ref);
    for (size_t i = 0; i < N_ref; ++i) {
      if (i == N_ref - 1) {
        lb_blk(i) = -bias(i) + bounds[i].c2.lb;
        lb_blk(N_ref + i) = bias(i) - bounds[i].c2.ub;
      } else {
        lb_blk(i) = -bias(i) + bounds[i + 1].c2.lb;
        lb_blk(N_ref + i) = bias(i) - bounds[i + 1].c2.ub;
      }
    }
    // Assign
    A.block(2 * 3 * N_ref, 0, 3 * N_ref, N_dec) = A_blk;
    lb.segment(2 * 3 * N_ref, 3 * N_ref) = lb_blk;
  }

  // Fixed points constraint
  {
    // C := [I | O | O]
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(N_ref, N_ref * N_state);
    for (size_t i = 0; i < N_ref; ++i) {
      C(i, N_state * i) = 1;
    }
    // bias := Cast * (Aex * x0 + Wex)
    Eigen::VectorXd bias = C * (m.Aex * x0 + m.Wex);

    // Assign
    A.block(3 * N_point * N_ref, 0, N_ref, N_ref) = C * m.Bex;
    for (size_t i = 0; i < N_ref; ++i) {
      if (ref_points[i + 1].fix_state && i + 1 < N_ref) {
        lb(3 * N_point * N_ref + i) = ref_points[i + 1].fix_state.get()(0) - bias(i);
        ub(3 * N_point * N_ref + i) = ref_points[i + 1].fix_state.get()(0) - bias(i);
      } else if (i == ref_points.size() - 1 && mpt_param_ptr_->is_hard_fixing_terminal_point) {
        lb(3 * N_point * N_ref + i) = -bias(i);
        ub(3 * N_point * N_ref + i) = -bias(i);
      }
    }
  }

  ConstraintMatrix constraint_matrix;

  constraint_matrix.linear = A;

  for (int i = 0; i < lb.size(); ++i) {
    constraint_matrix.lower_bound.push_back(lb(i));
    constraint_matrix.upper_bound.push_back(ub(i));
  }

  return constraint_matrix;
}

std::vector<ReferencePoint> MPTOptimizer::getBaseReferencePoints(
  const std::vector<geometry_msgs::msg::Point> & interpolated_points,
  const std::unique_ptr<Trajectories> & prev_trajs, DebugData * debug_data) const
{
  std::vector<ReferencePoint> reference_points;
  for (const auto & e : interpolated_points) {
    ReferencePoint ref_point;
    ref_point.p = e;
    reference_points.push_back(ref_point);
  }
  if (!prev_trajs) {
    return reference_points;
  }
  if (prev_trajs->model_predictive_trajectory.size() != prev_trajs->mpt_ref_points.size()) {
    return reference_points;
  }

  // re-calculating points' position for fixing
  std::vector<geometry_msgs::msg::Point> cropped_interpolated_points;
  double accum_s_for_interpolated = 0;
  for (std::size_t i = 0; i < interpolated_points.size(); i++) {
    if (i > 0) {
      accum_s_for_interpolated +=
        util::calculate2DDistance(interpolated_points[i], interpolated_points[i - 1]);
    }
    if (
      accum_s_for_interpolated >
      traj_param_ptr_->num_sampling_points * traj_param_ptr_->delta_arc_length_for_mpt_points) {
      break;
    }
    cropped_interpolated_points.push_back(interpolated_points[i]);
  }
  constexpr double fine_resolution = 0.005;
  std::vector<geometry_msgs::msg::Point> fine_interpolated_points;
  fine_interpolated_points =
    util::getInterpolatedPoints(cropped_interpolated_points, fine_resolution);
  const double max_s =
    traj_param_ptr_->backward_fixing_distance + traj_param_ptr_->forward_fixing_mpt_distance;
  const int num_points = std::min(
    static_cast<int>(reference_points.size()),
    static_cast<int>(max_s / traj_param_ptr_->delta_arc_length_for_mpt_points));

  for (int i = 0; i < num_points; i++) {
    const int nearest_prev_idx =
      util::getNearestPointIdx(prev_trajs->mpt_ref_points, reference_points[i].p);
    if (
      util::calculate2DDistance(
        prev_trajs->mpt_ref_points[nearest_prev_idx].p, reference_points[i].p) >=
      std::fabs(traj_param_ptr_->delta_arc_length_for_mpt_points)) {
      continue;
    }
    const int nearest_idx =
      util::getNearestIdx(fine_interpolated_points, prev_trajs->mpt_ref_points[nearest_prev_idx].p);
    if (
      util::calculate2DDistance(
        fine_interpolated_points[nearest_idx], prev_trajs->mpt_ref_points[nearest_prev_idx].p) <
      fine_resolution) {
      reference_points[i] = prev_trajs->mpt_ref_points[nearest_prev_idx];
      reference_points[i].fix_state = prev_trajs->mpt_ref_points[nearest_prev_idx].optimized_state;
    }
  }

  std::vector<ReferencePoint> trimmed_reference_points;
  for (std::size_t i = 0; i < reference_points.size(); i++) {
    if (i > 0) {
      const double dx = reference_points[i].p.x - reference_points[i - 1].p.x;
      const double dy = reference_points[i].p.y - reference_points[i - 1].p.y;
      if (std::fabs(dx) < 1e-6 && std::fabs(dy) < 1e-6) {
        continue;
      }
    }
    trimmed_reference_points.push_back(reference_points[i]);
  }

  for (const auto & e : trimmed_reference_points) {
    if (e.fix_state) {
      geometry_msgs::msg::Point rel_point;
      rel_point.y = e.fix_state.get()(0);
      geometry_msgs::msg::Pose origin;
      origin.position = e.p;
      origin.orientation = e.q;
      geometry_msgs::msg::Pose debug_pose;
      debug_pose.position = util::transformToAbsoluteCoordinate2D(rel_point, origin);
      debug_data->fixed_mpt_points.push_back(debug_pose);
      continue;
    }
  }
  return trimmed_reference_points;
}

double MPTOptimizer::getTraversedDistance(
  const bool enable_avoidance, const ReferencePoint & ref_point, const double traverse_angle,
  const double initial_value, const CVMaps & maps, const double min_road_clearance,
  const double min_obj_clearance, const bool search_expanding_side) const
{
  constexpr double ds = 0.1;
  constexpr double lane_width = 7.5;
  int n = static_cast<int>(lane_width / ds);

  double traversed_dist = initial_value;
  for (int i = 0; i < n; ++i) {
    traversed_dist += ds;
    geometry_msgs::msg::Point new_position;
    new_position.x = ref_point.p.x + traversed_dist * std::cos(traverse_angle);
    new_position.y = ref_point.p.y + traversed_dist * std::sin(traverse_angle);
    auto clearance = getClearance(maps.clearance_map, new_position, maps.map_info);
    auto obj_clearance = getClearance(maps.only_objects_clearance_map, new_position, maps.map_info);
    if (!clearance || !obj_clearance) {
      clearance.emplace(0);
      obj_clearance.emplace(0);
    }
    if (!enable_avoidance) {
      obj_clearance.emplace(std::numeric_limits<double>::max());
    }
    if (search_expanding_side) {
      if (clearance.get() > min_road_clearance && obj_clearance.get() > min_obj_clearance) {
        break;
      }
    } else {
      if (clearance.get() < min_road_clearance || obj_clearance.get() < min_obj_clearance) {
        break;
      }
    }
  }
  return traversed_dist;
}

boost::optional<double> MPTOptimizer::getValidLatError(
  const bool enable_avoidance, const ReferencePoint & ref_point, const double initial_value,
  const CVMaps & maps, const double min_road_clearance, const double min_obj_clearance,
  const std::vector<double> & rough_road_bound, const bool search_expanding_side) const
{
  constexpr double ds = 0.01;
  constexpr double lane_width = 7.5;
  int n = static_cast<int>(lane_width / ds);

  double rel_value = initial_value;
  for (int i = 0; i < n; ++i) {
    rel_value -= ds;
    if (rel_value > rough_road_bound[0] || rel_value < rough_road_bound[1]) {
      return boost::none;
    }
    geometry_msgs::msg::Point rel_point;
    rel_point.y = rel_value;
    geometry_msgs::msg::Pose origin;
    origin.position = ref_point.p;
    origin.orientation = util::getQuaternionFromYaw(ref_point.yaw);
    const auto new_position = util::transformToAbsoluteCoordinate2D(rel_point, origin);
    auto clearance = getClearance(maps.clearance_map, new_position, maps.map_info);
    auto obj_clearance = getClearance(maps.only_objects_clearance_map, new_position, maps.map_info);
    if (!clearance || !obj_clearance) {
      return boost::none;
    }
    if (!enable_avoidance) {
      obj_clearance.emplace(std::numeric_limits<double>::max());
    }
    if (search_expanding_side) {
      if (clearance.get() > min_road_clearance && obj_clearance.get() > min_obj_clearance) {
        break;
      }
    } else {
      if (clearance.get() < min_road_clearance || obj_clearance.get() < min_obj_clearance) {
        break;
      }
    }
  }
  return rel_value;
}
