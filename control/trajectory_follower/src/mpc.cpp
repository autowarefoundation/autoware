// Copyright 2018-2021 The Autoware Foundation
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

#include "trajectory_follower/mpc.hpp"

#include "motion_utils/motion_utils.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using namespace std::literals::chrono_literals;

bool MPC::calculateMPC(
  const autoware_auto_vehicle_msgs::msg::SteeringReport & current_steer,
  const double current_velocity, const geometry_msgs::msg::Pose & current_pose,
  autoware_auto_control_msgs::msg::AckermannLateralCommand & ctrl_cmd,
  autoware_auto_planning_msgs::msg::Trajectory & predicted_traj,
  tier4_debug_msgs::msg::Float32MultiArrayStamped & diagnostic)
{
  /* recalculate velocity from ego-velocity with dynamics */
  trajectory_follower::MPCTrajectory reference_trajectory =
    applyVelocityDynamicsFilter(m_ref_traj, current_pose, current_velocity);

  MPCData mpc_data;
  if (!getData(reference_trajectory, current_steer, current_pose, &mpc_data)) {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 1000 /*ms*/, "fail to get Data.");
    return false;
  }

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = getInitialState(mpc_data);

  /* delay compensation */

  if (!updateStateForDelayCompensation(reference_trajectory, mpc_data.nearest_time, &x0)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, 1000 /*ms*/, "updateStateForDelayCompensation failed. stop computation.");
    return false;
  }

  /* resample ref_traj with mpc sampling time */
  trajectory_follower::MPCTrajectory mpc_resampled_ref_traj;
  const double mpc_start_time = mpc_data.nearest_time + m_param.input_delay;
  const double prediction_dt =
    getPredictionDeltaTime(mpc_start_time, reference_trajectory, current_pose);
  if (!resampleMPCTrajectoryByTime(
        mpc_start_time, prediction_dt, reference_trajectory, &mpc_resampled_ref_traj)) {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 1000 /*ms*/, "trajectory resampling failed.");
    return false;
  }

  /* generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex */
  MPCMatrix mpc_matrix = generateMPCMatrix(mpc_resampled_ref_traj, prediction_dt);

  /* solve quadratic optimization */
  Eigen::VectorXd Uex;
  if (!executeOptimization(mpc_matrix, x0, prediction_dt, &Uex)) {
    RCLCPP_WARN_THROTTLE(m_logger, *m_clock, 1000 /*ms*/, "optimization failed.");
    return false;
  }

  /* apply saturation and filter */
  const double u_saturated = std::max(std::min(Uex(0), m_steer_lim), -m_steer_lim);
  const double u_filtered = m_lpf_steering_cmd.filter(u_saturated);

  /* set control command */
  {
    ctrl_cmd.steering_tire_angle = static_cast<float>(u_filtered);
    ctrl_cmd.steering_tire_rotation_rate = static_cast<float>(calcDesiredSteeringRate(
      mpc_matrix, x0, Uex, u_filtered, current_steer.steering_tire_angle, prediction_dt));
  }

  storeSteerCmd(u_filtered);

  /* save input to buffer for delay compensation*/
  m_input_buffer.push_back(ctrl_cmd.steering_tire_angle);
  m_input_buffer.pop_front();
  m_raw_steer_cmd_pprev = m_raw_steer_cmd_prev;
  m_raw_steer_cmd_prev = Uex(0);

  /* calculate predicted trajectory */
  Eigen::VectorXd Xex = mpc_matrix.Aex * x0 + mpc_matrix.Bex * Uex + mpc_matrix.Wex;
  trajectory_follower::MPCTrajectory mpc_predicted_traj;
  const auto & traj = mpc_resampled_ref_traj;
  for (size_t i = 0; i < static_cast<size_t>(m_param.prediction_horizon); ++i) {
    const int DIM_X = m_vehicle_model_ptr->getDimX();
    const double lat_error = Xex(static_cast<int>(i) * DIM_X);
    const double yaw_error = Xex(static_cast<int>(i) * DIM_X + 1);
    const double x = traj.x[i] - std::sin(traj.yaw[i]) * lat_error;
    const double y = traj.y[i] + std::cos(traj.yaw[i]) * lat_error;
    const double z = traj.z[i];
    const double yaw = traj.yaw[i] + yaw_error;
    const double vx = traj.vx[i];
    const double k = traj.k[i];
    const double smooth_k = traj.smooth_k[i];
    const double relative_time = traj.relative_time[i];
    mpc_predicted_traj.push_back(x, y, z, yaw, vx, k, smooth_k, relative_time);
  }
  trajectory_follower::MPCUtils::convertToAutowareTrajectory(mpc_predicted_traj, predicted_traj);

  /* prepare diagnostic message */
  const double nearest_k = reference_trajectory.k[static_cast<size_t>(mpc_data.nearest_idx)];
  const double nearest_smooth_k =
    reference_trajectory.smooth_k[static_cast<size_t>(mpc_data.nearest_idx)];
  const double steer_cmd = ctrl_cmd.steering_tire_angle;
  const double wb = m_vehicle_model_ptr->getWheelbase();

  typedef decltype(diagnostic.data)::value_type DiagnosticValueType;
  auto append_diag_data = [&](const auto & val) -> void {
    diagnostic.data.push_back(static_cast<DiagnosticValueType>(val));
  };
  // [0] final steering command (MPC + LPF)
  append_diag_data(steer_cmd);
  // [1] mpc calculation result
  append_diag_data(Uex(0));
  // [2] feedforward steering value
  append_diag_data(mpc_matrix.Uref_ex(0));
  // [3] feedforward steering value raw
  append_diag_data(std::atan(nearest_smooth_k * wb));
  // [4] current steering angle
  append_diag_data(mpc_data.steer);
  // [5] lateral error
  append_diag_data(mpc_data.lateral_err);
  // [6] current_pose yaw
  append_diag_data(tf2::getYaw(current_pose.orientation));
  // [7] nearest_pose yaw
  append_diag_data(tf2::getYaw(mpc_data.nearest_pose.orientation));
  // [8] yaw error
  append_diag_data(mpc_data.yaw_err);
  // [9] reference velocity
  append_diag_data(reference_trajectory.vx[static_cast<size_t>(mpc_data.nearest_idx)]);
  // [10] measured velocity
  append_diag_data(current_velocity);
  // [11] angvel from steer command
  append_diag_data(current_velocity * tan(steer_cmd) / wb);
  // [12] angvel from measured steer
  append_diag_data(current_velocity * tan(mpc_data.steer) / wb);
  // [13] angvel from path curvature
  append_diag_data(current_velocity * nearest_smooth_k);
  // [14] nearest path curvature (used for feedforward)
  append_diag_data(nearest_smooth_k);
  // [15] nearest path curvature (not smoothed)
  append_diag_data(nearest_k);
  // [16] predicted steer
  append_diag_data(mpc_data.predicted_steer);
  // [17] angvel from predicted steer
  append_diag_data(current_velocity * tan(mpc_data.predicted_steer) / wb);

  return true;
}

void MPC::setReferenceTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory & trajectory_msg,
  const double traj_resample_dist, const bool enable_path_smoothing,
  const int path_filter_moving_ave_num, const int curvature_smoothing_num_traj,
  const int curvature_smoothing_num_ref_steer)
{
  trajectory_follower::MPCTrajectory mpc_traj_raw;        // received raw trajectory
  trajectory_follower::MPCTrajectory mpc_traj_resampled;  // resampled trajectory
  trajectory_follower::MPCTrajectory mpc_traj_smoothed;   // smooth filtered trajectory

  /* resampling */
  trajectory_follower::MPCUtils::convertToMPCTrajectory(trajectory_msg, mpc_traj_raw);
  if (!trajectory_follower::MPCUtils::resampleMPCTrajectoryByDistance(
        mpc_traj_raw, traj_resample_dist, &mpc_traj_resampled)) {
    RCLCPP_WARN(m_logger, "[setReferenceTrajectory] spline error when resampling by distance");
    return;
  }

  const auto is_forward_shift =
    motion_utils::isDrivingForward(mpc_traj_resampled.toTrajectoryPoints());
  // if driving direction is unknown, use previous value
  m_is_forward_shift = is_forward_shift ? is_forward_shift.get() : m_is_forward_shift;

  /* path smoothing */
  mpc_traj_smoothed = mpc_traj_resampled;
  const int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (enable_path_smoothing && mpc_traj_resampled_size > 2 * path_filter_moving_ave_num) {
    if (
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num, mpc_traj_smoothed.x) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num, mpc_traj_smoothed.y) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num, mpc_traj_smoothed.yaw) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num, mpc_traj_smoothed.vx)) {
      RCLCPP_DEBUG(m_logger, "path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /* calculate yaw angle */
  trajectory_follower::MPCUtils::calcTrajectoryYawFromXY(&mpc_traj_smoothed, m_is_forward_shift);
  trajectory_follower::MPCUtils::convertEulerAngleToMonotonic(&mpc_traj_smoothed.yaw);

  /* calculate curvature */
  trajectory_follower::MPCUtils::calcTrajectoryCurvature(
    static_cast<size_t>(curvature_smoothing_num_traj),
    static_cast<size_t>(curvature_smoothing_num_ref_steer), &mpc_traj_smoothed);

  /* add end point with vel=0 on traj for mpc prediction */
  {
    auto & t = mpc_traj_smoothed;
    const double t_ext = 100.0;  // extra time to prevent mpc calc failure due to short time
    const double t_end = t.relative_time.back() + t_ext;
    const double v_end = 0.0;
    t.vx.back() = v_end;  // set for end point
    t.push_back(
      t.x.back(), t.y.back(), t.z.back(), t.yaw.back(), v_end, t.k.back(), t.smooth_k.back(),
      t_end);
  }

  if (!mpc_traj_smoothed.size()) {
    RCLCPP_DEBUG(m_logger, "path callback: trajectory size is undesired.");
    return;
  }

  m_ref_traj = mpc_traj_smoothed;
}

void MPC::resetPrevResult(const autoware_auto_vehicle_msgs::msg::SteeringReport & current_steer)
{
  m_raw_steer_cmd_prev = current_steer.steering_tire_angle;
  m_raw_steer_cmd_pprev = current_steer.steering_tire_angle;
}

bool MPC::getData(
  const trajectory_follower::MPCTrajectory & traj,
  const autoware_auto_vehicle_msgs::msg::SteeringReport & current_steer,
  const geometry_msgs::msg::Pose & current_pose, MPCData * data)
{
  static constexpr auto duration = 5000 /*ms*/;
  size_t nearest_idx;
  if (!trajectory_follower::MPCUtils::calcNearestPoseInterp(
        traj, current_pose, &(data->nearest_pose), &(nearest_idx), &(data->nearest_time),
        ego_nearest_dist_threshold, ego_nearest_yaw_threshold, m_logger, *m_clock)) {
    // reset previous MPC result
    // Note: When a large deviation from the trajectory occurs, the optimization stops and
    // the vehicle will return to the path by re-planning the trajectory or external operation.
    // After the recovery, the previous value of the optimization may deviate greatly from
    // the actual steer angle, and it may make the optimization result unstable.
    resetPrevResult(current_steer);
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, duration, "calculateMPC: error in calculating nearest pose. stop mpc.");
    return false;
  }

  /* get data */
  data->nearest_idx = static_cast<int>(nearest_idx);
  data->steer = static_cast<double>(current_steer.steering_tire_angle);
  data->lateral_err =
    trajectory_follower::MPCUtils::calcLateralError(current_pose, data->nearest_pose);
  data->yaw_err = autoware::common::helper_functions::wrap_angle(
    tf2::getYaw(current_pose.orientation) - tf2::getYaw(data->nearest_pose.orientation));

  /* get predicted steer */
  if (!m_steer_prediction_prev) {
    m_steer_prediction_prev = std::make_shared<double>(current_steer.steering_tire_angle);
  }
  data->predicted_steer = calcSteerPrediction();
  *m_steer_prediction_prev = data->predicted_steer;

  /* check error limit */
  const double dist_err = autoware::common::geometry::distance_2d<double>(
    current_pose.position, data->nearest_pose.position);
  if (dist_err > m_admissible_position_error) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, duration, "position error is over limit. error = %fm, limit: %fm",
      dist_err, m_admissible_position_error);
    return false;
  }

  /* check yaw error limit */
  if (std::fabs(data->yaw_err) > m_admissible_yaw_error_rad) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, duration, "yaw error is over limit. error = %f deg, limit %f deg",
      RAD2DEG * data->yaw_err, RAD2DEG * m_admissible_yaw_error_rad);
    return false;
  }

  /* check trajectory time length */
  const double max_prediction_time =
    m_param.min_prediction_length / static_cast<double>(m_param.prediction_horizon - 1);
  auto end_time = data->nearest_time + m_param.input_delay + m_ctrl_period + max_prediction_time;
  if (end_time > traj.relative_time.back()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, 1000 /*ms*/, "path is too short for prediction.");
    return false;
  }
  return true;
}

double MPC::calcSteerPrediction()
{
  auto t_start = m_time_prev;
  auto t_end = m_clock->now();
  m_time_prev = t_end;

  const double duration = (t_end - t_start).seconds();
  const double time_constant = m_param.steer_tau;

  const double initial_response = std::exp(-duration / time_constant) * (*m_steer_prediction_prev);

  if (m_ctrl_cmd_vec.size() <= 2) {
    return initial_response;
  }

  return initial_response + getSteerCmdSum(t_start, t_end, time_constant);
}

double MPC::getSteerCmdSum(
  const rclcpp::Time & t_start, const rclcpp::Time & t_end, const double time_constant) const
{
  if (m_ctrl_cmd_vec.size() <= 2) {
    return 0.0;
  }

  // Find first index of control command container
  size_t idx = 1;
  while (t_start > rclcpp::Time(m_ctrl_cmd_vec.at(idx).stamp)) {
    if ((idx + 1) >= m_ctrl_cmd_vec.size()) {
      return 0.0;
    }
    ++idx;
  }

  // Compute steer command input response
  double steer_sum = 0.0;
  auto t = t_start;
  while (t_end > rclcpp::Time(m_ctrl_cmd_vec.at(idx).stamp)) {
    const double duration = (rclcpp::Time(m_ctrl_cmd_vec.at(idx).stamp) - t).seconds();
    t = rclcpp::Time(m_ctrl_cmd_vec.at(idx).stamp);
    steer_sum += (1 - std::exp(-duration / time_constant)) *
                 static_cast<double>(m_ctrl_cmd_vec.at(idx - 1).steering_tire_angle);
    ++idx;
    if (idx >= m_ctrl_cmd_vec.size()) {
      break;
    }
  }

  const double duration = (t_end - t).seconds();
  steer_sum += (1 - std::exp(-duration / time_constant)) *
               static_cast<double>(m_ctrl_cmd_vec.at(idx - 1).steering_tire_angle);

  return steer_sum;
}

void MPC::storeSteerCmd(const double steer)
{
  const auto time_delayed = m_clock->now() + rclcpp::Duration::from_seconds(m_param.input_delay);
  autoware_auto_control_msgs::msg::AckermannLateralCommand cmd;
  cmd.stamp = time_delayed;
  cmd.steering_tire_angle = static_cast<float>(steer);

  // store published ctrl cmd
  m_ctrl_cmd_vec.emplace_back(cmd);

  if (m_ctrl_cmd_vec.size() <= 2) {
    return;
  }

  // remove unused ctrl cmd
  constexpr double store_time = 0.3;
  if ((time_delayed - m_ctrl_cmd_vec.at(1).stamp).seconds() > m_param.input_delay + store_time) {
    m_ctrl_cmd_vec.erase(m_ctrl_cmd_vec.begin());
  }
}

bool MPC::resampleMPCTrajectoryByTime(
  const double ts, const double prediction_dt, const trajectory_follower::MPCTrajectory & input,
  trajectory_follower::MPCTrajectory * output) const
{
  std::vector<double> mpc_time_v;
  for (double i = 0; i < static_cast<double>(m_param.prediction_horizon); ++i) {
    mpc_time_v.push_back(ts + i * prediction_dt);
  }
  if (!trajectory_follower::MPCUtils::linearInterpMPCTrajectory(
        input.relative_time, input, mpc_time_v, output)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, 1000 /*ms*/,
      "calculateMPC: mpc resample error. stop mpc calculation. check code!");
    return false;
  }
  return true;
}

Eigen::VectorXd MPC::getInitialState(const MPCData & data)
{
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);

  const auto & lat_err = data.lateral_err;
  const auto & steer = m_use_steer_prediction ? data.predicted_steer : data.steer;
  const auto & yaw_err = data.yaw_err;

  if (m_vehicle_model_type == "kinematics") {
    x0 << lat_err, yaw_err, steer;
  } else if (m_vehicle_model_type == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (m_vehicle_model_type == "dynamics") {
    double dlat = (lat_err - m_lateral_error_prev) / m_ctrl_period;
    double dyaw = (yaw_err - m_yaw_error_prev) / m_ctrl_period;
    m_lateral_error_prev = lat_err;
    m_yaw_error_prev = yaw_err;
    dlat = m_lpf_lateral_error.filter(dlat);
    dyaw = m_lpf_yaw_error.filter(dyaw);
    x0 << lat_err, dlat, yaw_err, dyaw;
    RCLCPP_DEBUG(m_logger, "(before lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
    RCLCPP_DEBUG(m_logger, "(after lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
  } else {
    RCLCPP_ERROR(m_logger, "vehicle_model_type is undefined");
  }
  return x0;
}

bool MPC::updateStateForDelayCompensation(
  const trajectory_follower::MPCTrajectory & traj, const double & start_time, Eigen::VectorXd * x)
{
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);

  Eigen::MatrixXd x_curr = *x;
  double mpc_curr_time = start_time;
  for (uint i = 0; i < m_input_buffer.size(); ++i) {
    double k = 0.0;
    double v = 0.0;
    if (
      !trajectory_follower::linearInterpolate(traj.relative_time, traj.k, mpc_curr_time, k) ||
      !trajectory_follower::linearInterpolate(traj.relative_time, traj.vx, mpc_curr_time, v)) {
      RCLCPP_ERROR(
        m_logger, "mpc resample error at delay compensation, stop mpc calculation. check code!");
      return false;
    }

    /* get discrete state matrix A, B, C, W */
    m_vehicle_model_ptr->setVelocity(v);
    m_vehicle_model_ptr->setCurvature(k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, m_ctrl_period);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = m_input_buffer.at(i);  // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += m_ctrl_period;
  }
  *x = x_curr;
  return true;
}

trajectory_follower::MPCTrajectory MPC::applyVelocityDynamicsFilter(
  const trajectory_follower::MPCTrajectory & input, const geometry_msgs::msg::Pose & current_pose,
  const double v0) const
{
  autoware_auto_planning_msgs::msg::Trajectory autoware_traj;
  autoware::motion::control::trajectory_follower::MPCUtils::convertToAutowareTrajectory(
    input, autoware_traj);
  if (autoware_traj.points.empty()) {
    return input;
  }

  const size_t nearest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const double acc_lim = m_param.acceleration_limit;
  const double tau = m_param.velocity_time_constant;

  trajectory_follower::MPCTrajectory output = input;
  trajectory_follower::MPCUtils::dynamicSmoothingVelocity(
    static_cast<size_t>(nearest_idx), v0, acc_lim, tau, output);
  const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
  const double t_end = output.relative_time.back() + t_ext;
  const double v_end = 0.0;
  output.vx.back() = v_end;  // set for end point
  output.push_back(
    output.x.back(), output.y.back(), output.z.back(), output.yaw.back(), v_end, output.k.back(),
    output.smooth_k.back(), t_end);
  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCMatrix MPC::generateMPCMatrix(
  const trajectory_follower::MPCTrajectory & reference_trajectory, const double prediction_dt)
{
  using Eigen::MatrixXd;

  const int N = m_param.prediction_horizon;
  const double DT = prediction_dt;
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  MPCMatrix m;
  m.Aex = MatrixXd::Zero(DIM_X * N, DIM_X);
  m.Bex = MatrixXd::Zero(DIM_X * N, DIM_U * N);
  m.Wex = MatrixXd::Zero(DIM_X * N, 1);
  m.Cex = MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  m.Qex = MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  m.R1ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.R2ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.Uref_ex = MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  MatrixXd Q = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R = MatrixXd::Zero(DIM_U, DIM_U);
  MatrixXd Q_adaptive = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R_adaptive = MatrixXd::Zero(DIM_U, DIM_U);

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);
  MatrixXd Uref(DIM_U, 1);

  const double sign_vx = m_is_forward_shift ? 1 : -1;

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    const double ref_vx = reference_trajectory.vx[static_cast<size_t>(i)];
    const double ref_vx_squared = ref_vx * ref_vx;

    const double ref_k = reference_trajectory.k[static_cast<size_t>(i)] * sign_vx;
    const double ref_smooth_k = reference_trajectory.smooth_k[static_cast<size_t>(i)] * sign_vx;

    /* get discrete state matrix A, B, C, W */
    m_vehicle_model_ptr->setVelocity(ref_vx);
    m_vehicle_model_ptr->setCurvature(ref_k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
    R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
    Q(0, 0) = getWeightLatError(ref_k);
    Q(1, 1) = getWeightHeadingError(ref_k);
    R(0, 0) = getWeightSteerInput(ref_k);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1) {
      Q_adaptive(0, 0) = m_param.weight_terminal_lat_error;
      Q_adaptive(1, 1) = m_param.weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * getWeightHeadingErrorSqVel(ref_k);
    R_adaptive(0, 0) += ref_vx_squared * getWeightSteerInputSqVel(ref_k);

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      m.Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      m.Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      m.Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      m.Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * m.Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        m.Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * m.Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      m.Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * m.Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    m.Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    m.Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    m.Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    m.R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    m_vehicle_model_ptr->setCurvature(ref_smooth_k);
    m_vehicle_model_ptr->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < DEG2RAD * m_param.zero_ff_steer_deg) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    m.Uref_ex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double ref_vx = reference_trajectory.vx[static_cast<size_t>(i)];
    const double ref_k = reference_trajectory.k[static_cast<size_t>(i)] * sign_vx;
    const double j = ref_vx * ref_vx * getWeightLatJerk(ref_k) / (DT * DT);
    const Eigen::Matrix2d J = (Eigen::Matrix2d() << j, -j, -j, j).finished();
    m.R2ex.block(i, i, 2, 2) += J;
  }

  addSteerWeightR(prediction_dt, &m.R1ex);

  return m;
}

/*
 * solve quadratic optimization.
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 *                , Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 * constraint matrix : lb < U < ub, lbA < A*U < ubA
 * current considered constraint
 *  - steering limit
 *  - steering rate limit
 *
 * (1)lb < u < ub && (2)lbA < Au < ubA --> (3)[lb, lbA] < [I, A]u < [ub, ubA]
 * (1)lb < u < ub ...
 * [-u_lim] < [ u0 ] < [u_lim]
 * [-u_lim] < [ u1 ] < [u_lim]
 *              ~~~
 * [-u_lim] < [ uN ] < [u_lim] (*N... DIM_U)
 * (2)lbA < Au < ubA ...
 * [prev_u0 - au_lim*ctp] < [   u0  ] < [prev_u0 + au_lim*ctp] (*ctp ... ctrl_period)
 * [    -au_lim * dt    ] < [u1 - u0] < [     au_lim * dt    ]
 * [    -au_lim * dt    ] < [u2 - u1] < [     au_lim * dt    ]
 *                            ~~~
 * [    -au_lim * dt    ] < [uN-uN-1] < [     au_lim * dt    ] (*N... DIM_U)
 */
bool MPC::executeOptimization(
  const MPCMatrix & m, const Eigen::VectorXd & x0, const double prediction_dt,
  Eigen::VectorXd * Uex)
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  if (!isValid(m)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, 1000 /*ms*/, "model matrix is invalid. stop MPC.");
    return false;
  }

  const int DIM_U_N = m_param.prediction_horizon * m_vehicle_model_ptr->getDimU();

  // cost function: 1/2 * Uex' * H * Uex + f' * Uex,  H = B' * C' * Q * C * B + R
  const MatrixXd CB = m.Cex * m.Bex;
  const MatrixXd QCB = m.Qex * CB;
  // MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for
  // a good way.  //NOLINT
  MatrixXd H = MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  MatrixXd f = (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Uref_ex.transpose() * m.R1ex;
  addSteerWeightF(prediction_dt, &f);

  MatrixXd A = MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  VectorXd lb = VectorXd::Constant(DIM_U_N, -m_steer_lim);  // min steering angle
  VectorXd ub = VectorXd::Constant(DIM_U_N, m_steer_lim);   // max steering angle
  VectorXd lbA = VectorXd::Constant(DIM_U_N, -m_steer_rate_lim * prediction_dt);
  VectorXd ubA = VectorXd::Constant(DIM_U_N, m_steer_rate_lim * prediction_dt);
  lbA(0, 0) = m_raw_steer_cmd_prev - m_steer_rate_lim * m_ctrl_period;
  ubA(0, 0) = m_raw_steer_cmd_prev + m_steer_rate_lim * m_ctrl_period;

  auto t_start = std::chrono::system_clock::now();
  bool solve_result = m_qpsolver_ptr->solve(H, f.transpose(), A, lb, ub, lbA, ubA, *Uex);
  auto t_end = std::chrono::system_clock::now();
  if (!solve_result) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(m_logger, *m_clock, 1000 /*ms*/, "qp solver error");
    return false;
  }

  {
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    RCLCPP_DEBUG(m_logger, "qp solver calculation time = %ld [ms]", t);
  }

  if (Uex->array().isNaN().any()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      m_logger, *m_clock, 1000 /*ms*/, "model Uex includes NaN, stop MPC.");
    return false;
  }
  return true;
}

void MPC::addSteerWeightR(const double prediction_dt, Eigen::MatrixXd * R_ptr) const
{
  const int N = m_param.prediction_horizon;
  const double DT = prediction_dt;

  auto & R = *R_ptr;

  /* add steering rate : weight for (u(i) - u(i-1) / dt )^2 */
  {
    const double steer_rate_r = m_param.weight_steer_rate / (DT * DT);
    const Eigen::Matrix2d D = steer_rate_r * (Eigen::Matrix2d() << 1.0, -1.0, -1.0, 1.0).finished();
    for (int i = 0; i < N - 1; ++i) {
      R.block(i, i, 2, 2) += D;
    }
    if (N > 1) {
      // steer rate i = 0
      R(0, 0) += m_param.weight_steer_rate / (m_ctrl_period * m_ctrl_period);
    }
  }

  /* add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2 */
  {
    const double w = m_param.weight_steer_acc;
    const double steer_acc_r = w / std::pow(DT, 4);
    const double steer_acc_r_cp1 = w / (std::pow(DT, 3) * m_ctrl_period);
    const double steer_acc_r_cp2 = w / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
    const double steer_acc_r_cp4 = w / std::pow(m_ctrl_period, 4);
    const Eigen::Matrix3d D =
      steer_acc_r *
      (Eigen::Matrix3d() << 1.0, -2.0, 1.0, -2.0, 4.0, -2.0, 1.0, -2.0, 1.0).finished();
    for (int i = 1; i < N - 1; ++i) {
      R.block(i - 1, i - 1, 3, 3) += D;
    }
    if (N > 1) {
      // steer acc i = 1
      R(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
      R(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(1, 1) += steer_acc_r * 1.0;
      // steer acc i = 0
      R(0, 0) += steer_acc_r_cp4 * 1.0;
    }
  }
}

void MPC::addSteerWeightF(const double prediction_dt, Eigen::MatrixXd * f_ptr) const
{
  if (f_ptr->rows() < 2) {
    return;
  }

  const double DT = prediction_dt;
  auto & f = *f_ptr;

  // steer rate for i = 0
  f(0, 0) += -2.0 * m_param.weight_steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = m_param.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = m_param.weight_steer_acc / (std::pow(DT, 3) * m_ctrl_period);
  const double steer_acc_r_cp2 =
    m_param.weight_steer_acc / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
  const double steer_acc_r_cp4 = m_param.weight_steer_acc / std::pow(m_ctrl_period, 4);

  // steer acc  i = 0
  f(0, 0) += ((-2.0 * m_raw_steer_cmd_prev + m_raw_steer_cmd_pprev) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  f(0, 0) += (-2.0 * m_raw_steer_cmd_prev * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  f(0, 1) += (2.0 * m_raw_steer_cmd_prev * steer_acc_r_cp1) * 0.5;
}

double MPC::getPredictionDeltaTime(
  const double start_time, const trajectory_follower::MPCTrajectory & input,
  const geometry_msgs::msg::Pose & current_pose) const
{
  // Calculate the time min_prediction_length ahead from current_pose
  autoware_auto_planning_msgs::msg::Trajectory autoware_traj;
  autoware::motion::control::trajectory_follower::MPCUtils::convertToAutowareTrajectory(
    input, autoware_traj);
  const size_t nearest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  double sum_dist = 0;
  const double target_time = [&]() {
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    for (size_t i = nearest_idx + 1; i < input.relative_time.size(); i++) {
      const double segment_dist =
        std::hypot(input.x.at(i) - input.x.at(i - 1), input.y.at(i) - input.y.at(i - 1));
      sum_dist += segment_dist;
      if (m_param.min_prediction_length < sum_dist) {
        const double prev_sum_dist = sum_dist - segment_dist;
        const double ratio = (m_param.min_prediction_length - prev_sum_dist) / segment_dist;
        const double relative_time_at_i = i == input.relative_time.size() - 1
                                            ? input.relative_time.at(i) - t_ext
                                            : input.relative_time.at(i);
        return input.relative_time.at(i - 1) +
               (relative_time_at_i - input.relative_time.at(i - 1)) * ratio;
      }
    }
    return input.relative_time.back() - t_ext;
  }();

  // Calculate delta time for min_prediction_length
  const double dt =
    (target_time - start_time) / static_cast<double>(m_param.prediction_horizon - 1);

  return std::max(dt, m_param.prediction_dt);
}

double MPC::calcDesiredSteeringRate(
  const MPCMatrix & mpc_matrix, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
  const double u_filtered, const float current_steer, const double predict_dt) const
{
  if (m_vehicle_model_type != "kinematics") {
    // not supported yet. Use old implementation.
    return (u_filtered - current_steer) / predict_dt;
  }

  // calculate predicted states to get the steering motion
  const auto & m = mpc_matrix;
  const Eigen::MatrixXd Xex = m.Aex * x0 + m.Bex * Uex + m.Wex;

  const size_t STEER_IDX = 2;  // for kinematics model

  const auto steer_0 = x0(STEER_IDX, 0);
  const auto steer_1 = Xex(STEER_IDX, 0);

  const auto steer_rate = (steer_1 - steer_0) / predict_dt;

  return steer_rate;
}

bool MPC::isValid(const MPCMatrix & m) const
{
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Uref_ex.array().isNaN().any()) {
    return false;
  }

  if (
    m.Aex.array().isInf().any() || m.Bex.array().isInf().any() || m.Cex.array().isInf().any() ||
    m.Wex.array().isInf().any() || m.Qex.array().isInf().any() || m.R1ex.array().isInf().any() ||
    m.R2ex.array().isInf().any() || m.Uref_ex.array().isInf().any()) {
    return false;
  }

  return true;
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
