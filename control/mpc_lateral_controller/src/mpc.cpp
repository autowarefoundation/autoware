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

#include "mpc_lateral_controller/mpc.hpp"

#include "interpolation/linear_interpolation.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "mpc_lateral_controller/mpc_utils.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <algorithm>
#include <limits>

namespace autoware::motion::control::mpc_lateral_controller
{
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::normalizeRadian;
using tier4_autoware_utils::rad2deg;

bool MPC::calculateMPC(
  const SteeringReport & current_steer, const Odometry & current_kinematics,
  AckermannLateralCommand & ctrl_cmd, Trajectory & predicted_trajectory,
  Float32MultiArrayStamped & diagnostic)
{
  // since the reference trajectory does not take into account the current velocity of the ego
  // vehicle, it needs to calculate the trajectory velocity considering the longitudinal dynamics.
  const auto reference_trajectory =
    applyVelocityDynamicsFilter(m_reference_trajectory, current_kinematics);

  // get the necessary data
  const auto [success_data, mpc_data] =
    getData(reference_trajectory, current_steer, current_kinematics);
  if (!success_data) {
    return fail_warn_throttle("fail to get MPC Data. Stop MPC.");
  }

  // calculate initial state of the error dynamics
  const auto x0 = getInitialState(mpc_data);

  // apply time delay compensation to the initial state
  const auto [success_delay, x0_delayed] =
    updateStateForDelayCompensation(reference_trajectory, mpc_data.nearest_time, x0);
  if (!success_delay) {
    return fail_warn_throttle("delay compensation failed. Stop MPC.");
  }

  // resample reference trajectory with mpc sampling time
  const double mpc_start_time = mpc_data.nearest_time + m_param.input_delay;
  const double prediction_dt =
    getPredictionDeltaTime(mpc_start_time, reference_trajectory, current_kinematics);

  const auto [success_resample, mpc_resampled_ref_trajectory] =
    resampleMPCTrajectoryByTime(mpc_start_time, prediction_dt, reference_trajectory);
  if (!success_resample) {
    return fail_warn_throttle("trajectory resampling failed. Stop MPC.");
  }

  // generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex
  const auto mpc_matrix = generateMPCMatrix(mpc_resampled_ref_trajectory, prediction_dt);

  // solve Optimization problem
  const auto [success_opt, Uex] = executeOptimization(
    mpc_matrix, x0_delayed, prediction_dt, mpc_resampled_ref_trajectory,
    current_kinematics.twist.twist.linear.x);
  if (!success_opt) {
    return fail_warn_throttle("optimization failed. Stop MPC.");
  }

  // apply filters for the input limitation and low pass filter
  const double u_saturated = std::clamp(Uex(0), -m_steer_lim, m_steer_lim);
  const double u_filtered = m_lpf_steering_cmd.filter(u_saturated);

  // set control command
  ctrl_cmd.steering_tire_angle = static_cast<float>(u_filtered);
  ctrl_cmd.steering_tire_rotation_rate = static_cast<float>(calcDesiredSteeringRate(
    mpc_matrix, x0_delayed, Uex, u_filtered, current_steer.steering_tire_angle, prediction_dt));

  // save the control command for the steering prediction
  m_steering_predictor->storeSteerCmd(u_filtered);

  // save input to buffer for delay compensation
  m_input_buffer.push_back(ctrl_cmd.steering_tire_angle);
  m_input_buffer.pop_front();

  // save previous input for the mpc rate limit
  m_raw_steer_cmd_pprev = m_raw_steer_cmd_prev;
  m_raw_steer_cmd_prev = Uex(0);

  // calculate predicted trajectory
  predicted_trajectory =
    calcPredictedTrajectory(mpc_resampled_ref_trajectory, mpc_matrix, x0_delayed, Uex);

  // prepare diagnostic message
  diagnostic =
    generateDiagData(reference_trajectory, mpc_data, mpc_matrix, ctrl_cmd, Uex, current_kinematics);

  return true;
}

Trajectory MPC::calcPredictedTrajectory(
  const MPCTrajectory & mpc_resampled_ref_trajectory, const MPCMatrix & mpc_matrix,
  const VectorXd & x0_delayed, const VectorXd & Uex) const
{
  const VectorXd Xex = mpc_matrix.Aex * x0_delayed + mpc_matrix.Bex * Uex + mpc_matrix.Wex;
  MPCTrajectory mpc_predicted_traj;
  const auto & traj = mpc_resampled_ref_trajectory;
  for (int i = 0; i < m_param.prediction_horizon; ++i) {
    const int DIM_X = m_vehicle_model_ptr->getDimX();
    const double lat_error = Xex(i * DIM_X);
    const double yaw_error = Xex(i * DIM_X + 1);
    const double x = traj.x.at(i) - std::sin(traj.yaw.at(i)) * lat_error;
    const double y = traj.y.at(i) + std::cos(traj.yaw.at(i)) * lat_error;
    const double z = traj.z.at(i);
    const double yaw = traj.yaw.at(i) + yaw_error;
    const double vx = traj.vx.at(i);
    const double k = traj.k.at(i);
    const double smooth_k = traj.smooth_k.at(i);
    const double relative_time = traj.relative_time.at(i);
    mpc_predicted_traj.push_back(x, y, z, yaw, vx, k, smooth_k, relative_time);
  }
  return MPCUtils::convertToAutowareTrajectory(mpc_predicted_traj);
}

Float32MultiArrayStamped MPC::generateDiagData(
  const MPCTrajectory & reference_trajectory, const MPCData & mpc_data,
  const MPCMatrix & mpc_matrix, const AckermannLateralCommand & ctrl_cmd, const VectorXd & Uex,
  const Odometry & current_kinematics) const
{
  Float32MultiArrayStamped diagnostic;

  // prepare diagnostic message
  const double nearest_k = reference_trajectory.k.at(mpc_data.nearest_idx);
  const double nearest_smooth_k = reference_trajectory.smooth_k.at(mpc_data.nearest_idx);
  const double wb = m_vehicle_model_ptr->getWheelbase();
  const double current_velocity = current_kinematics.twist.twist.linear.x;
  const double wz_predicted = current_velocity * std::tan(mpc_data.predicted_steer) / wb;
  const double wz_measured = current_velocity * std::tan(mpc_data.steer) / wb;
  const double wz_command = current_velocity * std::tan(ctrl_cmd.steering_tire_angle) / wb;
  const int iteration_num = m_qpsolver_ptr->getTakenIter();
  const double runtime = m_qpsolver_ptr->getRunTime();
  const double objective_value = m_qpsolver_ptr->getObjVal();

  typedef decltype(diagnostic.data)::value_type DiagnosticValueType;
  const auto append_diag = [&](const auto & val) -> void {
    diagnostic.data.push_back(static_cast<DiagnosticValueType>(val));
  };
  append_diag(ctrl_cmd.steering_tire_angle);      // [0] final steering command (MPC + LPF)
  append_diag(Uex(0));                            // [1] mpc calculation result
  append_diag(mpc_matrix.Uref_ex(0));             // [2] feed-forward steering value
  append_diag(std::atan(nearest_smooth_k * wb));  // [3] feed-forward steering value raw
  append_diag(mpc_data.steer);                    // [4] current steering angle
  append_diag(mpc_data.lateral_err);              // [5] lateral error
  append_diag(tf2::getYaw(current_kinematics.pose.pose.orientation));  // [6] current_pose yaw
  append_diag(tf2::getYaw(mpc_data.nearest_pose.orientation));         // [7] nearest_pose yaw
  append_diag(mpc_data.yaw_err);                                       // [8] yaw error
  append_diag(reference_trajectory.vx.at(mpc_data.nearest_idx));       // [9] reference velocity
  append_diag(current_velocity);                                       // [10] measured velocity
  append_diag(wz_command);                           // [11] angular velocity from steer command
  append_diag(wz_measured);                          // [12] angular velocity from measured steer
  append_diag(current_velocity * nearest_smooth_k);  // [13] angular velocity from path curvature
  append_diag(nearest_smooth_k);          // [14] nearest path curvature (used for feed-forward)
  append_diag(nearest_k);                 // [15] nearest path curvature (not smoothed)
  append_diag(mpc_data.predicted_steer);  // [16] predicted steer
  append_diag(wz_predicted);              // [17] angular velocity from predicted steer
  append_diag(iteration_num);             // [18] iteration number
  append_diag(runtime);                   // [19] runtime of the latest problem solved
  append_diag(objective_value);           // [20] objective value of the latest problem solved

  return diagnostic;
}

void MPC::setReferenceTrajectory(
  const Trajectory & trajectory_msg, const TrajectoryFilteringParam & param,
  const Odometry & current_kinematics)
{
  const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    trajectory_msg.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);
  const double ego_offset_to_segment = motion_utils::calcLongitudinalOffsetToSegment(
    trajectory_msg.points, nearest_seg_idx, current_kinematics.pose.pose.position);

  const auto mpc_traj_raw = MPCUtils::convertToMPCTrajectory(trajectory_msg);

  // resampling
  const auto [success_resample, mpc_traj_resampled] = MPCUtils::resampleMPCTrajectoryByDistance(
    mpc_traj_raw, param.traj_resample_dist, nearest_seg_idx, ego_offset_to_segment);
  if (!success_resample) {
    warn_throttle("[setReferenceTrajectory] spline error when resampling by distance");
    return;
  }

  const auto is_forward_shift =
    motion_utils::isDrivingForward(mpc_traj_resampled.toTrajectoryPoints());

  // if driving direction is unknown, use previous value
  m_is_forward_shift = is_forward_shift ? is_forward_shift.value() : m_is_forward_shift;

  // path smoothing
  MPCTrajectory mpc_traj_smoothed = mpc_traj_resampled;  // smooth filtered trajectory
  const int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (
    param.enable_path_smoothing && mpc_traj_resampled_size > 2 * param.path_filter_moving_ave_num) {
    using MoveAverageFilter::filt_vector;
    if (
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.x) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.y) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.yaw) ||
      !filt_vector(param.path_filter_moving_ave_num, mpc_traj_smoothed.vx)) {
      RCLCPP_DEBUG(m_logger, "path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /*
   * Extend terminal points
   * Note: The current MPC does not properly take into account the attitude angle at the end of the
   * path. By extending the end of the path in the attitude direction, the MPC can consider the
   * attitude angle well, resulting in improved control performance. If the trajectory is
   * well-defined considering the end point attitude angle, this feature is not necessary.
   */
  if (param.extend_trajectory_for_end_yaw_control) {
    MPCUtils::extendTrajectoryInYawDirection(
      mpc_traj_raw.yaw.back(), param.traj_resample_dist, m_is_forward_shift, mpc_traj_smoothed);
  }

  // calculate yaw angle
  MPCUtils::calcTrajectoryYawFromXY(mpc_traj_smoothed, m_is_forward_shift);
  MPCUtils::convertEulerAngleToMonotonic(mpc_traj_smoothed.yaw);

  // calculate curvature
  MPCUtils::calcTrajectoryCurvature(
    param.curvature_smoothing_num_traj, param.curvature_smoothing_num_ref_steer, mpc_traj_smoothed);

  // stop velocity at a terminal point
  mpc_traj_smoothed.vx.back() = 0.0;

  // add a extra point on back with extended time to make the mpc stable.
  auto last_point = mpc_traj_smoothed.back();
  last_point.relative_time += 100.0;  // extra time to prevent mpc calc failure due to short time
  last_point.vx = 0.0;                // stop velocity at a terminal point
  mpc_traj_smoothed.push_back(last_point);

  if (!mpc_traj_smoothed.size()) {
    RCLCPP_DEBUG(m_logger, "path callback: trajectory size is undesired.");
    return;
  }

  m_reference_trajectory = mpc_traj_smoothed;
}

void MPC::resetPrevResult(const SteeringReport & current_steer)
{
  // Consider limit. The prev value larger than limitation brakes the optimization constraint and
  // results in optimization failure.
  const float steer_lim_f = static_cast<float>(m_steer_lim);
  m_raw_steer_cmd_prev = std::clamp(current_steer.steering_tire_angle, -steer_lim_f, steer_lim_f);
  m_raw_steer_cmd_pprev = std::clamp(current_steer.steering_tire_angle, -steer_lim_f, steer_lim_f);
}

std::pair<bool, MPCData> MPC::getData(
  const MPCTrajectory & traj, const SteeringReport & current_steer,
  const Odometry & current_kinematics)
{
  const auto current_pose = current_kinematics.pose.pose;

  MPCData data;
  if (!MPCUtils::calcNearestPoseInterp(
        traj, current_pose, &(data.nearest_pose), &(data.nearest_idx), &(data.nearest_time),
        ego_nearest_dist_threshold, ego_nearest_yaw_threshold)) {
    warn_throttle("calculateMPC: error in calculating nearest pose. stop mpc.");
    return {false, MPCData{}};
  }

  // get data
  data.steer = static_cast<double>(current_steer.steering_tire_angle);
  data.lateral_err = MPCUtils::calcLateralError(current_pose, data.nearest_pose);
  data.yaw_err = normalizeRadian(
    tf2::getYaw(current_pose.orientation) - tf2::getYaw(data.nearest_pose.orientation));

  // get predicted steer
  data.predicted_steer = m_steering_predictor->calcSteerPrediction();

  // check error limit
  const double dist_err = calcDistance2d(current_pose, data.nearest_pose);
  if (dist_err > m_admissible_position_error) {
    warn_throttle("Too large position error: %fm > %fm", dist_err, m_admissible_position_error);
    return {false, MPCData{}};
  }

  // check yaw error limit
  if (std::fabs(data.yaw_err) > m_admissible_yaw_error_rad) {
    warn_throttle("Too large yaw error: %f > %f", data.yaw_err, m_admissible_yaw_error_rad);
    return {false, MPCData{}};
  }

  // check trajectory time length
  const double max_prediction_time =
    m_param.min_prediction_length / static_cast<double>(m_param.prediction_horizon - 1);
  auto end_time = data.nearest_time + m_param.input_delay + m_ctrl_period + max_prediction_time;
  if (end_time > traj.relative_time.back()) {
    warn_throttle("path is too short for prediction.");
    return {false, MPCData{}};
  }
  return {true, data};
}

std::pair<bool, MPCTrajectory> MPC::resampleMPCTrajectoryByTime(
  const double ts, const double prediction_dt, const MPCTrajectory & input) const
{
  MPCTrajectory output;
  std::vector<double> mpc_time_v;
  for (double i = 0; i < static_cast<double>(m_param.prediction_horizon); ++i) {
    mpc_time_v.push_back(ts + i * prediction_dt);
  }
  if (!MPCUtils::linearInterpMPCTrajectory(input.relative_time, input, mpc_time_v, output)) {
    warn_throttle("calculateMPC: mpc resample error. stop mpc calculation. check code!");
    return {false, {}};
  }
  return {true, output};
}

VectorXd MPC::getInitialState(const MPCData & data)
{
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  VectorXd x0 = VectorXd::Zero(DIM_X);

  const auto & lat_err = data.lateral_err;
  const auto & steer = m_use_steer_prediction ? data.predicted_steer : data.steer;
  const auto & yaw_err = data.yaw_err;

  const auto vehicle_model = m_vehicle_model_ptr->modelName();
  if (vehicle_model == "kinematics") {
    x0 << lat_err, yaw_err, steer;
  } else if (vehicle_model == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (vehicle_model == "dynamics") {
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

std::pair<bool, VectorXd> MPC::updateStateForDelayCompensation(
  const MPCTrajectory & traj, const double & start_time, const VectorXd & x0_orig)
{
  const int DIM_X = m_vehicle_model_ptr->getDimX();
  const int DIM_U = m_vehicle_model_ptr->getDimU();
  const int DIM_Y = m_vehicle_model_ptr->getDimY();

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);

  MatrixXd x_curr = x0_orig;
  double mpc_curr_time = start_time;
  for (size_t i = 0; i < m_input_buffer.size(); ++i) {
    double k, v = 0.0;
    try {
      k = interpolation::lerp(traj.relative_time, traj.k, mpc_curr_time);
      v = interpolation::lerp(traj.relative_time, traj.vx, mpc_curr_time);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(m_logger, "mpc resample failed at delay compensation, stop mpc: %s", e.what());
      return {false, {}};
    }

    // get discrete state matrix A, B, C, W
    m_vehicle_model_ptr->setVelocity(v);
    m_vehicle_model_ptr->setCurvature(k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, m_ctrl_period);
    MatrixXd ud = MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = m_input_buffer.at(i);  // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += m_ctrl_period;
  }
  return {true, x_curr};
}

MPCTrajectory MPC::applyVelocityDynamicsFilter(
  const MPCTrajectory & input, const Odometry & current_kinematics) const
{
  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(input);
  if (autoware_traj.points.empty()) {
    return input;
  }

  const size_t nearest_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    autoware_traj.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);

  MPCTrajectory output = input;
  MPCUtils::dynamicSmoothingVelocity(
    nearest_seg_idx, current_kinematics.twist.twist.linear.x, m_param.acceleration_limit,
    m_param.velocity_time_constant, output);

  auto last_point = output.back();
  last_point.relative_time += 100.0;  // extra time to prevent mpc calc failure due to short time
  last_point.vx = 0.0;                // stop velocity at a terminal point
  output.push_back(last_point);
  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Uref_ex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCMatrix MPC::generateMPCMatrix(
  const MPCTrajectory & reference_trajectory, const double prediction_dt)
{
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

  // weight matrix depends on the vehicle model
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

  // predict dynamics for N times
  for (int i = 0; i < N; ++i) {
    const double ref_vx = reference_trajectory.vx.at(i);
    const double ref_vx_squared = ref_vx * ref_vx;

    const double ref_k = reference_trajectory.k.at(i) * sign_vx;
    const double ref_smooth_k = reference_trajectory.smooth_k.at(i) * sign_vx;

    // get discrete state matrix A, B, C, W
    m_vehicle_model_ptr->setVelocity(ref_vx);
    m_vehicle_model_ptr->setCurvature(ref_k);
    m_vehicle_model_ptr->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q = MatrixXd::Zero(DIM_Y, DIM_Y);
    R = MatrixXd::Zero(DIM_U, DIM_U);
    const auto mpc_weight = getWeight(ref_k);
    Q(0, 0) = mpc_weight.lat_error;
    Q(1, 1) = mpc_weight.heading_error;
    R(0, 0) = mpc_weight.steering_input;

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1) {
      Q_adaptive(0, 0) = m_param.nominal_weight.terminal_lat_error;
      Q_adaptive(1, 1) = m_param.nominal_weight.terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * mpc_weight.heading_error_squared_vel;
    R_adaptive(0, 0) += ref_vx_squared * mpc_weight.steering_input_squared_vel;

    // update mpc matrix
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

    // get reference input (feed-forward)
    m_vehicle_model_ptr->setCurvature(ref_smooth_k);
    m_vehicle_model_ptr->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < tier4_autoware_utils::deg2rad(m_param.zero_ff_steer_deg)) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    m.Uref_ex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  // add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2
  for (int i = 0; i < N - 1; ++i) {
    const double ref_vx = reference_trajectory.vx.at(i);
    const double ref_k = reference_trajectory.k.at(i) * sign_vx;
    const double j = ref_vx * ref_vx * getWeight(ref_k).lat_jerk / (DT * DT);
    const Eigen::Matrix2d J = (Eigen::Matrix2d() << j, -j, -j, j).finished();
    m.R2ex.block(i, i, 2, 2) += J;
  }

  addSteerWeightR(prediction_dt, m.R1ex);

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
std::pair<bool, VectorXd> MPC::executeOptimization(
  const MPCMatrix & m, const VectorXd & x0, const double prediction_dt, const MPCTrajectory & traj,
  const double current_velocity)
{
  VectorXd Uex;

  if (!isValid(m)) {
    warn_throttle("model matrix is invalid. stop MPC.");
    return {false, {}};
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
  addSteerWeightF(prediction_dt, f);

  MatrixXd A = MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  // steering angle limit
  VectorXd lb = VectorXd::Constant(DIM_U_N, -m_steer_lim);  // min steering angle
  VectorXd ub = VectorXd::Constant(DIM_U_N, m_steer_lim);   // max steering angle

  // steering angle rate limit
  VectorXd steer_rate_limits = calcSteerRateLimitOnTrajectory(traj, current_velocity);
  VectorXd ubA = steer_rate_limits * prediction_dt;
  VectorXd lbA = -steer_rate_limits * prediction_dt;
  ubA(0) = m_raw_steer_cmd_prev + steer_rate_limits(0) * m_ctrl_period;
  lbA(0) = m_raw_steer_cmd_prev - steer_rate_limits(0) * m_ctrl_period;

  auto t_start = std::chrono::system_clock::now();
  bool solve_result = m_qpsolver_ptr->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex);
  auto t_end = std::chrono::system_clock::now();
  if (!solve_result) {
    warn_throttle("qp solver error");
    return {false, {}};
  }

  {
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    RCLCPP_DEBUG(m_logger, "qp solver calculation time = %ld [ms]", t);
  }

  if (Uex.array().isNaN().any()) {
    warn_throttle("model Uex includes NaN, stop MPC.");
    return {false, {}};
  }
  return {true, Uex};
}

void MPC::addSteerWeightR(const double prediction_dt, MatrixXd & R) const
{
  const int N = m_param.prediction_horizon;
  const double DT = prediction_dt;

  // add steering rate : weight for (u(i) - u(i-1) / dt )^2
  {
    const double steer_rate_r = m_param.nominal_weight.steer_rate / (DT * DT);
    const Eigen::Matrix2d D = steer_rate_r * (Eigen::Matrix2d() << 1.0, -1.0, -1.0, 1.0).finished();
    for (int i = 0; i < N - 1; ++i) {
      R.block(i, i, 2, 2) += D;
    }
    if (N > 1) {
      // steer rate i = 0
      R(0, 0) += m_param.nominal_weight.steer_rate / (m_ctrl_period * m_ctrl_period);
    }
  }

  // add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2
  {
    const double w = m_param.nominal_weight.steer_acc;
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

void MPC::addSteerWeightF(const double prediction_dt, MatrixXd & f) const
{
  if (f.rows() < 2) {
    return;
  }

  const double DT = prediction_dt;

  // steer rate for i = 0
  f(0, 0) += -2.0 * m_param.nominal_weight.steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = m_param.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 =
    m_param.nominal_weight.steer_acc / (std::pow(DT, 3) * m_ctrl_period);
  const double steer_acc_r_cp2 =
    m_param.nominal_weight.steer_acc / (std::pow(DT, 2) * std::pow(m_ctrl_period, 2));
  const double steer_acc_r_cp4 = m_param.nominal_weight.steer_acc / std::pow(m_ctrl_period, 4);

  // steer acc  i = 0
  f(0, 0) += ((-2.0 * m_raw_steer_cmd_prev + m_raw_steer_cmd_pprev) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  f(0, 0) += (-2.0 * m_raw_steer_cmd_prev * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  f(0, 1) += (2.0 * m_raw_steer_cmd_prev * steer_acc_r_cp1) * 0.5;
}

double MPC::getPredictionDeltaTime(
  const double start_time, const MPCTrajectory & input, const Odometry & current_kinematics) const
{
  // Calculate the time min_prediction_length ahead from current_pose
  const auto autoware_traj = MPCUtils::convertToAutowareTrajectory(input);
  const size_t nearest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    autoware_traj.points, current_kinematics.pose.pose, ego_nearest_dist_threshold,
    ego_nearest_yaw_threshold);
  double sum_dist = 0;
  const double target_time = [&]() {
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    for (size_t i = nearest_idx + 1; i < input.relative_time.size(); i++) {
      const double segment_dist = MPCUtils::calcDistance2d(input, i, i - 1);
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
  const MPCMatrix & mpc_matrix, const MatrixXd & x0, const MatrixXd & Uex, const double u_filtered,
  const float current_steer, const double predict_dt) const
{
  if (m_vehicle_model_ptr->modelName() != "kinematics") {
    // not supported yet. Use old implementation.
    return (u_filtered - current_steer) / predict_dt;
  }

  // calculate predicted states to get the steering motion
  const auto & m = mpc_matrix;
  const MatrixXd Xex = m.Aex * x0 + m.Bex * Uex + m.Wex;

  const size_t STEER_IDX = 2;  // for kinematics model

  const auto steer_0 = x0(STEER_IDX, 0);
  const auto steer_1 = Xex(STEER_IDX, 0);

  const auto steer_rate = (steer_1 - steer_0) / predict_dt;

  return steer_rate;
}

VectorXd MPC::calcSteerRateLimitOnTrajectory(
  const MPCTrajectory & trajectory, const double current_velocity) const
{
  const auto interp = [&](const auto & steer_rate_limit_map, const auto & current) {
    std::vector<double> reference, limits;
    for (const auto & p : steer_rate_limit_map) {
      reference.push_back(p.first);
      limits.push_back(p.second);
    }

    // If the speed is out of range of the reference, apply zero-order hold.
    if (current <= reference.front()) {
      return limits.front();
    }
    if (current >= reference.back()) {
      return limits.back();
    }

    // Apply linear interpolation
    for (size_t i = 0; i < reference.size() - 1; ++i) {
      if (reference.at(i) <= current && current <= reference.at(i + 1)) {
        auto ratio =
          (current - reference.at(i)) / std::max(reference.at(i + 1) - reference.at(i), 1.0e-5);
        ratio = std::clamp(ratio, 0.0, 1.0);
        const auto interp = limits.at(i) + ratio * (limits.at(i + 1) - limits.at(i));
        return interp;
      }
    }

    std::cerr << "MPC::calcSteerRateLimitOnTrajectory() interpolation logic is broken. Command "
                 "filter is not working. Please check the code."
              << std::endl;
    return reference.back();
  };

  // when the vehicle is stopped, no steering rate limit.
  const bool is_vehicle_stopped = std::fabs(current_velocity) < 0.01;
  if (is_vehicle_stopped) {
    return VectorXd::Zero(m_param.prediction_horizon);
  }

  // calculate steering rate limit
  VectorXd steer_rate_limits = VectorXd::Zero(m_param.prediction_horizon);
  for (int i = 0; i < m_param.prediction_horizon; ++i) {
    const auto limit_by_curvature = interp(m_steer_rate_lim_map_by_curvature, trajectory.k.at(i));
    const auto limit_by_velocity = interp(m_steer_rate_lim_map_by_velocity, trajectory.vx.at(i));
    steer_rate_limits(i) = std::min(limit_by_curvature, limit_by_velocity);
  }

  return steer_rate_limits;
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
}  // namespace autoware::motion::control::mpc_lateral_controller
