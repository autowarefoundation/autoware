// Copyright 2021 Tier IV, Inc.
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

#include "motion_velocity_smoother/smoother/jerk_filtered_smoother.hpp"

#include "eigen3/Eigen/Core"
#include "motion_velocity_smoother/trajectory_utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

#define VERBOSE_TRAJECTORY_VELOCITY false

namespace motion_velocity_smoother
{
JerkFilteredSmoother::JerkFilteredSmoother(rclcpp::Node & node) : SmootherBase(node)
{
  auto & p = smoother_param_;
  p.jerk_weight = node.declare_parameter("jerk_weight", 10.0);
  p.over_v_weight = node.declare_parameter("over_v_weight", 100000.0);
  p.over_a_weight = node.declare_parameter("over_a_weight", 5000.0);
  p.over_j_weight = node.declare_parameter("over_j_weight", 2000.0);
  p.jerk_filter_ds = node.declare_parameter("jerk_filter_ds", 0.1);

  qp_solver_.updateMaxIter(20000);
  qp_solver_.updateRhoInterval(0);  // 0 means automatic
  qp_solver_.updateEpsRel(1.0e-6);  // def: 1.0e-4
  qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
  qp_solver_.updateVerbose(false);
}

void JerkFilteredSmoother::setParam(const Param & smoother_param)
{
  smoother_param_ = smoother_param;
}

JerkFilteredSmoother::Param JerkFilteredSmoother::getParam() const { return smoother_param_; }

bool JerkFilteredSmoother::apply(
  const double v0, const double a0, const TrajectoryPoints & input, TrajectoryPoints & output,
  std::vector<TrajectoryPoints> & debug_trajectories)
{
  output = input;

  if (input.empty()) {
    RCLCPP_WARN(logger_, "Input TrajectoryPoints to the jerk filtered optimization is empty.");
    return false;
  }

  if (input.size() == 1) {
    // No need to do optimization
    output.front().longitudinal_velocity_mps = v0;
    output.front().acceleration_mps2 = a0;
    debug_trajectories.resize(3);
    debug_trajectories[0] = output;
    debug_trajectories[1] = output;
    debug_trajectories[2] = output;
    return true;
  }

  const auto ts = std::chrono::system_clock::now();

  const double a_max = base_param_.max_accel;
  const double a_min = base_param_.min_decel;
  const double a_stop_accel = 0.0;
  const double a_stop_decel = base_param_.stop_decel;
  const double j_max = base_param_.max_jerk;
  const double j_min = base_param_.min_jerk;
  const double over_j_weight = smoother_param_.over_j_weight;
  const double over_v_weight = smoother_param_.over_v_weight;
  const double over_a_weight = smoother_param_.over_a_weight;

  // jerk filter
  const auto forward_filtered =
    forwardJerkFilter(v0, std::max(a0, a_min), a_max, a_stop_accel, j_max, input);
  const auto backward_filtered = backwardJerkFilter(
    input.back().longitudinal_velocity_mps, a_stop_decel, a_min, a_stop_decel, j_min, input);
  const auto filtered =
    mergeFilteredTrajectory(v0, a0, a_min, j_min, forward_filtered, backward_filtered);

  // Resample TrajectoryPoints for Optimization
  // TODO(planning/control team) deal with overlapped lanes with the same direction
  const auto initial_traj_pose = filtered.front().pose;

  const auto resample = [&](const auto & trajectory) {
    return resampling::resampleTrajectory(
      trajectory, v0, initial_traj_pose, std::numeric_limits<double>::max(),
      std::numeric_limits<double>::max(), base_param_.resample_param);
  };

  auto opt_resampled_trajectory = resample(filtered);

  // Set debug trajectories
  debug_trajectories.resize(3);
  debug_trajectories[0] = resample(forward_filtered);
  debug_trajectories[1] = resample(backward_filtered);
  debug_trajectories[2] = resample(filtered);

  // Ensure terminal velocity is zero
  opt_resampled_trajectory.back().longitudinal_velocity_mps = 0.0;

  // If Resampled Size is too small, we don't do optimization
  if (opt_resampled_trajectory.size() == 1) {
    // No need to do optimization
    output.front().longitudinal_velocity_mps = v0;
    output.front().acceleration_mps2 = a0;
    debug_trajectories[0] = output;
    debug_trajectories[1] = output;
    debug_trajectories[2] = output;
    return true;
  }

  // to avoid getting 0 as a stop point, search zero velocity index from 1.
  // the size of the resampled trajectory must not be less than 2.
  const auto zero_vel_id = motion_utils::searchZeroVelocityIndex(
    opt_resampled_trajectory, 1, opt_resampled_trajectory.size());

  if (!zero_vel_id) {
    RCLCPP_WARN(logger_, "opt_resampled_trajectory must have stop point.");
    return false;
  }

  // Clip trajectory from 0 to zero_vel_id (the size becomes zero_vel_id_ + 1)
  const size_t N = *zero_vel_id + 1;

  output = opt_resampled_trajectory;

  const std::vector<double> interval_dist_arr =
    trajectory_utils::calcTrajectoryIntervalDistance(opt_resampled_trajectory);

  std::vector<double> v_max_arr(N, 0.0);
  for (size_t i = 0; i < N; ++i) {
    v_max_arr.at(i) = opt_resampled_trajectory.at(i).longitudinal_velocity_mps;
  }

  /*
   * x = [
   *      b[0], b[1], ..., b[N],               : 0~N
   *      a[0], a[1], .... a[N],               : N~2N
   *      delta[0], ..., delta[N],             : 2N~3N
   *      sigma[0], sigma[1], ...., sigma[N],  : 3N~4N
   *      gamma[0], gamma[1], ..., gamma[N]    : 4N~5N
   *     ]
   *
   * b[i]  : velocity^2
   * delta : 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
   * sigma : a_min < a[i] - sigma[i] < a_max
   * gamma : jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
   */
  const uint32_t IDX_B0 = 0;
  const uint32_t IDX_A0 = N;
  const uint32_t IDX_DELTA0 = 2 * N;
  const uint32_t IDX_SIGMA0 = 3 * N;
  const uint32_t IDX_GAMMA0 = 4 * N;

  const uint32_t l_variables = 5 * N;
  const uint32_t l_constraints = 4 * N + 1;

  // the matrix size depends on constraint numbers.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(l_constraints, l_variables);

  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  /**************************************************************/
  /**************************************************************/
  /**************** design objective function *******************/
  /**************************************************************/
  /**************************************************************/

  // jerk: d(ai)/ds * v_ref -> minimize weight * ((a1 - a0) / ds * v_ref)^2 * ds
  constexpr double ZERO_VEL_THR_FOR_DT_CALC = 0.3;
  const double smooth_weight = smoother_param_.jerk_weight;
  for (size_t i = 0; i < N - 1; ++i) {
    const double ref_vel = v_max_arr.at(i);
    const double interval_dist = std::max(interval_dist_arr.at(i), 0.0001);
    const double w_x_ds_inv = (1.0 / interval_dist) * ref_vel;
    P(IDX_A0 + i, IDX_A0 + i) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i, IDX_A0 + i + 1) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i) -= smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
    P(IDX_A0 + i + 1, IDX_A0 + i + 1) += smooth_weight * w_x_ds_inv * w_x_ds_inv * interval_dist;
  }

  for (size_t i = 0; i < N; ++i) {
    const double v_max = std::max(v_max_arr.at(i), 0.1);
    q.at(IDX_B0 + i) =
      -1.0 / (v_max * v_max);  // |v_max_i^2 - b_i|/v_max^2 -> minimize (-bi) * ds / v_max^2
    if (i < N - 1) {
      q.at(IDX_B0 + i) *= std::max(interval_dist_arr.at(i), 0.0001);
    }
    P(IDX_DELTA0 + i, IDX_DELTA0 + i) += over_v_weight;  // over velocity cost
    P(IDX_SIGMA0 + i, IDX_SIGMA0 + i) += over_a_weight;  // over acceleration cost
    P(IDX_GAMMA0 + i, IDX_GAMMA0 + i) += over_j_weight;  // over jerk cost
  }

  /**************************************************************/
  /**************************************************************/
  /**************** design constraint matrix ********************/
  /**************************************************************/
  /**************************************************************/

  /*
  NOTE: The delta allows b to be negative. This is actually invalid because the definition is b=v^2.
  But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  v=0 & a<0. To avoid the infeasibility, we allow b<0. The negative b is dealt as b=0 when it is
  converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  b is almost 0, and is not a big problem.
  */

  size_t constr_idx = 0;

  // Soft Constraint Velocity Limit: 0 < b - delta < v_max^2
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = 1.0;       // b_i
    A(constr_idx, IDX_DELTA0 + i) = -1.0;  // -delta_i
    upper_bound[constr_idx] = v_max_arr.at(i) * v_max_arr.at(i);
    lower_bound[constr_idx] = 0.0;
  }

  // Soft Constraint Acceleration Limit: a_min < a - sigma < a_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_A0 + i) = 1.0;       // a_i
    A(constr_idx, IDX_SIGMA0 + i) = -1.0;  // -sigma_i

    constexpr double stop_vel = 1e-3;
    if (v_max_arr.at(i) < stop_vel) {
      // Stop Point
      upper_bound[constr_idx] = a_stop_decel;
      lower_bound[constr_idx] = a_stop_decel;
    } else {
      upper_bound[constr_idx] = a_max;
      lower_bound[constr_idx] = a_min;
    }
  }

  // Soft Constraint Jerk Limit: jerk_min < pseudo_jerk[i] * ref_vel[i] - gamma[i] < jerk_max
  // -> jerk_min * ds < (a[i+1] - a[i]) * ref_vel[i] - gamma[i] * ds < jerk_max * ds
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double ref_vel = std::max(v_max_arr.at(i), ZERO_VEL_THR_FOR_DT_CALC);
    const double ds = interval_dist_arr.at(i);
    A(constr_idx, IDX_A0 + i) = -ref_vel;     // -a[i] * ref_vel
    A(constr_idx, IDX_A0 + i + 1) = ref_vel;  //  a[i+1] * ref_vel
    A(constr_idx, IDX_GAMMA0 + i) = -ds;      // -gamma[i] * ds
    upper_bound[constr_idx] = j_max * ds;     //  jerk_max * ds
    lower_bound[constr_idx] = j_min * ds;     //  jerk_min * ds
  }

  // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    A(constr_idx, IDX_B0 + i) = -1.0;                            // b(i)
    A(constr_idx, IDX_B0 + i + 1) = 1.0;                         // b(i+1)
    A(constr_idx, IDX_A0 + i) = -2.0 * interval_dist_arr.at(i);  // a(i) * ds
    upper_bound[constr_idx] = 0.0;
    lower_bound[constr_idx] = 0.0;
  }

  // initial condition
  {
    A(constr_idx, IDX_B0) = 1.0;  // b0
    upper_bound[constr_idx] = v0 * v0;
    lower_bound[constr_idx] = v0 * v0;
    ++constr_idx;

    A(constr_idx, IDX_A0) = 1.0;  // a0
    upper_bound[constr_idx] = a0;
    lower_bound[constr_idx] = a0;
    ++constr_idx;
  }

  // execute optimization
  const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);
  const std::vector<double> optval = std::get<0>(result);

  const auto tf1 = std::chrono::system_clock::now();
  const double dt_ms1 =
    std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;
  RCLCPP_DEBUG(logger_, "optimization time = %f [ms]", dt_ms1);

  // get velocity & acceleration
  for (size_t i = 0; i < N; ++i) {
    double b = optval.at(IDX_B0 + i);
    output.at(i).longitudinal_velocity_mps = std::sqrt(std::max(b, 0.0));
    output.at(i).acceleration_mps2 = optval.at(IDX_A0 + i);
  }
  for (size_t i = N; i < output.size(); ++i) {
    output.at(i).longitudinal_velocity_mps = 0.0;
    output.at(i).acceleration_mps2 = a_stop_decel;
  }

  qp_solver_.logUnsolvedStatus("[motion_velocity_smoother]");

  const int status_polish = std::get<2>(result);
  if (status_polish != 1) {
    const auto msg = status_polish == 0    ? "unperformed"
                     : status_polish == -1 ? "unsuccessful"
                                           : "unknown";
    RCLCPP_WARN(logger_, "osqp polish process failed : %s. The result may be inaccurate", msg);
  }

  if (VERBOSE_TRAJECTORY_VELOCITY) {
    const auto s_output = trajectory_utils::calcArclengthArray(output);

    std::cerr << "\n\n" << std::endl;
    for (size_t i = 0; i < N; ++i) {
      const auto v_opt = output.at(i).longitudinal_velocity_mps;
      const auto a_opt = output.at(i).acceleration_mps2;
      const auto ds = i < interval_dist_arr.size() ? interval_dist_arr.at(i) : 0.0;
      const auto v_rs = i < opt_resampled_trajectory.size()
                          ? opt_resampled_trajectory.at(i).longitudinal_velocity_mps
                          : 0.0;
      RCLCPP_INFO(
        logger_, "i =  %4lu | s: %5f | ds: %5f | rs: %9f | op_v: %10f | op_a: %10f |", i,
        s_output.at(i), ds, v_rs, v_opt, a_opt);
    }
  }

  return true;
}

TrajectoryPoints JerkFilteredSmoother::forwardJerkFilter(
  const double v0, const double a0, const double a_max, const double a_start, const double j_max,
  const TrajectoryPoints & input) const
{
  auto applyLimits = [&input, &a_start](double & v, double & a, size_t i) {
    double v_lim = input.at(i).longitudinal_velocity_mps;
    static constexpr double ep = 1.0e-5;
    if (v > v_lim + ep) {
      v = v_lim;
      a = 0.0;

      if (v_lim < 1e-3 && i < input.size() - 1) {
        double next_v_lim = input.at(i + 1).longitudinal_velocity_mps;
        if (next_v_lim >= 1e-3) {
          a = a_start;  // start from stop velocity
        }
      }
    }

    if (v < 0.0) {
      v = a = 0.0;
    }
  };

  auto output = input;

  double current_vel = v0;
  double current_acc = a0;
  applyLimits(current_vel, current_acc, 0);

  output.front().longitudinal_velocity_mps = current_vel;
  output.front().acceleration_mps2 = current_acc;
  for (size_t i = 1; i < input.size(); ++i) {
    const double ds = tier4_autoware_utils::calcDistance2d(input.at(i), input.at(i - 1));
    const double max_dt = std::pow(6.0 * ds / j_max, 1.0 / 3.0);  // assuming v0 = a0 = 0.
    const double dt = std::min(ds / std::max(current_vel, 1.0e-6), max_dt);

    if (current_acc + j_max * dt >= a_max) {
      const double tmp_jerk = std::min((a_max - current_acc) / dt, j_max);
      current_vel = current_vel + current_acc * dt + 0.5 * tmp_jerk * dt * dt;
      current_acc = a_max;
    } else {
      current_vel = current_vel + current_acc * dt + 0.5 * j_max * dt * dt;
      current_acc = current_acc + j_max * dt;
    }
    applyLimits(current_vel, current_acc, i);
    output.at(i).longitudinal_velocity_mps = current_vel;
    output.at(i).acceleration_mps2 = current_acc;
  }
  return output;
}

TrajectoryPoints JerkFilteredSmoother::backwardJerkFilter(
  const double v0, const double a0, const double a_min, const double a_stop, const double j_min,
  const TrajectoryPoints & input) const
{
  auto input_rev = input;
  std::reverse(input_rev.begin(), input_rev.end());
  auto filtered = forwardJerkFilter(
    v0, std::fabs(a0), std::fabs(a_min), std::fabs(a_stop), std::fabs(j_min), input_rev);
  std::reverse(filtered.begin(), filtered.end());
  for (size_t i = 0; i < filtered.size(); ++i) {
    filtered.at(i).acceleration_mps2 *= -1.0;  // Deceleration
  }
  return filtered;
}

TrajectoryPoints JerkFilteredSmoother::mergeFilteredTrajectory(
  const double v0, const double a0, const double a_min, const double j_min,
  const TrajectoryPoints & forward_filtered, const TrajectoryPoints & backward_filtered) const
{
  TrajectoryPoints merged;
  merged = forward_filtered;

  auto getVx = [](const TrajectoryPoints & trajectory, int i) {
    return trajectory.at(i).longitudinal_velocity_mps;
  };

  size_t i = 0;

  if (getVx(backward_filtered, 0) < v0) {
    double current_vel = v0;
    double current_acc = a0;
    while (getVx(backward_filtered, i) < current_vel && i < merged.size() - 1) {
      merged.at(i).longitudinal_velocity_mps = current_vel;
      merged.at(i).acceleration_mps2 = current_acc;

      const double ds =
        tier4_autoware_utils::calcDistance2d(forward_filtered.at(i + 1), forward_filtered.at(i));
      const double max_dt =
        std::pow(6.0 * ds / std::fabs(j_min), 1.0 / 3.0);  // assuming v0 = a0 = 0.
      const double dt = std::min(ds / std::max(current_vel, 1.0e-6), max_dt);

      if (current_acc + j_min * dt < a_min) {
        const double tmp_jerk = std::max((a_min - current_acc) / dt, j_min);
        current_vel = current_vel + current_acc * dt + 0.5 * tmp_jerk * dt * dt;
        current_acc = std::max(current_acc + tmp_jerk * dt, a_min);
      } else {
        current_vel = current_vel + current_acc * dt + 0.5 * j_min * dt * dt;
        current_acc = current_acc + j_min * dt;
      }

      if (current_vel > getVx(forward_filtered, i)) {
        current_vel = getVx(forward_filtered, i);
      }
      ++i;
    }
  }

  // take smaller velocity point
  for (; i < merged.size(); ++i) {
    merged.at(i) = (getVx(forward_filtered, i) < getVx(backward_filtered, i))
                     ? forward_filtered.at(i)
                     : backward_filtered.at(i);
  }
  return merged;
}

TrajectoryPoints JerkFilteredSmoother::resampleTrajectory(
  const TrajectoryPoints & input, [[maybe_unused]] const double v0,
  const geometry_msgs::msg::Pose & current_pose, const double nearest_dist_threshold,
  const double nearest_yaw_threshold) const
{
  return resampling::resampleTrajectory(
    input, current_pose, nearest_dist_threshold, nearest_yaw_threshold, base_param_.resample_param,
    smoother_param_.jerk_filter_ds);
}

}  // namespace motion_velocity_smoother
