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

#include "autoware/obstacle_cruise_planner/optimization_based_planner/velocity_optimizer.hpp"

#include <Eigen/Core>

#include <iostream>

VelocityOptimizer::VelocityOptimizer(
  const double max_s_weight, const double max_v_weight, const double over_s_safety_weight,
  const double over_s_ideal_weight, const double over_v_weight, const double over_a_weight,
  const double over_j_weight)
: max_s_weight_(max_s_weight),
  max_v_weight_(max_v_weight),
  over_s_safety_weight_(over_s_safety_weight),
  over_s_ideal_weight_(over_s_ideal_weight),
  over_v_weight_(over_v_weight),
  over_a_weight_(over_a_weight),
  over_j_weight_(over_j_weight)
{
  qp_solver_.updateMaxIter(200000);
  qp_solver_.updateRhoInterval(0);  // 0 means automatic
  qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
  qp_solver_.updateEpsAbs(1.0e-8);  // def: 1.0e-4
  qp_solver_.updateVerbose(false);
}

VelocityOptimizer::OptimizationResult VelocityOptimizer::optimize(const OptimizationData & data)
{
  const std::vector<double> time_vec = data.time_vec;
  const size_t N = time_vec.size();
  const double s0 = data.s0;
  const double v0 = data.v0;
  const double a0 = data.a0;
  const double v_max = std::max(data.v_max, 0.1);
  const double a_max = data.a_max;
  const double a_min = data.a_min;
  const double limit_a_max = data.limit_a_max;
  const double limit_a_min = data.limit_a_min;
  const double limit_j_max = data.limit_j_max;
  const double limit_j_min = data.limit_j_min;
  const double j_max = data.j_max;
  const double j_min = data.j_min;
  const double a_range = std::max(a_max - a_min, 0.1);
  const double j_range = std::max(j_max - j_min, 0.1);
  const double t_dangerous = data.t_dangerous;
  const double t_idling = data.idling_time;
  const auto s_boundary = data.s_boundary;

  // Variables: s_i, v_i, a_i, j_i, over_s_safety_i, over_s_ideal_i, over_v_i, over_a_i, over_j_i
  const int IDX_S0 = 0;
  const int IDX_V0 = N;
  const int IDX_A0 = 2 * N;
  const int IDX_J0 = 3 * N;
  const int IDX_OVER_S_SAFETY0 = 4 * N;
  const int IDX_OVER_S_IDEAL0 = 5 * N;
  const int IDX_OVER_V0 = 6 * N;
  const int IDX_OVER_A0 = 7 * N;
  const int IDX_OVER_J0 = 8 * N;
  const int l_variables = 9 * N;
  const int l_constraints = 7 * N + 3 * (N - 1) + 3;

  // the matrix size depends on constraint numbers.
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(l_constraints, l_variables);
  std::vector<double> lower_bound(l_constraints, 0.0);
  std::vector<double> upper_bound(l_constraints, 0.0);

  // Object Variables
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
  std::vector<double> q(l_variables, 0.0);

  // Object Function
  // min w_s*(s_i - s_ideal_i)^2 + w_v * v0 * ((v_i - v_max)^2 / v_max^2)
  // + over_s_safety^2 + over_s_ideal^2 + over_v_ideal^2 + over_a_ideal^2
  // + over_j_ideal^2
  constexpr double MINIMUM_MAX_S_BOUND = 0.01;
  for (size_t i = 0; i < N; ++i) {
    const double dt =
      i < N - 1 ? time_vec.at(i + 1) - time_vec.at(i) : time_vec.at(N - 1) - time_vec.at(N - 2);
    const double max_s = std::max(s_boundary.at(i).max_s, MINIMUM_MAX_S_BOUND);
    const double v_coeff = v0 / (2 * std::fabs(a_min)) + t_idling;
    if (s_boundary.at(i).is_object) {
      q.at(IDX_S0 + i) += -max_s_weight_ / max_s * dt;
      q.at(IDX_V0 + i) += -max_s_weight_ / max_s * v_coeff * dt;
    } else {
      q.at(IDX_S0 + i) += -max_s_weight_ / max_s * dt;
    }

    q.at(IDX_V0 + i) += -max_v_weight_ / v_max * dt;
  }

  for (size_t i = 0; i < N; ++i) {
    const double dt =
      i < N - 1 ? time_vec.at(i + 1) - time_vec.at(i) : time_vec.at(N - 1) - time_vec.at(N - 2);
    const double max_s = std::max(s_boundary.at(i).max_s, MINIMUM_MAX_S_BOUND);
    P(IDX_OVER_S_SAFETY0 + i, IDX_OVER_S_SAFETY0 + i) +=
      over_s_safety_weight_ / (max_s * max_s) * dt;
    P(IDX_OVER_S_IDEAL0 + i, IDX_OVER_S_IDEAL0 + i) += over_s_ideal_weight_ / (max_s * max_s) * dt;
    P(IDX_OVER_V0 + i, IDX_OVER_V0 + i) += over_v_weight_ / (v_max * v_max) * dt;
    P(IDX_OVER_A0 + i, IDX_OVER_A0 + i) += over_a_weight_ / a_range * dt;
    P(IDX_OVER_J0 + i, IDX_OVER_J0 + i) += over_j_weight_ / j_range * dt;

    if (s_boundary.at(i).is_object) {
      const double v_coeff = v0 / (2 * std::fabs(a_min)) + t_idling;
      P(IDX_S0 + i, IDX_S0 + i) += max_s_weight_ / (max_s * max_s) * dt;
      P(IDX_V0 + i, IDX_V0 + i) += max_s_weight_ / (max_s * max_s) * v_coeff * v_coeff * dt;
      P(IDX_S0 + i, IDX_V0 + i) += max_s_weight_ / (max_s * max_s) * v_coeff * dt;
      P(IDX_V0 + i, IDX_S0 + i) += max_s_weight_ / (max_s * max_s) * v_coeff * dt;
    } else {
      P(IDX_S0 + i, IDX_S0 + i) += max_s_weight_ / (max_s * max_s) * dt;
    }

    P(IDX_V0 + i, IDX_V0 + i) += max_v_weight_ / (v_max * v_max) * dt;
  }

  // Constraint
  size_t constr_idx = 0;

  // Safety Position Constraint: s_boundary_min < s_i + v_i*t_dangerous + v0*v_i/(2*|a_min|) -
  // over_s_safety_i < s_boundary_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    const double v_coeff = v0 / (2 * std::fabs(a_min)) + t_dangerous;
    A(constr_idx, IDX_S0 + i) = 1.0;  // s_i
    if (s_boundary.at(i).is_object) {
      A(constr_idx, IDX_V0 + i) = v_coeff;  // v_i * (t_dangerous + v0/(2*|a_min|))
    }
    A(constr_idx, IDX_OVER_S_SAFETY0 + i) = -1.0;  // over_s_safety_i
    upper_bound.at(constr_idx) = s_boundary.at(i).max_s;
    lower_bound.at(constr_idx) = 0.0;
  }

  // Ideal Position Constraint: s_boundary_min < s_i  + v_i * t_idling + v0*v_i/(2*|a_min|) -
  // over_s_ideal_i < s_boundary_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    const double v_coeff = v0 / (2 * std::fabs(a_min)) + t_idling;
    A(constr_idx, IDX_S0 + i) = 1.0;  // s_i
    if (s_boundary.at(i).is_object) {
      A(constr_idx, IDX_V0 + i) = v_coeff;  // v_i * (t_idling + v0/(2*|a_min|))
    }
    A(constr_idx, IDX_OVER_S_IDEAL0 + i) = -1.0;  // over_s_ideal_i
    upper_bound.at(constr_idx) = s_boundary.at(i).max_s;
    lower_bound.at(constr_idx) = 0.0;
  }

  // Soft Velocity Constraint: 0 < v_i - over_v_i < v_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_V0 + i) = 1.0;        // v_i
    A(constr_idx, IDX_OVER_V0 + i) = -1.0;  // over_v_i
    upper_bound.at(constr_idx) = i == N - 1 ? 0.0 : v_max;
    lower_bound.at(constr_idx) = 0.0;
  }

  // Soft Acceleration Constraint: a_min < a_i - over_a_i < a_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_A0 + i) = 1.0;        // a_i
    A(constr_idx, IDX_OVER_A0 + i) = -1.0;  // over_a_i
    upper_bound.at(constr_idx) = i == N - 1 ? 0.0 : a_max;
    lower_bound.at(constr_idx) = i == N - 1 ? 0.0 : a_min;
  }

  // Hard Acceleration Constraint: limit_a_min < a_i < a_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_A0 + i) = 1.0;  // a_i
    upper_bound.at(constr_idx) = limit_a_max;
    lower_bound.at(constr_idx) = limit_a_min;
  }

  // Soft Jerk Constraint: j_min < j_i - over_j_i < j_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_J0 + i) = 1.0;        // j_i
    A(constr_idx, IDX_OVER_J0 + i) = -1.0;  // over_j_i
    upper_bound.at(constr_idx) = i == N - 1 ? 0.0 : j_max;
    lower_bound.at(constr_idx) = i == N - 1 ? 0.0 : j_min;
  }

  // Hard Jerk Constraint: limit_j_min < j_i < limit_j_max
  for (size_t i = 0; i < N; ++i, ++constr_idx) {
    A(constr_idx, IDX_J0 + i) = 1.0;  // j_i
    upper_bound.at(constr_idx) = limit_j_max;
    lower_bound.at(constr_idx) = limit_j_min;
  }

  // Dynamic Constraint
  // s_i+1 = s_i + v_i * dt + 0.5 * a_i * dt^2 + 1/6 * j_i * dt^3
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double dt = time_vec.at(i + 1) - time_vec.at(i);
    A(constr_idx, IDX_S0 + i + 1) = 1.0;                    // s_i+1
    A(constr_idx, IDX_S0 + i) = -1.0;                       // -s_i
    A(constr_idx, IDX_V0 + i) = -dt;                        // -v_i*dt
    A(constr_idx, IDX_A0 + i) = -0.5 * dt * dt;             // -0.5 * a_i * dt^2
    A(constr_idx, IDX_J0 + i) = -1.0 / 6.0 * dt * dt * dt;  // -1.0/6.0 * j_i * dt^3
    upper_bound.at(constr_idx) = 0.0;
    lower_bound.at(constr_idx) = 0.0;
  }

  // v_i+1 = v_i + a_i * dt + 0.5 * j_i * dt^2
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double dt = time_vec.at(i + 1) - time_vec.at(i);
    A(constr_idx, IDX_V0 + i + 1) = 1.0;         // v_i+1
    A(constr_idx, IDX_V0 + i) = -1.0;            // -v_i
    A(constr_idx, IDX_A0 + i) = -dt;             // -a_i * dt
    A(constr_idx, IDX_J0 + i) = -0.5 * dt * dt;  // -0.5 * j_i * dt^2
    upper_bound.at(constr_idx) = 0.0;
    lower_bound.at(constr_idx) = 0.0;
  }

  // a_i+1 = a_i + j_i * dt
  for (size_t i = 0; i < N - 1; ++i, ++constr_idx) {
    const double dt = time_vec.at(i + 1) - time_vec.at(i);
    A(constr_idx, IDX_A0 + i + 1) = 1.0;  // a_i+1
    A(constr_idx, IDX_A0 + i) = -1.0;     // -a_i
    A(constr_idx, IDX_J0 + i) = -dt;      // -j_i * dt
    upper_bound.at(constr_idx) = 0.0;
    lower_bound.at(constr_idx) = 0.0;
  }

  // initial condition
  {
    A(constr_idx, IDX_S0) = 1.0;  // s0
    upper_bound[constr_idx] = s0;
    lower_bound[constr_idx] = s0;
    ++constr_idx;

    A(constr_idx, IDX_V0) = 1.0;  // v0
    upper_bound[constr_idx] = v0;
    lower_bound[constr_idx] = v0;
    ++constr_idx;

    A(constr_idx, IDX_A0) = 1.0;  // a0
    upper_bound[constr_idx] = a0;
    lower_bound[constr_idx] = a0;
  }

  // execute optimization
  const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);
  const std::vector<double> optval = std::get<0>(result);

  const int status_val = std::get<3>(result);
  if (status_val != 1)
    std::cerr << "optimization failed : " << qp_solver_.getStatusMessage().c_str() << std::endl;

  const auto has_nan =
    std::any_of(optval.begin(), optval.end(), [](const auto v) { return std::isnan(v); });
  if (has_nan) std::cerr << "optimization failed : result contains NaN values\n";

  OptimizationResult optimized_result;
  const auto is_optimization_failed = status_val != 1 || has_nan;
  if (!is_optimization_failed) {
    std::vector<double> opt_time = time_vec;
    std::vector<double> opt_pos(N);
    std::vector<double> opt_vel(N);
    std::vector<double> opt_acc(N);
    std::vector<double> opt_jerk(N);
    for (size_t i = 0; i < N; ++i) {
      opt_pos.at(i) = optval.at(IDX_S0 + i);
      opt_vel.at(i) = std::max(optval.at(IDX_V0 + i), 0.0);
      opt_acc.at(i) = optval.at(IDX_A0 + i);
      opt_jerk.at(i) = optval.at(IDX_J0 + i);
    }
    opt_vel.back() = 0.0;

    optimized_result.t = opt_time;
    optimized_result.s = opt_pos;
    optimized_result.v = opt_vel;
    optimized_result.a = opt_acc;
    optimized_result.j = opt_jerk;
  }

  return optimized_result;
}
