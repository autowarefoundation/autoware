// Copyright 2018-2021 Tier IV, Inc. All rights reserved.
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

#include "pid_longitudinal_controller/pid.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
PIDController::PIDController()
: m_error_integral(0.0),
  m_prev_error(0.0),
  m_is_first_time(true),
  m_is_gains_set(false),
  m_is_limits_set(false)
{
}

double PIDController::calculate(
  const double error, const double dt, const bool enable_integration,
  std::vector<double> & pid_contributions)
{
  if (!m_is_gains_set || !m_is_limits_set) {
    throw std::runtime_error("Trying to calculate uninitialized PID");
  }

  const auto & p = m_params;

  double ret_p = p.kp * error;
  ret_p = std::min(std::max(ret_p, p.min_ret_p), p.max_ret_p);

  if (enable_integration) {
    m_error_integral += error * dt;
    m_error_integral = std::min(std::max(m_error_integral, p.min_ret_i / p.ki), p.max_ret_i / p.ki);
  }
  const double ret_i = p.ki * m_error_integral;

  double error_differential;
  if (m_is_first_time) {
    error_differential = 0;
    m_is_first_time = false;
  } else {
    error_differential = (error - m_prev_error) / dt;
  }
  double ret_d = p.kd * error_differential;
  ret_d = std::min(std::max(ret_d, p.min_ret_d), p.max_ret_d);

  m_prev_error = error;

  pid_contributions.resize(3);
  pid_contributions.at(0) = ret_p;
  pid_contributions.at(1) = ret_i;
  pid_contributions.at(2) = ret_d;

  double ret = ret_p + ret_i + ret_d;
  ret = std::min(std::max(ret, p.min_ret), p.max_ret);

  return ret;
}

void PIDController::setGains(const double kp, const double ki, const double kd)
{
  m_params.kp = kp;
  m_params.ki = ki;
  m_params.kd = kd;
  m_is_gains_set = true;
}

void PIDController::setLimits(
  const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
  const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d)
{
  m_params.max_ret = max_ret;
  m_params.min_ret = min_ret;
  m_params.max_ret_p = max_ret_p;
  m_params.min_ret_p = min_ret_p;
  m_params.max_ret_d = max_ret_d;
  m_params.min_ret_d = min_ret_d;
  m_params.max_ret_i = max_ret_i;
  m_params.min_ret_i = min_ret_i;
  m_is_limits_set = true;
}

void PIDController::reset()
{
  m_error_integral = 0.0;
  m_prev_error = 0.0;
  m_is_first_time = true;
}
}  // namespace autoware::motion::control::pid_longitudinal_controller
