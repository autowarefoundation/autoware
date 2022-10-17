//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "raw_vehicle_cmd_converter/pid.hpp"

#include <algorithm>
#include <vector>

namespace raw_vehicle_cmd_converter
{

double PIDController::calculateFB(
  const double target_value, const double dt, const double reset_trigger_value,
  const double current_value, std::vector<double> & pid_contributions, std::vector<double> & errors)
{
  const double error = target_value - current_value;
  const bool enable_integration = (std::abs(reset_trigger_value) < 0.01) ? false : true;
  return calculatePID(error, dt, enable_integration, pid_contributions, errors, false);
}

double PIDController::calculatePID(
  const double error, const double dt, const bool enable_integration,
  std::vector<double> & pid_contributions, std::vector<double> & errors, bool is_debugging)
{
  double ret_p = kp_ * error;
  ret_p = std::min(std::max(ret_p, min_ret_p_), max_ret_p_);

  if (enable_integration) {
    error_integral_ += error * dt;
    error_integral_ = std::min(std::max(error_integral_, min_ret_i_ / ki_), max_ret_i_ / ki_);
    if (is_debugging) {
      std::cout << "error: " << error << ", dt: " << dt << ", integ_error: " << error_integral_
                << std::endl;
    }
  } else {
    error_integral_ *= invalid_integration_decay_;
  }
  double ret_i = ki_ * error_integral_;

  double error_differential;
  if (is_first_time_) {
    error_differential = 0;
    is_first_time_ = false;
  } else {
    error_differential = (error - prev_error_) / dt;
  }
  double ret_d = kd_ * error_differential;
  ret_d = std::min(std::max(ret_d, min_ret_d_), max_ret_d_);

  prev_error_ = error;

  pid_contributions.at(0) = ret_p;
  pid_contributions.at(1) = ret_i;
  pid_contributions.at(2) = ret_d;
  errors.at(0) = error;
  errors.at(1) = error_integral_;
  errors.at(2) = error_differential;

  double ret = ret_p + ret_i + ret_d;
  ret = std::min(std::max(ret, min_ret_), max_ret_);

  return ret;
}

void PIDController::setGains(const double kp, const double ki, const double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setLimits(
  const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
  const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d)
{
  max_ret_ = max_ret;
  min_ret_ = min_ret;
  max_ret_p_ = max_ret_p;
  min_ret_p_ = min_ret_p;
  max_ret_d_ = max_ret_d;
  min_ret_d_ = min_ret_d;
  max_ret_i_ = max_ret_i;
  min_ret_i_ = min_ret_i;
}

void PIDController::reset()
{
  error_integral_ = 0;
  prev_error_ = 0;
  is_first_time_ = true;
}
}  // namespace raw_vehicle_cmd_converter
