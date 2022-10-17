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

#ifndef RAW_VEHICLE_CMD_CONVERTER__PID_HPP_
#define RAW_VEHICLE_CMD_CONVERTER__PID_HPP_

#include <algorithm>
#include <iostream>
#include <vector>

namespace raw_vehicle_cmd_converter
{
class PIDController
{
public:
  double calculateFB(
    const double target_value, const double dt, const double reset_trigger_value,
    const double current_value, std::vector<double> & pid_contributions,
    std::vector<double> & errors);
  double calculatePID(
    const double error, const double dt, const bool enable_integration,
    std::vector<double> & pid_contributions, std::vector<double> & errors, bool is_debugging);
  void setGains(const double kp, const double ki, const double kd);
  void setLimits(
    const double max_ret, const double min_ret, const double max_ret_p, const double min_ret_p,
    const double max_ret_i, const double min_ret_i, const double max_ret_d, const double min_ret_d);
  void reset();
  void setDecay(const double decay) { invalid_integration_decay_ = decay; }
  void setInitialized() { is_initialized_ = true; }
  bool getInitialized() { return is_initialized_; }

private:
  // parameters
  double kp_;
  double ki_;
  double kd_;
  double max_ret_p_;
  double min_ret_p_;
  double max_ret_i_;
  double min_ret_i_;
  double max_ret_d_;
  double min_ret_d_;
  double max_ret_;
  double min_ret_;
  // states
  double error_integral_{0.0};
  double prev_error_{0.0};
  bool is_first_time_{true};
  double invalid_integration_decay_{0.0};
  double is_initialized_{false};
};
}  // namespace raw_vehicle_cmd_converter

#endif  // RAW_VEHICLE_CMD_CONVERTER__PID_HPP_
