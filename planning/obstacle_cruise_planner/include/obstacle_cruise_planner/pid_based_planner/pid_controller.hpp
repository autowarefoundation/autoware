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

#ifndef OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_CONTROLLER_HPP_
#define OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_CONTROLLER_HPP_

#include <optional>

class PIDController
{
public:
  PIDController(const double kp, const double ki, const double kd)
  : kp_(kp), ki_(ki), kd_(kd), error_sum_(0.0)
  {
  }

  double calc(const double error)
  {
    error_sum_ += error;

    // TODO(murooka) use time for d gain calculation
    const double output =
      kp_ * error + ki_ * error_sum_ + (prev_error_ ? kd_ * (error - *prev_error_) : 0.0);
    prev_error_ = error;
    return output;
  }

  double getKp() const { return kp_; }
  double getKi() const { return ki_; }
  double getKd() const { return kd_; }

  void updateParam(const double kp, const double ki, const double kd)
  {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    error_sum_ = 0.0;
    prev_error_ = {};
  }

private:
  double kp_;
  double ki_;
  double kd_;

  double error_sum_;
  std::optional<double> prev_error_;
};

#endif  // OBSTACLE_CRUISE_PLANNER__PID_BASED_PLANNER__PID_CONTROLLER_HPP_
