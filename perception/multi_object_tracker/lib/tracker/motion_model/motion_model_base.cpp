// Copyright 2024 Tier IV, Inc.
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
//
//
// Author: v1.0 Taekjin Lee
//

#include "autoware/multi_object_tracker/tracker/motion_model/motion_model_base.hpp"

namespace autoware::multi_object_tracker
{

MotionModel::MotionModel() : last_update_time_(rclcpp::Time(0, 0))
{
}

bool MotionModel::initialize(
  const rclcpp::Time & time, const Eigen::MatrixXd & X, const Eigen::MatrixXd & P)
{
  // initialize Kalman filter
  if (!ekf_.init(X, P)) return false;

  // set last_update_time_
  last_update_time_ = time;

  // set initialized flag
  is_initialized_ = true;

  return true;
}

bool MotionModel::predictState(const rclcpp::Time & time)
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  const double dt = getDeltaTime(time);
  if (dt < 0.0) {
    return false;
  }
  if (dt < 1e-6 /*1usec*/) {
    return true;
  }

  // multi-step prediction
  // if dt is too large, shorten dt and repeat prediction
  const uint32_t repeat = std::floor(dt / dt_max_) + 1;
  const double dt_ = dt / repeat;
  for (uint32_t i = 0; i < repeat; ++i) {
    if (!predictStateStep(dt_, ekf_)) return false;
    last_update_time_ += rclcpp::Duration::from_seconds(dt_);
  }
  // reset the last_update_time_ to the prediction time
  last_update_time_ = time;
  return true;
}

bool MotionModel::getPredictedState(
  const rclcpp::Time & time, Eigen::MatrixXd & X, Eigen::MatrixXd & P) const
{
  // check if the state is initialized
  if (!checkInitialized()) return false;

  const double dt = getDeltaTime(time);
  if (dt < 1e-6 /*1usec*/) {
    // no prediction, return the current state
    ekf_.getX(X);
    ekf_.getP(P);
    return true;
  }

  // copy the predicted state and covariance
  KalmanFilter tmp_ekf_for_no_update = ekf_;
  // multi-step prediction
  // if dt is too large, shorten dt and repeat prediction
  const uint32_t repeat = std::floor(dt / dt_max_) + 1;
  const double dt_ = dt / repeat;
  for (uint32_t i = 0; i < repeat; ++i) {
    if (!predictStateStep(dt_, tmp_ekf_for_no_update)) return false;
  }
  tmp_ekf_for_no_update.getX(X);
  tmp_ekf_for_no_update.getP(P);
  return true;
}

}  // namespace autoware::multi_object_tracker
