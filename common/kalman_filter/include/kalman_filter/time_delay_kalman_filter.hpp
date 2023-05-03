// Copyright 2018-2019 Autoware Foundation
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

#ifndef KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_HPP_
#define KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_HPP_

#include "kalman_filter/kalman_filter.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <iostream>

/**
 * @file time_delay_kalman_filter.h
 * @brief kalman filter with delayed measurement class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

class TimeDelayKalmanFilter : public KalmanFilter
{
public:
  /**
   * @brief No initialization constructor.
   */
  TimeDelayKalmanFilter();

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P0 initial covariance of estimated state
   * @param max_delay_step Maximum number of delay steps, which determines the dimension of the
   * extended kalman filter
   */
  void init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P, const int max_delay_step);

  /**
   * @brief get latest time estimated state
   */
  Eigen::MatrixXd getLatestX() const;

  /**
   * @brief get latest time estimation covariance
   */
  Eigen::MatrixXd getLatestP() const;

  /**
   * @brief calculate kalman filter covariance by precision model with time delay. This is mainly
   * for EKF of nonlinear process model.
   * @param x_next predicted state by prediction model
   * @param A coefficient matrix of x for process model
   * @param Q covariance matrix for process model
   */
  bool predictWithDelay(
    const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q);

  /**
   * @brief calculate kalman filter covariance by measurement model with time delay. This is mainly
   * for EKF of nonlinear process model.
   * @param y measured values
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @param delay_step measurement delay
   */
  bool updateWithDelay(
    const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
    const int delay_step);

private:
  int max_delay_step_;  //!< @brief maximum number of delay steps
  int dim_x_;           //!< @brief dimension of latest state
  int dim_x_ex_;        //!< @brief dimension of extended state with dime delay
};
#endif  // KALMAN_FILTER__TIME_DELAY_KALMAN_FILTER_HPP_
