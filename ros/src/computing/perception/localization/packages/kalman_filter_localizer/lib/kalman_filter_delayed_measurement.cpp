/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "kalman_filter/kalman_filter_delayed_measurement.h"

KalmanFilterDelayedMeasurement::KalmanFilterDelayedMeasurement() {}

void KalmanFilterDelayedMeasurement::init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0,
                                          const int max_delay_step)
{
  max_delay_step_ = max_delay_step;
  dim_x_ = x.rows();
  dim_x_ex_ = dim_x_ * max_delay_step;

  x_ = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  P_ = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);

  for (int i = 0; i < max_delay_step_; ++i)
  {
    x_.block(i * dim_x_, 0, dim_x_, 1) = x;
    P_.block(i * dim_x_, i * dim_x_, dim_x_, dim_x_) = P0;
  }
};

void KalmanFilterDelayedMeasurement::getCurrentX(Eigen::MatrixXd &x) { x = x_.block(0, 0, dim_x_, 1); };
void KalmanFilterDelayedMeasurement::getCurrentP(Eigen::MatrixXd &P) { P = P_.block(0, 0, dim_x_, dim_x_); };

bool KalmanFilterDelayedMeasurement::predictWithDelay(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
                                                      const Eigen::MatrixXd &Q)
{
  const int d_dim_x = dim_x_ex_ - dim_x_;

  /* slide states in the time direction */
  Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, 1);
  x_tmp.block(0, 0, dim_x_, 1) = x_next;
  x_tmp.block(dim_x_, 0, d_dim_x, 1) = x_.block(0, 0, d_dim_x, 1);
  x_ = x_tmp;

  /* set extended Q matrix */
  Eigen::MatrixXd Q_ex = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  Q_ex.block(0, 0, dim_x_, dim_x_) = Q;

  /* update P with delayed measurement A matrix structure */
  Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex_, dim_x_ex_);
  P_tmp.block(0, 0, dim_x_, dim_x_) = A * P_.block(0, 0, dim_x_, dim_x_) * A.transpose();
  P_tmp.block(0, dim_x_, dim_x_, d_dim_x) = A * P_.block(0, 0, dim_x_, d_dim_x);
  P_tmp.block(dim_x_, 0, d_dim_x, dim_x_) = P_.block(0, 0, d_dim_x, dim_x_) * A.transpose();
  P_tmp.block(dim_x_, dim_x_, d_dim_x, d_dim_x) = P_.block(0, 0, d_dim_x, d_dim_x);
  P_tmp += Q_ex;
  P_ = P_tmp;

  return true;
};

bool KalmanFilterDelayedMeasurement::updateWithDelay(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                                                     const Eigen::MatrixXd &R, const int delay_step)
{
  if (delay_step >= max_delay_step_)
  {
    printf("delay step is larger than max_delay_step. ignore update.\n");
    return false;
  }

  const int dim_y = y.rows();

  /* set measurement matrix */
  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(dim_y, dim_x_ex_);
  C_ex.block(0, dim_x_ * delay_step, dim_y, dim_x_) = C;

  /* update */
  if (!update(y, C_ex, R))
    return false;

  return true;
};