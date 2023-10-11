// Copyright 2023 The Autoware Foundation
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

#include "kalman_filter/time_delay_kalman_filter.hpp"

#include <gtest/gtest.h>

TEST(time_delay_kalman_filter, td_kf)
{
  TimeDelayKalmanFilter td_kf_;

  Eigen::MatrixXd x_t(3, 1);
  x_t << 1.0, 2.0, 3.0;
  Eigen::MatrixXd P_t(3, 3);
  P_t << 0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3;
  const int max_delay_step = 5;
  const int dim_x = x_t.rows();
  const int dim_x_ex = dim_x * max_delay_step;
  // Initialize the filter
  td_kf_.init(x_t, P_t, max_delay_step);

  // Check if initialization was successful
  Eigen::MatrixXd x_init = td_kf_.getLatestX();
  Eigen::MatrixXd P_init = td_kf_.getLatestP();
  Eigen::MatrixXd x_ex_t = Eigen::MatrixXd::Zero(dim_x_ex, 1);
  Eigen::MatrixXd P_ex_t = Eigen::MatrixXd::Zero(dim_x_ex, dim_x_ex);
  for (int i = 0; i < max_delay_step; ++i) {
    x_ex_t.block(i * dim_x, 0, dim_x, 1) = x_t;
    P_ex_t.block(i * dim_x, i * dim_x, dim_x, dim_x) = P_t;
  }

  EXPECT_EQ(x_init.rows(), 3);
  EXPECT_EQ(x_init.cols(), 1);
  EXPECT_EQ(P_init.rows(), 3);
  EXPECT_EQ(P_init.cols(), 3);
  EXPECT_NEAR(x_init(0, 0), x_t(0, 0), 1e-5);
  EXPECT_NEAR(x_init(1, 0), x_t(1, 0), 1e-5);
  EXPECT_NEAR(x_init(2, 0), x_t(2, 0), 1e-5);
  EXPECT_NEAR(P_init(0, 0), P_t(0, 0), 1e-5);
  EXPECT_NEAR(P_init(1, 1), P_t(1, 1), 1e-5);
  EXPECT_NEAR(P_init(2, 2), P_t(2, 2), 1e-5);

  // Define prediction parameters
  Eigen::MatrixXd A_t(3, 3);
  A_t << 2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0;
  Eigen::MatrixXd Q_t(3, 3);
  Q_t << 0.01, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.03;
  Eigen::MatrixXd x_next(3, 1);
  x_next << 2.0, 4.0, 6.0;

  // Perform prediction
  EXPECT_TRUE(td_kf_.predictWithDelay(x_next, A_t, Q_t));

  // Check the prediction state and covariance matrix
  Eigen::MatrixXd x_predict = td_kf_.getLatestX();
  Eigen::MatrixXd P_predict = td_kf_.getLatestP();
  Eigen::MatrixXd x_tmp = Eigen::MatrixXd::Zero(dim_x_ex, 1);
  x_tmp.block(0, 0, dim_x, 1) = A_t * x_t;
  x_tmp.block(dim_x, 0, dim_x_ex - dim_x, 1) = x_ex_t.block(0, 0, dim_x_ex - dim_x, 1);
  x_ex_t = x_tmp;
  Eigen::MatrixXd x_predict_expected = x_ex_t.block(0, 0, dim_x, 1);
  Eigen::MatrixXd P_tmp = Eigen::MatrixXd::Zero(dim_x_ex, dim_x_ex);
  P_tmp.block(0, 0, dim_x, dim_x) = A_t * P_ex_t.block(0, 0, dim_x, dim_x) * A_t.transpose() + Q_t;
  P_tmp.block(0, dim_x, dim_x, dim_x_ex - dim_x) =
    A_t * P_ex_t.block(0, 0, dim_x, dim_x_ex - dim_x);
  P_tmp.block(dim_x, 0, dim_x_ex - dim_x, dim_x) =
    P_ex_t.block(0, 0, dim_x_ex - dim_x, dim_x) * A_t.transpose();
  P_tmp.block(dim_x, dim_x, dim_x_ex - dim_x, dim_x_ex - dim_x) =
    P_ex_t.block(0, 0, dim_x_ex - dim_x, dim_x_ex - dim_x);
  P_ex_t = P_tmp;
  Eigen::MatrixXd P_predict_expected = P_ex_t.block(0, 0, dim_x, dim_x);
  EXPECT_NEAR(x_predict(0, 0), x_predict_expected(0, 0), 1e-5);
  EXPECT_NEAR(x_predict(1, 0), x_predict_expected(1, 0), 1e-5);
  EXPECT_NEAR(x_predict(2, 0), x_predict_expected(2, 0), 1e-5);
  EXPECT_NEAR(P_predict(0, 0), P_predict_expected(0, 0), 1e-5);
  EXPECT_NEAR(P_predict(1, 1), P_predict_expected(1, 1), 1e-5);
  EXPECT_NEAR(P_predict(2, 2), P_predict_expected(2, 2), 1e-5);

  // Define update parameters
  Eigen::MatrixXd C_t(3, 3);
  C_t << 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5;
  Eigen::MatrixXd R_t(3, 3);
  R_t << 0.001, 0.0, 0.0, 0.0, 0.002, 0.0, 0.0, 0.0, 0.003;
  Eigen::MatrixXd y_t(3, 1);
  y_t << 1.05, 2.05, 3.05;
  const int delay_step = 2;  // Choose an appropriate delay step
  const int dim_y = y_t.rows();

  // Perform update
  EXPECT_TRUE(td_kf_.updateWithDelay(y_t, C_t, R_t, delay_step));

  // Check the updated state and covariance matrix
  Eigen::MatrixXd x_update = td_kf_.getLatestX();
  Eigen::MatrixXd P_update = td_kf_.getLatestP();

  Eigen::MatrixXd C_ex_t = Eigen::MatrixXd::Zero(dim_y, dim_x_ex);
  const Eigen::MatrixXd PCT_t = P_ex_t * C_ex_t.transpose();
  const Eigen::MatrixXd K_t = PCT_t * ((R_t + C_ex_t * PCT_t).inverse());
  const Eigen::MatrixXd y_pred = C_ex_t * x_ex_t;
  x_ex_t = x_ex_t + K_t * (y_t - y_pred);
  P_ex_t = P_ex_t - K_t * (C_ex_t * P_ex_t);
  Eigen::MatrixXd x_update_expected = x_ex_t.block(0, 0, dim_x, 1);
  Eigen::MatrixXd P_update_expected = P_ex_t.block(0, 0, dim_x, dim_x);
  EXPECT_NEAR(x_update(0, 0), x_update_expected(0, 0), 1e-5);
  EXPECT_NEAR(x_update(1, 0), x_update_expected(1, 0), 1e-5);
  EXPECT_NEAR(x_update(2, 0), x_update_expected(2, 0), 1e-5);
  EXPECT_NEAR(P_update(0, 0), P_update_expected(0, 0), 1e-5);
  EXPECT_NEAR(P_update(1, 1), P_update_expected(1, 1), 1e-5);
  EXPECT_NEAR(P_update(2, 2), P_update_expected(2, 2), 1e-5);
}
