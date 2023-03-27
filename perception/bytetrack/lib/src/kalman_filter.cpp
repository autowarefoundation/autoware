/*
 * MIT License
 *
 * Copyright (c) 2021 Yifu Zhang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Copyright 2023 TIER IV, Inc.
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

#include "kalman_filter.h"

#include <eigen3/Eigen/Cholesky>

namespace byte_kalman
{
const double KalmanFilter::chi2inv95[10] = {0,      3.8415, 5.9915, 7.8147, 9.4877,
                                            11.070, 12.592, 14.067, 15.507, 16.919};
KalmanFilter::KalmanFilter()
{
  int ndim = 4;
  double dt = 1.;

  _motion_mat = Eigen::MatrixXf::Identity(8, 8);
  for (int i = 0; i < ndim; i++) {
    _motion_mat(i, ndim + i) = dt;
  }
  _update_mat = Eigen::MatrixXf::Identity(4, 8);

  this->_std_weight_position = 1. / 20;
  this->_std_weight_velocity = 1. / 160;
}

KAL_DATA KalmanFilter::initiate(const DETECTBOX & measurement)
{
  DETECTBOX mean_pos = measurement;
  DETECTBOX mean_vel;
  for (int i = 0; i < 4; i++) mean_vel(i) = 0;

  KAL_MEAN mean;
  for (int i = 0; i < 8; i++) {
    if (i < 4)
      mean(i) = mean_pos(i);
    else
      mean(i) = mean_vel(i - 4);
  }

  KAL_MEAN std;
  std(0) = 2 * _std_weight_position * measurement[3];
  std(1) = 2 * _std_weight_position * measurement[3];
  std(2) = 1e-2;
  std(3) = 2 * _std_weight_position * measurement[3];
  std(4) = 10 * _std_weight_velocity * measurement[3];
  std(5) = 10 * _std_weight_velocity * measurement[3];
  std(6) = 1e-5;
  std(7) = 10 * _std_weight_velocity * measurement[3];

  KAL_MEAN tmp = std.array().square();
  KAL_COVA var = tmp.asDiagonal();
  return std::make_pair(mean, var);
}

void KalmanFilter::predict(KAL_MEAN & mean, KAL_COVA & covariance)
{
  // revise the data;
  DETECTBOX std_pos;
  std_pos << _std_weight_position * mean(3), _std_weight_position * mean(3), 1e-2,
    _std_weight_position * mean(3);
  DETECTBOX std_vel;
  std_vel << _std_weight_velocity * mean(3), _std_weight_velocity * mean(3), 1e-5,
    _std_weight_velocity * mean(3);
  KAL_MEAN tmp;
  tmp.block<1, 4>(0, 0) = std_pos;
  tmp.block<1, 4>(0, 4) = std_vel;
  tmp = tmp.array().square();
  KAL_COVA motion_cov = tmp.asDiagonal();
  KAL_MEAN mean1 = this->_motion_mat * mean.transpose();
  KAL_COVA covariance1 = this->_motion_mat * covariance * (_motion_mat.transpose());
  covariance1 += motion_cov;

  mean = mean1;
  covariance = covariance1;
}

KAL_HDATA KalmanFilter::project(const KAL_MEAN & mean, const KAL_COVA & covariance)
{
  DETECTBOX std;
  std << _std_weight_position * mean(3), _std_weight_position * mean(3), 1e-1,
    _std_weight_position * mean(3);
  KAL_HMEAN mean1 = _update_mat * mean.transpose();
  KAL_HCOVA covariance1 = _update_mat * covariance * (_update_mat.transpose());
  Eigen::Matrix<float, 4, 4> diag = std.asDiagonal();
  diag = diag.array().square().matrix();
  covariance1 += diag;
  //    covariance1.diagonal() << diag;
  return std::make_pair(mean1, covariance1);
}

KAL_DATA
KalmanFilter::update(
  const KAL_MEAN & mean, const KAL_COVA & covariance, const DETECTBOX & measurement)
{
  KAL_HDATA pa = project(mean, covariance);
  KAL_HMEAN projected_mean = pa.first;
  KAL_HCOVA projected_cov = pa.second;

  // chol_factor, lower =
  // scipy.linalg.cho_factor(projected_cov, lower=True, check_finite=False)
  // kalmain_gain =
  // scipy.linalg.cho_solve((cho_factor, lower),
  // np.dot(covariance, self._upadte_mat.T).T,
  // check_finite=False).T
  Eigen::Matrix<float, 4, 8> B = (covariance * (_update_mat.transpose())).transpose();
  Eigen::Matrix<float, 8, 4> kalman_gain = (projected_cov.llt().solve(B)).transpose();  // eg.8x4
  Eigen::Matrix<float, 1, 4> innovation = measurement - projected_mean;                 // eg.1x4
  auto tmp = innovation * (kalman_gain.transpose());
  KAL_MEAN new_mean = (mean.array() + tmp.array()).matrix();
  KAL_COVA new_covariance = covariance - kalman_gain * projected_cov * (kalman_gain.transpose());
  return std::make_pair(new_mean, new_covariance);
}

Eigen::Matrix<float, 1, -1> KalmanFilter::gating_distance(
  const KAL_MEAN & mean, const KAL_COVA & covariance, const std::vector<DETECTBOX> & measurements,
  bool only_position)
{
  KAL_HDATA pa = this->project(mean, covariance);
  if (only_position) {
    printf("not implement!");
    exit(0);
  }
  KAL_HMEAN mean1 = pa.first;
  KAL_HCOVA covariance1 = pa.second;

  //    Eigen::Matrix<float, -1, 4, Eigen::RowMajor> d(size, 4);
  DETECTBOXSS d(measurements.size(), 4);
  int pos = 0;
  for (DETECTBOX box : measurements) {
    d.row(pos++) = box - mean1;
  }
  Eigen::Matrix<float, -1, -1, Eigen::RowMajor> factor = covariance1.llt().matrixL();
  Eigen::Matrix<float, -1, -1> z =
    factor.triangularView<Eigen::Lower>().solve<Eigen::OnTheRight>(d).transpose();
  auto zz = ((z.array()) * (z.array())).matrix();
  auto square_maha = zz.colwise().sum();
  return square_maha;
}
}  // namespace byte_kalman
