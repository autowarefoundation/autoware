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

#pragma once

#include "data_type.h"

#include <vector>
namespace byte_kalman
{
class KalmanFilter
{
public:
  static const double chi2inv95[10];
  KalmanFilter();
  KAL_DATA initiate(const DETECTBOX & measurement);
  void predict(KAL_MEAN & mean, KAL_COVA & covariance);
  KAL_HDATA project(const KAL_MEAN & mean, const KAL_COVA & covariance);
  KAL_DATA update(
    const KAL_MEAN & mean, const KAL_COVA & covariance, const DETECTBOX & measurement);

  Eigen::Matrix<float, 1, -1> gating_distance(
    const KAL_MEAN & mean, const KAL_COVA & covariance, const std::vector<DETECTBOX> & measurements,
    bool only_position = false);

private:
  Eigen::Matrix<float, 8, 8, Eigen::RowMajor> _motion_mat;
  Eigen::Matrix<float, 4, 8, Eigen::RowMajor> _update_mat;
  float _std_weight_position;
  float _std_weight_velocity;
};
}  // namespace byte_kalman
