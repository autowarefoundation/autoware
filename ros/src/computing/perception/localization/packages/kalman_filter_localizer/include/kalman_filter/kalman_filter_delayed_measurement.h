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

#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "kalman_filter/kalman_filter.h"

class KalmanFilterDelayedMeasurement : public KalmanFilter
{
public:
  KalmanFilterDelayedMeasurement();

  void init(const Eigen::MatrixXd &x, const Eigen::MatrixXd &P0, const int max_delay_step);

  void getCurrentX(Eigen::MatrixXd &x);
  void getCurrentP(Eigen::MatrixXd &P);

  bool predictWithDelay(const Eigen::MatrixXd &x_next, const Eigen::MatrixXd &A,
                        const Eigen::MatrixXd &Q);
  bool updateWithDelay(const Eigen::MatrixXd &y, const Eigen::MatrixXd &C,
                       const Eigen::MatrixXd &R, const int delay_step);

private:
  int max_delay_step_;
  int dim_x_;
  int dim_x_ex_;
};