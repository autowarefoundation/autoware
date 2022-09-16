// Copyright 2022 Autoware Foundation
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

#include "ekf_localizer/mahalanobis.hpp"

double squaredMahalanobis(
  const Eigen::VectorXd & x, const Eigen::VectorXd & y, const Eigen::MatrixXd & C)
{
  const Eigen::VectorXd d = x - y;
  return d.dot(C.inverse() * d);
}

bool mahalanobisGate(
  const double & dist_max, const Eigen::MatrixXd & x, const Eigen::MatrixXd & obj_x,
  const Eigen::MatrixXd & cov)
{
  return squaredMahalanobis(x, obj_x, cov) <= dist_max * dist_max;
}
