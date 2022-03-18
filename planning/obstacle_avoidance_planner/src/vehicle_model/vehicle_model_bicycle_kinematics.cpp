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

#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_bicycle_kinematics.hpp"

#include <iostream>
#include <limits>
#include <vector>

KinematicsBicycleModel::KinematicsBicycleModel(const double wheel_base, const double steer_limit)
: VehicleModelInterface(/* dim_x */ 2, /* dim_u */ 1, /* dim_y */ 2, wheel_base, steer_limit)
{
}

void KinematicsBicycleModel::calculateStateEquationMatrix(
  Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double ds)
{
  // const double epsilon = std::numeric_limits<double>::epsilon();
  // constexpr double dt = 0.03;  // assuming delta time for steer tau

  /*
  const double lf = wheel_base_ - center_offset_from_base_;
  const double lr = center_offset_from_base_;
  */

  const double delta_r = std::atan(wheel_base_ * curvature_);
  const double cropped_delta_r = std::clamp(delta_r, -steer_limit_, steer_limit_);

  // NOTE: cos(delta_r) will not be zero since that happens only when curvature is infinity
  Ad << 1.0, ds,  //
    0.0, 1.0;

  Bd << 0.0, ds / wheel_base_ / std::pow(std::cos(delta_r), 2.0);

  Wd << 0.0, -ds * curvature_ + ds / wheel_base_ *
                                  (std::tan(cropped_delta_r) -
                                   cropped_delta_r / std::pow(std::cos(cropped_delta_r), 2.0));
}

void KinematicsBicycleModel::calculateObservationMatrix(Eigen::MatrixXd & Cd)
{
  Cd << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
}

void KinematicsBicycleModel::calculateObservationSparseMatrix(
  std::vector<Eigen::Triplet<double>> & Cd_vec)
{
  Cd_vec.clear();
  Cd_vec.push_back({0, 0, 1.0});
  Cd_vec.push_back({1, 1, 1.0});
}

void KinematicsBicycleModel::calculateReferenceInput(Eigen::MatrixXd * Uref)
{
  (*Uref)(0, 0) = std::atan(wheel_base_ * curvature_);
}
