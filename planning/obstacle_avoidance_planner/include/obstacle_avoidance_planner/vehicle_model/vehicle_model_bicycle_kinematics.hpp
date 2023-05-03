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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_

#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_interface.hpp"

#include <Eigen/Core>
#include <Eigen/LU>

#include <vector>

class KinematicsBicycleModel : public VehicleModelInterface
{
public:
  KinematicsBicycleModel(const double wheelbase, const double steer_limit);
  virtual ~KinematicsBicycleModel() = default;

  void calculateStateEquationMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double curvature,
    const double ds) const override;
};
#endif  // OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_
