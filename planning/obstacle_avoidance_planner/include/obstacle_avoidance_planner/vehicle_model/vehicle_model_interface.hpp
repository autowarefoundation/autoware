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

#ifndef OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <vector>

class VehicleModelInterface
{
protected:
  const int dim_x_;
  const int dim_u_;
  const int dim_y_;
  const double wheelbase_;
  const double steer_limit_;

public:
  VehicleModelInterface(
    const int dim_x, const int dim_u, const int dim_y, const double wheelbase,
    const double steer_limit);
  virtual ~VehicleModelInterface() = default;

  int getDimX() const;
  int getDimU() const;
  int getDimY() const;

  virtual void calculateStateEquationMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double k,
    const double ds) const = 0;
};

#endif  // OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
