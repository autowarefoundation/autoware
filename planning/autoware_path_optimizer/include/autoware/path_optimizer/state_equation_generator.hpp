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

#ifndef AUTOWARE__PATH_OPTIMIZER__STATE_EQUATION_GENERATOR_HPP_
#define AUTOWARE__PATH_OPTIMIZER__STATE_EQUATION_GENERATOR_HPP_

#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "autoware/path_optimizer/vehicle_model/vehicle_model_interface.hpp"
#include "autoware/universe_utils/system/time_keeper.hpp"

#include <memory>
#include <vector>

namespace autoware::path_optimizer
{
struct ReferencePoint;

class StateEquationGenerator
{
public:
  struct Matrix
  {
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::VectorXd W;
  };

  StateEquationGenerator() = default;
  StateEquationGenerator(
    const double wheel_base, const double max_steer_rad,
    const std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper)
  : vehicle_model_ptr_(std::make_unique<KinematicsBicycleModel>(wheel_base, max_steer_rad)),
    time_keeper_(time_keeper)
  {
  }

  int getDimX() const { return vehicle_model_ptr_->getDimX(); }
  int getDimU() const { return vehicle_model_ptr_->getDimU(); }

  // time-series state equation: x = B u + W (u includes x_0)
  // NOTE: one-step state equation is x_t+1 = Ad x_t + Bd u + Wd.
  Matrix calcMatrix(const std::vector<ReferencePoint> & ref_points) const;

  Eigen::VectorXd predict(const Matrix & mat, const Eigen::VectorXd U) const;

private:
  std::unique_ptr<VehicleModelInterface> vehicle_model_ptr_;
  mutable std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;
};
}  // namespace autoware::path_optimizer
#endif  // AUTOWARE__PATH_OPTIMIZER__STATE_EQUATION_GENERATOR_HPP_
