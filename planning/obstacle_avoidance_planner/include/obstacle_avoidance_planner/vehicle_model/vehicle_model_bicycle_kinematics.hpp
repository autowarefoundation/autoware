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

/**
 * @file vehicle_model_bicycle_dynamics.h
 * @brief vehicle model class of bicycle kinematics
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

/*
 *    Representation
 * e      : lateral error
 * th     : heading angle error
 * steer  : steering angle
 * steer_d: desired steering angle (input)
 * v      : velocity
 * W      : wheelbase length
 * tau    : time constant for steering dynamics
 *
 *    State & Input
 * x = [e, th, steer]^T
 * u = steer_d
 *
 *    Nonlinear model
 * dx1/dt = v * sin(x2)
 * dx2/dt = v * tan(x3) / W
 * dx3/dt = -(x3 - u) / tau
 *
 *    Linearized model around reference point (v = v_r, th = th_r, steer = steer_r)
 *         [0,  vr,                   0]       [    0]       [                           0]
 * dx/dt = [0,   0, vr/W/cos(steer_r)^2] * x + [    0] * u + [-vr*steer_r/W/cos(steer_r)^2]
 *         [0,   0,               1/tau]       [1/tau]       [                           0]
 *
 */

#ifndef OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_
#define OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "obstacle_avoidance_planner/vehicle_model/vehicle_model_interface.hpp"

#include <vector>

/**
 * @class vehicle model class of bicycle kinematics
 * @brief calculate model-related values
 */
class KinematicsBicycleModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim steering angle limit [rad]
   */
  KinematicsBicycleModel(const double wheel_base, const double steer_limit);

  /**
   * @brief destructor
   */
  virtual ~KinematicsBicycleModel() = default;

  /**
   * @brief calculate discrete kinematics equation matrix of x_k+1 = Ad * x_k + Bd * uk + Wd
   * @param [out] Ad coefficient matrix
   * @param [out] Bd coefficient matrix
   * @param [out] Wd coefficient matrix
   * @param [in] ds discretization arc length
   */
  void calculateStateEquationMatrix(
    Eigen::MatrixXd & Ad, Eigen::MatrixXd & Bd, Eigen::MatrixXd & Wd, const double ds) override;

  /**
   * @brief calculate discrete observation matrix of y_k = Cd * x_k
   * @param [out] Cd coefficient matrix
   */
  void calculateObservationMatrix(Eigen::MatrixXd & Cd) override;

  /**
   * @brief calculate discrete observation matrix of y_k = Cd * x_k
   * @param [out] Cd_vec sparse matrix information of coefficient matrix
   */
  void calculateObservationSparseMatrix(std::vector<Eigen::Triplet<double>> & Cd_vec) override;

  /**
   * @brief calculate reference input
   * @param [out] reference input
   */
  void calculateReferenceInput(Eigen::MatrixXd * Uref) override;
};
#endif  // OBSTACLE_AVOIDANCE_PLANNER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_KINEMATICS_HPP_
