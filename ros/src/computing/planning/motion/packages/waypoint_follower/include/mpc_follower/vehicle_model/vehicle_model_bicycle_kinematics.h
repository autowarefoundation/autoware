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
 * steer_d: desired steering agle (input)
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


#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

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
   * @param [in] steer_tau steering time constant for 1d-model
   */
  KinematicsBicycleModel(const double &wheelbase, const double &steer_lim, const double &steer_tau);

  /**
   * @brief destructor
   */
  ~KinematicsBicycleModel();

  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk 
   * @param [out] Ad coefficient matrix
   * @param [out] Bd coefficient matrix
   * @param [out] Cd coefficient matrix
   * @param [out] Wd coefficient matrix
   * @param [in] dt Discretization time
   */
  void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                               Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, const double &dt) override;

  /**
   * @brief calculate reference input
   * @param [out] reference input
   */
  void calculateReferenceInput(Eigen::MatrixXd &Uref) override;

private:
  double wheelbase_; //!< @brief wheelbase length [m]
  double steer_lim_; //!< @brief steering angle limit [rad]
  double steer_tau_; //!< @brief steering time constant for 1d-model
};
