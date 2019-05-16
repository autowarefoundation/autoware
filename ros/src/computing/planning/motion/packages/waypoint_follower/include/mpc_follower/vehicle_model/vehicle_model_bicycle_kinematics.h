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

#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

class KinematicsBicycleModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param wheelbase wheelbase length [m]
   * @param steer_lim_deg steering angle limit [deg]
   * @param steer_tau steering time constant for 1d-model
   */
  KinematicsBicycleModel(const double &wheelbase_, const double &steer_lim_deg, const double &steer_tau);

  /**
   * @brief destructor
   */
  ~KinematicsBicycleModel();

  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk 
   * @param Ad coefficient matrix
   * @param Bd coefficient matrix
   * @param Cd coefficient matrix
   * @param Wd coefficient matrix
   * @param dt Discretization time
   */
  void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                               Eigen::MatrixXd &Cd, Eigen::MatrixXd &Wd, double &dt) override;

  /**
   * @brief calculate reference input
   * @param [out] reference input
   */
  void calculateReferenceInput(Eigen::MatrixXd &Uref) override;

private:
  double wheelbase_;     //!< @brief wheelbase length [m]
  double steer_lim_deg_; //!< @brief steering angle limit [deg]
  double steer_tau_;     //!< @brief steering time constant for 1d-model
};
