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
 * @brief vehicle model class of bicycle dynamics
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "mpc_follower/vehicle_model/vehicle_model_interface.h"

/** 
 * @class vehicle model class of bicycle dynamics
 * @brief calculate model-related values
 */
class DynamicsBicycleModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] steer_lim_deg steering angle limit [deg]
   * @param [in] mass_fl mass applied to front left tire [kg]
   * @param [in] mass_fr mass applied to front right tire [kg]
   * @param [in] mass_rl mass applied to rear left tire [kg]
   * @param [in] mass_rr mass applied to rear right tire [kg]
   * @param [in] cf front cornering power
   * @param [in] cr rear cornering power
   */
  DynamicsBicycleModel(double &wheelbase, double &steer_lim_deg, double &mass_fl, double &mass_fr,
                       double &mass_rl, double &mass_rr, double &cf, double &cr);

  /**
   * @brief destructor
   */
  ~DynamicsBicycleModel();

  /**
   * @brief calculate discrete model matrix of x_k+1 = Ad * xk + Bd * uk + Wd, yk = Cd * xk 
   * @param [in] Ad coefficient matrix
   * @param [in] Bd coefficient matrix
   * @param [in] Cd coefficient matrix
   * @param [in] Wd coefficient matrix
   * @param [in] dt Discretization time
   */
  void calculateDiscreteMatrix(Eigen::MatrixXd &Ad, Eigen::MatrixXd &Bd,
                               Eigen::MatrixXd &Wd, Eigen::MatrixXd &Cd, const double &dt) override;

  /**
   * @brief calculate reference input
   * @param [out] reference input
   */
  void calculateReferenceInput(Eigen::MatrixXd &Uref) override;

private:
  double wheelbase_;     //!< @brief wheelbase length [m]
  double lf_;            //!< @brief length from centor of mass to front wheel [m]
  double lr_;            //!< @brief length from centor of mass to rear wheel [m]
  double steer_lim_deg_; //!< @brief steering angle limit [deg]
  double mass_;          //!< @brief total mass of vehicle [kg]
  double iz_;            //!< @brief moment of inertia [kg * m2]
  double cf_;            //!< @brief front cornering power [N/rad]
  double cr_;            //!< @brief rear cornering power [N/rad]
};
