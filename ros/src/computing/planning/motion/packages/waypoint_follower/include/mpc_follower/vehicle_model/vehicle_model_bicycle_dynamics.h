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

/*
 *    Representation
 * e      : lateral error
 * de     : derivative of lateral error
 * th     : heading angle error 
 * dth    : derivative of heading angle error 
 * steer  : steering angle (input)
 * v      : velocity 
 * m      : mass
 * Iz     : inertia
 * lf     : length from center to front tire
 * lr     : length from center to rear tire
 * cf     : front tire cornering power
 * cr     : rear tire cornering power
 * k      : curvature on reference trajectory point
 * 
 *    State & Input
 * x = [e, de, th, dth]^T
 * u = steer
 * 
 *    Linearized model around reference point (v=vr)
 *          [0,                   1,                0,                        0]       [       0]       [                          0]
 *  dx/dt = [0,       -(cf+cr)/m/vr,        (cf+cr)/m,       (lr*cr-lf*cf)/m/vr] * x + [    cf/m] * u + [(lr*cr-lf*cf)/m/vr*k - vr*k]
 *          [0,                   0,                0,                        1]       [       0]       [                          0]
 *          [0, (lr*cr-lf*cf)/Iz/vr, (lf*cf-lr*cr)/Iz, -(lf^2*cf+lr^2*cr)/Iz/vr]       [lf*cf/Iz]       [   -(lf^2*cf+lr^2*cr)/Iz/vr]
 * 
 * Reference : Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", Robotics Institute, Carnegie Mellon University, February 2009.
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
   * @param [in] mass_fl mass applied to front left tire [kg]
   * @param [in] mass_fr mass applied to front right tire [kg]
   * @param [in] mass_rl mass applied to rear left tire [kg]
   * @param [in] mass_rr mass applied to rear right tire [kg]
   * @param [in] cf front cornering power
   * @param [in] cr rear cornering power
   */
  DynamicsBicycleModel(double &wheelbase, double &mass_fl, double &mass_fr,
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
  double wheelbase_; //!< @brief wheelbase length [m]
  double lf_;        //!< @brief length from centor of mass to front wheel [m]
  double lr_;        //!< @brief length from centor of mass to rear wheel [m]
  double mass_;      //!< @brief total mass of vehicle [kg]
  double iz_;        //!< @brief moment of inertia [kg * m2]
  double cf_;        //!< @brief front cornering power [N/rad]
  double cr_;        //!< @brief rear cornering power [N/rad]
};
