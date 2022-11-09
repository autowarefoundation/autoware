// Copyright 2018-2021 The Autoware Foundation
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
 *          [0,                   1,                0,                        0]       [       0] [
 * 0] dx/dt = [0,
 * -(cf+cr)/m/vr,        (cf+cr)/m,       (lr*cr-lf*cf)/m/vr] * x + [    cf/m] * u +
 * [(lr*cr-lf*cf)/m/vr*k - vr*k] [0, 0,                0,                        1]       [       0]
 * [                          0] [0, (lr*cr-lf*cf)/Iz/vr, (lf*cf-lr*cr)/Iz,
 * -(lf^2*cf+lr^2*cr)/Iz/vr]       [lf*cf/Iz]       [   -(lf^2*cf+lr^2*cr)/Iz/vr]
 *
 * Reference : Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path
 * Tracking", Robotics Institute, Carnegie Mellon University, February 2009.
 */

#ifndef TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_DYNAMICS_HPP_
#define TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_DYNAMICS_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"
#include "trajectory_follower/vehicle_model/vehicle_model_interface.hpp"
#include "trajectory_follower/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

/**
 * Vehicle model class of bicycle dynamics
 * @brief calculate model-related values
 */
class TRAJECTORY_FOLLOWER_PUBLIC DynamicsBicycleModel : public VehicleModelInterface
{
public:
  /**
   * @brief constructor with parameter initialization
   * @param [in] wheelbase wheelbase length [m]
   * @param [in] mass_fl mass applied to front left tire [kg]
   * @param [in] mass_fr mass applied to front right tire [kg]
   * @param [in] mass_rl mass applied to rear left tire [kg]
   * @param [in] mass_rr mass applied to rear right tire [kg]
   * @param [in] cf front cornering power [N/rad]
   * @param [in] cr rear cornering power [N/rad]
   */
  DynamicsBicycleModel(
    const double wheelbase, const double mass_fl, const double mass_fr, const double mass_rl,
    const double mass_rr, const double cf, const double cr);

  /**
   * @brief destructor
   */
  ~DynamicsBicycleModel() = default;

  /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk + w_d, yk = c_d * xk
   * @param [in] a_d coefficient matrix
   * @param [in] b_d coefficient matrix
   * @param [in] c_d coefficient matrix
   * @param [in] w_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
  void calculateDiscreteMatrix(
    Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & w_d, Eigen::MatrixXd & c_d,
    const double dt) override;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  void calculateReferenceInput(Eigen::MatrixXd & u_ref) override;

private:
  double m_lf;    //!< @brief length from center of mass to front wheel [m]
  double m_lr;    //!< @brief length from center of mass to rear wheel [m]
  double m_mass;  //!< @brief total mass of vehicle [kg]
  double m_iz;    //!< @brief moment of inertia [kg * m2]
  double m_cf;    //!< @brief front cornering power [N/rad]
  double m_cr;    //!< @brief rear cornering power [N/rad]
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_BICYCLE_DYNAMICS_HPP_
