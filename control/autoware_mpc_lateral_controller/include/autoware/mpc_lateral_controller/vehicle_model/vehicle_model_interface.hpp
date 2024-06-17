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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_

#include "autoware/mpc_lateral_controller/mpc_trajectory.hpp"

#include <Eigen/Core>

#include <string>

namespace autoware::motion::control::mpc_lateral_controller
{

/**
 * Vehicle model class
 * @brief calculate model-related values
 */
class VehicleModelInterface
{
protected:
  const int m_dim_x;   //!< @brief dimension of state x
  const int m_dim_u;   //!< @brief dimension of input u
  const int m_dim_y;   //!< @brief dimension of output y
  double m_velocity;   //!< @brief vehicle velocity [m/s]
  double m_curvature;  //!< @brief curvature on the linearized point on path
  double m_wheelbase;  //!< @brief wheelbase of the vehicle [m]

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   * @param [in] dim_y dimension of output y
   * @param [in] wheelbase wheelbase of the vehicle [m]
   */
  VehicleModelInterface(int dim_x, int dim_u, int dim_y, double wheelbase);

  /**
   * @brief destructor
   */
  virtual ~VehicleModelInterface() = default;

  /**
   * @brief get state x dimension
   * @return state dimension
   */
  int getDimX() const;

  /**
   * @brief get input u dimension
   * @return input dimension
   */
  int getDimU() const;

  /**
   * @brief get output y dimension
   * @return output dimension
   */
  int getDimY() const;

  /**
   * @brief get wheelbase of the vehicle
   * @return wheelbase value [m]
   */
  double getWheelbase() const;

  /**
   * @brief set velocity
   * @param [in] velocity vehicle velocity
   */
  void setVelocity(const double velocity);

  /**
   * @brief set curvature
   * @param [in] curvature curvature on the linearized point on path
   */
  void setCurvature(const double curvature);

  /**
   * @brief calculate discrete model matrix of x_k+1 = a_d * xk + b_d * uk + w_d, yk = c_d * xk
   * @param [out] a_d coefficient matrix
   * @param [out] b_d coefficient matrix
   * @param [out] c_d coefficient matrix
   * @param [out] w_d coefficient matrix
   * @param [in] dt Discretization time [s]
   */
  virtual void calculateDiscreteMatrix(
    Eigen::MatrixXd & a_d, Eigen::MatrixXd & b_d, Eigen::MatrixXd & c_d, Eigen::MatrixXd & w_d,
    const double dt) = 0;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  virtual void calculateReferenceInput(Eigen::MatrixXd & u_ref) = 0;

  /**
   * @brief returns model name e.g. kinematics, dynamics
   */
  virtual std::string modelName() = 0;

  /**
   * @brief Calculate the predicted trajectory for the ego vehicle based on the MPC result in world
   * coordinate
   * @param a_d The MPC A matrix used for optimization.
   * @param b_d The MPC B matrix used for optimization.
   * @param c_d The MPC C matrix used for optimization.
   * @param w_d The MPC W matrix used for optimization.
   * @param x0 initial state vector.
   * @param Uex The optimized input vector.
   * @param reference_trajectory The resampled reference trajectory.
   * @param dt delta time used in the optimization
   * @return The predicted trajectory.
   */
  virtual MPCTrajectory calculatePredictedTrajectoryInWorldCoordinate(
    const Eigen::MatrixXd & a_d, const Eigen::MatrixXd & b_d, const Eigen::MatrixXd & c_d,
    const Eigen::MatrixXd & w_d, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
    const MPCTrajectory & reference_trajectory, const double dt) const = 0;

  /**
   * @brief Calculate the predicted trajectory for the ego vehicle based on the MPC result in Frenet
   * Coordinate
   * @param a_d The MPC A matrix used for optimization.
   * @param b_d The MPC B matrix used for optimization.
   * @param c_d The MPC C matrix used for optimization.
   * @param w_d The MPC W matrix used for optimization.
   * @param x0 initial state vector.
   * @param Uex The optimized input vector.
   * @param reference_trajectory The resampled reference trajectory.
   * @param dt delta time used in the optimization
   * @return The predicted trajectory.
   */
  virtual MPCTrajectory calculatePredictedTrajectoryInFrenetCoordinate(
    const Eigen::MatrixXd & a_d, const Eigen::MatrixXd & b_d, const Eigen::MatrixXd & c_d,
    const Eigen::MatrixXd & w_d, const Eigen::MatrixXd & x0, const Eigen::MatrixXd & Uex,
    const MPCTrajectory & reference_trajectory, const double dt) const = 0;
};
}  // namespace autoware::motion::control::mpc_lateral_controller
#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
