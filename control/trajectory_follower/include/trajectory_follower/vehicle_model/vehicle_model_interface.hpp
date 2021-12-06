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

#ifndef TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
#define TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_

#include "common/types.hpp"
#include "eigen3/Eigen/Core"
#include "trajectory_follower/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
using autoware::common::types::float64_t;
/**
 * Vehicle model class
 * @brief calculate model-related values
 */
class TRAJECTORY_FOLLOWER_PUBLIC VehicleModelInterface
{
protected:
  const int64_t m_dim_x;   //!< @brief dimension of state x
  const int64_t m_dim_u;   //!< @brief dimension of input u
  const int64_t m_dim_y;   //!< @brief dimension of output y
  float64_t m_velocity;   //!< @brief vehicle velocity [m/s]
  float64_t m_curvature;  //!< @brief curvature on the linearized point on path
  float64_t m_wheelbase;  //!< @brief wheelbase of the vehicle [m]

public:
  /**
   * @brief constructor
   * @param [in] dim_x dimension of state x
   * @param [in] dim_u dimension of input u
   * @param [in] dim_y dimension of output y
   * @param [in] wheelbase wheelbase of the vehicle [m]
   */
  VehicleModelInterface(int64_t dim_x, int64_t dim_u, int64_t dim_y, float64_t wheelbase);

  /**
   * @brief destructor
   */
  virtual ~VehicleModelInterface() = default;

  /**
   * @brief get state x dimension
   * @return state dimension
   */
  int64_t getDimX();

  /**
   * @brief get input u dimension
   * @return input dimension
   */
  int64_t getDimU();

  /**
   * @brief get output y dimension
   * @return output dimension
   */
  int64_t getDimY();

  /**
   * @brief get wheelbase of the vehicle
   * @return wheelbase value [m]
   */
  float64_t getWheelbase();

  /**
   * @brief set velocity
   * @param [in] velocity vehicle velocity
   */
  void setVelocity(const float64_t velocity);

  /**
   * @brief set curvature
   * @param [in] curvature curvature on the linearized point on path
   */
  void setCurvature(const float64_t curvature);

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
    const float64_t dt) = 0;

  /**
   * @brief calculate reference input
   * @param [out] u_ref input
   */
  virtual void calculateReferenceInput(Eigen::MatrixXd & u_ref) = 0;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__VEHICLE_MODEL__VEHICLE_MODEL_INTERFACE_HPP_
