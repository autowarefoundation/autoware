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

#ifndef TRAJECTORY_FOLLOWER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
#define TRAJECTORY_FOLLOWER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/LU"
#include "trajectory_follower/visibility_control.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{

/// Interface for solvers of Quadratic Programming (QP) problems
class TRAJECTORY_FOLLOWER_PUBLIC QPSolverInterface
{
public:
  /**
   * @brief destructor
   */
  virtual ~QPSolverInterface() = default;

  /**
   * @brief solve QP problem : minimize J = u' * h_mat * u + f_vec' * u without constraint
   * @param [in] h_mat parameter matrix in object function
   * @param [in] f_vec parameter matrix in object function
   * @param [in] a parameter matrix for constraint lb_a < a*u < ub_a
   * @param [in] lb parameter matrix for constraint lb < u < ub
   * @param [in] ub parameter matrix for constraint lb < u < ub
   * @param [in] lb_a parameter matrix for constraint lb_a < a*u < ub_a
   * @param [in] ub_a parameter matrix for constraint lb_a < a*u < ub_a
   * @param [out] u optimal variable vector
   * @return ture if the problem was solved
   */
  virtual bool solve(
    const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
    const Eigen::VectorXd & ub_a, Eigen::VectorXd & u) = 0;
};
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
#endif  // TRAJECTORY_FOLLOWER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
