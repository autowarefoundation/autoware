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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_

#include <Eigen/Core>

namespace autoware::motion::control::mpc_lateral_controller
{

/// Interface for solvers of Quadratic Programming (QP) problems
class QPSolverInterface
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
   * @return true if the problem was solved
   */
  virtual bool solve(
    const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
    const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
    const Eigen::VectorXd & ub_a, Eigen::VectorXd & u) = 0;

  virtual int64_t getTakenIter() const { return 0; }
  virtual double getRunTime() const { return 0.0; }
  virtual double getObjVal() const { return 0.0; }
};
}  // namespace autoware::motion::control::mpc_lateral_controller
#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__QP_SOLVER__QP_SOLVER_INTERFACE_HPP_
