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

#include "mpc_lateral_controller/qp_solver/qp_solver_unconstraint_fast.hpp"

#include <Eigen/Dense>

namespace autoware::motion::control::mpc_lateral_controller
{
QPSolverEigenLeastSquareLLT::QPSolverEigenLeastSquareLLT()
{
}
bool QPSolverEigenLeastSquareLLT::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & /*a*/,
  const Eigen::VectorXd & /*lb*/, const Eigen::VectorXd & /*ub*/, const Eigen::VectorXd & /*lb_a*/,
  const Eigen::VectorXd & /*ub_a*/, Eigen::VectorXd & u)
{
  if (std::fabs(h_mat.determinant()) < 1.0E-9) {
    return false;
  }

  u = -h_mat.llt().solve(f_vec);

  return true;
}
}  // namespace autoware::motion::control::mpc_lateral_controller
