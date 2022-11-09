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

#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"

#include <string>
#include <vector>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
QPSolverOSQP::QPSolverOSQP(const rclcpp::Logger & logger) : logger_{logger} {}
bool QPSolverOSQP::solve(
  const Eigen::MatrixXd & h_mat, const Eigen::MatrixXd & f_vec, const Eigen::MatrixXd & a,
  const Eigen::VectorXd & lb, const Eigen::VectorXd & ub, const Eigen::VectorXd & lb_a,
  const Eigen::VectorXd & ub_a, Eigen::VectorXd & u)
{
  const Eigen::Index raw_a = a.rows();
  const Eigen::Index col_a = a.cols();
  const Eigen::Index dim_u = ub.size();
  Eigen::MatrixXd Identity = Eigen::MatrixXd::Identity(dim_u, dim_u);

  // convert matrix to vector for osqpsolver
  std::vector<double> f(&f_vec(0), f_vec.data() + f_vec.cols() * f_vec.rows());

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;

  for (int i = 0; i < dim_u; ++i) {
    lower_bound.push_back(lb(i));
    upper_bound.push_back(ub(i));
  }

  for (int i = 0; i < col_a; ++i) {
    lower_bound.push_back(lb_a(i));
    upper_bound.push_back(ub_a(i));
  }

  Eigen::MatrixXd osqpA = Eigen::MatrixXd(dim_u + col_a, raw_a);
  osqpA << Identity, a;

  /* execute optimization */
  auto result = osqpsolver_.optimize(h_mat, osqpA, f, lower_bound, upper_bound);

  std::vector<double> U_osqp = std::get<0>(result);
  u = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>(
    &U_osqp[0], static_cast<Eigen::Index>(U_osqp.size()), 1);

  const int status_val = std::get<3>(result);
  if (status_val != 1) {
    // TODO(Horibe): Should return false and the failure must be handled in an appropriate way.
    RCLCPP_WARN(logger_, "optimization failed : %s", osqpsolver_.getStatusMessage().c_str());
  }

  // polish status: successful (1), unperformed (0), (-1) unsuccessful
  int status_polish = std::get<2>(result);
  if (status_polish == -1) {
    RCLCPP_WARN(logger_, "osqp status_polish = %d (unsuccessful)", status_polish);
    return false;
  }
  if (status_polish == 0) {
    RCLCPP_WARN(logger_, "osqp status_polish = %d (unperformed)", status_polish);
    return true;
  }
  return true;
}
}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware
