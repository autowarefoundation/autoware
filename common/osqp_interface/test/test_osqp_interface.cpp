// Copyright 2021 The Autoware Foundation
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

#include "gtest/gtest.h"
#include "osqp_interface/osqp_interface.hpp"

#include <Eigen/Core>

#include <tuple>
#include <vector>

namespace
{
// Problem taken from https://github.com/osqp/osqp/blob/master/tests/basic_qp/generate_problem.py
//
// min  1/2 * x' * P * x  + q' * x
// s.t. lb <= A * x <= ub
//
// P = [4, 1], q = [1], A = [1, 1], lb = [   1], ub = [1.0]
//     [1, 2]      [1]      [1, 0]       [   0]       [0.7]
//                          [0, 1]       [   0]       [0.7]
//                          [0, 1]       [-inf]       [inf]
//
// The optimal solution is
// x = [0.3, 0.7]'
// y = [-2.9, 0.0, 0.2, 0.0]`
// obj = 1.88

// cppcheck-suppress syntaxError
TEST(TestOsqpInterface, BasicQp)
{
  using autoware::common::osqp::calCSCMatrix;
  using autoware::common::osqp::calCSCMatrixTrapezoidal;
  using autoware::common::osqp::CSC_Matrix;

  auto check_result =
    [](const std::tuple<std::vector<double>, std::vector<double>, int, int, int> & result) {
      EXPECT_EQ(std::get<2>(result), 1);  // polish succeeded
      EXPECT_EQ(std::get<3>(result), 1);  // solution succeeded

      static const auto ep = 1.0e-8;

      const auto prime_val = std::get<0>(result);
      ASSERT_EQ(prime_val.size(), size_t(2));
      EXPECT_NEAR(prime_val[0], 0.3, ep);
      EXPECT_NEAR(prime_val[1], 0.7, ep);

      const auto dual_val = std::get<1>(result);
      ASSERT_EQ(dual_val.size(), size_t(4));
      EXPECT_NEAR(dual_val[0], -2.9, ep);
      EXPECT_NEAR(dual_val[1], 0.0, ep);
      EXPECT_NEAR(dual_val[2], 0.2, ep);
      EXPECT_NEAR(dual_val[3], 0.0, ep);
    };

  const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(4, 2) << 1, 1, 1, 0, 0, 1, 0, 1).finished();
  const std::vector<double> q = {1.0, 1.0};
  const std::vector<double> l = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
  const std::vector<double> u = {1.0, 0.7, 0.7, autoware::common::osqp::INF};

  {
    // Define problem during optimization
    autoware::common::osqp::OSQPInterface osqp;
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result =
      osqp.optimize(P, A, q, l, u);
    check_result(result);
  }

  {
    // Define problem during initialization
    autoware::common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result = osqp.optimize();
    check_result(result);
  }

  {
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result;
    // Dummy initial problem
    Eigen::MatrixXd P_ini = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd A_ini = Eigen::MatrixXd::Zero(4, 2);
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    autoware::common::osqp::OSQPInterface osqp(P_ini, A_ini, q_ini, l_ini, u_ini, 1e-6);
    osqp.optimize();

    // Redefine problem before optimization
    osqp.initializeProblem(P, A, q, l, u);
    result = osqp.optimize();
    check_result(result);
  }

  {
    // Define problem during initialization with csc matrix
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    autoware::common::osqp::OSQPInterface osqp(P_csc, A_csc, q, l, u, 1e-6);
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result = osqp.optimize();
    check_result(result);
  }

  {
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result;
    // Dummy initial problem with csc matrix
    CSC_Matrix P_ini_csc = calCSCMatrixTrapezoidal(Eigen::MatrixXd::Zero(2, 2));
    CSC_Matrix A_ini_csc = calCSCMatrix(Eigen::MatrixXd::Zero(4, 2));
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    autoware::common::osqp::OSQPInterface osqp(P_ini_csc, A_ini_csc, q_ini, l_ini, u_ini, 1e-6);
    osqp.optimize();

    // Redefine problem before optimization
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    osqp.initializeProblem(P_csc, A_csc, q, l, u);
    result = osqp.optimize();
    check_result(result);
  }

  // add warm startup
  {
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result;
    // Dummy initial problem with csc matrix
    CSC_Matrix P_ini_csc = calCSCMatrixTrapezoidal(Eigen::MatrixXd::Zero(2, 2));
    CSC_Matrix A_ini_csc = calCSCMatrix(Eigen::MatrixXd::Zero(4, 2));
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    autoware::common::osqp::OSQPInterface osqp(P_ini_csc, A_ini_csc, q_ini, l_ini, u_ini, 1e-6);
    osqp.optimize();

    // Redefine problem before optimization
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    osqp.initializeProblem(P_csc, A_csc, q, l, u);
    result = osqp.optimize();
    check_result(result);

    osqp.updateCheckTermination(1);
    const auto primal_val = std::get<0>(result);
    const auto dual_val = std::get<1>(result);
    for (size_t i = 0; i < primal_val.size(); ++i) {
      std::cerr << primal_val.at(i) << std::endl;
    }
    osqp.setWarmStart(primal_val, dual_val);
    result = osqp.optimize();
    check_result(result);
    EXPECT_EQ(osqp.getTakenIter(), 1);
  }
}
}  // namespace
