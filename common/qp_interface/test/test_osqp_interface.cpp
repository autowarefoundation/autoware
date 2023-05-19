// Copyright 2023 TIER IV, Inc.
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
#include "qp_interface/osqp_interface.hpp"

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
  using qp::calCSCMatrix;
  using qp::calCSCMatrixTrapezoidal;
  using qp::CSC_Matrix;

  auto check_result = [](
                        const auto & solution, const int solution_status, const int polish_status) {
    EXPECT_EQ(solution_status, 1);
    EXPECT_EQ(polish_status, 1);

    static const auto ep = 1.0e-8;

    ASSERT_EQ(solution.size(), size_t(2));
    EXPECT_NEAR(solution[0], 0.3, ep);
    EXPECT_NEAR(solution[1], 0.7, ep);
  };

  const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(4, 2) << 1, 1, 1, 0, 0, 1, 0, 1).finished();
  const std::vector<double> q = {1.0, 1.0};
  const std::vector<double> l = {1.0, 0.0, 0.0, -qp::INF};
  const std::vector<double> u = {1.0, 0.7, 0.7, qp::INF};

  {
    // Define problem during optimization
    qp::OSQPInterface osqp(false, 1e-6);
    const auto solution = osqp.QPInterface::optimize(P, A, q, l, u);
    const auto solution_status = osqp.getStatus();
    const auto polish_status = osqp.getPolishStatus();
    check_result(solution, solution_status, polish_status);
  }

  {
    // Define problem during initialization
    qp::OSQPInterface osqp(false, 1e-6);
    const auto solution = osqp.QPInterface::optimize(P, A, q, l, u);
    const auto solution_status = osqp.getStatus();
    const auto polish_status = osqp.getPolishStatus();
    check_result(solution, solution_status, polish_status);
  }

  {
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result;
    // Dummy initial problem
    Eigen::MatrixXd P_ini = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd A_ini = Eigen::MatrixXd::Zero(4, 2);
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    qp::OSQPInterface osqp(false, 1e-6);
    osqp.QPInterface::optimize(P_ini, A_ini, q_ini, l_ini, u_ini);
  }

  {
    // Define problem during initialization with csc matrix
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    qp::OSQPInterface osqp(false, 1e-6);

    const auto solution = osqp.optimize(P_csc, A_csc, q, l, u);
    const auto solution_status = osqp.getStatus();
    const auto polish_status = osqp.getPolishStatus();
    check_result(solution, solution_status, polish_status);
  }

  {
    std::tuple<std::vector<double>, std::vector<double>, int, int, int> result;
    // Dummy initial problem with csc matrix
    CSC_Matrix P_ini_csc = calCSCMatrixTrapezoidal(Eigen::MatrixXd::Zero(2, 2));
    CSC_Matrix A_ini_csc = calCSCMatrix(Eigen::MatrixXd::Zero(4, 2));
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    qp::OSQPInterface osqp(false, 1e-6);
    osqp.optimize(P_ini_csc, A_ini_csc, q_ini, l_ini, u_ini);

    // Redefine problem before optimization
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);

    const auto solution = osqp.optimize(P_csc, A_csc, q, l, u);
    const auto solution_status = osqp.getStatus();
    const auto polish_status = osqp.getPolishStatus();
    check_result(solution, solution_status, polish_status);
  }

  // add warm startup
  {
    // Dummy initial problem with csc matrix
    CSC_Matrix P_ini_csc = calCSCMatrixTrapezoidal(Eigen::MatrixXd::Zero(2, 2));
    CSC_Matrix A_ini_csc = calCSCMatrix(Eigen::MatrixXd::Zero(4, 2));
    std::vector<double> q_ini(2, 0.0);
    std::vector<double> l_ini(4, 0.0);
    std::vector<double> u_ini(4, 0.0);
    qp::OSQPInterface osqp(true, 1e-6);  // enable warm start
    osqp.optimize(P_ini_csc, A_ini_csc, q_ini, l_ini, u_ini);

    // Redefine problem before optimization
    CSC_Matrix P_csc = calCSCMatrixTrapezoidal(P);
    CSC_Matrix A_csc = calCSCMatrix(A);
    {
      const auto solution = osqp.optimize(P_csc, A_csc, q, l, u);
      const auto solution_status = osqp.getStatus();
      const auto polish_status = osqp.getPolishStatus();
      check_result(solution, solution_status, polish_status);

      osqp.updateCheckTermination(1);
      const auto primal_val = solution;
      const auto dual_val = osqp.getDualSolution();
      for (size_t i = 0; i < primal_val.size(); ++i) {
        std::cerr << primal_val.at(i) << std::endl;
      }
      osqp.setWarmStart(primal_val, dual_val);
    }

    {
      const auto solution = osqp.optimize(P_csc, A_csc, q, l, u);
      const auto solution_status = osqp.getStatus();
      const auto polish_status = osqp.getPolishStatus();
      check_result(solution, solution_status, polish_status);
    }

    // NOTE: This should be true, but currently optimize function reset the workspace, which
    // disables warm start.
    // EXPECT_EQ(osqp.getTakenIter(), 1);
  }
}
}  // namespace
