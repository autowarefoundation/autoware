// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <tuple>
#include <vector>

#include "eigen3/Eigen/Core"
#include "gtest/gtest.h"
#include "osqp_interface/osqp_interface.hpp"


namespace
{
using autoware::common::osqp::float64_t;
/// Problem taken from https://github.com/osqp/osqp/blob/master/tests/basic_qp/generate_problem.py
// cppcheck-suppress syntaxError
TEST(TestOsqpInterface, BasicQp) {
  auto check_result =
    [](const std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int> & result) {
      EXPECT_EQ(std::get<2>(result), 1);
      EXPECT_EQ(std::get<3>(result), 1);
      ASSERT_EQ(std::get<0>(result).size(), size_t(2));
      ASSERT_EQ(std::get<1>(result).size(), size_t(2));
      EXPECT_DOUBLE_EQ(std::get<0>(result)[0], 0.3);
      EXPECT_DOUBLE_EQ(std::get<0>(result)[1], 0.7);
      EXPECT_DOUBLE_EQ(std::get<1>(result)[0], -2.9);
      EXPECT_NEAR(std::get<1>(result)[1], 0.2, 1e-6);
    };

  {
    // Define problem during optimization
    autoware::common::osqp::OSQPInterface osqp;
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    Eigen::MatrixXd A(2, 4);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<float64_t> q = {1.0, 1.0};
    std::vector<float64_t> l = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int> result = osqp.optimize(
      P, A, q, l, u);
    check_result(result);
  }
  {
    // Define problem during initialization
    Eigen::MatrixXd P(2, 2);
    P << 4, 1, 1, 2;
    Eigen::MatrixXd A(2, 4);
    A << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<float64_t> q = {1.0, 1.0};
    std::vector<float64_t> l = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    autoware::common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int> result = osqp.optimize();
    check_result(result);
  }
  {
    std::tuple<std::vector<float64_t>, std::vector<float64_t>, int, int> result;
    // Dummy initial problem
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(2, 2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 4);
    std::vector<float64_t> q(2, 0.0);
    std::vector<float64_t> l(4, 0.0);
    std::vector<float64_t> u(4, 0.0);
    autoware::common::osqp::OSQPInterface osqp(P, A, q, l, u, 1e-6);
    osqp.optimize();
    // Redefine problem before optimization
    Eigen::MatrixXd P_new(2, 2);
    P_new << 4, 1, 1, 2;
    Eigen::MatrixXd A_new(2, 4);
    A_new << 1, 1, 1, 0, 0, 1, 0, 1;
    std::vector<float64_t> q_new = {1.0, 1.0};
    std::vector<float64_t> l_new = {1.0, 0.0, 0.0, -autoware::common::osqp::INF};
    std::vector<float64_t> u_new = {1.0, 0.7, 0.7, autoware::common::osqp::INF};
    osqp.initializeProblem(P_new, A_new, q_new, l_new, u_new);
    result = osqp.optimize();
    check_result(result);
  }
}
}  // namespace
