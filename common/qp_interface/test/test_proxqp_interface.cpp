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
#include "qp_interface/proxqp_interface.hpp"

#include <Eigen/Core>

#include <tuple>
#include <vector>

namespace
{
// Problem taken from
// https://github.com/proxqp/proxqp/blob/master/tests/basic_qp/generate_problem.py
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

TEST(TestProxqpInterface, BasicQp)
{
  auto check_result = [](const auto & solution, const std::string & status) {
    EXPECT_EQ(status, "PROXQP_SOLVED");

    static const auto ep = 1.0e-8;
    ASSERT_EQ(solution.size(), size_t(2));
    EXPECT_NEAR(solution[0], 0.3, ep);
    EXPECT_NEAR(solution[1], 0.7, ep);
  };

  const Eigen::MatrixXd P = (Eigen::MatrixXd(2, 2) << 4, 1, 1, 2).finished();
  const Eigen::MatrixXd A = (Eigen::MatrixXd(4, 2) << 1, 1, 1, 0, 0, 1, 0, 1).finished();
  const std::vector<double> q = {1.0, 1.0};
  const std::vector<double> l = {1.0, 0.0, 0.0, -std::numeric_limits<double>::max()};
  const std::vector<double> u = {1.0, 0.7, 0.7, std::numeric_limits<double>::max()};

  {
    // Define problem during optimization
    autoware::common::ProxQPInterface proxqp(false, 4000, 1e-9, 1e-9, false);
    const auto solution = proxqp.QPInterface::optimize(P, A, q, l, u);
    const auto status = proxqp.getStatus();
    check_result(solution, status);
  }

  {
    // Define problem during optimization with warm start
    autoware::common::ProxQPInterface proxqp(true, 4000, 1e-9, 1e-9, false);
    {
      const auto solution = proxqp.QPInterface::optimize(P, A, q, l, u);
      const auto status = proxqp.getStatus();
      check_result(solution, status);
      EXPECT_NE(proxqp.getIterationNumber(), 1);
    }
    {
      const auto solution = proxqp.QPInterface::optimize(P, A, q, l, u);
      const auto status = proxqp.getStatus();
      check_result(solution, status);
      EXPECT_EQ(proxqp.getIterationNumber(), 0);
    }
  }
}
}  // namespace
