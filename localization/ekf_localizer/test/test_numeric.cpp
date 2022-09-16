// Copyright 2022 Autoware Foundation
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

#include "ekf_localizer/numeric.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>

TEST(Numeric, HasNan)
{
  const Eigen::VectorXd empty(0);
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::nan("");

  EXPECT_FALSE(hasNan(empty));
  EXPECT_FALSE(hasNan(Eigen::Vector3d(0., 0., 1.)));
  EXPECT_FALSE(hasNan(Eigen::Vector3d(1e16, 0., 1.)));
  EXPECT_FALSE(hasNan(Eigen::Vector3d(0., 1., inf)));

  EXPECT_TRUE(hasNan(Eigen::Vector3d(nan, 1., 0.)));
}

TEST(Numeric, HasInf)
{
  const Eigen::VectorXd empty(0);
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::nan("");

  EXPECT_FALSE(hasInf(empty));
  EXPECT_FALSE(hasInf(Eigen::Vector3d(0., 0., 1.)));
  EXPECT_FALSE(hasInf(Eigen::Vector3d(1e16, 0., 1.)));
  EXPECT_FALSE(hasInf(Eigen::Vector3d(nan, 1., 0.)));

  EXPECT_TRUE(hasInf(Eigen::Vector3d(0., 1., inf)));
}
