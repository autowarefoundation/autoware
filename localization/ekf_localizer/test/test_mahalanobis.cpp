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

#include "ekf_localizer/mahalanobis.hpp"

#include <gtest/gtest.h>

constexpr double tolerance = 1e-8;

TEST(SquaredMahalanobis, SmokeTest)
{
  {
    Eigen::Vector2d x(0, 1);
    Eigen::Vector2d y(3, 2);
    Eigen::Matrix2d C;
    C << 10, 0, 0, 10;

    EXPECT_NEAR(squaredMahalanobis(x, y, C), 1.0, tolerance);
  }

  {
    Eigen::Vector2d x(4, 1);
    Eigen::Vector2d y(1, 5);
    Eigen::Matrix2d C;
    C << 5, 0, 0, 5;

    EXPECT_NEAR(squaredMahalanobis(x, y, C), 5.0, tolerance);
  }
}

TEST(Mahalanobis, SmokeTest)
{
  {
    Eigen::Vector2d x(0, 1);
    Eigen::Vector2d y(3, 2);
    Eigen::Matrix2d C;
    C << 10, 0, 0, 10;

    EXPECT_NEAR(mahalanobis(x, y, C), 1.0, tolerance);
  }

  {
    Eigen::Vector2d x(4, 1);
    Eigen::Vector2d y(1, 5);
    Eigen::Matrix2d C;
    C << 5, 0, 0, 5;

    EXPECT_NEAR(mahalanobis(x, y, C), std::sqrt(5.0), tolerance);
  }
}
