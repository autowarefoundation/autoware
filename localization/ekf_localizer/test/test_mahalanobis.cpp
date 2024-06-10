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

TEST(squared_mahalanobis, SmokeTest)
{
  {
    Eigen::Vector2d x(0, 1);
    Eigen::Vector2d y(3, 2);
    Eigen::Matrix2d c;
    c << 10, 0, 0, 10;

    EXPECT_NEAR(squared_mahalanobis(x, y, c), 1.0, tolerance);
  }

  {
    Eigen::Vector2d x(4, 1);
    Eigen::Vector2d y(1, 5);
    Eigen::Matrix2d c;
    c << 5, 0, 0, 5;

    EXPECT_NEAR(squared_mahalanobis(x, y, c), 5.0, tolerance);
  }
}

TEST(mahalanobis, SmokeTest)
{
  {
    Eigen::Vector2d x(0, 1);
    Eigen::Vector2d y(3, 2);
    Eigen::Matrix2d c;
    c << 10, 0, 0, 10;

    EXPECT_NEAR(mahalanobis(x, y, c), 1.0, tolerance);
  }

  {
    Eigen::Vector2d x(4, 1);
    Eigen::Vector2d y(1, 5);
    Eigen::Matrix2d c;
    c << 5, 0, 0, 5;

    EXPECT_NEAR(mahalanobis(x, y, c), std::sqrt(5.0), tolerance);
  }
}
