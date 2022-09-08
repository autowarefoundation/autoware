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

#include "ekf_localizer/measurement.hpp"

#include <gtest/gtest.h>

TEST(Measurement, PoseMeasurementMatrix)
{
  const Eigen::Matrix<double, 3, 6> M = poseMeasurementMatrix();
  Eigen::Matrix<double, 3, 6> expected;
  expected << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  EXPECT_EQ((M - expected).norm(), 0);
}

TEST(Measurement, TwistMeasurementMatrix)
{
  const Eigen::Matrix<double, 2, 6> M = twistMeasurementMatrix();
  Eigen::Matrix<double, 2, 6> expected;
  expected << 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
  EXPECT_EQ((M - expected).norm(), 0);
}

TEST(Measurement, PoseMeasurementCovariance)
{
  {
    const std::array<double, 36> covariance = {1, 2, 0, 0, 0, 3, 4, 5, 0, 0, 0, 6,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 7, 8, 0, 0, 0, 9};

    const Eigen::Matrix3d M = poseMeasurementCovariance(covariance, 2);

    Eigen::Matrix3d expected;
    expected << 2, 4, 6, 8, 10, 12, 14, 16, 18;

    EXPECT_EQ((M - expected).norm(), 0.);
  }

  {
    // Make sure that other elements are not changed
    std::array<double, 36> covariance;
    covariance.fill(0);
    const Eigen::Matrix3d M = poseMeasurementCovariance(covariance, 2.);
    EXPECT_EQ(M.norm(), 0);
  }
}

TEST(Measurement, TwistMeasurementCovariance)
{
  {
    const std::array<double, 36> covariance = {1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 6,
                                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4};

    const Eigen::Matrix2d M = twistMeasurementCovariance(covariance, 2);

    Eigen::Matrix2d expected;
    expected << 2, 4, 6, 8;

    EXPECT_EQ((M - expected).norm(), 0.);
  }

  {
    // Make sure that other elements are not changed
    std::array<double, 36> covariance;
    covariance.fill(0);
    const Eigen::Matrix2d M = twistMeasurementCovariance(covariance, 2.);
    EXPECT_EQ(M.norm(), 0);
  }
}
