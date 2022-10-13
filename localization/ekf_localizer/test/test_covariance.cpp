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

#include "ekf_localizer/covariance.hpp"

#include <gtest/gtest.h>

TEST(EKFCovarianceToPoseMessageCovariance, SmokeTest)
{
  {
    Matrix6d P = Matrix6d::Zero();
    P(0, 0) = 1.;
    P(0, 1) = 2.;
    P(0, 2) = 3.;
    P(1, 0) = 4.;
    P(1, 1) = 5.;
    P(1, 2) = 6.;
    P(2, 0) = 7.;
    P(2, 1) = 8.;
    P(2, 2) = 9.;

    std::array<double, 36> covariance = ekfCovarianceToPoseMessageCovariance(P);
    EXPECT_EQ(covariance(0), 1.);
    EXPECT_EQ(covariance(1), 2.);
    EXPECT_EQ(covariance(5), 3.);
    EXPECT_EQ(covariance(6), 4.);
    EXPECT_EQ(covariance(7), 5.);
    EXPECT_EQ(covariance(11), 6.);
    EXPECT_EQ(covariance(30), 7.);
    EXPECT_EQ(covariance(31), 8.);
    EXPECT_EQ(covariance(35), 9.);
  }

  // ensure other elements are zero
  {
    Matrix6d P = Matrix6d::Zero();
    std::array<double, 36> covariance = ekfCovarianceToPoseMessageCovariance(P);
    for (double e : covariance) {
      EXPECT_EQ(e, 0.);
    }
  }
}

TEST(EKFCovarianceToTwistMessageCovariance, SmokeTest)
{
  {
    Matrix6d P = Matrix6d::Zero();
    P(4, 4) = 1.;
    P(4, 5) = 2.;
    P(5, 4) = 3.;
    P(5, 5) = 4.;

    std::array<double, 36> covariance = ekfCovarianceToTwistMessageCovariance(P);
    EXPECT_EQ(covariance(0), 1.);
    EXPECT_EQ(covariance(5), 2.);
    EXPECT_EQ(covariance(30), 3.);
    EXPECT_EQ(covariance(35), 4.);
  }

  // ensure other elements are zero
  {
    Matrix6d P = Matrix6d::Zero();
    std::array<double, 36> covariance = ekfCovarianceToTwistMessageCovariance(P);
    for (double e : covariance) {
      EXPECT_EQ(e, 0.);
    }
  }
}
