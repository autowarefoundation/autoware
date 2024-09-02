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

namespace autoware::ekf_localizer
{

TEST(EKFCovarianceToPoseMessageCovariance, SmokeTest)
{
  {
    Matrix6d p = Matrix6d::Zero();
    p(0, 0) = 1.;
    p(0, 1) = 2.;
    p(0, 2) = 3.;
    p(1, 0) = 4.;
    p(1, 1) = 5.;
    p(1, 2) = 6.;
    p(2, 0) = 7.;
    p(2, 1) = 8.;
    p(2, 2) = 9.;

    std::array<double, 36> covariance = ekf_covariance_to_pose_message_covariance(p);
    EXPECT_EQ(covariance[0], 1.);
    EXPECT_EQ(covariance[1], 2.);
    EXPECT_EQ(covariance[5], 3.);
    EXPECT_EQ(covariance[6], 4.);
    EXPECT_EQ(covariance[7], 5.);
    EXPECT_EQ(covariance[11], 6.);
    EXPECT_EQ(covariance[30], 7.);
    EXPECT_EQ(covariance[31], 8.);
    EXPECT_EQ(covariance[35], 9.);
  }

  // ensure other elements are zero
  {
    Matrix6d p = Matrix6d::Zero();
    std::array<double, 36> covariance = ekf_covariance_to_pose_message_covariance(p);
    for (double e : covariance) {
      EXPECT_EQ(e, 0.);
    }
  }
}

TEST(EKFCovarianceToTwistMessageCovariance, SmokeTest)
{
  {
    Matrix6d p = Matrix6d::Zero();
    p(4, 4) = 1.;
    p(4, 5) = 2.;
    p(5, 4) = 3.;
    p(5, 5) = 4.;

    std::array<double, 36> covariance = ekf_covariance_to_twist_message_covariance(p);
    EXPECT_EQ(covariance[0], 1.);
    EXPECT_EQ(covariance[5], 2.);
    EXPECT_EQ(covariance[30], 3.);
    EXPECT_EQ(covariance[35], 4.);
  }

  // ensure other elements are zero
  {
    Matrix6d p = Matrix6d::Zero();
    std::array<double, 36> covariance = ekf_covariance_to_twist_message_covariance(p);
    for (double e : covariance) {
      EXPECT_EQ(e, 0.);
    }
  }
}

}  // namespace autoware::ekf_localizer
