// Copyright 2023 Autoware Foundation
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

#include "ekf_localizer/warning_message.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

TEST(PoseDelayStepWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    poseDelayStepWarningMessage(6.0, 4.0).c_str(),
    "Pose delay exceeds the compensation limit, ignored. "
    "delay: 6.000[s], limit: 4.000[s]");
}

TEST(TwistDelayStepWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    twistDelayStepWarningMessage(10.0, 6.0).c_str(),
    "Twist delay exceeds the compensation limit, ignored. "
    "delay: 10.000[s], limit: 6.000[s]");
}

TEST(PoseDelayTimeWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    poseDelayTimeWarningMessage(-1.0).c_str(),
    "Pose time stamp is inappropriate, set delay to 0[s]. delay = -1.000");
  EXPECT_STREQ(
    poseDelayTimeWarningMessage(-0.4).c_str(),
    "Pose time stamp is inappropriate, set delay to 0[s]. delay = -0.400");
}

TEST(TwistDelayTimeWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    twistDelayTimeWarningMessage(-1.0).c_str(),
    "Twist time stamp is inappropriate, set delay to 0[s]. delay = -1.000");
  EXPECT_STREQ(
    twistDelayTimeWarningMessage(-0.4).c_str(),
    "Twist time stamp is inappropriate, set delay to 0[s]. delay = -0.400");
}

TEST(MahalanobisWarningMessage, SmokeTest)
{
  EXPECT_STREQ(
    mahalanobisWarningMessage(1.0, 0.5).c_str(),
    "The Mahalanobis distance 1.0000 is over the limit 0.5000.");
}
