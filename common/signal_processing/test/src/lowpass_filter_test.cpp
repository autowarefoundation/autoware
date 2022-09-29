// Copyright 2022 Tier IV, Inc.
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

#include "signal_processing/lowpass_filter.hpp"

#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

geometry_msgs::msg::Twist createTwist(
  const double lx, const double ly, const double lz, const double ax, const double ay,
  const double az)
{
  geometry_msgs::msg::Twist twist;

  twist.linear.x = lx;
  twist.linear.y = ly;
  twist.linear.z = lz;
  twist.angular.x = ax;
  twist.angular.y = ay;
  twist.angular.z = az;

  return twist;
}

TEST(lowpass_filter_twist, filter)
{
  LowpassFilterTwist lowpass_filter_(0.1);

  {  // initial state
    EXPECT_EQ(lowpass_filter_.getValue(), boost::none);
  }

  {  // random filter
    geometry_msgs::msg::Twist twist = createTwist(0.0, 0.1, 0.2, 0.3, 0.4, 0.5);

    const auto filtered_twist = lowpass_filter_.filter(twist);
    EXPECT_NEAR(filtered_twist.linear.x, 0.0, epsilon);
    EXPECT_NEAR(filtered_twist.linear.y, 0.1, epsilon);
    EXPECT_NEAR(filtered_twist.linear.z, 0.2, epsilon);
    EXPECT_NEAR(filtered_twist.angular.x, 0.3, epsilon);
    EXPECT_NEAR(filtered_twist.angular.y, 0.4, epsilon);
    EXPECT_NEAR(filtered_twist.angular.z, 0.5, epsilon);
  }

  {  // reset without value
    lowpass_filter_.reset();
    EXPECT_EQ(lowpass_filter_.getValue(), boost::none);
  }

  {  // reset with value
    geometry_msgs::msg::Twist twist = createTwist(0.0, 0.1, 0.2, 0.3, 0.4, 0.5);

    lowpass_filter_.reset(twist);

    const auto filtered_twist = lowpass_filter_.getValue();
    EXPECT_NEAR(filtered_twist->linear.x, 0.0, epsilon);
    EXPECT_NEAR(filtered_twist->linear.y, 0.1, epsilon);
    EXPECT_NEAR(filtered_twist->linear.z, 0.2, epsilon);
    EXPECT_NEAR(filtered_twist->angular.x, 0.3, epsilon);
    EXPECT_NEAR(filtered_twist->angular.y, 0.4, epsilon);
    EXPECT_NEAR(filtered_twist->angular.z, 0.5, epsilon);
  }
}
