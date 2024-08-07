// Copyright 2022 The Autoware Contributors
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

#include "pose2twist_core.hpp"

#include <gtest/gtest.h>

// 1e-3 radian = 0.057 degrees
constexpr double acceptable_error = 1e-3;

TEST(AngularVelocityFromQuaternion, CheckMultiplicationOrder)
{
  // If we define q2 as the rotation obtained by applying dq after applying q1, then q2 = q1 * dq .
  //
  // IT IS NOT q2 = dq * q1 .
  //
  // This test checks that the multiplication order is correct.

  const tf2::Vector3 target_vector(1, 2, 3);
  // initial state
  // Now, car is facing to the +x direction
  //         z
  //         ^     y
  //         |    ^
  //         |   /
  //         |  /
  //        car -> x
  //
  //
  //

  tf2::Quaternion q1;
  q1.setRPY(0., 0., M_PI / 2.);  // yaw = 90 degrees
  const tf2::Vector3 initially_rotated_vector = tf2::quatRotate(q1, target_vector);
  // after applying q1
  // Now, car is facing to the +y direction
  //         z
  //         ^
  //         |    y
  //         |  ^
  //         | /
  //     <--car    x
  //
  //
  //
  EXPECT_NEAR(initially_rotated_vector.x(), -2., acceptable_error);
  EXPECT_NEAR(initially_rotated_vector.y(), 1., acceptable_error);
  EXPECT_NEAR(initially_rotated_vector.z(), 3., acceptable_error);

  tf2::Quaternion dq;
  dq.setRPY(0., M_PI / 2., 0.);  // pitch = 90 degrees
  const tf2::Vector3 finally_rotated_vector = tf2::quatRotate(q1 * dq, target_vector);
  // after applying dq
  // Now, car is facing to the -z direction
  //         z     y
  //              ^
  //             /
  //            /
  //           /
  //     <--car    x
  //         |
  //         v
  //
  EXPECT_NEAR(finally_rotated_vector.x(), -2., acceptable_error);
  EXPECT_NEAR(finally_rotated_vector.y(), 3., acceptable_error);
  EXPECT_NEAR(finally_rotated_vector.z(), -1., acceptable_error);

  // Failure case
  {
    const tf2::Vector3 false_rotated_vector = tf2::quatRotate(dq * q1, target_vector);

    EXPECT_FALSE(std::abs(false_rotated_vector.x() - (-2)) < acceptable_error);
    EXPECT_FALSE(std::abs(false_rotated_vector.y() - (3)) < acceptable_error);
    EXPECT_FALSE(std::abs(false_rotated_vector.z() - (-1)) < acceptable_error);
  }
}

TEST(AngularVelocityFromQuaternion, CheckNumericalValidity)
{
  auto test = [](const tf2::Vector3 & expected_axis, const double expected_angle) -> void {
    tf2::Quaternion expected_q;
    expected_q.setRotation(expected_axis, expected_angle);

    // Create a random initial quaternion
    tf2::Quaternion initial_q;
    initial_q.setRPY(0.2, 0.3, 0.4);

    // Calculate the final quaternion by rotating the initial quaternion by the expected
    // quaternion
    const tf2::Quaternion final_q = initial_q * expected_q;

    // Calculate the relative rotation between the initial and final quaternion
    const geometry_msgs::msg::Vector3 rotation_vector =
      autoware::pose2twist::compute_relative_rotation_vector(initial_q, final_q);

    EXPECT_NEAR(rotation_vector.x, expected_axis.x() * expected_angle, acceptable_error);
    EXPECT_NEAR(rotation_vector.y, expected_axis.y() * expected_angle, acceptable_error);
    EXPECT_NEAR(rotation_vector.z, expected_axis.z() * expected_angle, acceptable_error);
  };

  test(tf2::Vector3(1.0, 0.0, 0.0).normalized(), 0.1);   // 0.1 radian =  5.7 degrees
  test(tf2::Vector3(1.0, 1.0, 0.0).normalized(), -0.2);  // 0.2 radian = 11.4 degrees
  test(tf2::Vector3(1.0, 2.0, 3.0).normalized(), 0.3);   // 0.3 radian = 17.2 degrees
}
