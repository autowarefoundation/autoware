// Copyright 2021 Tier IV, Inc.
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

#include "interpolation/spherical_linear_interpolation.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

constexpr double epsilon = 1e-6;

namespace
{
inline geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}
}  // namespace

TEST(slerp, spline_scalar)
{
  using interpolation::slerp;

  // Same value
  {
    const double src_yaw = 0.0;
    const double dst_yaw = 0.0;
    const auto src_quat = createQuaternionFromRPY(0.0, 0.0, src_yaw);
    const auto dst_quat = createQuaternionFromRPY(0.0, 0.0, dst_yaw);

    const auto ans_quat = createQuaternionFromRPY(0.0, 0.0, 0.0);

    for (double ratio = -2.0; ratio < 2.0 + epsilon; ratio += 0.1) {
      const auto interpolated_quat = slerp(src_quat, dst_quat, ratio);
      EXPECT_NEAR(ans_quat.x, interpolated_quat.x, epsilon);
      EXPECT_NEAR(ans_quat.y, interpolated_quat.y, epsilon);
      EXPECT_NEAR(ans_quat.z, interpolated_quat.z, epsilon);
      EXPECT_NEAR(ans_quat.w, interpolated_quat.w, epsilon);
    }
  }

  // Random Value
  {
    const double src_yaw = 0.0;
    const double dst_yaw = M_PI;
    const auto src_quat = createQuaternionFromRPY(0.0, 0.0, src_yaw);
    const auto dst_quat = createQuaternionFromRPY(0.0, 0.0, dst_yaw);

    for (double ratio = -2.0; ratio < 2.0 + epsilon; ratio += 0.1) {
      const auto interpolated_quat = slerp(src_quat, dst_quat, ratio);

      const double ans_yaw = M_PI * ratio;
      tf2::Quaternion ans;
      ans.setRPY(0, 0, ans_yaw);
      const geometry_msgs::msg::Quaternion ans_quat = tf2::toMsg(ans);

      EXPECT_NEAR(ans_quat.x, interpolated_quat.x, epsilon);
      EXPECT_NEAR(ans_quat.y, interpolated_quat.y, epsilon);
      EXPECT_NEAR(ans_quat.z, interpolated_quat.z, epsilon);
      EXPECT_NEAR(ans_quat.w, interpolated_quat.w, epsilon);
    }
  }
}

TEST(slerp, spline_vector)
{
  using interpolation::slerp;

  // query keys are same as base keys
  {
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<geometry_msgs::msg::Quaternion> base_values;
    for (size_t i = 0; i < 5; ++i) {
      const auto quat = createQuaternionFromRPY(0.0, 0.0, i * M_PI / 5.0);
      base_values.push_back(quat);
    }
    const std::vector<double> query_keys = base_keys;
    const auto ans = base_values;

    const auto results = slerp(base_keys, base_values, query_keys);

    for (size_t i = 0; i < results.size(); ++i) {
      const auto interpolated_quat = results.at(i);
      const auto ans_quat = ans.at(i);

      EXPECT_NEAR(ans_quat.x, interpolated_quat.x, epsilon);
      EXPECT_NEAR(ans_quat.y, interpolated_quat.y, epsilon);
      EXPECT_NEAR(ans_quat.z, interpolated_quat.z, epsilon);
      EXPECT_NEAR(ans_quat.w, interpolated_quat.w, epsilon);
    }
  }

  // random
  {
    const std::vector<double> base_keys{0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<geometry_msgs::msg::Quaternion> base_values;
    for (size_t i = 0; i < 5; ++i) {
      const auto quat = createQuaternionFromRPY(0.0, 0.0, i * M_PI / 5.0);
      base_values.push_back(quat);
    }
    const std::vector<double> query_keys = {0.0, 0.1, 1.5, 2.6, 3.1, 3.8};
    std::vector<geometry_msgs::msg::Quaternion> ans(query_keys.size());
    ans.at(0) = createQuaternionFromRPY(0.0, 0.0, 0.0);
    ans.at(1) = createQuaternionFromRPY(0.0, 0.0, 0.1 * M_PI / 5.0);
    ans.at(2) = createQuaternionFromRPY(0.0, 0.0, 0.5 * M_PI / 5.0 + M_PI / 5.0);
    ans.at(3) = createQuaternionFromRPY(0.0, 0.0, 0.6 * M_PI / 5.0 + 2.0 * M_PI / 5.0);
    ans.at(4) = createQuaternionFromRPY(0.0, 0.0, 0.1 * M_PI / 5.0 + 3.0 * M_PI / 5.0);
    ans.at(5) = createQuaternionFromRPY(0.0, 0.0, 0.8 * M_PI / 5.0 + 3.0 * M_PI / 5.0);

    const auto results = slerp(base_keys, base_values, query_keys);

    for (size_t i = 0; i < results.size(); ++i) {
      const auto interpolated_quat = results.at(i);
      const auto ans_quat = ans.at(i);

      EXPECT_NEAR(ans_quat.x, interpolated_quat.x, epsilon);
      EXPECT_NEAR(ans_quat.y, interpolated_quat.y, epsilon);
      EXPECT_NEAR(ans_quat.z, interpolated_quat.z, epsilon);
      EXPECT_NEAR(ans_quat.w, interpolated_quat.w, epsilon);
    }
  }
}
