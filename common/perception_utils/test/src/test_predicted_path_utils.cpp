// Copyright 2022 TIER IV, Inc.
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

#include "perception_utils/predicted_path_utils.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>

using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Point3d;

constexpr double epsilon = 1e-06;

namespace
{
using autoware_auto_perception_msgs::msg::PredictedPath;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::transformPoint;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

PredictedPath createTestPredictedPath(
  const size_t num_points, const double point_time_interval, const double vel,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(point_time_interval);

  const double point_interval = vel * point_time_interval;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    const auto p = createPose(x, y, 0.0, 0.0, 0.0, theta);
    path.path.push_back(p);
  }
  return path;
}
}  // namespace

TEST(predicted_path_utils, testCalcInterpolatedPose)
{
  using perception_utils::calcInterpolatedPose;
  using tier4_autoware_utils::createQuaternionFromRPY;
  using tier4_autoware_utils::createQuaternionFromYaw;
  using tier4_autoware_utils::deg2rad;

  const auto path = createTestPredictedPath(100, 0.1, 1.0);

  // Normal Case (same point as the original point)
  {
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    for (double t = 0.0; t < 9.0 + 1e-6; t += 1.0) {
      const auto p = calcInterpolatedPose(path, t);

      EXPECT_NE(p, boost::none);
      EXPECT_NEAR(p->position.x, t * 1.0, epsilon);
      EXPECT_NEAR(p->position.y, 0.0, epsilon);
      EXPECT_NEAR(p->position.z, 0.0, epsilon);
      EXPECT_NEAR(p->orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p->orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p->orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p->orientation.w, ans_quat.w, epsilon);
    }
  }

  // Normal Case (random case)
  {
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    for (double t = 0.0; t < 9.0; t += 0.3) {
      const auto p = calcInterpolatedPose(path, t);

      EXPECT_NE(p, boost::none);
      EXPECT_NEAR(p->position.x, t * 1.0, epsilon);
      EXPECT_NEAR(p->position.y, 0.0, epsilon);
      EXPECT_NEAR(p->position.z, 0.0, epsilon);
      EXPECT_NEAR(p->orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p->orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p->orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p->orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Interpolation
  {
    // Negative time
    {
      const auto p = calcInterpolatedPose(path, -1.0);
      EXPECT_EQ(p, boost::none);
    }

    // Over the time horizon
    {
      const auto p = calcInterpolatedPose(path, 11.0);
      EXPECT_EQ(p, boost::none);
    }

    // Empty Path
    {
      PredictedPath empty_path;
      const auto p = calcInterpolatedPose(empty_path, 5.0);
      EXPECT_EQ(p, boost::none);
    }
  }
}

TEST(predicted_path_utils, resamplePredictedPath_by_vector)
{
  using perception_utils::resamplePredictedPath;
  using tier4_autoware_utils::createQuaternionFromRPY;
  using tier4_autoware_utils::createQuaternionFromYaw;
  using tier4_autoware_utils::deg2rad;

  const auto path = createTestPredictedPath(10, 1.0, 1.0);

  // Resample Same Points
  {
    const std::vector<double> resampling_vec = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
    const auto resampled_path = resamplePredictedPath(path, resampling_vec);

    EXPECT_EQ(resampled_path.path.size(), path.path.size());
    EXPECT_NEAR(path.confidence, resampled_path.confidence, epsilon);

    for (size_t i = 0; i < path.path.size(); ++i) {
      EXPECT_NEAR(path.path.at(i).position.x, resampled_path.path.at(i).position.x, epsilon);
      EXPECT_NEAR(path.path.at(i).position.y, resampled_path.path.at(i).position.y, epsilon);
      EXPECT_NEAR(path.path.at(i).position.z, resampled_path.path.at(i).position.z, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.x, resampled_path.path.at(i).orientation.x, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.y, resampled_path.path.at(i).orientation.y, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.z, resampled_path.path.at(i).orientation.z, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.w, resampled_path.path.at(i).orientation.w, epsilon);
    }
  }

  // Resample random case
  {
    const std::vector<double> resampling_vec = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 1.3, 2.8,
                                                3.7, 4.2, 5.1, 6.9, 7.3, 8.5, 9.0};
    const auto resampled_path = resamplePredictedPath(path, resampling_vec);

    EXPECT_EQ(resampled_path.path.size(), resampling_vec.size());
    EXPECT_NEAR(path.confidence, resampled_path.confidence, epsilon);

    for (size_t i = 0; i < resampled_path.path.size(); ++i) {
      EXPECT_NEAR(resampled_path.path.at(i).position.x, resampling_vec.at(i) * 1.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.x, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.w, 1.0, epsilon);
    }
  }

  // Resample which exceeds the maximum size
  {
    std::vector<double> resampling_vec(101);
    for (size_t i = 0; i < 101; ++i) {
      resampling_vec.at(i) = i * 0.05;
    }

    const auto resampled_path = resamplePredictedPath(path, resampling_vec);

    EXPECT_EQ(resampled_path.path.size(), resampled_path.path.max_size());
    EXPECT_NEAR(path.confidence, resampled_path.confidence, epsilon);

    for (size_t i = 0; i < resampled_path.path.max_size(); ++i) {
      EXPECT_NEAR(resampled_path.path.at(i).position.x, resampling_vec.at(i), epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.x, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.w, 1.0, epsilon);
    }
  }

  // Some points are out of range
  {
    const std::vector<double> resampling_vec = {-1.0, 0.0, 5.0, 9.0, 9.1};
    EXPECT_THROW(resamplePredictedPath(path, resampling_vec), std::invalid_argument);
  }
}

TEST(predicted_path_utils, resamplePredictedPath_by_sampling_time)
{
  using perception_utils::resamplePredictedPath;
  using tier4_autoware_utils::createQuaternionFromRPY;
  using tier4_autoware_utils::createQuaternionFromYaw;
  using tier4_autoware_utils::deg2rad;

  const auto path = createTestPredictedPath(10, 1.0, 1.0);

  // Sample same points
  {
    const auto resampled_path = resamplePredictedPath(path, 1.0, 9.0);

    EXPECT_EQ(resampled_path.path.size(), path.path.size());
    EXPECT_NEAR(rclcpp::Duration(resampled_path.time_step).seconds(), 1.0, epsilon);
    for (size_t i = 0; i < path.path.size(); ++i) {
      EXPECT_NEAR(path.path.at(i).position.x, resampled_path.path.at(i).position.x, epsilon);
      EXPECT_NEAR(path.path.at(i).position.y, resampled_path.path.at(i).position.y, epsilon);
      EXPECT_NEAR(path.path.at(i).position.z, resampled_path.path.at(i).position.z, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.x, resampled_path.path.at(i).orientation.x, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.y, resampled_path.path.at(i).orientation.y, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.z, resampled_path.path.at(i).orientation.z, epsilon);
      EXPECT_NEAR(path.path.at(i).orientation.w, resampled_path.path.at(i).orientation.w, epsilon);
    }
  }

  // Fine sampling
  {
    const auto resampled_path = resamplePredictedPath(path, 0.1, 9.0);

    EXPECT_EQ(resampled_path.path.size(), static_cast<size_t>(91));
    EXPECT_NEAR(rclcpp::Duration(resampled_path.time_step).seconds(), 0.1, epsilon);
    for (size_t i = 0; i < resampled_path.path.size(); ++i) {
      EXPECT_NEAR(resampled_path.path.at(i).position.x, 0.1 * i, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.x, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.w, 1.0, epsilon);
    }
  }

  // Coarse Sampling
  {
    const auto resampled_path = resamplePredictedPath(path, 2.0, 9.0);

    EXPECT_EQ(resampled_path.path.size(), static_cast<size_t>(5));
    EXPECT_NEAR(rclcpp::Duration(resampled_path.time_step).seconds(), 2.0, epsilon);
    for (size_t i = 0; i < resampled_path.path.size(); ++i) {
      EXPECT_NEAR(resampled_path.path.at(i).position.x, 2.0 * i, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.x, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.w, 1.0, epsilon);
    }
  }

  // Shorter horizon
  {
    const auto resampled_path = resamplePredictedPath(path, 1.5, 7.0);

    EXPECT_EQ(resampled_path.path.size(), static_cast<size_t>(5));
    EXPECT_NEAR(rclcpp::Duration(resampled_path.time_step).seconds(), 1.5, epsilon);
    for (size_t i = 0; i < resampled_path.path.size(); ++i) {
      EXPECT_NEAR(resampled_path.path.at(i).position.x, 1.5 * i, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).position.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.x, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.y, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.z, 0.0, epsilon);
      EXPECT_NEAR(resampled_path.path.at(i).orientation.w, 1.0, epsilon);
    }
  }

  // No Sampling
  {
    // Negative resampling time or resampling time horizon
    EXPECT_THROW(resamplePredictedPath(path, 0.0, 9.0), std::invalid_argument);
    EXPECT_THROW(resamplePredictedPath(path, -1.0, 9.0), std::invalid_argument);
    EXPECT_THROW(resamplePredictedPath(path, 1.0, 0.0), std::invalid_argument);
    EXPECT_THROW(resamplePredictedPath(path, 1.0, -9.0), std::invalid_argument);

    PredictedPath empty_path;
    EXPECT_THROW(resamplePredictedPath(empty_path, 1.0, 10.0), std::invalid_argument);
  }
}
