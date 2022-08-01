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

#include "motion_utils/resample/resample.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/math/constants.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>

namespace
{
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::transformPoint;

constexpr double epsilon = 1e-6;

geometry_msgs::msg::Pose createPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose p;
  p.position = createPoint(x, y, z);
  p.orientation = createQuaternionFromRPY(roll, pitch, yaw);
  return p;
}

PathPoint generateTestPathPoint(
  const double x, const double y, const double z, const double theta = 0.0,
  const double vel_lon = 0.0, const double vel_lat = 0.0, const double heading_rate = 0.0)
{
  PathPoint p;
  p.pose = createPose(x, y, z, 0.0, 0.0, theta);
  p.longitudinal_velocity_mps = vel_lon;
  p.lateral_velocity_mps = vel_lat;
  p.heading_rate_rps = heading_rate;
  return p;
}

TrajectoryPoint generateTestTrajectoryPoint(
  const double x, const double y, const double z, const double theta = 0.0,
  const double vel_lon = 0.0, const double vel_lat = 0.0, const double heading_rate = 0.0,
  const double acc = 0.0)
{
  TrajectoryPoint p;
  p.pose = createPose(x, y, z, 0.0, 0.0, theta);
  p.longitudinal_velocity_mps = vel_lon;
  p.lateral_velocity_mps = vel_lat;
  p.heading_rate_rps = heading_rate;
  p.acceleration_mps2 = acc;
  return p;
}

template <class T>
T generateTestPath(
  const size_t num_points, const double point_interval, const double vel_lon = 0.0,
  const double vel_lat = 0.0, const double heading_rate_rps = 0.0, const double init_theta = 0.0,
  const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel_lon;
    p.lateral_velocity_mps = vel_lat;
    p.heading_rate_rps = heading_rate_rps;
    traj.points.push_back(p);
  }

  return traj;
}

template <class T>
T generateTestTrajectory(
  const size_t num_points, const double point_interval, const double vel_lon = 0.0,
  const double vel_lat = 0.0, const double heading_rate_rps = 0.0, const double acc = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  using Point = typename T::_points_type::value_type;

  T traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    Point p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel_lon;
    p.lateral_velocity_mps = vel_lat;
    p.heading_rate_rps = heading_rate_rps;
    p.acceleration_mps2 = acc;
    traj.points.push_back(p);
  }

  return traj;
}

std::vector<double> generateArclength(const size_t num_points, const double interval)
{
  std::vector<double> resampled_arclength(num_points);
  for (size_t i = 0; i < num_points; ++i) {
    resampled_arclength.at(i) = i * interval;
  }

  return resampled_arclength;
}
}  // namespace

TEST(resample_path, resample_path_by_vector)
{
  using motion_utils::resamplePath;
  // Output is same as input
  {
    auto path = generateTestPath<Path>(10, 1.0, 3.0, 1.0, 0.01);
    std::vector<double> resampled_arclength = generateArclength(10, 1.0);

    {
      const auto resampled_path = resamplePath(path, resampled_arclength);
      for (size_t i = 0; i < resampled_path.points.size(); ++i) {
        const auto p = resampled_path.points.at(i);
        const auto ans_p = path.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
      }
    }

    // Change the last point orientation
    path.points.back() =
      generateTestPathPoint(9.0, 0.0, 0.0, tier4_autoware_utils::pi / 3.0, 3.0, 1.0, 0.01);
    {
      const auto resampled_path = resamplePath(path, resampled_arclength);
      for (size_t i = 0; i < resampled_path.points.size() - 1; ++i) {
        const auto p = resampled_path.points.at(i);
        const auto ans_p = path.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
      }

      const auto p = resampled_path.points.back();
      const auto ans_p = path.points.back();
      const auto ans_quat = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
      EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
      EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
      EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, ans_quat.w, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
    }
  }

  // Output key is not same as input
  {
    autoware_auto_planning_msgs::msg::Path path;
    path.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      path.points.at(i) = generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 1.5, 5.3, 7.5, 9.0};

    const auto resampled_path = resamplePath(path, resampled_arclength);

    {
      const auto p = resampled_path.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
    }

    {
      const auto p = resampled_path.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
    }

    {
      const auto p = resampled_path.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 1.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.15, epsilon);
    }

    {
      const auto p = resampled_path.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
    }

    {
      const auto p = resampled_path.points.at(4);
      EXPECT_NEAR(p.pose.position.x, 7.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 7.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 3.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.75, epsilon);
    }

    {
      const auto p = resampled_path.points.at(5);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
    }

    for (size_t i = 0; i < resampled_path.points.size(); ++i) {
      const auto p = resampled_path.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);
    }
  }

  // No Interpolation
  {
    // Input path size is not enough for interpolation
    {
      autoware_auto_planning_msgs::msg::Path path;
      path.points.resize(1);
      for (size_t i = 0; i < 1; ++i) {
        path.points.at(i) =
          generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
      }
      std::vector<double> resampled_arclength = generateArclength(10, 1.0);

      const auto resampled_path = resamplePath(path, resampled_arclength);
      EXPECT_EQ(resampled_path.points.size(), path.points.size());
      for (size_t i = 0; i < resampled_path.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.x, path.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.y, path.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.z, path.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.x, path.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.y, path.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.z, path.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.w, path.points.at(i).pose.orientation.w,
          epsilon);
      }
    }

    // Resampled Arclength size is not enough for interpolation
    {
      autoware_auto_planning_msgs::msg::Path path;
      path.points.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        path.points.at(i) =
          generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
      }
      std::vector<double> resampled_arclength = generateArclength(1, 1.0);

      const auto resampled_path = resamplePath(path, resampled_arclength);
      EXPECT_EQ(resampled_path.points.size(), path.points.size());
      for (size_t i = 0; i < resampled_path.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.x, path.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.y, path.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.z, path.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.x, path.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.y, path.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.z, path.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.w, path.points.at(i).pose.orientation.w,
          epsilon);
      }
    }

    // Resampled Arclength is longer than input path
    {
      autoware_auto_planning_msgs::msg::Path path;
      path.points.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        path.points.at(i) =
          generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
      }
      std::vector<double> resampled_arclength = generateArclength(3, 5.0);

      const auto resampled_path = resamplePath(path, resampled_arclength);
      EXPECT_EQ(resampled_path.points.size(), path.points.size());
      for (size_t i = 0; i < resampled_path.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.x, path.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.y, path.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.position.z, path.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.x, path.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.y, path.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.z, path.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_path.points.at(i).pose.orientation.w, path.points.at(i).pose.orientation.w,
          epsilon);
      }
    }
  }
}

TEST(resample_path, resample_path_by_vector_non_default)
{
  using motion_utils::resamplePath;

  // Lerp x, y
  {
    autoware_auto_planning_msgs::msg::Path path;
    path.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      path.points.at(i) = generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 5.3, 9.0};

    const auto resampled_path = resamplePath(path, resampled_arclength, true);
    {
      const auto p = resampled_path.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
    }

    {
      const auto p = resampled_path.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
    }

    {
      const auto p = resampled_path.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
    }

    {
      const auto p = resampled_path.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
    }

    for (size_t i = 0; i < resampled_path.points.size(); ++i) {
      const auto p = resampled_path.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);
    }
  }

  // Slerp z
  {
    autoware_auto_planning_msgs::msg::Path path;
    path.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      path.points.at(i) =
        generateTestPathPoint(i * 1.0, 0.0, i * 1.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 5.3, 9.0};

    const auto resampled_path = resamplePath(path, resampled_arclength, false, false);
    {
      const auto p = resampled_path.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
    }

    {
      const auto p = resampled_path.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 1.2, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
    }

    {
      const auto p = resampled_path.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 5.3, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
    }

    {
      const auto p = resampled_path.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 9.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
    }

    const double pitch = std::atan(1.0);
    const auto ans_quat = tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, 0.0);
    for (size_t i = 0; i < resampled_path.points.size(); ++i) {
      const auto p = resampled_path.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Lerp v
  {
    autoware_auto_planning_msgs::msg::Path path;
    path.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      path.points.at(i) = generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 5.3, 9.0};

    const auto resampled_path = resamplePath(path, resampled_arclength, false, true, false);
    {
      const auto p = resampled_path.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
    }

    {
      const auto p = resampled_path.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.2, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.6, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
    }

    {
      const auto p = resampled_path.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.3, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.65, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
    }

    {
      const auto p = resampled_path.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
    }

    for (size_t i = 0; i < resampled_path.points.size(); ++i) {
      const auto p = resampled_path.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);
    }
  }
}

TEST(resample_path, resample_trajectory_by_vector)
{
  using motion_utils::resampleTrajectory;
  // Output is same as input
  {
    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 3.0, 1.0, 0.01, 0.5);
    std::vector<double> resampled_arclength = generateArclength(10, 1.0);

    {
      const auto resampled_traj = resampleTrajectory(traj, resampled_arclength);
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        const auto p = resampled_traj.points.at(i);
        const auto ans_p = traj.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
        EXPECT_NEAR(p.acceleration_mps2, ans_p.acceleration_mps2, epsilon);
      }
    }

    // Change the last point orientation
    traj.points.back() = generateTestTrajectoryPoint(
      9.0, 0.0, 0.0, tier4_autoware_utils::pi / 3.0, 3.0, 1.0, 0.01, 0.5);
    {
      const auto resampled_path = resampleTrajectory(traj, resampled_arclength);
      for (size_t i = 0; i < resampled_path.points.size() - 1; ++i) {
        const auto p = resampled_path.points.at(i);
        const auto ans_p = traj.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
        EXPECT_NEAR(p.acceleration_mps2, ans_p.acceleration_mps2, epsilon);
      }

      const auto p = resampled_path.points.back();
      const auto ans_p = traj.points.back();
      const auto ans_quat = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
      EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
      EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
      EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, ans_quat.w, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, ans_p.acceleration_mps2, epsilon);
    }
  }

  // Output key is not same as input
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 1.5, 5.3, 7.5, 9.0};

    const auto resampled_path = resampleTrajectory(traj, resampled_arclength);

    {
      const auto p = resampled_path.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.0, epsilon);
    }

    {
      const auto p = resampled_path.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.05, epsilon);
    }

    {
      const auto p = resampled_path.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 1.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.15, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.05, epsilon);
    }

    {
      const auto p = resampled_path.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.25, epsilon);
    }

    {
      const auto p = resampled_path.points.at(4);
      EXPECT_NEAR(p.pose.position.x, 7.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 7.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 3.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.75, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.35, epsilon);
    }

    {
      const auto p = resampled_path.points.at(5);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.45, epsilon);
    }

    for (size_t i = 0; i < resampled_path.points.size(); ++i) {
      const auto p = resampled_path.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);
    }
  }

  // No Interpolation
  {
    // Input path size is not enough for interpolation
    {
      autoware_auto_planning_msgs::msg::Trajectory traj;
      traj.points.resize(1);
      for (size_t i = 0; i < 1; ++i) {
        traj.points.at(i) =
          generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
      }
      std::vector<double> resampled_arclength = generateArclength(10, 1.0);

      const auto resampled_traj = resampleTrajectory(traj, resampled_arclength);
      EXPECT_EQ(resampled_traj.points.size(), traj.points.size());
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.x, traj.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.y, traj.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.z, traj.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.x, traj.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.y, traj.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.z, traj.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.w, traj.points.at(i).pose.orientation.w,
          epsilon);
      }
    }

    // Resampled Arclength size is not enough for interpolation
    {
      autoware_auto_planning_msgs::msg::Trajectory traj;
      traj.points.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        traj.points.at(i) =
          generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
      }
      std::vector<double> resampled_arclength = generateArclength(1, 1.0);

      const auto resampled_traj = resampleTrajectory(traj, resampled_arclength);
      EXPECT_EQ(resampled_traj.points.size(), traj.points.size());
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.x, traj.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.y, traj.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.z, traj.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.x, traj.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.y, traj.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.z, traj.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.w, traj.points.at(i).pose.orientation.w,
          epsilon);
      }
    }

    // Resampled Arclength is longer than input path
    {
      autoware_auto_planning_msgs::msg::Trajectory traj;
      traj.points.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        traj.points.at(i) =
          generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
      }
      std::vector<double> resampled_arclength = generateArclength(3, 5.0);

      const auto resampled_traj = resampleTrajectory(traj, resampled_arclength);
      EXPECT_EQ(resampled_traj.points.size(), traj.points.size());
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.x, traj.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.y, traj.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.z, traj.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.x, traj.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.y, traj.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.z, traj.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.w, traj.points.at(i).pose.orientation.w,
          epsilon);
      }
    }
  }
}

TEST(resample_trajectory, resample_trajectory_by_vector_non_default)
{
  using motion_utils::resampleTrajectory;

  // Lerp x, y
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 5.3, 9.0};

    const auto resampled_traj = resampleTrajectory(traj, resampled_arclength, true);
    {
      const auto p = resampled_traj.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.0, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.05, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.25, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.45, epsilon);
    }

    for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);
    }
  }

  // Slerp z
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) = generateTestTrajectoryPoint(
        i * 1.0, 0.0, i * 1.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 5.3, 9.0};

    const auto resampled_traj = resampleTrajectory(traj, resampled_arclength, false, false);
    {
      const auto p = resampled_traj.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.0, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 1.2, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.05, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 5.3, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.25, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 9.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.45, epsilon);
    }

    const double pitch = std::atan(1.0);
    const auto ans_quat = tier4_autoware_utils::createQuaternionFromRPY(0.0, pitch, 0.0);
    for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Lerp twist
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    std::vector<double> resampled_arclength = {0.0, 1.2, 5.3, 9.0};

    const auto resampled_traj = resampleTrajectory(traj, resampled_arclength, false, true, false);
    {
      const auto p = resampled_traj.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.0, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.2, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 1.2, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 0.6, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.12, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.06, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 5.3, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 5.3, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 2.65, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.53, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.265, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, 0.45, epsilon);
    }

    for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);
    }
  }
}

TEST(resample_trajectory, resample_trajectory_by_same_interval)
{
  using motion_utils::resampleTrajectory;

  // Same point resampling
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }

    {
      const auto resampled_traj = resampleTrajectory(traj, 1.0);
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        const auto p = resampled_traj.points.at(i);
        const auto ans_p = traj.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
        EXPECT_NEAR(p.acceleration_mps2, ans_p.acceleration_mps2, epsilon);
      }
    }

    // Change the last point orientation
    traj.points.back() = generateTestTrajectoryPoint(
      9.0, 0.0, 0.0, tier4_autoware_utils::pi / 3.0, 3.0, 1.0, 0.01, 0.5);
    {
      const auto resampled_path = resampleTrajectory(traj, 1.0);
      for (size_t i = 0; i < resampled_path.points.size() - 1; ++i) {
        const auto p = resampled_path.points.at(i);
        const auto ans_p = traj.points.at(i);
        EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
        EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
        EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.x, ans_p.pose.orientation.x, epsilon);
        EXPECT_NEAR(p.pose.orientation.y, ans_p.pose.orientation.y, epsilon);
        EXPECT_NEAR(p.pose.orientation.z, ans_p.pose.orientation.z, epsilon);
        EXPECT_NEAR(p.pose.orientation.w, ans_p.pose.orientation.w, epsilon);
        EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
        EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
        EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
        EXPECT_NEAR(p.acceleration_mps2, ans_p.acceleration_mps2, epsilon);
      }

      const auto p = resampled_path.points.back();
      const auto ans_p = traj.points.back();
      const auto ans_quat = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
      EXPECT_NEAR(p.pose.position.x, ans_p.pose.position.x, epsilon);
      EXPECT_NEAR(p.pose.position.y, ans_p.pose.position.y, epsilon);
      EXPECT_NEAR(p.pose.position.z, ans_p.pose.position.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, ans_quat.w, epsilon);
      EXPECT_NEAR(p.longitudinal_velocity_mps, ans_p.longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, ans_p.lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, ans_p.heading_rate_rps, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, ans_p.acceleration_mps2, epsilon);
    }
  }

  // Normal Case without zero point
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    traj.points.at(0).longitudinal_velocity_mps = 5.0;

    const auto resampled_traj = resampleTrajectory(traj, 0.1);
    for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.position.x, 0.1 * i, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      const size_t idx = i / 10;
      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(idx).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(idx).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.01 * i, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(idx).acceleration_mps2, epsilon);
    }
  }

  // Normal Case without stop point but with terminal point
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    traj.points.at(0).longitudinal_velocity_mps = 5.0;

    const auto resampled_traj = resampleTrajectory(traj, 0.4);
    for (size_t i = 0; i < resampled_traj.points.size() - 1; ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.position.x, 0.4 * i, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      const size_t idx = i / 2.5;
      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(idx).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(idx).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.04 * i, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(idx).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(23);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      const size_t idx = 9;
      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(idx).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(idx).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(idx).acceleration_mps2, epsilon);
    }
  }

  // Normal Case without stop point but with terminal point (Boundary Condition)
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    traj.points.at(0).longitudinal_velocity_mps = 5.0;

    const auto resampled_traj = resampleTrajectory(traj, 0.999);
    for (size_t i = 0; i < resampled_traj.points.size() - 1; ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.position.x, 0.999 * i, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      std::cout << i << std::endl;
      const size_t idx = i == 0 ? 0 : i - 1;
      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(idx).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(idx).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0999 * i, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(idx).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(10);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      const size_t idx = 9;
      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(idx).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(idx).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(idx).acceleration_mps2, epsilon);
    }
  }

  // Normal Case with duplicated zero point
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    traj.points.at(0).longitudinal_velocity_mps = 5.0;
    traj.points.at(5).longitudinal_velocity_mps = 0.0;

    const auto resampled_traj = resampleTrajectory(traj, 0.1);
    EXPECT_EQ(resampled_traj.points.size(), static_cast<size_t>(91));
    for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
      const auto p = resampled_traj.points.at(i);
      EXPECT_NEAR(p.pose.position.x, 0.1 * i, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      const size_t idx = i / 10;
      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(idx).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(idx).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.01 * i, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(idx).acceleration_mps2, epsilon);
    }
  }

  // Normal Case with zero point
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    traj.points.at(0).longitudinal_velocity_mps = 8.0;
    traj.points.at(5).longitudinal_velocity_mps = 0.0;

    const auto resampled_traj = resampleTrajectory(traj, 1.5);
    EXPECT_EQ(resampled_traj.points.size(), static_cast<size_t>(8));
    {
      const auto p = resampled_traj.points.at(0);
      EXPECT_NEAR(p.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(0).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(0).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(0).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(1);
      EXPECT_NEAR(p.pose.position.x, 1.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(1).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(1).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.15, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(1).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(2);
      EXPECT_NEAR(p.pose.position.x, 3.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(3).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(3).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.30, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(3).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(3);
      EXPECT_NEAR(p.pose.position.x, 4.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(4).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(4).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.45, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(4).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(4);
      EXPECT_NEAR(p.pose.position.x, 5.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(5).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.50, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(5).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(5);
      EXPECT_NEAR(p.pose.position.x, 6.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(6).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(6).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.60, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(6).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(6);
      EXPECT_NEAR(p.pose.position.x, 7.5, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(7).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(7).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.75, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(7).acceleration_mps2, epsilon);
    }

    {
      const auto p = resampled_traj.points.at(7);
      EXPECT_NEAR(p.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(p.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p.pose.orientation.w, 1.0, epsilon);

      EXPECT_NEAR(
        p.longitudinal_velocity_mps, traj.points.at(9).longitudinal_velocity_mps, epsilon);
      EXPECT_NEAR(p.lateral_velocity_mps, traj.points.at(9).lateral_velocity_mps, epsilon);
      EXPECT_NEAR(p.heading_rate_rps, 0.90, epsilon);
      EXPECT_NEAR(p.acceleration_mps2, traj.points.at(9).acceleration_mps2, epsilon);
    }
  }

  // No Resample
  {
    // Input path size is not enough for resample
    {
      autoware_auto_planning_msgs::msg::Trajectory traj;
      traj.points.resize(1);
      for (size_t i = 0; i < 1; ++i) {
        traj.points.at(i) =
          generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
      }

      const auto resampled_traj = resampleTrajectory(traj, 1.0);
      EXPECT_EQ(resampled_traj.points.size(), traj.points.size());
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.x, traj.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.y, traj.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.z, traj.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.x, traj.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.y, traj.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.z, traj.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.w, traj.points.at(i).pose.orientation.w,
          epsilon);
      }
    }

    // Resample interval is invalid
    {
      autoware_auto_planning_msgs::msg::Trajectory traj;
      traj.points.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        traj.points.at(i) =
          generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
      }

      const auto resampled_traj = resampleTrajectory(traj, 1e-4);
      EXPECT_EQ(resampled_traj.points.size(), traj.points.size());
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.x, traj.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.y, traj.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.z, traj.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.x, traj.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.y, traj.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.z, traj.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.w, traj.points.at(i).pose.orientation.w,
          epsilon);
      }
    }

    // Resample interval is invalid (Negative value)
    {
      autoware_auto_planning_msgs::msg::Trajectory traj;
      traj.points.resize(10);
      for (size_t i = 0; i < 10; ++i) {
        traj.points.at(i) =
          generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
      }

      const auto resampled_traj = resampleTrajectory(traj, -5.0);
      EXPECT_EQ(resampled_traj.points.size(), traj.points.size());
      for (size_t i = 0; i < resampled_traj.points.size(); ++i) {
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.x, traj.points.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.y, traj.points.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.position.z, traj.points.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.x, traj.points.at(i).pose.orientation.x,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.y, traj.points.at(i).pose.orientation.y,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.z, traj.points.at(i).pose.orientation.z,
          epsilon);
        EXPECT_NEAR(
          resampled_traj.points.at(i).pose.orientation.w, traj.points.at(i).pose.orientation.w,
          epsilon);
      }
    }
  }
}
