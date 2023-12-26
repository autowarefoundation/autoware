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

#include "motion_utils/trajectory/conversion.hpp"
#include "motion_utils/trajectory/interpolation.hpp"
#include "motion_utils/trajectory/trajectory.hpp"
#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <gtest/gtest.h>
#include <gtest/internal/gtest-port.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
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

PathPointWithLaneId generateTestPathPoint(
  const double x, const double y, const double z, const double theta = 0.0,
  const double vel_lon = 0.0, const double vel_lat = 0.0, const double heading_rate = 0.0)
{
  PathPointWithLaneId p;
  p.point.pose = createPose(x, y, z, 0.0, 0.0, theta);
  p.point.longitudinal_velocity_mps = vel_lon;
  p.point.lateral_velocity_mps = vel_lat;
  p.point.heading_rate_rps = heading_rate;
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
    p.point.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.point.longitudinal_velocity_mps = vel_lon;
    p.point.lateral_velocity_mps = vel_lat;
    p.point.heading_rate_rps = heading_rate_rps;
    traj.points.push_back(p);
  }
  return traj;
}
}  // namespace

TEST(Interpolation, interpolate_path_for_trajectory)
{
  using motion_utils::calcInterpolatedPoint;

  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }

    // Same points as the trajectory point
    {
      const auto target_pose = createPose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose);

      EXPECT_NEAR(result.pose.position.x, 3.0, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 3.0, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 1.5, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.3, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.15, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }

    // Random Point
    {
      const auto target_pose = createPose(2.5, 5.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose);

      EXPECT_NEAR(result.pose.position.x, 2.5, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 1.25, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.25, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.125, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }

    // Random Point with zero order hold
    {
      const auto target_pose = createPose(2.5, 5.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose, true);

      EXPECT_NEAR(result.pose.position.x, 2.5, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 2.0, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.25, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.1, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }

    // Initial Point
    {
      const auto target_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose);

      EXPECT_NEAR(result.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.0, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }

    // Terminal Point
    {
      const auto target_pose = createPose(9.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose);

      EXPECT_NEAR(result.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.45, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }

    // Outside of initial point
    {
      const auto target_pose = createPose(-2.0, -9.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose);

      EXPECT_NEAR(result.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.0, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.0, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }

    // Outside of terminal point
    {
      const auto target_pose = createPose(10.0, 5.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(traj, target_pose);

      EXPECT_NEAR(result.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(result.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(result.heading_rate_rps, 0.9, epsilon);
      EXPECT_NEAR(result.acceleration_mps2, 0.45, epsilon);
      EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
      EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
    }
  }

  // Empty Point
  {
    const Trajectory traj;
    const auto target_pose = createPose(3.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    const auto result = calcInterpolatedPoint(traj, target_pose);

    EXPECT_NEAR(result.pose.position.x, 3.0, epsilon);
    EXPECT_NEAR(result.pose.position.y, 5.0, epsilon);
    EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
    EXPECT_NEAR(result.longitudinal_velocity_mps, 0.0, epsilon);
    EXPECT_NEAR(result.lateral_velocity_mps, 0.0, epsilon);
    EXPECT_NEAR(result.heading_rate_rps, 0.0, epsilon);
    EXPECT_NEAR(result.acceleration_mps2, 0.0, epsilon);
    EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
    EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
  }

  // One point
  {
    const auto traj = generateTestTrajectory<Trajectory>(1, 1.0, 1.0, 0.5, 0.1, 0.05);
    const auto target_pose = createPose(3.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    const auto result = calcInterpolatedPoint(traj, target_pose);

    EXPECT_NEAR(result.pose.position.x, 0.0, epsilon);
    EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
    EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
    EXPECT_NEAR(result.longitudinal_velocity_mps, 1.0, epsilon);
    EXPECT_NEAR(result.lateral_velocity_mps, 0.5, epsilon);
    EXPECT_NEAR(result.heading_rate_rps, 0.1, epsilon);
    EXPECT_NEAR(result.acceleration_mps2, 0.05, epsilon);
    EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
    EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
  }

  // Duplicated Points
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }
    traj.points.at(4) = traj.points.at(3);

    const auto target_pose = createPose(3.2, 5.0, 0.0, 0.0, 0.0, 0.0);
    const auto result = calcInterpolatedPoint(traj, target_pose);

    EXPECT_NEAR(result.pose.position.x, 3.0, epsilon);
    EXPECT_NEAR(result.pose.position.y, 0.0, epsilon);
    EXPECT_NEAR(result.pose.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.pose.orientation.w, 1.0, epsilon);
    EXPECT_NEAR(result.longitudinal_velocity_mps, 3.0, epsilon);
    EXPECT_NEAR(result.lateral_velocity_mps, 1.5, epsilon);
    EXPECT_NEAR(result.heading_rate_rps, 0.3, epsilon);
    EXPECT_NEAR(result.acceleration_mps2, 0.15, epsilon);
    EXPECT_NEAR(result.front_wheel_angle_rad, 0.0, epsilon);
    EXPECT_NEAR(result.rear_wheel_angle_rad, 0.0, epsilon);
  }
}

TEST(Interpolation, interpolate_path_for_path)
{
  using motion_utils::calcInterpolatedPoint;

  {
    autoware_auto_planning_msgs::msg::PathWithLaneId path;
    path.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      path.points.at(i) = generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
    }

    // Same points as the path point
    {
      const auto target_pose = createPose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose);

      EXPECT_NEAR(result.point.pose.position.x, 3.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 3.0, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 1.5, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.3, epsilon);
    }

    // Random Point
    {
      const auto target_pose = createPose(2.5, 5.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose);

      EXPECT_NEAR(result.point.pose.position.x, 2.5, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 2.5, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 1.25, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.25, epsilon);
    }

    // Random Point with zero order hold
    {
      const auto target_pose = createPose(2.5, 5.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose, true);

      EXPECT_NEAR(result.point.pose.position.x, 2.5, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 2.0, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 1.0, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.25, epsilon);
    }

    // Initial Point
    {
      const auto target_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose);

      EXPECT_NEAR(result.point.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.0, epsilon);
    }

    // Terminal Point
    {
      const auto target_pose = createPose(9.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose);

      EXPECT_NEAR(result.point.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.9, epsilon);
    }

    // Outside of initial point
    {
      const auto target_pose = createPose(-2.0, -9.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose);

      EXPECT_NEAR(result.point.pose.position.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 0.0, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.0, epsilon);
    }

    // Outside of terminal point
    {
      const auto target_pose = createPose(10.0, 5.0, 0.0, 0.0, 0.0, 0.0);
      const auto result = calcInterpolatedPoint(path, target_pose);

      EXPECT_NEAR(result.point.pose.position.x, 9.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
      EXPECT_NEAR(result.point.longitudinal_velocity_mps, 9.0, epsilon);
      EXPECT_NEAR(result.point.lateral_velocity_mps, 4.5, epsilon);
      EXPECT_NEAR(result.point.heading_rate_rps, 0.9, epsilon);
    }
  }

  // Empty Point
  {
    const PathWithLaneId path;
    const auto target_pose = createPose(3.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    const auto result = calcInterpolatedPoint(path, target_pose);

    EXPECT_NEAR(result.point.pose.position.x, 3.0, epsilon);
    EXPECT_NEAR(result.point.pose.position.y, 5.0, epsilon);
    EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
    EXPECT_NEAR(result.point.longitudinal_velocity_mps, 0.0, epsilon);
    EXPECT_NEAR(result.point.lateral_velocity_mps, 0.0, epsilon);
    EXPECT_NEAR(result.point.heading_rate_rps, 0.0, epsilon);
  }

  // One point
  {
    const auto path = generateTestPath<PathWithLaneId>(1, 1.0, 1.0, 0.5, 0.1);
    const auto target_pose = createPose(3.0, 5.0, 0.0, 0.0, 0.0, 0.0);
    const auto result = calcInterpolatedPoint(path, target_pose);

    EXPECT_NEAR(result.point.pose.position.x, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
    EXPECT_NEAR(result.point.longitudinal_velocity_mps, 1.0, epsilon);
    EXPECT_NEAR(result.point.lateral_velocity_mps, 0.5, epsilon);
    EXPECT_NEAR(result.point.heading_rate_rps, 0.1, epsilon);
  }

  // Duplicated Points
  {
    autoware_auto_planning_msgs::msg::PathWithLaneId path;
    path.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      path.points.at(i) = generateTestPathPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1);
    }
    path.points.at(4) = path.points.at(3);

    const auto target_pose = createPose(3.2, 5.0, 0.0, 0.0, 0.0, 0.0);
    const auto result = calcInterpolatedPoint(path, target_pose);

    EXPECT_NEAR(result.point.pose.position.x, 3.0, epsilon);
    EXPECT_NEAR(result.point.pose.position.y, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.point.pose.orientation.w, 1.0, epsilon);
    EXPECT_NEAR(result.point.longitudinal_velocity_mps, 3.0, epsilon);
    EXPECT_NEAR(result.point.lateral_velocity_mps, 1.5, epsilon);
    EXPECT_NEAR(result.point.heading_rate_rps, 0.3, epsilon);
  }
}

TEST(Interpolation, interpolate_points_with_length)
{
  using motion_utils::calcInterpolatedPose;

  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(10);
    for (size_t i = 0; i < 10; ++i) {
      traj.points.at(i) =
        generateTestTrajectoryPoint(i * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }

    // Same points as the trajectory point
    {
      const auto result = calcInterpolatedPose(traj.points, 3.0);

      EXPECT_NEAR(result.position.x, 3.0, epsilon);
      EXPECT_NEAR(result.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
    }

    // Random Point
    {
      const auto result = calcInterpolatedPose(traj.points, 2.5);

      EXPECT_NEAR(result.position.x, 2.5, epsilon);
      EXPECT_NEAR(result.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
    }

    // Negative length
    {
      const auto result = calcInterpolatedPose(traj.points, -3.0);

      EXPECT_NEAR(result.position.x, 0.0, epsilon);
      EXPECT_NEAR(result.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
    }

    // Boundary value (0.0)
    {
      const auto result = calcInterpolatedPose(traj.points, 0.0);

      EXPECT_NEAR(result.position.x, 0.0, epsilon);
      EXPECT_NEAR(result.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
    }

    // Terminal Point
    {
      const auto result = calcInterpolatedPose(traj.points, 9.0);

      EXPECT_NEAR(result.position.x, 9.0, epsilon);
      EXPECT_NEAR(result.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
    }

    // Outside of terminal point
    {
      const auto result = calcInterpolatedPose(traj.points, 10.0);

      EXPECT_NEAR(result.position.x, 9.0, epsilon);
      EXPECT_NEAR(result.position.y, 0.0, epsilon);
      EXPECT_NEAR(result.position.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
      EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
    }
  }

  // one point
  {
    autoware_auto_planning_msgs::msg::Trajectory traj;
    traj.points.resize(1);
    for (size_t i = 0; i < 1; ++i) {
      traj.points.at(i) = generateTestTrajectoryPoint(
        (i + 1) * 1.0, 0.0, 0.0, 0.0, i * 1.0, i * 0.5, i * 0.1, i * 0.05);
    }

    const auto result = calcInterpolatedPose(traj.points, 2.0);
    EXPECT_NEAR(result.position.x, 1.0, epsilon);
    EXPECT_NEAR(result.position.y, 0.0, epsilon);
    EXPECT_NEAR(result.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
  }

  // Empty Point
  {
    const Trajectory traj;
    const auto result = calcInterpolatedPose(traj.points, 2.0);

    EXPECT_NEAR(result.position.x, 0.0, epsilon);
    EXPECT_NEAR(result.position.y, 0.0, epsilon);
    EXPECT_NEAR(result.position.z, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.x, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.y, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.z, 0.0, epsilon);
    EXPECT_NEAR(result.orientation.w, 1.0, epsilon);
  }
}
