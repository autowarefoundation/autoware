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
using autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPointArray = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
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

template <class T>
T generateTestTrajectory(
  const size_t num_points, const double point_interval, const double vel = 0.0,
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
    p.longitudinal_velocity_mps = vel;
    traj.points.push_back(p);
  }

  return traj;
}

TrajectoryPointArray generateTestTrajectoryPointArray(
  const size_t num_points, const double point_interval, const double vel = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  TrajectoryPointArray traj;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    TrajectoryPoint p;
    p.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.longitudinal_velocity_mps = vel;
    traj.push_back(p);
  }

  return traj;
}

template <class T>
void updateTrajectoryVelocityAt(T & points, const size_t idx, const double vel)
{
  points.at(idx).longitudinal_velocity_mps = vel;
}
}  // namespace

TEST(trajectory, validateNonEmpty)
{
  using motion_utils::validateNonEmpty;

  // Empty
  EXPECT_THROW(validateNonEmpty(Trajectory{}.points), std::invalid_argument);

  // Non-empty
  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  EXPECT_NO_THROW(validateNonEmpty(traj.points));
}

TEST(trajectory, validateNonSharpAngle_DefaultThreshold)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::validateNonSharpAngle;

  TrajectoryPoint p1;
  p1.pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.longitudinal_velocity_mps = 0.0;

  TrajectoryPoint p2;
  p2.pose = createPose(1.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  p2.longitudinal_velocity_mps = 0.0;

  TrajectoryPoint p3;
  p3.pose = createPose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p3.longitudinal_velocity_mps = 0.0;

  // Non sharp angle
  {
    EXPECT_NO_THROW(validateNonSharpAngle(p1, p2, p3));
  }

  // Sharp angle
  {
    EXPECT_THROW(validateNonSharpAngle(p2, p1, p3), std::invalid_argument);
    EXPECT_THROW(validateNonSharpAngle(p1, p3, p2), std::invalid_argument);
  }
}

TEST(trajectory, validateNonSharpAngle_SetThreshold)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::validateNonSharpAngle;
  using tier4_autoware_utils::pi;

  TrajectoryPoint p1;
  p1.pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  p1.longitudinal_velocity_mps = 0.0;

  TrajectoryPoint p2;
  p2.pose = createPose(1.73205080756887729, 0.0, 0.0, 0.0, 0.0, 0.0);
  p2.longitudinal_velocity_mps = 0.0;

  TrajectoryPoint p3;
  p3.pose = createPose(1.73205080756887729, 1.0, 0.0, 0.0, 0.0, 0.0);
  p3.longitudinal_velocity_mps = 0.0;

  // Non sharp angle
  {
    EXPECT_NO_THROW(validateNonSharpAngle(p1, p2, p3, pi / 6));
    EXPECT_NO_THROW(validateNonSharpAngle(p2, p3, p1, pi / 6));
  }

  // Sharp angle
  {
    EXPECT_THROW(validateNonSharpAngle(p3, p1, p2, pi / 6), std::invalid_argument);
  }
}

TEST(trajectory, searchZeroVelocityIndex)
{
  using motion_utils::searchZeroVelocityIndex;

  // Empty
  EXPECT_FALSE(searchZeroVelocityIndex(Trajectory{}.points));

  // No zero velocity point
  {
    const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    EXPECT_FALSE(searchZeroVelocityIndex(traj.points));
  }

  // Only start point is zero
  {
    const size_t idx_ans = 0;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Only end point is zero
  {
    const size_t idx_ans = 9;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Only middle point is zero
  {
    const size_t idx_ans = 5;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Two points are zero
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);
    updateTrajectoryVelocityAt(traj.points, 6, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Negative velocity point is before zero velocity point
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, 2, -1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points), idx_ans);
  }

  // Search from src_idx to dst_idx
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_FALSE(searchZeroVelocityIndex(traj.points, 0, 3));
    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 0, 4), idx_ans);
    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 3, 10), idx_ans);
    EXPECT_FALSE(searchZeroVelocityIndex(traj.points, 4, 10));
  }
}

TEST(trajectory, searchZeroVelocityIndex_from_pose)
{
  using motion_utils::searchZeroVelocityIndex;

  // No zero velocity point
  {
    const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    EXPECT_FALSE(searchZeroVelocityIndex(traj.points, 0));
  }

  // Only start point is zero
  {
    const size_t idx_ans = 0;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 0), idx_ans);
  }

  // Only end point is zero
  {
    const size_t idx_ans = 9;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 0), idx_ans);
  }

  // Only middle point is zero
  {
    const size_t idx_ans = 5;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 0), idx_ans);
  }

  // Two points are zero
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);
    updateTrajectoryVelocityAt(traj.points, 6, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 0), idx_ans);
  }

  // Negative velocity point is before zero velocity point
  {
    const size_t idx_ans = 3;

    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    updateTrajectoryVelocityAt(traj.points, 2, -1.0);
    updateTrajectoryVelocityAt(traj.points, idx_ans, 0.0);

    EXPECT_EQ(*searchZeroVelocityIndex(traj.points, 0), idx_ans);
  }
}

TEST(trajectory, findNearestIndex_Pos_StraightTrajectory)
{
  using motion_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    findNearestIndex(Trajectory{}.points, geometry_msgs::msg::Point{}), std::invalid_argument);

  // Start point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(0.0, 0.0, 0.0)), 0U);

  // End point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(9.0, 0.0, 0.0)), 9U);

  // Boundary conditions
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(0.5, 0.0, 0.0)), 0U);
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(0.51, 0.0, 0.0)), 1U);

  // Point before start point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(-4.0, 5.0, 0.0)), 0U);

  // Point after end point
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(100.0, -3.0, 0.0)), 9U);

  // Random cases
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(2.4, 1.3, 0.0)), 2U);
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(4.0, 0.0, 0.0)), 4U);
}

TEST(trajectory, findNearestIndex_Pos_CurvedTrajectory)
{
  using motion_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(5.1, 3.4, 0.0)), 6U);
}

TEST(trajectory, findNearestIndex_Pose_NoThreshold)
{
  using motion_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_FALSE(findNearestIndex(Trajectory{}.points, geometry_msgs::msg::Pose{}, {}));

  // Start point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)), 0U);

  // End point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(9.0, 0.0, 0.0, 0.0, 0.0, 0.0)), 9U);

  // Boundary conditions
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)), 0U);
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(0.51, 0.0, 0.0, 0.0, 0.0, 0.0)), 1U);

  // Point before start point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(-4.0, 5.0, 0.0, 0.0, 0.0, 0.0)), 0U);

  // Point after end point
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(100.0, -3.0, 0.0, 0.0, 0.0, 0.0)), 9U);
}

TEST(trajectory, findNearestIndex_Pose_DistThreshold)
{
  using motion_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Out of threshold
  EXPECT_FALSE(findNearestIndex(traj.points, createPose(3.0, 0.6, 0.0, 0.0, 0.0, 0.0), 0.5));

  // On threshold
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(3.0, 0.5, 0.0, 0.0, 0.0, 0.0), 0.5), 3U);

  // Within threshold
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(3.0, 0.4, 0.0, 0.0, 0.0, 0.0), 0.5), 3U);
}

TEST(trajectory, findNearestIndex_Pose_YawThreshold)
{
  using motion_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto max_d = std::numeric_limits<double>::max();

  // Out of threshold
  EXPECT_FALSE(findNearestIndex(traj.points, createPose(3.0, 0.0, 0.0, 0.0, 0.0, 1.1), max_d, 1.0));

  // On threshold
  EXPECT_EQ(
    *findNearestIndex(traj.points, createPose(3.0, 0.0, 0.0, 0.0, 0.0, 1.0), max_d, 1.0), 3U);

  // Within threshold
  EXPECT_EQ(
    *findNearestIndex(traj.points, createPose(3.0, 0.0, 0.0, 0.0, 0.0, 0.9), max_d, 1.0), 3U);
}

TEST(trajectory, findNearestIndex_Pose_DistAndYawThreshold)
{
  using motion_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Random cases
  EXPECT_EQ(*findNearestIndex(traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0, 0.4), 2U);
  EXPECT_EQ(
    *findNearestIndex(traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8), 0.5, 1.0), 4U);
  EXPECT_EQ(
    *findNearestIndex(traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0), 1.0, 0.1), 8U);
}

TEST(trajectory, findNearestSegmentIndex)
{
  using motion_utils::findNearestSegmentIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    findNearestSegmentIndex(Trajectory{}.points, geometry_msgs::msg::Point{}),
    std::invalid_argument);

  // Start point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(0.0, 0.0, 0.0)), 0U);

  // End point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(9.0, 0.0, 0.0)), 8U);

  // Boundary conditions
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(1.0, 0.0, 0.0)), 0U);
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(1.1, 0.0, 0.0)), 1U);

  // Point before start point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(-4.0, 5.0, 0.0)), 0U);

  // Point after end point
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(100.0, -3.0, 0.0)), 8U);

  // Random cases
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(2.4, 1.0, 0.0)), 2U);
  EXPECT_EQ(findNearestSegmentIndex(traj.points, createPoint(4.0, 0.0, 0.0)), 3U);

  // Two nearest trajectory points are not the edges of the nearest segment.
  std::vector<geometry_msgs::msg::Point> sparse_points{
    createPoint(0.0, 0.0, 0.0),
    createPoint(10.0, 0.0, 0.0),
    createPoint(11.0, 0.0, 0.0),
  };
  EXPECT_EQ(findNearestSegmentIndex(sparse_points, createPoint(9.0, 1.0, 0.0)), 0U);
}

TEST(trajectory, calcLongitudinalOffsetToSegment_StraightTrajectory)
{
  using motion_utils::calcLongitudinalOffsetToSegment;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const bool throw_exception = true;

  // Empty
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(
      Trajectory{}.points, {}, geometry_msgs::msg::Point{}, throw_exception),
    std::invalid_argument);

  // Out of range
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(traj.points, -1, geometry_msgs::msg::Point{}, throw_exception),
    std::out_of_range);
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(
      traj.points, traj.points.size() - 1, geometry_msgs::msg::Point{}, throw_exception),
    std::out_of_range);

  // Same close points in trajectory
  {
    const auto invalid_traj = generateTestTrajectory<Trajectory>(10, 0.0);
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_THROW(
      calcLongitudinalOffsetToSegment(invalid_traj.points, 3, p, throw_exception),
      std::runtime_error);
  }

  // Same point
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 3, createPoint(3.0, 0.0, 0.0)), 0.0, epsilon);

  // Point before start point
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 6, createPoint(-3.9, 3.0, 0.0)), -9.9, epsilon);

  // Point after start point
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 7, createPoint(13.3, -10.0, 0.0)), 6.3, epsilon);

  // Random cases
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 2, createPoint(4.3, 7.0, 0.0)), 2.3, epsilon);
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 8, createPoint(1.0, 3.0, 0.0)), -7, epsilon);
}

TEST(trajectory, calcLongitudinalOffsetToSegment_CurveTrajectory)
{
  using motion_utils::calcLongitudinalOffsetToSegment;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 2, createPoint(2.0, 0.5, 0.0)), 0.083861449,
    epsilon);
}

TEST(trajectory, calcLateralOffset)
{
  using motion_utils::calcLateralOffset;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const bool throw_exception = true;

  // Empty
  EXPECT_THROW(
    calcLateralOffset(Trajectory{}.points, geometry_msgs::msg::Point{}, throw_exception),
    std::invalid_argument);

  // Trajectory size is 1
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_THROW(
      calcLateralOffset(one_point_traj.points, geometry_msgs::msg::Point{}, throw_exception),
      std::runtime_error);
  }

  // Same close points in trajectory
  {
    const auto invalid_traj = generateTestTrajectory<Trajectory>(10, 0.0);
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_THROW(calcLateralOffset(invalid_traj.points, p, throw_exception), std::runtime_error);
  }

  // Point on trajectory
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(3.1, 0.0, 0.0)), 0.0, epsilon);

  // Point before start point
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(-3.9, 3.0, 0.0)), 3.0, epsilon);

  // Point after start point
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(13.3, -10.0, 0.0)), -10.0, epsilon);

  // Random cases
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(4.3, 7.0, 0.0)), 7.0, epsilon);
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(1.0, -3.0, 0.0)), -3.0, epsilon);
}

TEST(trajectory, calcLateralOffset_without_segment_idx)
{
  using motion_utils::calcLateralOffset;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const bool throw_exception = true;

  // Empty
  EXPECT_THROW(
    calcLateralOffset(Trajectory{}.points, geometry_msgs::msg::Point{}, throw_exception),
    std::invalid_argument);

  // Trajectory size is 1
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_THROW(
      calcLateralOffset(
        one_point_traj.points, geometry_msgs::msg::Point{}, static_cast<size_t>(0),
        throw_exception),
      std::runtime_error);
  }

  // Same close points in trajectory
  {
    const auto invalid_traj = generateTestTrajectory<Trajectory>(10, 0.0);
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_THROW(
      calcLateralOffset(invalid_traj.points, p, static_cast<size_t>(2), throw_exception),
      std::runtime_error);
    EXPECT_THROW(
      calcLateralOffset(invalid_traj.points, p, static_cast<size_t>(3), throw_exception),
      std::runtime_error);
  }

  // Point on trajectory
  EXPECT_NEAR(
    calcLateralOffset(traj.points, createPoint(3.1, 0.0, 0.0), static_cast<size_t>(3)), 0.0,
    epsilon);

  // Point before start point
  EXPECT_NEAR(
    calcLateralOffset(traj.points, createPoint(-3.9, 3.0, 0.0), static_cast<size_t>(0)), 3.0,
    epsilon);

  // Point after start point
  EXPECT_NEAR(
    calcLateralOffset(traj.points, createPoint(13.3, -10.0, 0.0), static_cast<size_t>(8)), -10.0,
    epsilon);

  // Random cases
  EXPECT_NEAR(
    calcLateralOffset(traj.points, createPoint(4.3, 7.0, 0.0), static_cast<size_t>(4)), 7.0,
    epsilon);
  EXPECT_NEAR(
    calcLateralOffset(traj.points, createPoint(1.0, -3.0, 0.0), static_cast<size_t>(0)), -3.0,
    epsilon);
  EXPECT_NEAR(
    calcLateralOffset(traj.points, createPoint(1.0, -3.0, 0.0), static_cast<size_t>(1)), -3.0,
    epsilon);
}

TEST(trajectory, calcLateralOffset_CurveTrajectory)
{
  using motion_utils::calcLateralOffset;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(2.0, 0.5, 0.0)), 0.071386083, epsilon);
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(5.0, 1.0, 0.0)), -1.366602819, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromIndexToIndex)
{
  using motion_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcSignedArcLength(Trajectory{}.points, {}, {}), 0.0);

  // Out of range
  EXPECT_THROW(calcSignedArcLength(traj.points, -1, 1), std::out_of_range);
  EXPECT_THROW(calcSignedArcLength(traj.points, 0, traj.points.size() + 1), std::out_of_range);

  // Same point
  EXPECT_NEAR(calcSignedArcLength(traj.points, 3, 3), 0, epsilon);

  // Forward
  EXPECT_NEAR(calcSignedArcLength(traj.points, 0, 3), 3, epsilon);

  // Backward
  EXPECT_NEAR(calcSignedArcLength(traj.points, 9, 5), -4, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromPointToIndex)
{
  using motion_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcSignedArcLength(Trajectory{}.points, {}, {}), 0.0);

  // Same point
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(3.0, 0.0, 0.0), 3), 0, epsilon);

  // Forward
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(0.0, 0.0, 0.0), 3), 3, epsilon);

  // Backward
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(9.0, 0.0, 0.0), 5), -4, epsilon);

  // Point before start point
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(-3.9, 3.0, 0.0), 6), 9.9, epsilon);

  // Point after end point
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(13.3, -10.0, 0.0), 7), -6.3, epsilon);

  // Random cases
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(1.0, 3.0, 0.0), 9), 8, epsilon);
  EXPECT_NEAR(calcSignedArcLength(traj.points, createPoint(4.3, 7.0, 0.0), 2), -2.3, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromIndexToPoint)
{
  using motion_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcSignedArcLength(Trajectory{}.points, {}, {}), 0.0);

  // Same point
  EXPECT_NEAR(calcSignedArcLength(traj.points, 3, createPoint(3.0, 0.0, 0.0)), 0, epsilon);

  // Forward
  EXPECT_NEAR(calcSignedArcLength(traj.points, 0, createPoint(3.0, 0.0, 0.0)), 3, epsilon);

  // Backward
  EXPECT_NEAR(calcSignedArcLength(traj.points, 9, createPoint(5.0, 0.0, 0.0)), -4, epsilon);

  // Point before start point
  EXPECT_NEAR(calcSignedArcLength(traj.points, 6, createPoint(-3.9, 3.0, 0.0)), -9.9, epsilon);

  // Point after end point
  EXPECT_NEAR(calcSignedArcLength(traj.points, 7, createPoint(13.3, -10.0, 0.0)), 6.3, epsilon);

  // Random cases
  EXPECT_NEAR(calcSignedArcLength(traj.points, 1, createPoint(9.0, 3.0, 0.0)), 8, epsilon);
  EXPECT_NEAR(calcSignedArcLength(traj.points, 4, createPoint(1.7, 7.0, 0.0)), -2.3, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromPointToPoint)
{
  using motion_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcSignedArcLength(Trajectory{}.points, {}, {}), 0.0);

  // Same point
  {
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p, p), 0, epsilon);
  }

  // Forward
  {
    const auto p1 = createPoint(0.0, 0.0, 0.0);
    const auto p2 = createPoint(3.0, 1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 3, epsilon);
  }

  // Backward
  {
    const auto p1 = createPoint(8.0, 0.0, 0.0);
    const auto p2 = createPoint(9.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 1, epsilon);
  }

  // Point before start point
  {
    const auto p1 = createPoint(-3.9, 3.0, 0.0);
    const auto p2 = createPoint(6.0, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 9.9, epsilon);
  }

  // Point after end point
  {
    const auto p1 = createPoint(7.0, -5.0, 0.0);
    const auto p2 = createPoint(13.3, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 6.3, epsilon);
  }

  // Point before start point and after end point
  {
    const auto p1 = createPoint(-4.3, 10.0, 0.0);
    const auto p2 = createPoint(13.8, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 18.1, epsilon);
  }

  // Random cases
  {
    const auto p1 = createPoint(1.0, 3.0, 0.0);
    const auto p2 = createPoint(9.0, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), 8, epsilon);
  }
  {
    const auto p1 = createPoint(4.3, 7.0, 0.0);
    const auto p2 = createPoint(2.0, 3.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, p2), -2.3, epsilon);
  }
}

TEST(trajectory, calcArcLength)
{
  using motion_utils::calcArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcArcLength(Trajectory{}.points), 0.0);

  // Whole Length
  EXPECT_NEAR(calcArcLength(traj.points), 9.0, epsilon);
}

TEST(trajectory, convertToTrajectory)
{
  using motion_utils::convertToTrajectory;

  // Size check
  {
    const auto traj_input = generateTestTrajectoryPointArray(50, 1.0);
    const auto traj = convertToTrajectory(traj_input);
    EXPECT_EQ(traj.points.size(), traj_input.size());
  }
}

TEST(trajectory, convertToTrajectoryPointArray)
{
  using motion_utils::convertToTrajectoryPointArray;

  const auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0);
  const auto traj = convertToTrajectoryPointArray(traj_input);

  // Size check
  EXPECT_EQ(traj.size(), traj_input.points.size());

  // Value check
  for (size_t i = 0; i < traj.size(); ++i) {
    EXPECT_EQ(traj.at(i), traj_input.points.at(i));
  }
}

TEST(trajectory, calcDistanceToForwardStopPointFromIndex)
{
  using motion_utils::calcDistanceToForwardStopPoint;

  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Empty
  {
    EXPECT_FALSE(calcDistanceToForwardStopPoint(Trajectory{}.points));
  }

  // No Stop Point
  {
    const auto traj_no_stop_point = generateTestTrajectory<Trajectory>(10, 1.0, 3.0);
    for (size_t i = 0; i < 10; ++i) {
      const auto dist = calcDistanceToForwardStopPoint(traj_no_stop_point.points, i);
      EXPECT_FALSE(dist);
    }
  }

  // Boundary1 (Edge of the input trajectory)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 0);
    EXPECT_NEAR(dist.value(), 50.0, epsilon);
  }

  // Boundary2 (Edge of the input trajectory)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 99);
    EXPECT_FALSE(dist);
  }

  // Boundary3 (On the Stop Point)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 50);
    EXPECT_NEAR(dist.value(), 0.0, epsilon);
  }

  // Boundary4 (Right before the stop point)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 49);
    EXPECT_NEAR(dist.value(), 1.0, epsilon);
  }

  // Boundary5 (Right behind the stop point)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 51);
    EXPECT_FALSE(dist);
  }

  // Random
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 20);
    EXPECT_NEAR(dist.value(), 30.0, epsilon);
  }
}

TEST(trajectory, calcDistanceToForwardStopPointFromPose)
{
  using motion_utils::calcDistanceToForwardStopPoint;

  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Empty
  {
    EXPECT_FALSE(calcDistanceToForwardStopPoint(Trajectory{}.points, geometry_msgs::msg::Pose{}));
  }

  // No Stop Point
  {
    const auto traj_no_stop_point = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
    const auto pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_no_stop_point.points, pose);
    EXPECT_FALSE(dist);
  }

  // Trajectory Edge1
  {
    const auto pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.value(), 50.0, epsilon);
  }

  // Trajectory Edge2
  {
    const auto pose = createPose(99.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_FALSE(dist);
  }

  // Out of Trajectory1
  {
    const auto pose = createPose(-10.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.value(), 60.0, epsilon);
  }

  // Out of Trajectory2
  {
    const auto pose = createPose(200.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_FALSE(dist);
  }

  // Out of Trajectory3
  {
    const auto pose = createPose(-30.0, 50.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.value(), 80.0, epsilon);
  }

  // Boundary Condition1
  {
    const auto pose = createPose(50.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.value(), 0.0, epsilon);
  }

  // Boundary Condition2
  {
    const auto pose = createPose(50.1, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_FALSE(dist);
  }

  // Boundary Condition3
  {
    const auto pose = createPose(49.9, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.value(), 0.1, epsilon);
  }

  // Random
  {
    const auto pose = createPose(3.0, 2.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.value(), 47.0, epsilon);
  }
}

TEST(trajectory, calcDistanceToForwardStopPoint_DistThreshold)
{
  using motion_utils::calcDistanceToForwardStopPoint;

  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Boundary Condition1
  {
    const auto pose = createPose(-4.9, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
    EXPECT_NEAR(dist.value(), 54.9, epsilon);
  }

  // Boundary Condition2
  {
    const auto pose = createPose(-5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
    EXPECT_NEAR(dist.value(), 55.0, epsilon);
  }

  // Boundary Condition3
  {
    const auto pose = createPose(-5.1, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
    EXPECT_FALSE(dist);
  }

  // Random
  {
    {
      const auto pose = createPose(3.0, 2.0, 0.0, 0.0, 0.0, 0.0);
      const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
      EXPECT_NEAR(dist.value(), 47.0, epsilon);
    }

    {
      const auto pose = createPose(200.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
      EXPECT_FALSE(dist);
    }
  }
}

TEST(trajectory, calcDistanceToForwardStopPoint_YawThreshold)
{
  using motion_utils::calcDistanceToForwardStopPoint;
  using tier4_autoware_utils::deg2rad;

  const auto max_d = std::numeric_limits<double>::max();
  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Boundary Condition
  {
    const double x = 2.0;
    {
      const auto pose = createPose(x, 0.0, 0.0, 0.0, 0.0, deg2rad(29.9));
      const auto dist =
        calcDistanceToForwardStopPoint(traj_input.points, pose, max_d, deg2rad(30.0));
      EXPECT_NEAR(dist.value(), 48.0, epsilon);
    }

    {
      const auto pose = createPose(x, 0.0, 0.0, 0.0, 0.0, deg2rad(30.0));
      const auto dist =
        calcDistanceToForwardStopPoint(traj_input.points, pose, max_d, deg2rad(30.0));
      EXPECT_NEAR(dist.value(), 48.0, epsilon);
    }

    {
      const auto pose = createPose(x, 0.0, 0.0, 0.0, 0.0, deg2rad(30.1));
      const auto dist =
        calcDistanceToForwardStopPoint(traj_input.points, pose, max_d, deg2rad(30.0));
      EXPECT_FALSE(dist);
    }
  }

  // Random
  {
    {
      const auto pose = createPose(3.0, 2.0, 0.0, 0.0, 0.0, deg2rad(15.0));
      const auto dist =
        calcDistanceToForwardStopPoint(traj_input.points, pose, max_d, deg2rad(20.0));
      EXPECT_NEAR(dist.value(), 47.0, epsilon);
    }

    {
      const auto pose = createPose(15.0, 30.0, 0.0, 0.0, 0.0, deg2rad(45.0));
      const auto dist =
        calcDistanceToForwardStopPoint(traj_input.points, pose, max_d, deg2rad(10.0));
      EXPECT_FALSE(dist);
    }
  }
}

TEST(trajectory, calcLongitudinalOffsetPointFromIndex)
{
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPoint;
  using motion_utils::calcSignedArcLength;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_FALSE(calcLongitudinalOffsetPoint(Trajectory{}.points, {}, {}));

  // Out of range
  EXPECT_FALSE(calcLongitudinalOffsetPoint(traj.points, traj.points.size() + 1, 1.0));
  EXPECT_FALSE(calcLongitudinalOffsetPoint(traj.points, -1, 1.0));

  // Found Pose(forward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_back = calcSignedArcLength(traj.points, i, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, i, std::min(len, d_back));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().z, 0.0, epsilon);

      x_ans += 0.1;
    }
  }

  // Found Pose(backward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_front = calcSignedArcLength(traj.points, i, 0);

    for (double len = 0.0; d_front - epsilon < len; len -= 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, i, std::max(len, d_front));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().z, 0.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, 0, total_length + epsilon);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, 9, -total_length - epsilon);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // No found(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPoint(one_point_traj.points, 0.0, 0.0);

    EXPECT_EQ(p_out, std::nullopt);
  }
}

TEST(trajectory, calcLongitudinalOffsetPointFromPoint)
{
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPoint;
  using motion_utils::calcSignedArcLength;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_FALSE(calcLongitudinalOffsetPoint(Trajectory{}.points, {}, {}));

  // Found Pose(forward)
  for (double x_start = 0.0; x_start < total_length + epsilon; x_start += 0.1) {
    constexpr double lateral_deviation = 0.5;
    double x_ans = x_start;

    const auto p_src = createPoint(x_start, lateral_deviation, 0.0);
    const auto d_back = calcSignedArcLength(traj.points, p_src, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, std::min(len, d_back));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().z, 0.0, epsilon);

      x_ans += 0.1;
    }
  }

  // Found Pose(backward)
  for (double x_start = 0.0; x_start < total_length + epsilon; x_start += 0.1) {
    constexpr double lateral_deviation = 0.5;
    double x_ans = x_start;

    const auto p_src = createPoint(x_start, lateral_deviation, 0.0);
    const auto d_front = calcSignedArcLength(traj.points, p_src, 0);

    for (double len = 0.0; d_front - epsilon < len; len -= 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, std::max(len, d_front));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().z, 0.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_src = createPoint(0.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, total_length + 1.0);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // No found
  {
    const auto p_src = createPoint(9.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, -total_length - 1.0);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // Out of range(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_FALSE(
      calcLongitudinalOffsetPoint(one_point_traj.points, geometry_msgs::msg::Point{}, {}));
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex)
{
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPose;
  using motion_utils::calcSignedArcLength;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_FALSE(calcLongitudinalOffsetPose(Trajectory{}.points, {}, {}));

  // Out of range
  EXPECT_FALSE(calcLongitudinalOffsetPose(traj.points, traj.points.size() + 1, 1.0));
  EXPECT_FALSE(calcLongitudinalOffsetPose(traj.points, -1, 1.0));

  // Found Pose(forward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_back = calcSignedArcLength(traj.points, i, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, i, std::min(len, d_back));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.w, 1.0, epsilon);

      x_ans += 0.1;
    }
  }

  // Found Pose(backward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_front = calcSignedArcLength(traj.points, i, 0);

    for (double len = 0.0; d_front - epsilon < len; len -= 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, i, std::max(len, d_front));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.w, 1.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, total_length + epsilon);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 9, -total_length - epsilon);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // No found(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPose(one_point_traj.points, 0.0, 0.0);

    EXPECT_EQ(p_out, std::nullopt);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex_quatInterpolation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::deg2rad;

  Trajectory traj{};

  {
    TrajectoryPoint p;
    p.pose = createPose(0.0, 0.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  {
    TrajectoryPoint p;
    p.pose = createPose(1.0, 1.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  const auto total_length = calcArcLength(traj.points);

  // Found pose(forward)
  for (double len = 0.0; len < total_length; len += 0.1) {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, len);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, len * std::cos(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.y, len * std::sin(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }

  // Found pose(backward)
  for (double len = total_length; 0.0 < len; len -= 0.1) {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 1, -len);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0 - len * std::cos(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0 - len * std::sin(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }

  // Boundary condition
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, total_length);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }

  // Boundary condition
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 1, 0.0);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex_quatSphericalInterpolation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::deg2rad;

  Trajectory traj{};

  {
    TrajectoryPoint p;
    p.pose = createPose(0.0, 0.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  {
    TrajectoryPoint p;
    p.pose = createPose(1.0, 1.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  const auto total_length = calcArcLength(traj.points);

  // Found pose(forward)
  for (double len = 0.0; len < total_length; len += 0.1) {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, len, false);
    // ratio between two points
    const auto ratio = len / total_length;
    const auto ans_quat =
      createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0 - 45.0 * ratio));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, len * std::cos(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.y, len * std::sin(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }

  // Found pose(backward)
  for (double len = total_length; 0.0 < len; len -= 0.1) {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 1, -len, false);
    // ratio between two points
    const auto ratio = len / total_length;
    const auto ans_quat =
      createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0 * ratio));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0 - len * std::cos(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0 - len * std::sin(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }

  // Boundary condition
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, total_length, false);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }

  // Boundary condition
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 1, 0.0, false);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint)
{
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPose;
  using motion_utils::calcSignedArcLength;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_FALSE(calcLongitudinalOffsetPose(Trajectory{}.points, {}, {}));

  // Found Pose(forward)
  for (double x_start = 0.0; x_start < total_length + epsilon; x_start += 0.1) {
    constexpr double lateral_deviation = 0.5;
    double x_ans = x_start;

    const auto p_src = createPoint(x_start, lateral_deviation, 0.0);
    const auto d_back = calcSignedArcLength(traj.points, p_src, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, std::min(len, d_back));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.w, 1.0, epsilon);

      x_ans += 0.1;
    }
  }

  // Found Pose(backward)
  for (double x_start = 0.0; x_start < total_length + epsilon; x_start += 0.1) {
    constexpr double lateral_deviation = 0.5;
    double x_ans = x_start;

    const auto p_src = createPoint(x_start, lateral_deviation, 0.0);
    const auto d_front = calcSignedArcLength(traj.points, p_src, 0);

    for (double len = 0.0; d_front - epsilon < len; len -= 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, std::max(len, d_front));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(p_out.value().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.value().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.w, 1.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_src = createPoint(0.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, total_length + 1.0);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // No found
  {
    const auto p_src = createPoint(9.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, -total_length - 1.0);

    EXPECT_EQ(p_out, std::nullopt);
  }

  // Out of range(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_FALSE(
      calcLongitudinalOffsetPose(one_point_traj.points, geometry_msgs::msg::Point{}, {}));
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint_quatInterpolation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPose;
  using motion_utils::calcLongitudinalOffsetToSegment;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;

  Trajectory traj{};

  {
    TrajectoryPoint p;
    p.pose = createPose(0.0, 0.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  {
    TrajectoryPoint p;
    p.pose = createPose(1.0, 1.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  const auto total_length = calcArcLength(traj.points);

  // Found pose
  for (double len_start = 0.0; len_start < total_length; len_start += 0.1) {
    constexpr double deviation = 0.1;

    const auto p_src = createPoint(
      len_start * std::cos(deg2rad(45.0)) + deviation,
      len_start * std::sin(deg2rad(45.0)) - deviation, 0.0);
    const auto src_offset = calcLongitudinalOffsetToSegment(traj.points, 0, p_src);

    for (double len = -src_offset; len < total_length - src_offset; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, len);
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(
        p_out.value().position.x, p_src.x + len * std::cos(deg2rad(45.0)) - deviation, epsilon);
      EXPECT_NEAR(
        p_out.value().position.y, p_src.y + len * std::sin(deg2rad(45.0)) + deviation, epsilon);
      EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary condition
  {
    constexpr double deviation = 0.1;

    const auto p_src = createPoint(1.0 + deviation, 1.0 - deviation, 0.0);
    const auto src_offset = calcLongitudinalOffsetToSegment(traj.points, 0, p_src);

    const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, total_length - src_offset);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint_quatSphericalInterpolation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::calcArcLength;
  using motion_utils::calcLongitudinalOffsetPose;
  using motion_utils::calcLongitudinalOffsetToSegment;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;

  Trajectory traj{};

  {
    TrajectoryPoint p;
    p.pose = createPose(0.0, 0.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  {
    TrajectoryPoint p;
    p.pose = createPose(1.0, 1.0, 0.0, deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
    p.longitudinal_velocity_mps = 0.0;
    traj.points.push_back(p);
  }

  const auto total_length = calcArcLength(traj.points);

  // Found pose
  for (double len_start = 0.0; len_start < total_length; len_start += 0.1) {
    constexpr double deviation = 0.1;

    const auto p_src = createPoint(
      len_start * std::cos(deg2rad(45.0)) + deviation,
      len_start * std::sin(deg2rad(45.0)) - deviation, 0.0);
    const auto src_offset = calcLongitudinalOffsetToSegment(traj.points, 0, p_src);

    for (double len = -src_offset; len < total_length - src_offset; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, len, false);
      // ratio between two points
      const auto ratio = (src_offset + len) / total_length;
      const auto ans_quat =
        createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0 - 45.0 * ratio));

      EXPECT_NE(p_out, std::nullopt);
      EXPECT_NEAR(
        p_out.value().position.x, p_src.x + len * std::cos(deg2rad(45.0)) - deviation, epsilon);
      EXPECT_NEAR(
        p_out.value().position.y, p_src.y + len * std::sin(deg2rad(45.0)) + deviation, epsilon);
      EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary condition
  {
    constexpr double deviation = 0.1;

    const auto p_src = createPoint(1.0 + deviation, 1.0 - deviation, 0.0);
    const auto src_offset = calcLongitudinalOffsetToSegment(traj.points, 0, p_src);

    const auto p_out =
      calcLongitudinalOffsetPose(traj.points, p_src, total_length - src_offset, false);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, std::nullopt);
    EXPECT_NEAR(p_out.value().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.value().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.value().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.value().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.value().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.value().orientation.w, ans_quat.w, epsilon);
  }
}

TEST(trajectory, insertTargetPoint)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Boundary condition)
  for (double x_start = 0.0; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1.1e-3, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Quaternion interpolation)
  for (double x_start = 0.25; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.25 * std::tan(deg2rad(60.0)), 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(-30.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(60.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Not insert(Overlap base_idx point)
  for (double x_start = 0.0; x_start < total_length - 1.0 + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
  }

  // Not insert(Overlap base_idx + 1 point)
  for (double x_start = 1.0; x_start < total_length + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start - 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
  }

  // Invalid target point(In front of begin point)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(-1.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle"), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Behind of end point)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle"), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Huge lateral offset)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(4.0, 10.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle"), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid base index
  {
    auto traj_out = traj;

    const size_t segment_idx = 9U;
    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(segment_idx, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory<Trajectory>(0, 1.0);
    const size_t segment_idx = 0;
    EXPECT_FALSE(insertTargetPoint(segment_idx, geometry_msgs::msg::Point{}, empty_traj.points));
  }
}

TEST(trajectory, insertTargetPoint_Reverse)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::createQuaternionFromYaw;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  constexpr double overlap_threshold = 1e-4;
  auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  for (size_t i = 0; i < traj.points.size(); ++i) {
    traj.points.at(i).pose.orientation = createQuaternionFromYaw(tier4_autoware_utils::pi);
  }
  const auto total_length = calcArcLength(traj.points);

  for (double x_start = 0.0; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1.1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, traj_out.points, overlap_threshold);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(
        calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > overlap_threshold);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(180));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(180));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }
}

TEST(trajectory, insertTargetPoint_OverlapThreshold)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  constexpr double overlap_threshold = 1e-4;
  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert(Boundary condition)
  for (double x_start = 0.0; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1.1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, traj_out.points, overlap_threshold);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(
        calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > overlap_threshold);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Not insert(Overlap base_idx point)
  for (double x_start = 0.0; x_start < total_length - 1.0 + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1e-5, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, traj_out.points, overlap_threshold);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(
        calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > overlap_threshold);
    }
  }

  // Not insert(Overlap base_idx + 1 point)
  for (double x_start = 1.0; x_start < total_length + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start - 1e-5, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx =
      insertTargetPoint(base_idx, p_target, traj_out.points, overlap_threshold);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(
        calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > overlap_threshold);
    }
  }
}

TEST(trajectory, insertTargetPoint_Length)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(x_start, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Boundary condition)
  for (double x_start = 0.0; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1.1e-3, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(x_start + 1.1e-3, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Right on the terminal point
  {
    auto traj_out = traj;

    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(9.0, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Quaternion interpolation)
  for (double x_start = 0.25; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.25 * std::tan(deg2rad(60.0)), 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(x_start, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(-30.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(60.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Not insert(Overlap base_idx point)
  for (double x_start = 0.0; x_start < total_length - 1.0 + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(x_start + 1e-4, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
  }

  // Not insert(Overlap base_idx + 1 point)
  for (double x_start = 1.0; x_start < total_length + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start - 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(x_start - 1e-4, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
  }

  // Invalid target point(In front of the beginning point)
  {
    auto traj_out = traj;

    const auto p_target = createPoint(-1.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(-1.0, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Behind the end point)
  {
    auto traj_out = traj;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(10.0, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Huge lateral offset)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(4.0, 10.0, 0.0);
    const auto insert_idx = insertTargetPoint(4.0, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle."), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory<Trajectory>(0, 1.0);
    EXPECT_THROW(
      insertTargetPoint(0.0, geometry_msgs::msg::Point{}, empty_traj.points),
      std::invalid_argument);
  }
}

TEST(trajectory, insertTargetPoint_Length_Without_Target_Point)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(0, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Boundary condition)
  for (double x_start = 0.0; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1.1e-3, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(0, x_start + 1.1e-3, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Right on the terminal point
  {
    auto traj_out = traj;

    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(0, 9.0, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Not insert(Overlap base_idx point)
  for (double x_start = 0.0; x_start < total_length - 1.0 + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(0, x_start + 1e-4, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
  }

  // Not insert(Overlap base_idx + 1 point)
  for (double x_start = 1.0; x_start < total_length + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start - 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(0, x_start - 1e-4, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
  }

  // Invalid target point(In front of the beginning point)
  {
    auto traj_out = traj;

    const auto insert_idx = insertTargetPoint(0, -1.0, traj_out.points);

    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Behind the end point)
  {
    auto traj_out = traj;

    const auto insert_idx = insertTargetPoint(0, 10.0, traj_out.points);

    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory<Trajectory>(0, 1.0);
    EXPECT_THROW(insertTargetPoint(0, 0.0, empty_traj.points), std::invalid_argument);
  }
}

TEST(trajectory, insertTargetPoint_Length_Without_Target_Point_Non_Zero_Start_Idx)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Insert
  for (double x_start = 0.5; x_start < 5.0; x_start += 1.0) {
    auto traj_out = traj;

    const size_t start_idx = 2;
    const auto p_target = createPoint(x_start + 2.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Before End Point)
  {
    auto traj_out = traj;
    const double x_start = 1.0 - 1e-2;

    const size_t start_idx = 8;
    const auto p_target = createPoint(9.0 - 1e-2, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Right on End Point)
  {
    auto traj_out = traj;
    const double x_start = 1.0;

    const size_t start_idx = 8;
    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Insert on end point)
  {
    auto traj_out = traj;
    const double x_start = 4.0;

    const size_t start_idx = 5;
    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(start point)
  {
    auto traj_out = traj;
    const double x_start = 0.0;

    const size_t start_idx = 0;
    const auto p_target = createPoint(0.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Insert (Index Out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertTargetPoint(9, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(9, 1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(10, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(10, 1.0, traj_out.points), std::nullopt);
  }

  // No Insert (Length out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertTargetPoint(0, 10.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(0, 9.0001, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(5, 5.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(8, 1.0001, traj_out.points), std::nullopt);
  }
}

TEST(trajectory, insertTargetPoint_Negative_Length_Without_Target_Point_Non_Zero_Start_Idx)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Insert
  for (double x_start = -0.5; x_start < -5.0; x_start -= 1.0) {
    auto traj_out = traj;

    const size_t start_idx = 7;
    const auto p_target = createPoint(7.0 + x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Before End Point)
  {
    auto traj_out = traj;
    const double x_start = -1.0 - 1e-2;

    const size_t start_idx = 8;
    const auto p_target = createPoint(7.0 - 1e-2, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Right on End Point)
  {
    auto traj_out = traj;
    const double x_start = -1.0;

    const size_t start_idx = 8;
    const auto p_target = createPoint(7.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(start point)
  {
    auto traj_out = traj;
    const double x_start = -5.0;

    const size_t start_idx = 5;
    const auto p_target = createPoint(0.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Insert (Index Out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertTargetPoint(0, -1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(0, -1.0, traj_out.points), std::nullopt);
  }

  // No Insert (Length out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertTargetPoint(9, -10.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(9, -9.0001, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(1, -1.0001, traj_out.points), std::nullopt);
  }
}

TEST(trajectory, insertTargetPoint_Length_from_a_pose)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertTargetPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert (From Zero Point)
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const geometry_msgs::msg::Pose src_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert (From Non-Zero Point)
  {
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    for (double x_start = 0.5; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      }

      {
        const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_EQ(p_insert.position.x, p_target.x);
        EXPECT_EQ(p_insert.position.y, p_target.y);
        EXPECT_EQ(p_insert.position.z, p_target.z);
        EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
      }

      {
        const auto p_base = getPose(traj_out.points.at(base_idx));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
      }
    }
  }

  // No Insert
  {
    const double src_pose_x = 2.0;
    const double src_pose_y = 3.0;
    for (double x_start = 1e-3; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx);
      EXPECT_EQ(traj_out.points.size(), traj.points.size());

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      }
    }
  }

  // Insert (Boundary Condition)
  {
    const double src_pose_x = 2.0;
    const double src_pose_y = 3.0;
    for (double x_start = 1e-2; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      }

      {
        const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_EQ(p_insert.position.x, p_target.x);
        EXPECT_EQ(p_insert.position.y, p_target.y);
        EXPECT_EQ(p_insert.position.z, p_target.z);
        EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
      }

      {
        const auto p_base = getPose(traj_out.points.at(base_idx));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
      }
    }
  }

  // In Front of the beginning point of the trajectory
  {
    const double src_pose_x = -1.0;
    const double src_pose_y = 0.0;
    for (double x_start = 0.5 - src_pose_x; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      }

      {
        const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_EQ(p_insert.position.x, p_target.x);
        EXPECT_EQ(p_insert.position.y, p_target.y);
        EXPECT_EQ(p_insert.position.z, p_target.z);
        EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
      }

      {
        const auto p_base = getPose(traj_out.points.at(base_idx));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
      }
    }
  }

  // Insert from the point in front of the trajectory (Invalid)
  {
    auto traj_out = traj;
    const double src_pose_x = -1.0;
    const double src_pose_y = 0.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertTargetPoint(src_pose, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(src_pose, 0.5, traj_out.points), std::nullopt);
  }

  // Insert from the point behind the trajectory (Invalid)
  {
    auto traj_out = traj;
    const double src_pose_x = 10.0;
    const double src_pose_y = 3.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertTargetPoint(src_pose, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(src_pose, 1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(src_pose, 10.0, traj_out.points), std::nullopt);
  }

  // Insert from the point in front of the trajectory (Boundary Condition)
  {
    auto traj_out = traj;
    const double src_pose_x = -1.0;
    const double src_pose_y = 0.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    const double x_start = -src_pose_x;
    const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert from the end point (Boundary Condition)
  {
    auto traj_out = traj;
    const double src_pose_x = 9.0;
    const double src_pose_y = 0.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    const double x_start = 0.0;
    const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(src_pose, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Insert (Negative Insert Length)
  {
    auto traj_out = traj;
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertTargetPoint(src_pose, -1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertTargetPoint(src_pose, -10.0, traj_out.points), std::nullopt);
  }

  // No Insert (Too Far from the source point)
  {
    auto traj_out = traj;
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertTargetPoint(src_pose, 1.0, traj_out.points, 1.0), std::nullopt);
    EXPECT_EQ(insertTargetPoint(src_pose, 10.0, traj_out.points, 1.0), std::nullopt);
  }

  // No Insert (Too large yaw deviation)
  {
    auto traj_out = traj;
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    const double src_yaw = deg2rad(60.0);
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, src_yaw);
    const double max_dist = std::numeric_limits<double>::max();
    EXPECT_EQ(
      insertTargetPoint(src_pose, 1.0, traj_out.points, max_dist, deg2rad(45)), std::nullopt);
    EXPECT_EQ(
      insertTargetPoint(src_pose, 10.0, traj_out.points, max_dist, deg2rad(45)), std::nullopt);
  }
}

TEST(trajectory, insertStopPoint_from_a_source_index)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertStopPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 10.0);

  // Insert
  for (double x_start = 0.5; x_start < 5.0; x_start += 1.0) {
    auto traj_out = traj;

    const size_t start_idx = 2;
    const auto p_target = createPoint(x_start + 2.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Before End Point)
  {
    auto traj_out = traj;
    const double x_start = 1.0 - 1e-2;

    const size_t start_idx = 8;
    const auto p_target = createPoint(9.0 - 1e-2, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Right on End Point)
  {
    auto traj_out = traj;
    const double x_start = 1.0;

    const size_t start_idx = 8;
    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Insert on end point)
  {
    auto traj_out = traj;
    const double x_start = 4.0;

    const size_t start_idx = 5;
    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(start point)
  {
    auto traj_out = traj;
    const double x_start = 0.0;

    const size_t start_idx = 0;
    const auto p_target = createPoint(0.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(start_idx, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Insert (Index Out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertStopPoint(9, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(9, 1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(10, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(10, 1.0, traj_out.points), std::nullopt);
  }

  // No Insert (Length out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertStopPoint(0, 10.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(0, 9.0001, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(5, 5.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(8, 1.0001, traj_out.points), std::nullopt);
  }
}

TEST(trajectory, insertStopPoint_from_front_point)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertStopPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 10.0);

  // Insert
  for (double x_start = 0.5; x_start < 5.0; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 2.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(x_start + 2.0, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Before End Point)
  {
    auto traj_out = traj;
    const double x_start = 1.0 - 1e-2;

    const auto p_target = createPoint(9.0 - 1e-2, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(8.0 + x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(Right on End Point)
  {
    auto traj_out = traj;

    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(9.0, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary Condition(start point)
  {
    auto traj_out = traj;
    const double x_start = 0.0;

    const auto p_target = createPoint(0.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Insert (Length out of range)
  {
    auto traj_out = traj;
    EXPECT_EQ(insertStopPoint(9.0 + 1e-2, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(10.0, traj_out.points), std::nullopt);
    EXPECT_EQ(
      insertStopPoint(-std::numeric_limits<double>::epsilon(), traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(-3.0, traj_out.points), std::nullopt);
  }
}

TEST(trajectory, insertStopPoint_from_a_pose)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertStopPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 10.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert (From Zero Point)
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const geometry_msgs::msg::Pose src_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert (From Non-Zero Point)
  {
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    for (double x_start = 0.5; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
        } else {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
        }
      }

      {
        const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_EQ(p_insert.position.x, p_target.x);
        EXPECT_EQ(p_insert.position.y, p_target.y);
        EXPECT_EQ(p_insert.position.z, p_target.z);
        EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
      }

      {
        const auto p_base = getPose(traj_out.points.at(base_idx));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
      }
    }
  }

  // No Insert
  {
    const double src_pose_x = 2.0;
    const double src_pose_y = 3.0;
    for (double x_start = 1e-3; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx);
      EXPECT_EQ(traj_out.points.size(), traj.points.size());

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
        } else {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
        }
      }
    }
  }

  // Insert (Boundary Condition)
  {
    const double src_pose_x = 2.0;
    const double src_pose_y = 3.0;
    for (double x_start = 1e-2; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
        } else {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
        }
      }

      {
        const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_EQ(p_insert.position.x, p_target.x);
        EXPECT_EQ(p_insert.position.y, p_target.y);
        EXPECT_EQ(p_insert.position.z, p_target.z);
        EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
      }

      {
        const auto p_base = getPose(traj_out.points.at(base_idx));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
      }
    }
  }

  // In Front of the beginning point of the trajectory
  {
    const double src_pose_x = -1.0;
    const double src_pose_y = 0.0;
    for (double x_start = 0.5 - src_pose_x; x_start < total_length - src_pose_x; x_start += 1.0) {
      auto traj_out = traj;

      const geometry_msgs::msg::Pose src_pose =
        createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
      const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
        } else {
          EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
        }
      }

      {
        const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_EQ(p_insert.position.x, p_target.x);
        EXPECT_EQ(p_insert.position.y, p_target.y);
        EXPECT_EQ(p_insert.position.z, p_target.z);
        EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
      }

      {
        const auto p_base = getPose(traj_out.points.at(base_idx));
        const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
        EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
        EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
        EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
        EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
      }
    }
  }

  // Insert from the point in front of the trajectory (Invalid)
  {
    auto traj_out = traj;
    const double src_pose_x = -1.0;
    const double src_pose_y = 0.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertStopPoint(src_pose, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(src_pose, 0.5, traj_out.points), std::nullopt);
  }

  // Insert from the point behind the trajectory (Invalid)
  {
    auto traj_out = traj;
    const double src_pose_x = 10.0;
    const double src_pose_y = 3.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertStopPoint(src_pose, 0.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(src_pose, 1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(src_pose, 10.0, traj_out.points), std::nullopt);
  }

  // Insert from the point in front of the trajectory (Boundary Condition)
  {
    auto traj_out = traj;
    const double src_pose_x = -1.0;
    const double src_pose_y = 0.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    const double x_start = -src_pose_x;
    const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert from the end point (Boundary Condition)
  {
    auto traj_out = traj;
    const double src_pose_x = 9.0;
    const double src_pose_y = 0.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    const double x_start = 0.0;
    const auto p_target = createPoint(src_pose_x + x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(src_pose, x_start, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
      if (i < insert_idx.value()) {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 10.0, epsilon);
      } else {
        EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
      }
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // No Insert (Negative Insert Length)
  {
    auto traj_out = traj;
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertStopPoint(src_pose, -1.0, traj_out.points), std::nullopt);
    EXPECT_EQ(insertStopPoint(src_pose, -10.0, traj_out.points), std::nullopt);
  }

  // No Insert (Too Far from the source point)
  {
    auto traj_out = traj;
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(insertStopPoint(src_pose, 1.0, traj_out.points, 1.0), std::nullopt);
    EXPECT_EQ(insertStopPoint(src_pose, 10.0, traj_out.points, 1.0), std::nullopt);
  }

  // No Insert (Too large yaw deviation)
  {
    auto traj_out = traj;
    const double src_pose_x = 5.0;
    const double src_pose_y = 3.0;
    const double src_yaw = deg2rad(60.0);
    const geometry_msgs::msg::Pose src_pose =
      createPose(src_pose_x, src_pose_y, 0.0, 0.0, 0.0, src_yaw);
    const double max_dist = std::numeric_limits<double>::max();
    EXPECT_EQ(insertStopPoint(src_pose, 1.0, traj_out.points, max_dist, deg2rad(45)), std::nullopt);
    EXPECT_EQ(
      insertStopPoint(src_pose, 10.0, traj_out.points, max_dist, deg2rad(45)), std::nullopt);
  }
}

TEST(trajectory, insertStopPoint_with_pose_and_segment_index)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertStopPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::getPose;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 3.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    for (size_t i = 0; i < insert_idx.value(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 3.0, epsilon);
    }
    for (size_t i = insert_idx.value(); i < traj_out.points.size(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Boundary condition)
  for (double x_start = 0.0; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1.1e-3, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
    for (size_t i = 0; i < insert_idx.value(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 3.0, epsilon);
    }
    for (size_t i = insert_idx.value(); i < traj_out.points.size(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Insert(Quaternion interpolation)
  for (double x_start = 0.25; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.25 * std::tan(deg2rad(60.0)), 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
    for (size_t i = 0; i < insert_idx.value(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 3.0, epsilon);
    }
    for (size_t i = insert_idx.value(); i < traj_out.points.size(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.value()));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(-30.0));
      EXPECT_EQ(p_insert.position.x, p_target.x);
      EXPECT_EQ(p_insert.position.y, p_target.y);
      EXPECT_EQ(p_insert.position.z, p_target.z);
      EXPECT_NEAR(p_insert.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_insert.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_insert.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_insert.orientation.w, ans_quat.w, epsilon);
    }

    {
      const auto p_base = getPose(traj_out.points.at(base_idx));
      const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(60.0));
      EXPECT_NEAR(p_base.orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_base.orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_base.orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_base.orientation.w, ans_quat.w, epsilon);
    }
  }

  // Not insert(Overlap base_idx point)
  for (double x_start = 0.0; x_start < total_length - 1.0 + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start + 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
    for (size_t i = 0; i < insert_idx.value(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 3.0, epsilon);
    }
    for (size_t i = insert_idx.value(); i < traj_out.points.size(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
    }
  }

  // Not insert(Overlap base_idx + 1 point)
  for (double x_start = 1.0; x_start < total_length + epsilon; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start - 1e-4, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, std::nullopt);
    EXPECT_EQ(insert_idx.value(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }
    for (size_t i = 0; i < insert_idx.value(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 3.0, epsilon);
    }
    for (size_t i = insert_idx.value(); i < traj_out.points.size(); ++i) {
      EXPECT_NEAR(traj_out.points.at(i).longitudinal_velocity_mps, 0.0, epsilon);
    }
  }

  // Invalid target point(In front of begin point)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(-1.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle."), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Behind of end point)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle."), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid target point(Huge lateral offset)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(4.0, 10.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertStopPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(testing::internal::GetCapturedStderr().find("sharp angle."), std::string::npos);
    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Invalid base index
  {
    auto traj_out = traj;

    const size_t segment_idx = 9U;
    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const auto insert_idx = insertStopPoint(segment_idx, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, std::nullopt);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory<Trajectory>(0, 1.0);
    const size_t segment_idx = 0;
    EXPECT_FALSE(insertStopPoint(segment_idx, geometry_msgs::msg::Point{}, empty_traj.points));
  }
}

TEST(trajectory, insertDecelPoint_from_a_point)
{
  using motion_utils::calcArcLength;
  using motion_utils::findNearestSegmentIndex;
  using motion_utils::insertDecelPoint;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getLongitudinalVelocity;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 10.0);
  const auto total_length = calcArcLength(traj.points);
  const double decel_velocity = 5.0;

  // Insert (From Zero Point)
  {
    for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
      auto traj_out = traj;
      const geometry_msgs::msg::Point src_point = createPoint(0.0, 0.0, 0.0);
      const auto p_target = createPoint(x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertDecelPoint(src_point, x_start, decel_velocity, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(getLongitudinalVelocity(traj_out.points.at(i)), 10.0, epsilon);
        } else {
          EXPECT_NEAR(getLongitudinalVelocity(traj_out.points.at(i)), decel_velocity, epsilon);
        }
      }
    }
  }

  // Insert (From Non-Zero Point)
  {
    const double src_point_x = 5.0;
    const double src_point_y = 3.0;
    for (double x_start = 0.5; x_start < total_length - src_point_x; x_start += 1.0) {
      auto traj_out = traj;
      const geometry_msgs::msg::Point src_point = createPoint(src_point_x, src_point_y, 0.0);
      const auto p_target = createPoint(src_point_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertDecelPoint(src_point, x_start, decel_velocity, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx + 1);
      EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(getLongitudinalVelocity(traj_out.points.at(i)), 10.0, epsilon);
        } else {
          EXPECT_NEAR(getLongitudinalVelocity(traj_out.points.at(i)), decel_velocity, epsilon);
        }
      }
    }
  }

  // No Insert
  {
    const double src_point_x = 2.0;
    const double src_point_y = 3.0;
    for (double x_start = 1e-3; x_start < total_length - src_point_x; x_start += 1.0) {
      auto traj_out = traj;
      const geometry_msgs::msg::Point src_point = createPoint(src_point_x, src_point_y, 0.0);
      const auto p_target = createPoint(src_point_x + x_start, 0.0, 0.0);
      const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
      const auto insert_idx = insertDecelPoint(src_point, x_start, decel_velocity, traj_out.points);

      EXPECT_NE(insert_idx, std::nullopt);
      EXPECT_EQ(insert_idx.value(), base_idx);
      EXPECT_EQ(traj_out.points.size(), traj.points.size());

      for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
        EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
        if (i < insert_idx.value()) {
          EXPECT_NEAR(getLongitudinalVelocity(traj_out.points.at(i)), 10.0, epsilon);
        } else {
          EXPECT_NEAR(getLongitudinalVelocity(traj_out.points.at(i)), decel_velocity, epsilon);
        }
      }
    }
  }
}

TEST(trajectory, findFirstNearestIndexWithSoftConstraints)
{
  using motion_utils::findFirstNearestIndexWithSoftConstraints;
  using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
  using tier4_autoware_utils::pi;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Non overlapped points
  {
    // 1. Dist and yaw thresholds are given
    // Normal cases
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0, 0.4),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0, 0.4),
      2U);

    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8), 0.5, 1.0),
      4U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8), 0.5, 1.0),
      4U);

    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0), 1.0, 0.1),
      8U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0), 1.0, 0.1),
      8U);

    // Dist is out of range
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.4),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.4),
      2U);

    // Yaw is out of range
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0, 0.2),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0, 0.2),
      2U);

    // Dist and yaw is out of range
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.2),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.2),
      2U);

    // Empty points
    EXPECT_THROW(
      findFirstNearestIndexWithSoftConstraints(
        Trajectory{}.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.2),
      std::invalid_argument);
    EXPECT_THROW(
      findFirstNearestSegmentIndexWithSoftConstraints(
        Trajectory{}.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.2),
      std::invalid_argument);

    // 2. Dist threshold is given
    // Normal cases
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 2.0),
      2U);

    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8), 0.5),
      4U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8), 0.5),
      4U);

    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0), 1.0),
      8U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0), 1.0),
      8U);

    // Dist is out of range
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3), 1.0),
      2U);

    // 3. No threshold is given
    // Normal cases
    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3)),
      2U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(2.4, 1.3, 0.0, 0.0, 0.0, 0.3)),
      2U);

    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8)),
      4U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(4.1, 0.3, 0.0, 0.0, 0.0, -0.8)),
      4U);

    EXPECT_EQ(
      findFirstNearestIndexWithSoftConstraints(
        traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0)),
      8U);
    EXPECT_EQ(
      findFirstNearestSegmentIndexWithSoftConstraints(
        traj.points, createPose(8.5, -0.5, 0.0, 0.0, 0.0, 0.0)),
      8U);
  }

  // Vertically crossing points
  {
    //       ___
    //      |  |
    //   S__|__|
    //      |
    //      |
    //      G
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.push_back(createPose(-2.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(2.0, 0.0, 0.0, 0.0, 0.0, pi / 2.0));
    poses.push_back(createPose(2.0, 1.0, 0.0, 0.0, 0.0, pi / 2.0));
    poses.push_back(createPose(2.0, 2.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(1.0, 2.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(0.0, 2.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(0.0, 1.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(0.0, 0.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(0.0, -1.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(0.0, -2.0, 0.0, 0.0, 0.0, -pi / 2.0));

    // 1. Dist and yaw thresholds are given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 1.0, 0.4),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 1.0, 0.4),
        2U);

      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, -pi / 2.0), 1.0, 0.4),
        10U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, -pi / 2.0), 1.0, 0.4),
        9U);

      // Several nearest index within threshold
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 10.0, pi * 2.0),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 10.0, pi * 2.0),
        2U);

      // Dist is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 0.0, 0.4),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 0.0, 0.4),
        2U);

      // Yaw is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.0),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.3), 1.0, 0.0),
        2U);

      // Dist and yaw is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.3), 0.0, 0.0),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.3), 0.0, 0.0),
        2U);
    }

    // 2. Dist threshold is given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 1.0),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 1.0),
        2U);

      // Several nearest index within threshold
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 10.0),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 10.0),
        2U);

      // Dist is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 0.0),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0), 0.0),
        2U);
    }

    // 3. No threshold is given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0)),
        2U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(0.3, 0.3, 0.0, 0.0, 0.0, 0.0)),
        2U);
    }
  }

  {
    // Points has a loop structure with the opposite direction (= u-turn)
    //         __
    // S/G ___|_|

    std::vector<geometry_msgs::msg::Pose> poses;
    poses.push_back(createPose(-3.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(-2.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(1.0, 0.0, 0.0, 0.0, 0.0, pi / 2.0));
    poses.push_back(createPose(1.0, 1.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(0.0, 1.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(0.0, 0.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(-1.0, 0.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(-2.0, 0.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(-3.0, 0.0, 0.0, 0.0, 0.0, pi));

    // 1. Dist and yaw thresholds are given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0, 0.4),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0, 0.4),
        0U);

      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi), 1.0, 0.4),
        9U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi), 1.0, 0.4),
        9U);

      // Several nearest index within threshold
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi), 10.0, pi * 2.0),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi), 10.0, pi * 2.0),
        0U);

      // Dist is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi), 0.0, 0.4),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi), 0.0, 0.4),
        0U);

      // Yaw is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi * 0.9), 1.0, 0.0),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi * 0.9), 1.0, 0.0),
        0U);

      // Dist and yaw is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi * 0.9), 0.0, 0.0),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, pi * 0.9), 0.0, 0.0),
        0U);
    }

    // 2. Dist threshold is given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0),
        0U);

      // Several nearest index within threshold
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 10.0),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 10.0),
        0U);

      // Dist is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0),
        0U);
    }

    // 3. No threshold is given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0)),
        1U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(-2.1, 0.1, 0.0, 0.0, 0.0, 0.0)),
        0U);
    }
  }

  {  // Points has a loop structure with the same direction
     //      ___
     //     |  |
     //  S__|__|__G
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.push_back(createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(4.0, 0.0, 0.0, 0.0, 0.0, pi / 2.0));
    poses.push_back(createPose(4.0, 1.0, 0.0, 0.0, 0.0, pi / 2.0));
    poses.push_back(createPose(4.0, 2.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(3.0, 2.0, 0.0, 0.0, 0.0, pi));
    poses.push_back(createPose(2.0, 2.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(2.0, 1.0, 0.0, 0.0, 0.0, -pi / 2.0));
    poses.push_back(createPose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(3.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(4.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0));
    poses.push_back(createPose(6.0, 0.0, 0.0, 0.0, 0.0, 0.0));

    // 1. Dist and yaw thresholds are given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0, 0.4),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0, 0.4),
        3U);

      // Several nearest index within threshold
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 10.0, pi * 2.0),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 10.0, pi * 2.0),
        3U);

      // Dist is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0, 0.4),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0, 0.4),
        3U);

      // Yaw is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0, 0.0),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0, 0.0),
        3U);

      // Dist and yaw is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0, 0.0),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0, 0.0),
        3U);
    }

    // 2. Dist threshold is given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 1.0),
        3U);

      // Several nearest index within threshold
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 10.0),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 10.0),
        3U);

      // Dist is out of range
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0), 0.0),
        3U);
    }

    // 3. No threshold is given
    {
      // Normal cases
      EXPECT_EQ(
        findFirstNearestIndexWithSoftConstraints(poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0)),
        3U);
      EXPECT_EQ(
        findFirstNearestSegmentIndexWithSoftConstraints(
          poses, createPose(3.1, 0.1, 0.0, 0.0, 0.0, 0.0)),
        3U);
    }
  }
}

TEST(trajectory, calcSignedArcLengthFromPointAndSegmentIndexToPointAndSegmentIndex)
{
  using motion_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcSignedArcLength(Trajectory{}.points, {}, {}), 0.0);

  // Same point
  {
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p, 2, p, 2), 0, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p, 3, p, 3), 0, epsilon);
  }

  // Forward
  {
    const auto p1 = createPoint(0.0, 0.0, 0.0);
    const auto p2 = createPoint(3.0, 1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, p2, 2), 3, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, p2, 3), 3, epsilon);
  }

  // Backward
  {
    const auto p1 = createPoint(9.0, 0.0, 0.0);
    const auto p2 = createPoint(8.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 8, p2, 7), -1, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 8, p2, 8), -1, epsilon);
  }

  // Point before start point
  {
    const auto p1 = createPoint(-3.9, 3.0, 0.0);
    const auto p2 = createPoint(6.0, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, p2, 5), 9.9, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, p2, 6), 9.9, epsilon);
  }

  // Point after end point
  {
    const auto p1 = createPoint(7.0, -5.0, 0.0);
    const auto p2 = createPoint(13.3, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 6, p2, 8), 6.3, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 7, p2, 8), 6.3, epsilon);
  }

  // Point before start point and after end point
  {
    const auto p1 = createPoint(-4.3, 10.0, 0.0);
    const auto p2 = createPoint(13.8, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, p2, 8), 18.1, epsilon);
  }

  // Random cases
  {
    const auto p1 = createPoint(1.0, 3.0, 0.0);
    const auto p2 = createPoint(9.0, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, p2, 8), 8, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 1, p2, 8), 8, epsilon);
  }
  {
    const auto p1 = createPoint(4.3, 7.0, 0.0);
    const auto p2 = createPoint(2.0, 3.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 4, p2, 2), -2.3, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 4, p2, 1), -2.3, epsilon);
  }
}

TEST(trajectory, calcSignedArcLengthFromPointAndSegmentIndexToPointIndex)
{
  using motion_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_DOUBLE_EQ(calcSignedArcLength(Trajectory{}.points, {}, {}), 0.0);

  // Same point
  {
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p, 2, 3), 0, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 3, p, 3), 0, epsilon);
  }

  // Forward
  {
    const auto p1 = createPoint(0.0, 0.0, 0.0);
    const auto p2 = createPoint(3.0, 1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, 3), 3, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 0, p2, 2), 3, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 0, p2, 3), 3, epsilon);
  }

  // Backward
  {
    const auto p1 = createPoint(9.0, 0.0, 0.0);
    const auto p2 = createPoint(8.0, 0.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 8, 8), -1, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 8, p2, 7), 0, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 8, p2, 8), 0, epsilon);
  }

  // Point before start point
  {
    const auto p1 = createPoint(-3.9, 3.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, 6), 9.9, epsilon);
  }

  // Point after end point
  {
    const auto p2 = createPoint(13.3, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 7, p2, 8), 6.3, epsilon);
  }

  // Start point
  {
    const auto p1 = createPoint(0.0, 3.0, 0.0);
    const auto p2 = createPoint(5.3, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, 5), 5, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 0, p2, 5), 5.3, epsilon);
  }

  // Point after end point
  {
    const auto p1 = createPoint(7.3, -5.0, 0.0);
    const auto p2 = createPoint(9.0, -10.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 7, 9), 1.7, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 7, p2, 8), 2.0, epsilon);
  }

  // Random cases
  {
    const auto p1 = createPoint(1.0, 3.0, 0.0);
    const auto p2 = createPoint(9.0, -1.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 0, 9), 8, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 1, 9), 8, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 1, p2, 8), 8, epsilon);
  }
  {
    const auto p1 = createPoint(4.3, 7.0, 0.0);
    const auto p2 = createPoint(2.3, 3.0, 0.0);
    EXPECT_NEAR(calcSignedArcLength(traj.points, p1, 4, 2), -2.3, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 4, p2, 2), -1.7, epsilon);
    EXPECT_NEAR(calcSignedArcLength(traj.points, 4, p2, 1), -1.7, epsilon);
  }
}

TEST(trajectory, removeOverlapPoints)
{
  using motion_utils::removeOverlapPoints;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
  const auto removed_traj = removeOverlapPoints(traj.points, 0);
  EXPECT_EQ(traj.points.size(), removed_traj.size());
  for (size_t i = 0; i < traj.points.size(); ++i) {
    EXPECT_NEAR(traj.points.at(i).pose.position.x, removed_traj.at(i).pose.position.x, epsilon);
    EXPECT_NEAR(traj.points.at(i).pose.position.y, removed_traj.at(i).pose.position.y, epsilon);
    EXPECT_NEAR(traj.points.at(i).pose.position.z, removed_traj.at(i).pose.position.z, epsilon);
    EXPECT_NEAR(
      traj.points.at(i).pose.orientation.x, removed_traj.at(i).pose.orientation.x, epsilon);
    EXPECT_NEAR(
      traj.points.at(i).pose.orientation.y, removed_traj.at(i).pose.orientation.y, epsilon);
    EXPECT_NEAR(
      traj.points.at(i).pose.orientation.z, removed_traj.at(i).pose.orientation.z, epsilon);
    EXPECT_NEAR(
      traj.points.at(i).pose.orientation.w, removed_traj.at(i).pose.orientation.w, epsilon);
    EXPECT_NEAR(
      traj.points.at(i).longitudinal_velocity_mps, removed_traj.at(i).longitudinal_velocity_mps,
      epsilon);
  }

  // No overlap points
  {
    const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    for (size_t start_idx = 0; start_idx < 10; ++start_idx) {
      const auto removed_traj = removeOverlapPoints(traj.points, start_idx);
      EXPECT_EQ(traj.points.size(), removed_traj.size());
      for (size_t i = 0; i < traj.points.size(); ++i) {
        EXPECT_NEAR(traj.points.at(i).pose.position.x, removed_traj.at(i).pose.position.x, epsilon);
        EXPECT_NEAR(traj.points.at(i).pose.position.y, removed_traj.at(i).pose.position.y, epsilon);
        EXPECT_NEAR(traj.points.at(i).pose.position.z, removed_traj.at(i).pose.position.z, epsilon);
        EXPECT_NEAR(
          traj.points.at(i).pose.orientation.x, removed_traj.at(i).pose.orientation.x, epsilon);
        EXPECT_NEAR(
          traj.points.at(i).pose.orientation.y, removed_traj.at(i).pose.orientation.y, epsilon);
        EXPECT_NEAR(
          traj.points.at(i).pose.orientation.z, removed_traj.at(i).pose.orientation.z, epsilon);
        EXPECT_NEAR(
          traj.points.at(i).pose.orientation.w, removed_traj.at(i).pose.orientation.w, epsilon);
        EXPECT_NEAR(
          traj.points.at(i).longitudinal_velocity_mps, removed_traj.at(i).longitudinal_velocity_mps,
          epsilon);
      }
    }
  }

  // Overlap points from certain point
  {
    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    traj.points.at(5) = traj.points.at(6);
    const auto removed_traj = removeOverlapPoints(traj.points);

    EXPECT_EQ(traj.points.size() - 1, removed_traj.size());
    for (size_t i = 0; i < 6; ++i) {
      EXPECT_NEAR(traj.points.at(i).pose.position.x, removed_traj.at(i).pose.position.x, epsilon);
      EXPECT_NEAR(traj.points.at(i).pose.position.y, removed_traj.at(i).pose.position.y, epsilon);
      EXPECT_NEAR(traj.points.at(i).pose.position.z, removed_traj.at(i).pose.position.z, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.x, removed_traj.at(i).pose.orientation.x, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.y, removed_traj.at(i).pose.orientation.y, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.z, removed_traj.at(i).pose.orientation.z, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.w, removed_traj.at(i).pose.orientation.w, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).longitudinal_velocity_mps, removed_traj.at(i).longitudinal_velocity_mps,
        epsilon);
    }

    for (size_t i = 6; i < 9; ++i) {
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.position.x, removed_traj.at(i).pose.position.x, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.position.y, removed_traj.at(i).pose.position.y, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.position.z, removed_traj.at(i).pose.position.z, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.orientation.x, removed_traj.at(i).pose.orientation.x, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.orientation.y, removed_traj.at(i).pose.orientation.y, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.orientation.z, removed_traj.at(i).pose.orientation.z, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).pose.orientation.w, removed_traj.at(i).pose.orientation.w, epsilon);
      EXPECT_NEAR(
        traj.points.at(i + 1).longitudinal_velocity_mps,
        removed_traj.at(i).longitudinal_velocity_mps, epsilon);
    }
  }

  // Overlap points from certain point
  {
    auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);
    traj.points.at(5) = traj.points.at(6);
    const auto removed_traj = removeOverlapPoints(traj.points, 6);

    EXPECT_EQ(traj.points.size(), removed_traj.size());
    for (size_t i = 0; i < traj.points.size(); ++i) {
      EXPECT_NEAR(traj.points.at(i).pose.position.x, removed_traj.at(i).pose.position.x, epsilon);
      EXPECT_NEAR(traj.points.at(i).pose.position.y, removed_traj.at(i).pose.position.y, epsilon);
      EXPECT_NEAR(traj.points.at(i).pose.position.z, removed_traj.at(i).pose.position.z, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.x, removed_traj.at(i).pose.orientation.x, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.y, removed_traj.at(i).pose.orientation.y, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.z, removed_traj.at(i).pose.orientation.z, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).pose.orientation.w, removed_traj.at(i).pose.orientation.w, epsilon);
      EXPECT_NEAR(
        traj.points.at(i).longitudinal_velocity_mps, removed_traj.at(i).longitudinal_velocity_mps,
        epsilon);
    }
  }

  // Empty Points
  {
    const Trajectory traj;
    const auto removed_traj = removeOverlapPoints(traj.points);
    EXPECT_TRUE(removed_traj.empty());
  }
}

TEST(trajectory, cropForwardPoints)
{
  using motion_utils::cropForwardPoints;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);

  {  // Normal case
    const auto cropped_traj_points =
      cropForwardPoints(traj.points, tier4_autoware_utils::createPoint(1.5, 1.5, 0.0), 1, 4.8);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(7));
  }

  {  // Forward length is longer than points arc length.
    const auto cropped_traj_points =
      cropForwardPoints(traj.points, tier4_autoware_utils::createPoint(1.5, 1.5, 0.0), 1, 10.0);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(10));
  }

  {  // Point is on the previous segment
    const auto cropped_traj_points =
      cropForwardPoints(traj.points, tier4_autoware_utils::createPoint(1.0, 1.0, 0.0), 0, 2.5);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(4));
  }

  {  // Point is on the next segment
    const auto cropped_traj_points =
      cropForwardPoints(traj.points, tier4_autoware_utils::createPoint(1.0, 1.0, 0.0), 1, 2.5);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(4));
  }
}

TEST(trajectory, cropBackwardPoints)
{
  using motion_utils::cropBackwardPoints;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);

  {  // Normal case
    const auto cropped_traj_points =
      cropBackwardPoints(traj.points, tier4_autoware_utils::createPoint(8.5, 8.5, 0.0), 8, 4.8);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(6));
  }

  {  // Backward length is longer than points arc length.
    const auto cropped_traj_points =
      cropBackwardPoints(traj.points, tier4_autoware_utils::createPoint(8.5, 8.5, 0.0), 8, 10.0);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(10));
  }

  {  // Point is on the previous segment
    const auto cropped_traj_points =
      cropBackwardPoints(traj.points, tier4_autoware_utils::createPoint(8.0, 8.0, 0.0), 7, 2.5);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(4));
  }

  {  // Point is on the next segment
    const auto cropped_traj_points =
      cropBackwardPoints(traj.points, tier4_autoware_utils::createPoint(8.0, 8.0, 0.0), 8, 2.5);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(4));
  }
}

TEST(trajectory, cropPoints)
{
  using motion_utils::cropPoints;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);

  {  // Normal case
    const auto cropped_traj_points =
      cropPoints(traj.points, tier4_autoware_utils::createPoint(3.5, 3.5, 0.0), 3, 2.3, 1.2);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(3));
  }

  {  // Length is longer than points arc length.
    const auto cropped_traj_points =
      cropPoints(traj.points, tier4_autoware_utils::createPoint(3.5, 3.5, 0.0), 3, 10.0, 10.0);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(10));
  }

  {  // Point is on the previous segment
    const auto cropped_traj_points =
      cropPoints(traj.points, tier4_autoware_utils::createPoint(3.0, 3.0, 0.0), 2, 2.2, 1.9);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(4));
  }

  {  // Point is on the next segment
    const auto cropped_traj_points =
      cropPoints(traj.points, tier4_autoware_utils::createPoint(3.0, 3.0, 0.0), 3, 2.2, 1.9);
    EXPECT_EQ(cropped_traj_points.size(), static_cast<size_t>(4));
  }
}

TEST(Trajectory, removeInvalidOrientationPoints)
{
  using motion_utils::insertOrientation;
  using motion_utils::removeInvalidOrientationPoints;

  const double max_yaw_diff = M_PI_2;

  auto testRemoveInvalidOrientationPoints = [&](
                                              const Trajectory & traj,
                                              std::function<void(Trajectory &)> modifyTrajectory,
                                              size_t expected_size) {
    auto modified_traj = traj;
    insertOrientation(modified_traj.points, true);
    modifyTrajectory(modified_traj);
    removeInvalidOrientationPoints(modified_traj.points, max_yaw_diff);
    EXPECT_EQ(modified_traj.points.size(), expected_size);
    for (size_t i = 0; i < modified_traj.points.size() - 1; ++i) {
      EXPECT_EQ(traj.points.at(i), modified_traj.points.at(i));
      const double yaw1 = tf2::getYaw(modified_traj.points.at(i).pose.orientation);
      const double yaw2 = tf2::getYaw(modified_traj.points.at(i + 1).pose.orientation);
      const double yaw_diff = std::abs(tier4_autoware_utils::normalizeRadian(yaw1 - yaw2));
      EXPECT_LE(yaw_diff, max_yaw_diff);
    }
  };

  auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 1.0);

  // no invalid points
  testRemoveInvalidOrientationPoints(
    traj, [](Trajectory &) {}, traj.points.size());

  // invalid point at the end
  testRemoveInvalidOrientationPoints(
    traj,
    [&](Trajectory & t) {
      auto invalid_point = t.points.back();
      invalid_point.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 3 * M_PI_2));
      t.points.push_back(invalid_point);
    },
    traj.points.size());

  // invalid point in the middle
  testRemoveInvalidOrientationPoints(
    traj,
    [&](Trajectory & t) {
      auto invalid_point = t.points[4];
      invalid_point.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 3 * M_PI_2));
      t.points.insert(t.points.begin() + 4, invalid_point);
    },
    traj.points.size());

  // invalid point at the beginning
  testRemoveInvalidOrientationPoints(
    traj,
    [&](Trajectory & t) {
      auto invalid_point = t.points.front();
      invalid_point.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), 3 * M_PI_2));
      t.points.insert(t.points.begin(), invalid_point);
    },
    1);  // expected size is 1 since only the first point remains
}

TEST(trajectory, calcYawDeviation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::calcYawDeviation;

  constexpr double tolerance = 1e-3;

  // Generate test trajectory
  const auto trajectory = generateTestTrajectory<Trajectory>(4, 10.0, 0.0, 0.0, M_PI / 8);

  // check with fist point
  {
    const auto input_pose = createPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const double yaw_deviation = calcYawDeviation(trajectory.points, input_pose);
    constexpr double expected_yaw_deviation = -M_PI / 8;
    EXPECT_NEAR(yaw_deviation, expected_yaw_deviation, tolerance);
  }

  // check with middle point
  {
    const auto input_pose = createPose(10.0, 10.0, 0.0, 0.0, 0.0, M_PI / 8);
    const double yaw_deviation = calcYawDeviation(trajectory.points, input_pose);
    constexpr double expected_yaw_deviation = -0.734;
    EXPECT_NEAR(yaw_deviation, expected_yaw_deviation, tolerance);
  }
}

TEST(trajectory, isTargetPointFront)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using motion_utils::isTargetPointFront;
  using tier4_autoware_utils::createPoint;

  // Generate test trajectory
  const auto trajectory = generateTestTrajectory<Trajectory>(10, 1.0);

  // Front point is base
  {
    const auto base_point = createPoint(5.0, 0.0, 0.0);
    const auto target_point = createPoint(1.0, 0.0, 0.0);

    EXPECT_FALSE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  // Front point is target
  {
    const auto base_point = createPoint(1.0, 0.0, 0.0);
    const auto target_point = createPoint(3.0, 0.0, 0.0);

    EXPECT_TRUE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  // boundary condition
  {
    const auto base_point = createPoint(1.0, 0.0, 0.0);
    const auto target_point = createPoint(1.0, 0.0, 0.0);

    EXPECT_FALSE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  // before the start point
  {
    const auto base_point = createPoint(1.0, 0.0, 0.0);
    const auto target_point = createPoint(-3.0, 0.0, 0.0);

    EXPECT_FALSE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  {
    const auto base_point = createPoint(-5.0, 0.0, 0.0);
    const auto target_point = createPoint(1.0, 0.0, 0.0);

    EXPECT_TRUE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  // after the end point
  {
    const auto base_point = createPoint(10.0, 0.0, 0.0);
    const auto target_point = createPoint(3.0, 0.0, 0.0);

    EXPECT_FALSE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  {
    const auto base_point = createPoint(2.0, 0.0, 0.0);
    const auto target_point = createPoint(14.0, 0.0, 0.0);

    EXPECT_TRUE(isTargetPointFront(trajectory.points, base_point, target_point));
  }

  // empty point
  {
    const Trajectory traj;
    const auto base_point = createPoint(2.0, 0.0, 0.0);
    const auto target_point = createPoint(5.0, 0.0, 0.0);
    EXPECT_FALSE(isTargetPointFront(traj.points, base_point, target_point));
  }

  // non-default threshold
  {
    const double threshold = 3.0;

    {
      const auto base_point = createPoint(5.0, 0.0, 0.0);
      const auto target_point = createPoint(3.0, 0.0, 0.0);

      EXPECT_FALSE(isTargetPointFront(trajectory.points, base_point, target_point, threshold));
    }

    {
      const auto base_point = createPoint(1.0, 0.0, 0.0);
      const auto target_point = createPoint(4.0, 0.0, 0.0);

      EXPECT_FALSE(isTargetPointFront(trajectory.points, base_point, target_point, threshold));
    }

    {
      const auto base_point = createPoint(1.0, 0.0, 0.0);
      const auto target_point = createPoint(4.1, 0.0, 0.0);

      EXPECT_TRUE(isTargetPointFront(trajectory.points, base_point, target_point, threshold));
    }
  }
}
