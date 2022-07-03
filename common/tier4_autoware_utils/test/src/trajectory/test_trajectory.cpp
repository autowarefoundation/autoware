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

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"
#include "tier4_autoware_utils/trajectory/tmp_conversion.hpp"
#include "tier4_autoware_utils/trajectory/trajectory.hpp"

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
  using tier4_autoware_utils::validateNonEmpty;

  // Empty
  EXPECT_THROW(validateNonEmpty(Trajectory{}.points), std::invalid_argument);

  // Non-empty
  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  EXPECT_NO_THROW(validateNonEmpty(traj.points));
}

TEST(trajectory, validateNonSharpAngle_DefaultThreshold)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using tier4_autoware_utils::validateNonSharpAngle;

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
  using tier4_autoware_utils::pi;
  using tier4_autoware_utils::validateNonSharpAngle;

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
  using tier4_autoware_utils::searchZeroVelocityIndex;

  // Empty
  EXPECT_THROW(searchZeroVelocityIndex(Trajectory{}.points), std::invalid_argument);

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

TEST(trajectory, findNearestIndex_Pos_StraightTrajectory)
{
  using tier4_autoware_utils::findNearestIndex;

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
  using tier4_autoware_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_EQ(findNearestIndex(traj.points, createPoint(5.1, 3.4, 0.0)), 6U);
}

TEST(trajectory, findNearestIndex_Pose_NoThreshold)
{
  using tier4_autoware_utils::findNearestIndex;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    findNearestIndex(Trajectory{}.points, geometry_msgs::msg::Pose{}, {}), std::invalid_argument);

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
  using tier4_autoware_utils::findNearestIndex;

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
  using tier4_autoware_utils::findNearestIndex;

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
  using tier4_autoware_utils::findNearestIndex;

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
  using tier4_autoware_utils::findNearestSegmentIndex;

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
  using tier4_autoware_utils::calcLongitudinalOffsetToSegment;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(Trajectory{}.points, {}, geometry_msgs::msg::Point{}),
    std::invalid_argument);

  // Out of range
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(traj.points, -1, geometry_msgs::msg::Point{}),
    std::out_of_range);
  EXPECT_THROW(
    calcLongitudinalOffsetToSegment(
      traj.points, traj.points.size() - 1, geometry_msgs::msg::Point{}),
    std::out_of_range);

  // Same close points in trajectory
  {
    const auto invalid_traj = generateTestTrajectory<Trajectory>(10, 0.0);
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_THROW(calcLongitudinalOffsetToSegment(invalid_traj.points, 3, p), std::runtime_error);
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
  using tier4_autoware_utils::calcLongitudinalOffsetToSegment;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_NEAR(
    calcLongitudinalOffsetToSegment(traj.points, 2, createPoint(2.0, 0.5, 0.0)), 0.083861449,
    epsilon);
}

TEST(trajectory, calcLateralOffset)
{
  using tier4_autoware_utils::calcLateralOffset;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(
    calcLateralOffset(Trajectory{}.points, geometry_msgs::msg::Point{}), std::invalid_argument);

  // Trajectory size is 1
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_THROW(
      calcLateralOffset(one_point_traj.points, geometry_msgs::msg::Point{}), std::out_of_range);
  }

  // Same close points in trajectory
  {
    const auto invalid_traj = generateTestTrajectory<Trajectory>(10, 0.0);
    const auto p = createPoint(3.0, 0.0, 0.0);
    EXPECT_THROW(calcLateralOffset(invalid_traj.points, p), std::runtime_error);
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

TEST(trajectory, calcLateralOffset_CurveTrajectory)
{
  using tier4_autoware_utils::calcLateralOffset;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0, 0.0, 0.0, 0.1);

  // Random cases
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(2.0, 0.5, 0.0)), 0.071386083, epsilon);
  EXPECT_NEAR(calcLateralOffset(traj.points, createPoint(5.0, 1.0, 0.0)), -1.366602819, epsilon);
}

TEST(trajectory, calcSignedArcLengthFromIndexToIndex)
{
  using tier4_autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

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
  using tier4_autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

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

TEST(trajectory, calcSignedArcLengthFromPoseToIndex_DistThreshold)
{
  using tier4_autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Out of threshold
  EXPECT_FALSE(calcSignedArcLength(traj.points, createPose(0.0, 0.6, 0.0, 0.0, 0.0, 0.0), 3, 0.5));

  // On threshold
  EXPECT_NEAR(
    *calcSignedArcLength(traj.points, createPose(0.0, 0.5, 0.0, 0.0, 0.0, 0.0), 3, 0.5), 3.0,
    epsilon);

  // Within threshold
  EXPECT_NEAR(
    *calcSignedArcLength(traj.points, createPose(0.0, 0.4, 0.0, 0.0, 0.0, 0.0), 3, 0.5), 3.0,
    epsilon);
}

TEST(trajectory, calcSignedArcLengthFromPoseToIndex_YawThreshold)
{
  using tier4_autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto max_d = std::numeric_limits<double>::max();

  // Out of threshold
  EXPECT_FALSE(
    calcSignedArcLength(traj.points, createPose(0.0, 0.5, 0.0, 0.0, 0.0, 1.1), 3, max_d, 1.0));

  // On threshold
  EXPECT_NEAR(
    *calcSignedArcLength(traj.points, createPose(0.0, 0.5, 0.0, 0.0, 0.0, 1.0), 3, max_d, 1.0), 3.0,
    epsilon);

  // Within threshold
  EXPECT_NEAR(
    *calcSignedArcLength(traj.points, createPose(0.0, 0.5, 0.0, 0.0, 0.0, 0.9), 3, max_d, 1.0), 3.0,
    epsilon);
}

TEST(trajectory, calcSignedArcLengthFromIndexToPoint)
{
  using tier4_autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

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
  using tier4_autoware_utils::calcSignedArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcSignedArcLength(Trajectory{}.points, {}, {}), std::invalid_argument);

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
  using tier4_autoware_utils::calcArcLength;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);

  // Empty
  EXPECT_THROW(calcArcLength(Trajectory{}.points), std::invalid_argument);

  // Whole Length
  EXPECT_NEAR(calcArcLength(traj.points), 9.0, epsilon);
}

TEST(trajectory, convertToTrajectory)
{
  using tier4_autoware_utils::convertToTrajectory;

  // Size check
  {
    const auto traj_input = generateTestTrajectoryPointArray(50, 1.0);
    const auto traj = convertToTrajectory(traj_input);
    EXPECT_EQ(traj.points.size(), traj_input.size());
  }

  // Clipping check
  {
    const auto traj_input = generateTestTrajectoryPointArray(10000, 1.0);
    const auto traj = convertToTrajectory(traj_input);
    EXPECT_EQ(traj.points.size(), traj.CAPACITY);
    // Value check
    for (size_t i = 0; i < traj.points.size(); ++i) {
      EXPECT_EQ(traj.points.at(i), traj_input.at(i));
    }
  }
}

TEST(trajectory, convertToTrajectoryPointArray)
{
  using tier4_autoware_utils::convertToTrajectoryPointArray;

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
  using tier4_autoware_utils::calcDistanceToForwardStopPoint;

  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Empty
  {
    EXPECT_THROW(calcDistanceToForwardStopPoint(Trajectory{}.points), std::invalid_argument);
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
    EXPECT_NEAR(dist.get(), 50.0, epsilon);
  }

  // Boundary2 (Edge of the input trajectory)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 99);
    EXPECT_FALSE(dist);
  }

  // Boundary3 (On the Stop Point)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 50);
    EXPECT_NEAR(dist.get(), 0.0, epsilon);
  }

  // Boundary4 (Right before the stop point)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 49);
    EXPECT_NEAR(dist.get(), 1.0, epsilon);
  }

  // Boundary5 (Right behind the stop point)
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 51);
    EXPECT_FALSE(dist);
  }

  // Random
  {
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, 20);
    EXPECT_NEAR(dist.get(), 30.0, epsilon);
  }
}

TEST(trajectory, calcDistanceToForwardStopPointFromPose)
{
  using tier4_autoware_utils::calcDistanceToForwardStopPoint;

  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Empty
  {
    EXPECT_THROW(
      calcDistanceToForwardStopPoint(Trajectory{}.points, geometry_msgs::msg::Pose{}),
      std::invalid_argument);
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
    EXPECT_NEAR(dist.get(), 50.0, epsilon);
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
    EXPECT_NEAR(dist.get(), 60.0, epsilon);
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
    EXPECT_NEAR(dist.get(), 80.0, epsilon);
  }

  // Boundary Condition1
  {
    const auto pose = createPose(50.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.get(), 0.0, epsilon);
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
    EXPECT_NEAR(dist.get(), 0.1, epsilon);
  }

  // Random
  {
    const auto pose = createPose(3.0, 2.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose);
    EXPECT_NEAR(dist.get(), 47.0, epsilon);
  }
}

TEST(trajectory, calcDistanceToForwardStopPoint_DistThreshold)
{
  using tier4_autoware_utils::calcDistanceToForwardStopPoint;

  auto traj_input = generateTestTrajectory<Trajectory>(100, 1.0, 3.0);
  traj_input.points.at(50).longitudinal_velocity_mps = 0.0;

  // Boundary Condition1
  {
    const auto pose = createPose(-4.9, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
    EXPECT_NEAR(dist.get(), 54.9, epsilon);
  }

  // Boundary Condition2
  {
    const auto pose = createPose(-5.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    const auto dist = calcDistanceToForwardStopPoint(traj_input.points, pose, 5.0);
    EXPECT_NEAR(dist.get(), 55.0, epsilon);
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
      EXPECT_NEAR(dist.get(), 47.0, epsilon);
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
  using tier4_autoware_utils::calcDistanceToForwardStopPoint;
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
      EXPECT_NEAR(dist.get(), 48.0, epsilon);
    }

    {
      const auto pose = createPose(x, 0.0, 0.0, 0.0, 0.0, deg2rad(30.0));
      const auto dist =
        calcDistanceToForwardStopPoint(traj_input.points, pose, max_d, deg2rad(30.0));
      EXPECT_NEAR(dist.get(), 48.0, epsilon);
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
      EXPECT_NEAR(dist.get(), 47.0, epsilon);
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
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPoint;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPoint(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Out of range
  EXPECT_THROW(
    calcLongitudinalOffsetPoint(traj.points, traj.points.size() + 1, 1.0), std::out_of_range);
  EXPECT_THROW(calcLongitudinalOffsetPoint(traj.points, -1, 1.0), std::out_of_range);

  // Found Pose(forward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_back = calcSignedArcLength(traj.points, i, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, i, std::min(len, d_back));

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().z, 0.0, epsilon);

      x_ans += 0.1;
    }
  }

  // Found Pose(backward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_front = calcSignedArcLength(traj.points, i, 0);

    for (double len = 0.0; d_front - epsilon < len; len -= 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, i, std::max(len, d_front));

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().z, 0.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, 0, total_length + epsilon);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, 9, -total_length - epsilon);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPoint(one_point_traj.points, 0.0, 0.0);

    EXPECT_EQ(p_out, boost::none);
  }
}

TEST(trajectory, calcLongitudinalOffsetPointFromPoint)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPoint;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPoint(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Found Pose(forward)
  for (double x_start = 0.0; x_start < total_length + epsilon; x_start += 0.1) {
    constexpr double lateral_deviation = 0.5;
    double x_ans = x_start;

    const auto p_src = createPoint(x_start, lateral_deviation, 0.0);
    const auto d_back = calcSignedArcLength(traj.points, p_src, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, std::min(len, d_back));

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().z, 0.0, epsilon);

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

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().z, 0.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_src = createPoint(0.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, total_length + 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found
  {
    const auto p_src = createPoint(9.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPoint(traj.points, p_src, -total_length - 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // Out of range(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_THROW(
      calcLongitudinalOffsetPoint(one_point_traj.points, geometry_msgs::msg::Point{}, {}),
      std::out_of_range);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPose(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Out of range
  EXPECT_THROW(
    calcLongitudinalOffsetPose(traj.points, traj.points.size() + 1, 1.0), std::out_of_range);
  EXPECT_THROW(calcLongitudinalOffsetPose(traj.points, -1, 1.0), std::out_of_range);

  // Found Pose(forward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_back = calcSignedArcLength(traj.points, i, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, i, std::min(len, d_back));

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);

      x_ans += 0.1;
    }
  }

  // Found Pose(backward)
  for (size_t i = 0; i < traj.points.size(); ++i) {
    double x_ans = getPoint(traj.points.at(i)).x;

    const auto d_front = calcSignedArcLength(traj.points, i, 0);

    for (double len = 0.0; d_front - epsilon < len; len -= 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, i, std::max(len, d_front));

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, total_length + epsilon);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 9, -total_length - epsilon);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPose(one_point_traj.points, 0.0, 0.0);

    EXPECT_EQ(p_out, boost::none);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex_quatInterpolation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
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

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, len * std::cos(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.get().position.y, len * std::sin(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, ans_quat.w, epsilon);
  }

  // Found pose(backward)
  for (double len = total_length; 0.0 < len; len -= 0.1) {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 1, -len);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(45.0));

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 1.0 - len * std::cos(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.get().position.y, 1.0 - len * std::sin(deg2rad(45.0)), epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, ans_quat.w, epsilon);
  }

  // Boundary condition
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 0, total_length);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, ans_quat.w, epsilon);
  }

  // Boundary condition
  {
    const auto p_out = calcLongitudinalOffsetPose(traj.points, 1, 0.0);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, ans_quat.w, epsilon);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPose(Trajectory{}.points, {}, {}), std::invalid_argument);

  // Found Pose(forward)
  for (double x_start = 0.0; x_start < total_length + epsilon; x_start += 0.1) {
    constexpr double lateral_deviation = 0.5;
    double x_ans = x_start;

    const auto p_src = createPoint(x_start, lateral_deviation, 0.0);
    const auto d_back = calcSignedArcLength(traj.points, p_src, traj.points.size() - 1);

    for (double len = 0.0; len < d_back + epsilon; len += 0.1) {
      const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, std::min(len, d_back));

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);

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

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(p_out.get().position.x, x_ans, epsilon);
      EXPECT_NEAR(p_out.get().position.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.x, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.y, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.w, 1.0, epsilon);

      x_ans -= 0.1;
    }
  }

  // No found
  {
    const auto p_src = createPoint(0.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, total_length + 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // No found
  {
    const auto p_src = createPoint(9.0, 0.0, 0.0);
    const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, -total_length - 1.0);

    EXPECT_EQ(p_out, boost::none);
  }

  // Out of range(Trajectory size is 1)
  {
    const auto one_point_traj = generateTestTrajectory<Trajectory>(1, 1.0);
    EXPECT_THROW(
      calcLongitudinalOffsetPose(one_point_traj.points, geometry_msgs::msg::Point{}, {}),
      std::out_of_range);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint_quatInterpolation)
{
  using autoware_auto_planning_msgs::msg::TrajectoryPoint;
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::calcLongitudinalOffsetToSegment;
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

      EXPECT_NE(p_out, boost::none);
      EXPECT_NEAR(
        p_out.get().position.x, p_src.x + len * std::cos(deg2rad(45.0)) - deviation, epsilon);
      EXPECT_NEAR(
        p_out.get().position.y, p_src.y + len * std::sin(deg2rad(45.0)) + deviation, epsilon);
      EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
      EXPECT_NEAR(p_out.get().orientation.x, ans_quat.x, epsilon);
      EXPECT_NEAR(p_out.get().orientation.y, ans_quat.y, epsilon);
      EXPECT_NEAR(p_out.get().orientation.z, ans_quat.z, epsilon);
      EXPECT_NEAR(p_out.get().orientation.w, ans_quat.w, epsilon);
    }
  }

  // Boundary condition
  {
    constexpr double deviation = 0.1;

    const auto p_src = createPoint(1.0 + deviation, 1.0 - deviation, 0.0);
    const auto src_offset = calcLongitudinalOffsetToSegment(traj.points, 0, p_src);

    const auto p_out = calcLongitudinalOffsetPose(traj.points, p_src, total_length - src_offset);
    const auto ans_quat = createQuaternionFromRPY(deg2rad(0.0), deg2rad(0.0), deg2rad(0.0));

    EXPECT_NE(p_out, boost::none);
    EXPECT_NEAR(p_out.get().position.x, 1.0, epsilon);
    EXPECT_NEAR(p_out.get().position.y, 1.0, epsilon);
    EXPECT_NEAR(p_out.get().position.z, 0.0, epsilon);
    EXPECT_NEAR(p_out.get().orientation.x, ans_quat.x, epsilon);
    EXPECT_NEAR(p_out.get().orientation.y, ans_quat.y, epsilon);
    EXPECT_NEAR(p_out.get().orientation.z, ans_quat.z, epsilon);
    EXPECT_NEAR(p_out.get().orientation.w, ans_quat.w, epsilon);
  }
}

TEST(trajectory, insertTargetPoint)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPose;
  using tier4_autoware_utils::insertTargetPoint;

  const auto traj = generateTestTrajectory<Trajectory>(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.get()));
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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.get()));
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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > 1e-3);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.get()));
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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx);
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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
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

    EXPECT_STREQ(testing::internal::GetCapturedStderr().c_str(), "Sharp angle.\n");
    EXPECT_EQ(insert_idx, boost::none);
  }

  // Invalid target point(Behind of end point)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_STREQ(testing::internal::GetCapturedStderr().c_str(), "Sharp angle.\n");
    EXPECT_EQ(insert_idx, boost::none);
  }

  // Invalid target point(Huge lateral offset)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(4.0, 10.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(base_idx, p_target, traj_out.points);

    EXPECT_STREQ(testing::internal::GetCapturedStderr().c_str(), "Sharp angle.\n");
    EXPECT_EQ(insert_idx, boost::none);
  }

  // Invalid base index
  {
    auto traj_out = traj;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(9U, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, boost::none);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory<Trajectory>(0, 1.0);
    EXPECT_THROW(
      insertTargetPoint({}, geometry_msgs::msg::Point{}, empty_traj.points), std::invalid_argument);
  }
}

TEST(trajectory, insertTargetPoint_OverlapThreshold)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPose;
  using tier4_autoware_utils::insertTargetPoint;

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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size() + 1);

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(
        calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > overlap_threshold);
    }

    {
      const auto p_insert = getPose(traj_out.points.at(insert_idx.get()));
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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx);
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

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

    for (size_t i = 0; i < traj_out.points.size() - 1; ++i) {
      EXPECT_TRUE(
        calcDistance2d(traj_out.points.at(i), traj_out.points.at(i + 1)) > overlap_threshold);
    }
  }
}
