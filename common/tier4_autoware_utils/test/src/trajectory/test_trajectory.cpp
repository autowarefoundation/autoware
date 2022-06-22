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

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"
#include "tier4_autoware_utils/trajectory/tmp_conversion.hpp"
#include "tier4_autoware_utils/trajectory/trajectory.hpp"

#include <gtest/gtest.h>
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
