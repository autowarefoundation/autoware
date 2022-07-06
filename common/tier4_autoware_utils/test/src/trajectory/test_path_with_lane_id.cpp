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

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"
#include "tier4_autoware_utils/trajectory/path_with_lane_id.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.h>

#include <limits>
#include <vector>

namespace
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
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

PathWithLaneId generateTestTrajectory(
  const size_t num_points, const double point_interval, const double vel = 0.0,
  const double init_theta = 0.0, const double delta_theta = 0.0)
{
  PathWithLaneId path_with_lane_id;
  for (size_t i = 0; i < num_points; ++i) {
    const double theta = init_theta + i * delta_theta;
    const double x = i * point_interval * std::cos(theta);
    const double y = i * point_interval * std::sin(theta);

    PathPointWithLaneId p;
    p.point.pose = createPose(x, y, 0.0, 0.0, 0.0, theta);
    p.point.longitudinal_velocity_mps = vel;
    path_with_lane_id.points.push_back(p);
  }

  return path_with_lane_id;
}
}  // namespace

TEST(trajectory, calcLongitudinalOffsetPointFromIndex_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPoint;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPoint(PathWithLaneId{}.points, {}, {}), std::invalid_argument);

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
    const auto one_point_traj = generateTestTrajectory(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPoint(one_point_traj.points, 0.0, 0.0);

    EXPECT_EQ(p_out, boost::none);
  }
}

TEST(trajectory, calcLongitudinalOffsetPointFromPoint_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPoint;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPoint(PathWithLaneId{}.points, {}, {}), std::invalid_argument);

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
    const auto one_point_traj = generateTestTrajectory(1, 1.0);
    EXPECT_THROW(
      calcLongitudinalOffsetPoint(one_point_traj.points, geometry_msgs::msg::Point{}, {}),
      std::out_of_range);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromIndex_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPose(PathWithLaneId{}.points, {}, {}), std::invalid_argument);

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
    const auto one_point_traj = generateTestTrajectory(1, 1.0);
    const auto p_out = calcLongitudinalOffsetPose(one_point_traj.points, 0.0, 0.0);

    EXPECT_EQ(p_out, boost::none);
  }
}

TEST(trajectory, calcLongitudinalOffsetPoseFromPoint_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcLongitudinalOffsetPose;
  using tier4_autoware_utils::calcSignedArcLength;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::getPoint;

  const auto traj = generateTestTrajectory(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Empty
  EXPECT_THROW(calcLongitudinalOffsetPose(PathWithLaneId{}.points, {}, {}), std::invalid_argument);

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
    const auto one_point_traj = generateTestTrajectory(1, 1.0);
    EXPECT_THROW(
      calcLongitudinalOffsetPose(one_point_traj.points, geometry_msgs::msg::Point{}, {}),
      std::out_of_range);
  }
}

TEST(trajectory, insertTargetPoint_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPose;
  using tier4_autoware_utils::insertTargetPoint;

  const auto traj = generateTestTrajectory(10, 1.0);
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
    const size_t segment_idx = 9U;
    const auto insert_idx = insertTargetPoint(segment_idx, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, boost::none);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory(0, 1.0);
    const size_t segment_idx = 0;
    EXPECT_THROW(
      insertTargetPoint(segment_idx, geometry_msgs::msg::Point{}, empty_traj.points),
      std::invalid_argument);
  }
}

TEST(trajectory, insertTargetPoint_OverlapThreshold_PathWithLaneId)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPose;
  using tier4_autoware_utils::insertTargetPoint;

  constexpr double overlap_threshold = 1e-4;
  const auto traj = generateTestTrajectory(10, 1.0);
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

TEST(trajectory, insertTargetPoint_PathWithLaneId_Length)
{
  using tier4_autoware_utils::calcArcLength;
  using tier4_autoware_utils::calcDistance2d;
  using tier4_autoware_utils::createPoint;
  using tier4_autoware_utils::deg2rad;
  using tier4_autoware_utils::findNearestSegmentIndex;
  using tier4_autoware_utils::getPose;
  using tier4_autoware_utils::insertTargetPoint;

  const auto traj = generateTestTrajectory(10, 1.0);
  const auto total_length = calcArcLength(traj.points);

  // Insert
  for (double x_start = 0.5; x_start < total_length; x_start += 1.0) {
    auto traj_out = traj;

    const auto p_target = createPoint(x_start, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(x_start, p_target, traj_out.points);

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
    const auto insert_idx = insertTargetPoint(x_start + 1.1e-3, p_target, traj_out.points);

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

  // Right on the terminal point
  {
    auto traj_out = traj;

    const auto p_target = createPoint(9.0, 0.0, 0.0);
    const size_t base_idx = findNearestSegmentIndex(traj.points, p_target);
    const auto insert_idx = insertTargetPoint(9.0, p_target, traj_out.points);

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
    EXPECT_EQ(traj_out.points.size(), traj.points.size());

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
    const auto insert_idx = insertTargetPoint(x_start, p_target, traj_out.points);

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
    const auto insert_idx = insertTargetPoint(x_start + 1e-4, p_target, traj_out.points);

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
    const auto insert_idx = insertTargetPoint(x_start - 1e-4, p_target, traj_out.points);

    EXPECT_NE(insert_idx, boost::none);
    EXPECT_EQ(insert_idx.get(), base_idx + 1);
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

    EXPECT_EQ(insert_idx, boost::none);
  }

  // Invalid target point(Behind the end point)
  {
    auto traj_out = traj;

    const auto p_target = createPoint(10.0, 0.0, 0.0);
    const auto insert_idx = insertTargetPoint(10.0, p_target, traj_out.points);

    EXPECT_EQ(insert_idx, boost::none);
  }

  // Invalid target point(Huge lateral offset)
  {
    testing::internal::CaptureStderr();
    auto traj_out = traj;

    const auto p_target = createPoint(4.0, 10.0, 0.0);
    const auto insert_idx = insertTargetPoint(4.0, p_target, traj_out.points);

    EXPECT_STREQ(testing::internal::GetCapturedStderr().c_str(), "Sharp angle.\n");
    EXPECT_EQ(insert_idx, boost::none);
  }

  // Empty
  {
    auto empty_traj = generateTestTrajectory(0, 1.0);
    EXPECT_THROW(
      insertTargetPoint(0.0, geometry_msgs::msg::Point{}, empty_traj.points),
      std::invalid_argument);
  }
}
