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

#include "autoware/universe_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

TEST(geometry, getPoint_PathWithLaneId)
{
  using autoware::universe_utils::getPoint;

  const double x_ans = 1.0;
  const double y_ans = 2.0;
  const double z_ans = 3.0;

  tier4_planning_msgs::msg::PathPointWithLaneId p;
  p.point.pose.position.x = x_ans;
  p.point.pose.position.y = y_ans;
  p.point.pose.position.z = z_ans;
  const geometry_msgs::msg::Point p_out = getPoint(p);
  EXPECT_DOUBLE_EQ(p_out.x, x_ans);
  EXPECT_DOUBLE_EQ(p_out.y, y_ans);
  EXPECT_DOUBLE_EQ(p_out.z, z_ans);
}

TEST(geometry, getPose_PathWithLaneId)
{
  using autoware::universe_utils::getPose;

  const double x_ans = 1.0;
  const double y_ans = 2.0;
  const double z_ans = 3.0;
  const double q_x_ans = 0.1;
  const double q_y_ans = 0.2;
  const double q_z_ans = 0.3;
  const double q_w_ans = 0.4;

  tier4_planning_msgs::msg::PathPointWithLaneId p;
  p.point.pose.position.x = x_ans;
  p.point.pose.position.y = y_ans;
  p.point.pose.position.z = z_ans;
  p.point.pose.orientation.x = q_x_ans;
  p.point.pose.orientation.y = q_y_ans;
  p.point.pose.orientation.z = q_z_ans;
  p.point.pose.orientation.w = q_w_ans;
  const geometry_msgs::msg::Pose p_out = getPose(p);
  EXPECT_DOUBLE_EQ(p_out.position.x, x_ans);
  EXPECT_DOUBLE_EQ(p_out.position.y, y_ans);
  EXPECT_DOUBLE_EQ(p_out.position.z, z_ans);
  EXPECT_DOUBLE_EQ(p_out.orientation.x, q_x_ans);
  EXPECT_DOUBLE_EQ(p_out.orientation.y, q_y_ans);
  EXPECT_DOUBLE_EQ(p_out.orientation.z, q_z_ans);
  EXPECT_DOUBLE_EQ(p_out.orientation.w, q_w_ans);
}

TEST(geometry, getLongitudinalVelocity_PathWithLaneId)
{
  using autoware::universe_utils::getLongitudinalVelocity;

  const double velocity = 1.0;

  tier4_planning_msgs::msg::PathPointWithLaneId p;
  p.point.longitudinal_velocity_mps = velocity;
  EXPECT_DOUBLE_EQ(getLongitudinalVelocity(p), velocity);
}

TEST(geometry, setPose_PathWithLaneId)
{
  using autoware::universe_utils::setPose;

  const double x_ans = 1.0;
  const double y_ans = 2.0;
  const double z_ans = 3.0;
  const double q_x_ans = 0.1;
  const double q_y_ans = 0.2;
  const double q_z_ans = 0.3;
  const double q_w_ans = 0.4;

  geometry_msgs::msg::Pose p;
  p.position.x = x_ans;
  p.position.y = y_ans;
  p.position.z = z_ans;
  p.orientation.x = q_x_ans;
  p.orientation.y = q_y_ans;
  p.orientation.z = q_z_ans;
  p.orientation.w = q_w_ans;

  tier4_planning_msgs::msg::PathPointWithLaneId p_out{};
  setPose(p, p_out);
  EXPECT_DOUBLE_EQ(p_out.point.pose.position.x, x_ans);
  EXPECT_DOUBLE_EQ(p_out.point.pose.position.y, y_ans);
  EXPECT_DOUBLE_EQ(p_out.point.pose.position.z, z_ans);
  EXPECT_DOUBLE_EQ(p_out.point.pose.orientation.x, q_x_ans);
  EXPECT_DOUBLE_EQ(p_out.point.pose.orientation.y, q_y_ans);
  EXPECT_DOUBLE_EQ(p_out.point.pose.orientation.z, q_z_ans);
  EXPECT_DOUBLE_EQ(p_out.point.pose.orientation.w, q_w_ans);
}

TEST(geometry, setLongitudinalVelocity_PathWithLaneId)
{
  using autoware::universe_utils::setLongitudinalVelocity;

  const double velocity = 1.0;

  tier4_planning_msgs::msg::PathPointWithLaneId p{};
  setLongitudinalVelocity(velocity, p);
  EXPECT_DOUBLE_EQ(p.point.longitudinal_velocity_mps, velocity);
}
