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

#include "perception_utils/geometry.hpp"

#include <gtest/gtest.h>

#include <string>

TEST(geometry, getPose)
{
  using perception_utils::getPose;

  const double x_ans = 1.0;
  const double y_ans = 2.0;
  const double z_ans = 3.0;
  const double q_x_ans = 0.1;
  const double q_y_ans = 0.2;
  const double q_z_ans = 0.3;
  const double q_w_ans = 0.4;

  {
    geometry_msgs::msg::Pose p;
    p.position.x = x_ans;
    p.position.y = y_ans;
    p.position.z = z_ans;
    p.orientation.x = q_x_ans;
    p.orientation.y = q_y_ans;
    p.orientation.z = q_z_ans;
    p.orientation.w = q_w_ans;
    const geometry_msgs::msg::Pose p_out = getPose(p);
    EXPECT_DOUBLE_EQ(p_out.position.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.position.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.position.z, z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.x, q_x_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.y, q_y_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.z, q_z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.w, q_w_ans);
  }

  {
    autoware_auto_perception_msgs::msg::DetectedObject p;
    p.kinematics.pose_with_covariance.pose.position.x = x_ans;
    p.kinematics.pose_with_covariance.pose.position.y = y_ans;
    p.kinematics.pose_with_covariance.pose.position.z = z_ans;
    p.kinematics.pose_with_covariance.pose.orientation.x = q_x_ans;
    p.kinematics.pose_with_covariance.pose.orientation.y = q_y_ans;
    p.kinematics.pose_with_covariance.pose.orientation.z = q_z_ans;
    p.kinematics.pose_with_covariance.pose.orientation.w = q_w_ans;

    const geometry_msgs::msg::Pose p_out = getPose(p);
    EXPECT_DOUBLE_EQ(p_out.position.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.position.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.position.z, z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.x, q_x_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.y, q_y_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.z, q_z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.w, q_w_ans);
  }

  {
    autoware_auto_perception_msgs::msg::TrackedObject p;
    p.kinematics.pose_with_covariance.pose.position.x = x_ans;
    p.kinematics.pose_with_covariance.pose.position.y = y_ans;
    p.kinematics.pose_with_covariance.pose.position.z = z_ans;
    p.kinematics.pose_with_covariance.pose.orientation.x = q_x_ans;
    p.kinematics.pose_with_covariance.pose.orientation.y = q_y_ans;
    p.kinematics.pose_with_covariance.pose.orientation.z = q_z_ans;
    p.kinematics.pose_with_covariance.pose.orientation.w = q_w_ans;

    const geometry_msgs::msg::Pose p_out = getPose(p);
    EXPECT_DOUBLE_EQ(p_out.position.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.position.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.position.z, z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.x, q_x_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.y, q_y_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.z, q_z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.w, q_w_ans);
  }

  {
    autoware_auto_perception_msgs::msg::PredictedObject p;
    p.kinematics.initial_pose_with_covariance.pose.position.x = x_ans;
    p.kinematics.initial_pose_with_covariance.pose.position.y = y_ans;
    p.kinematics.initial_pose_with_covariance.pose.position.z = z_ans;
    p.kinematics.initial_pose_with_covariance.pose.orientation.x = q_x_ans;
    p.kinematics.initial_pose_with_covariance.pose.orientation.y = q_y_ans;
    p.kinematics.initial_pose_with_covariance.pose.orientation.z = q_z_ans;
    p.kinematics.initial_pose_with_covariance.pose.orientation.w = q_w_ans;

    const geometry_msgs::msg::Pose p_out = getPose(p);
    EXPECT_DOUBLE_EQ(p_out.position.x, x_ans);
    EXPECT_DOUBLE_EQ(p_out.position.y, y_ans);
    EXPECT_DOUBLE_EQ(p_out.position.z, z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.x, q_x_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.y, q_y_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.z, q_z_ans);
    EXPECT_DOUBLE_EQ(p_out.orientation.w, q_w_ans);
  }
}
