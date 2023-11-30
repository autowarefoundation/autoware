// Copyright 2023- Autoware Foundation
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

#include "localization_util/smart_pose_buffer.hpp"
#include "localization_util/util_func.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

bool compare_pose(
  const PoseWithCovarianceStamped & pose_a, const PoseWithCovarianceStamped & pose_b)
{
  return pose_a.header.stamp == pose_b.header.stamp &&
         pose_a.header.frame_id == pose_b.header.frame_id &&
         pose_a.pose.covariance == pose_b.pose.covariance &&
         pose_a.pose.pose.position.x == pose_b.pose.pose.position.x &&
         pose_a.pose.pose.position.y == pose_b.pose.pose.position.y &&
         pose_a.pose.pose.position.z == pose_b.pose.pose.position.z &&
         pose_a.pose.pose.orientation.x == pose_b.pose.pose.orientation.x &&
         pose_a.pose.pose.orientation.y == pose_b.pose.pose.orientation.y &&
         pose_a.pose.pose.orientation.z == pose_b.pose.pose.orientation.z &&
         pose_a.pose.pose.orientation.w == pose_b.pose.pose.orientation.w;
}

class TestSmartPoseBuffer : public ::testing::Test
{
protected:
  void SetUp() override {}

  void TearDown() override {}
};

TEST_F(TestSmartPoseBuffer, interpolate_pose)  // NOLINT
{
  rclcpp::Logger logger = rclcpp::get_logger("test_logger");
  const double pose_timeout_sec = 10.0;
  const double pose_distance_tolerance_meters = 100.0;
  SmartPoseBuffer smart_pose_buffer(logger, pose_timeout_sec, pose_distance_tolerance_meters);

  // first data
  PoseWithCovarianceStamped::SharedPtr old_pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  old_pose_ptr->header.stamp.sec = 10;
  old_pose_ptr->header.stamp.nanosec = 0;
  old_pose_ptr->pose.pose.position.x = 10.0;
  old_pose_ptr->pose.pose.position.y = 20.0;
  old_pose_ptr->pose.pose.position.z = 0.0;
  old_pose_ptr->pose.pose.orientation = rpy_deg_to_quaternion(0.0, 0.0, 0.0);
  smart_pose_buffer.push_back(old_pose_ptr);

  // second data
  PoseWithCovarianceStamped::SharedPtr new_pose_ptr = std::make_shared<PoseWithCovarianceStamped>();
  new_pose_ptr->header.stamp.sec = 20;
  new_pose_ptr->header.stamp.nanosec = 0;
  new_pose_ptr->pose.pose.position.x = 20.0;
  new_pose_ptr->pose.pose.position.y = 40.0;
  new_pose_ptr->pose.pose.position.z = 0.0;
  new_pose_ptr->pose.pose.orientation = rpy_deg_to_quaternion(0.0, 0.0, 90.0);
  smart_pose_buffer.push_back(new_pose_ptr);

  // interpolate
  builtin_interfaces::msg::Time target_ros_time_msg;
  target_ros_time_msg.sec = 15;
  target_ros_time_msg.nanosec = 0;
  const std::optional<SmartPoseBuffer::InterpolateResult> & interpolate_result =
    smart_pose_buffer.interpolate(target_ros_time_msg);
  ASSERT_TRUE(interpolate_result.has_value());
  const SmartPoseBuffer::InterpolateResult result = interpolate_result.value();

  // check old
  EXPECT_TRUE(compare_pose(result.old_pose, *old_pose_ptr));

  // check new
  EXPECT_TRUE(compare_pose(result.new_pose, *new_pose_ptr));

  // check interpolated
  EXPECT_EQ(result.interpolated_pose.header.stamp.sec, 15);
  EXPECT_EQ(result.interpolated_pose.header.stamp.nanosec, static_cast<uint32_t>(0));
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.x, 15.0);
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.y, 30.0);
  EXPECT_EQ(result.interpolated_pose.pose.pose.position.z, 0.0);
  const auto rpy = get_rpy(result.interpolated_pose.pose.pose);
  EXPECT_NEAR(rpy.x * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.y * 180 / M_PI, 0.0, 1e-6);
  EXPECT_NEAR(rpy.z * 180 / M_PI, 45.0, 1e-6);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
