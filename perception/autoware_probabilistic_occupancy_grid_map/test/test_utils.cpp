// Copyright 2023 TIER IV, INC.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// autoware
#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

#include <gtest/gtest.h>

#include <memory>
// pcl
#include <pcl_ros/transforms.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// create pointcloud function
pcl::PointCloud<pcl::PointXYZ> createPCLPointCloudWithIteratedHeight(const size_t width)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.width = width;
  pcl_cloud.height = 1;  // assume height is 1
  pcl_cloud.points.resize(pcl_cloud.width * pcl_cloud.height);
  for (size_t i = 0; i < pcl_cloud.points.size(); ++i) {
    pcl_cloud.points[i].x = 1.0;
    pcl_cloud.points[i].y = 1.0;
    pcl_cloud.points[i].z = static_cast<float>(i);
  }
  pcl_cloud.header.frame_id = "base_link";
  return pcl_cloud;
}

// mock buffer to avoid tf2_ros::Buffer::lookupTransform() error
class MockBuffer : public tf2_ros::Buffer
{
public:
  MockBuffer() : Buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)) {}

  // override lookupTransform() to avoid error
  geometry_msgs::msg::TransformStamped lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const tf2::TimePoint & time) const override
  {
    (void)target_frame;
    (void)source_frame;
    (void)time;
    geometry_msgs::msg::TransformStamped dummy_transform;
    return dummy_transform;
  }
};

// test pointcloud cropping function
TEST(TestUtils, TestCropPointcloudByHeight)
{
  // mock buffer
  MockBuffer mock_buffer;
  // create pointcloud with pcl
  const auto pcl_cloud_10 = createPCLPointCloudWithIteratedHeight(10);
  const auto pcl_cloud_0 = createPCLPointCloudWithIteratedHeight(0);
  // convert to ros msg
  sensor_msgs::msg::PointCloud2 ros_cloud_10;
  sensor_msgs::msg::PointCloud2 ros_cloud_0;
  pcl::toROSMsg(pcl_cloud_10, ros_cloud_10);
  pcl::toROSMsg(pcl_cloud_0, ros_cloud_0);

  // test buffer
  sensor_msgs::msg::PointCloud2 test1_output, test2_output, test3_output;

  // case1: normal input, normal output
  EXPECT_NO_THROW(autoware::occupancy_grid_map::utils::cropPointcloudByHeight(
    ros_cloud_10, mock_buffer, "base_link", 0.0, 10.0, test1_output));

  // case2: normal input, empty output
  EXPECT_NO_THROW(autoware::occupancy_grid_map::utils::cropPointcloudByHeight(
    ros_cloud_10, mock_buffer, "base_link", -2.0, -0.1, test2_output));

  // case3: empty input, normal output
  EXPECT_NO_THROW(autoware::occupancy_grid_map::utils::cropPointcloudByHeight(
    ros_cloud_0, mock_buffer, "base_link", 0.0, 10.0, test3_output));
}
