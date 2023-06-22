// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_COMMON__PUB_SUB_HPP_
#define YABLOC_COMMON__PUB_SUB_HPP_

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace yabloc::common
{
void publish_image(
  rclcpp::Publisher<sensor_msgs::msg::Image> & publisher, const cv::Mat & image,
  const rclcpp::Time & stamp);

template <typename PointT>
void publish_cloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & publisher,
  const pcl::PointCloud<PointT> & cloud, const rclcpp::Time & stamp);
}  // namespace yabloc::common

#endif  // YABLOC_COMMON__PUB_SUB_HPP_
