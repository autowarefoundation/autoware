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

#include "yabloc_common/pub_sub.hpp"

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::common
{
void publish_image(
  rclcpp::Publisher<sensor_msgs::msg::Image> & publisher, const cv::Mat & image,
  const rclcpp::Time & stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  if (image.channels() == 3)
    raw_image.encoding = "bgr8";
  else
    raw_image.encoding = "mono8";

  if (image.depth() != CV_8U) {
    throw std::runtime_error("publish_image can publish only CV_8U");
  }

  raw_image.image = image;
  publisher.publish(*raw_image.toImageMsg());
}

template <typename PointT>
void publish_cloud(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> & publisher,
  const pcl::PointCloud<PointT> & cloud, const rclcpp::Time & stamp)
{
  // Convert to msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "map";
  publisher.publish(cloud_msg);
}

template void publish_cloud<pcl::PointXYZ>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZ> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointXYZL>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZL> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointNormal>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointNormal> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointXYZI>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZI> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointXYZLNormal>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZLNormal> &,
  const rclcpp::Time &);

template void publish_cloud<pcl::PointXYZRGB>(
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2> &, const pcl::PointCloud<pcl::PointXYZRGB> &,
  const rclcpp::Time &);

}  // namespace yabloc::common
