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

#ifndef YABLOC_IMAGE_PROCESSING__LINE_SEGMENTS_OVERLAY__LINE_SEGMENTS_OVERLAY_HPP_
#define YABLOC_IMAGE_PROCESSING__LINE_SEGMENTS_OVERLAY__LINE_SEGMENTS_OVERLAY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>

namespace yabloc::line_segments_overlay
{
class LineSegmentsOverlay : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using LineSegment = pcl::PointXYZLNormal;
  using LineSegments = pcl::PointCloud<LineSegment>;
  LineSegmentsOverlay();

private:
  void on_image(const Image::ConstSharedPtr & img_msg);
  void on_line_segments(const PointCloud2::ConstSharedPtr & line_segments_msg);
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_line_segments_;
  rclcpp::Publisher<Image>::SharedPtr pub_debug_image_;

  std::map<rclcpp::Time, Image::ConstSharedPtr> image_buffer_;
  size_t max_buffer_size_;
};
}  // namespace yabloc::line_segments_overlay

#endif  // YABLOC_IMAGE_PROCESSING__LINE_SEGMENTS_OVERLAY__LINE_SEGMENTS_OVERLAY_HPP_
