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

#include "yabloc_image_processing/line_segments_overlay/line_segments_overlay.hpp"

#include <opencv4/opencv2/core/eigen.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::line_segments_overlay
{
LineSegmentsOverlay::LineSegmentsOverlay()
: Node("overlay_lanelet2"),
  max_buffer_size_(static_cast<size_t>(declare_parameter<int>("max_buffer_size", 5)))
{
  using std::placeholders::_1;

  auto cb_image = std::bind(&LineSegmentsOverlay::on_image, this, _1);
  auto cb_line_segments_ = std::bind(&LineSegmentsOverlay::on_line_segments, this, _1);

  sub_image_ = create_subscription<Image>("~/input/image_raw", 10, cb_image);
  sub_line_segments_ =
    create_subscription<PointCloud2>("~/input/line_segments", 10, cb_line_segments_);

  pub_debug_image_ = create_publisher<Image>("~/debug/image_with_colored_line_segments", 10);
}

void LineSegmentsOverlay::on_image(const Image::ConstSharedPtr & img_msg)
{
  rclcpp::Time timestamp(img_msg->header.stamp);
  image_buffer_[timestamp] = img_msg;

  if (image_buffer_.size() > max_buffer_size_) {
    // The image_buffer is an ordered_map whose key is rclcpp::Time.
    // rclcpp::Time has a natural ordering system, so image_buffer.begin() always gives the
    // oldest message.
    image_buffer_.erase(image_buffer_.begin());
  }
}

void LineSegmentsOverlay::on_line_segments(const PointCloud2::ConstSharedPtr & line_segments_msg)
{
  const rclcpp::Time stamp = rclcpp::Time(line_segments_msg->header.stamp);
  auto iter = image_buffer_.find(stamp);
  // If the iterator reaches the end of the image_buffer_, it means the image with the given
  // timestamp is not found in the buffer. This assumption is based on the fact that images are
  // received and stored in the buffer BEFORE their corresponding line segments are processed. If
  // this assumption does not hold, the function may throw a runtime error indicating that the image
  // with the given timestamp was not found.
  if (iter == image_buffer_.end()) {
    // However, the above assumption may be violated just after launch.
    RCLCPP_ERROR_STREAM(get_logger(), "Image with the given timestamp not found.");
    return;
  }

  auto image_ptr = iter->second;
  cv::Mat image = cv_bridge::toCvShare(image_ptr, "bgr8")->image;

  LineSegments line_segments_cloud;
  pcl::fromROSMsg(*line_segments_msg, line_segments_cloud);

  for (size_t index = 0; index < line_segments_cloud.size(); ++index) {
    const LineSegment & pn = line_segments_cloud.at(index);
    Eigen::Vector3f xy1 = pn.getVector3fMap();
    Eigen::Vector3f xy2 = pn.getNormalVector3fMap();

    cv::Scalar color(0, 255, 0);      // Green
    if (pn.label == 0) {              // if unreliable
      color = cv::Scalar(0, 0, 255);  // Red
    }

    cv::line(image, cv::Point(xy1(0), xy1(1)), cv::Point(xy2(0), xy2(1)), color, 2);
  }

  common::publish_image(*pub_debug_image_, image, stamp);
}

}  // namespace yabloc::line_segments_overlay
