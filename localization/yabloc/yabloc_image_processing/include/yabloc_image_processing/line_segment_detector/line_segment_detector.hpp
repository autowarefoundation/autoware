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

#ifndef YABLOC_IMAGE_PROCESSING__LINE_SEGMENT_DETECTOR__LINE_SEGMENT_DETECTOR_HPP_
#define YABLOC_IMAGE_PROCESSING__LINE_SEGMENT_DETECTOR__LINE_SEGMENT_DETECTOR_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core/eigen.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
#include <vector>

namespace yabloc::line_segment_detector
{
class LineSegmentDetector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

  LineSegmentDetector();

private:
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_with_line_segments_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;

  cv::Ptr<cv::LineSegmentDetector> line_segment_detector_;

  std::vector<cv::Mat> remove_too_outer_elements(
    const cv::Mat & lines, const cv::Size & size) const;
  void on_image(const sensor_msgs::msg::Image & msg);
  void execute(const cv::Mat & image, const rclcpp::Time & stamp);
};
}  // namespace yabloc::line_segment_detector

#endif  // YABLOC_IMAGE_PROCESSING__LINE_SEGMENT_DETECTOR__LINE_SEGMENT_DETECTOR_HPP_
