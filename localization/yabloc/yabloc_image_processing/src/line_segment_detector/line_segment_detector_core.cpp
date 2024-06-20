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

#include "yabloc_image_processing/line_segment_detector/line_segment_detector.hpp"

#include <autoware/universe_utils/system/stop_watch.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace yabloc::line_segment_detector
{
LineSegmentDetector::LineSegmentDetector(const rclcpp::NodeOptions & options)
: Node("line_detector", options)
{
  using std::placeholders::_1;

  // Subscriber
  auto cb_image = std::bind(&LineSegmentDetector::on_image, this, _1);
  sub_image_ = create_subscription<Image>("~/input/image_raw", 10, cb_image);

  // Publisher
  pub_image_with_line_segments_ = create_publisher<Image>("~/debug/image_with_line_segments", 10);
  pub_cloud_ = create_publisher<PointCloud2>("~/output/line_segments_cloud", 10);

  line_segment_detector_ =
    cv::createLineSegmentDetector(cv::LSD_REFINE_STD, 0.8, 0.6, 2.0, 22.5, 0, 0.7, 1024);
}

void LineSegmentDetector::on_image(const sensor_msgs::msg::Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  execute(image, msg.header.stamp);
}

void LineSegmentDetector::execute(const cv::Mat & image, const rclcpp::Time & stamp)
{
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

  cv::Mat lines;
  {
    autoware::universe_utils::StopWatch stop_watch;
    line_segment_detector_->detect(gray_image, lines);
    if (lines.size().width != 0) {
      line_segment_detector_->drawSegments(gray_image, lines);
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "lsd: " << stop_watch.toc() << "[ms]");
  }

  common::publish_image(*pub_image_with_line_segments_, gray_image, stamp);

  pcl::PointCloud<pcl::PointNormal> line_cloud;
  std::vector<cv::Mat> filtered_lines = remove_too_outer_elements(lines, image.size());

  for (const cv::Mat & xy_xy : filtered_lines) {
    Eigen::Vector3f xy1;
    Eigen::Vector3f xy2;
    xy1 << xy_xy.at<float>(0), xy_xy.at<float>(1), 0;
    xy2 << xy_xy.at<float>(2), xy_xy.at<float>(3), 0;
    pcl::PointNormal pn;
    pn.getVector3fMap() = xy1;
    pn.getNormalVector3fMap() = xy2;
    line_cloud.push_back(pn);
  }
  common::publish_cloud(*pub_cloud_, line_cloud, stamp);
}

std::vector<cv::Mat> LineSegmentDetector::remove_too_outer_elements(
  const cv::Mat & lines, const cv::Size & size)
{
  std::vector<cv::Rect2i> rect_vector;
  rect_vector.emplace_back(0, 0, size.width, 3);
  rect_vector.emplace_back(0, size.height - 3, size.width, 3);
  rect_vector.emplace_back(0, 0, 3, size.height);
  rect_vector.emplace_back(size.width - 3, 0, 3, size.height);

  std::vector<cv::Mat> filtered_lines;
  for (int i = 0; i < lines.rows; i++) {
    cv::Mat xy_xy = lines.row(i);
    cv::Point2f xy1(xy_xy.at<float>(0), xy_xy.at<float>(1));
    cv::Point2f xy2(xy_xy.at<float>(2), xy_xy.at<float>(3));

    bool enabled = true;
    for (const cv::Rect2i & rect : rect_vector) {
      if (rect.contains(xy1) && rect.contains(xy2)) {
        enabled = false;
        break;
      }
    }
    if (enabled) filtered_lines.push_back(xy_xy);
  }

  return filtered_lines;
}

}  // namespace yabloc::line_segment_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yabloc::line_segment_detector::LineSegmentDetector)
