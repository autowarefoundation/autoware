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

#ifndef YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__GRAPH_SEGMENT_HPP_
#define YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__GRAPH_SEGMENT_HPP_

#include "yabloc_image_processing/graph_segment/similar_area_searcher.hpp"

#include <opencv4/opencv2/ximgproc/segmentation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>

namespace yabloc::graph_segment
{
class GraphSegment : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  GraphSegment();

private:
  const float target_height_ratio_;
  const int target_candidate_box_width_;

  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_mask_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_debug_image_;
  cv::Ptr<cv::ximgproc::segmentation::GraphSegmentation> segmentation_;
  std::unique_ptr<SimilarAreaSearcher> similar_area_searcher_{nullptr};

  void on_image(const Image & msg);

  int search_most_road_like_class(const cv::Mat & segmented) const;

  void draw_and_publish_image(
    const cv::Mat & raw_image, const cv::Mat & debug_image, const rclcpp::Time & stamp);
};
}  // namespace yabloc::graph_segment

#endif  // YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__GRAPH_SEGMENT_HPP_
