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

#include "yabloc_image_processing/graph_segment/graph_segment.hpp"
#include "yabloc_image_processing/graph_segment/histogram.hpp"

#include <autoware/universe_utils/system/stop_watch.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

namespace yabloc::graph_segment
{
GraphSegment::GraphSegment(const rclcpp::NodeOptions & options)
: Node("graph_segment", options),
  target_height_ratio_(static_cast<float>(declare_parameter<float>("target_height_ratio"))),
  target_candidate_box_width_(
    static_cast<int>(declare_parameter<int>("target_candidate_box_width")))
{
  using std::placeholders::_1;

  // Subscriber
  sub_image_ = create_subscription<Image>(
    "~/input/image_raw", 10, std::bind(&GraphSegment::on_image, this, _1));

  pub_mask_image_ = create_publisher<Image>("~/output/mask_image", 10);
  pub_debug_image_ = create_publisher<Image>("~/debug/segmented_image", 10);

  const double sigma = declare_parameter<double>("sigma");
  const float k = static_cast<float>(declare_parameter<float>("k"));
  const int min_size = static_cast<int>(declare_parameter<double>("min_size"));
  segmentation_ = cv::ximgproc::segmentation::createGraphSegmentation(sigma, k, min_size);

  // additional area pickup module
  if (declare_parameter<bool>("pickup_additional_areas", true)) {
    similar_area_searcher_ =
      std::make_unique<SimilarAreaSearcher>(declare_parameter<float>("similarity_score_threshold"));
  }
}

cv::Vec3b random_hsv(int index)
{
  // It generates colors that are not too bright or too vivid, but rich in hues.
  auto base = static_cast<double>(index);
  return {
    static_cast<unsigned char>(std::fmod(base * 0.7, 1.0) * 180),
    static_cast<unsigned char>(0.7 * 255), static_cast<unsigned char>(0.5 * 255)};
};

int GraphSegment::search_most_road_like_class(const cv::Mat & segmented) const
{
  const int bw = target_candidate_box_width_;
  const float r = target_height_ratio_;
  cv::Point2i target_px(
    static_cast<int>(static_cast<float>(segmented.cols) * 0.5),
    static_cast<int>(static_cast<float>(segmented.rows) * r));
  cv::Rect2i rect(target_px + cv::Point2i(-bw, -bw), target_px + cv::Point2i(bw, bw));

  std::unordered_map<int, int> areas;
  std::unordered_set<int> candidates;
  for (int h = 0; h < segmented.rows; h++) {
    const int * seg_ptr = segmented.ptr<int>(h);
    for (int w = 0; w < segmented.cols; w++) {
      int key = seg_ptr[w];
      areas.try_emplace(key, 0);
      areas[key]++;
      if (rect.contains(cv::Point2i{w, h})) candidates.insert(key);
    }
  }

  // Search the largest area and its class
  int max_area = 0;
  int max_area_class = -1;
  for (int c : candidates) {
    if (areas.at(c) < max_area) continue;
    max_area = areas.at(c);
    max_area_class = c;
  }
  return max_area_class;
}

void GraphSegment::on_image(const Image & msg)
{
  cv::Mat image = common::decompress_to_cv_mat(msg);
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(), 0.5, 0.5);

  // Execute graph-based segmentation
  autoware::universe_utils::StopWatch stop_watch;
  cv::Mat segmented;
  segmentation_->processImage(resized, segmented);
  RCLCPP_INFO_STREAM(get_logger(), "segmentation time: " << stop_watch.toc() * 1000 << "[ms]");

  //
  int target_class = search_most_road_like_class(segmented);
  //
  std::set<int> road_keys = {target_class};
  if (similar_area_searcher_) {
    road_keys = similar_area_searcher_->search(resized, segmented, target_class);
  }

  // Draw output image and debug image
  cv::Mat output_image = cv::Mat::zeros(resized.size(), CV_8UC1);
  cv::Mat debug_image = cv::Mat::zeros(resized.size(), CV_8UC3);
  for (int h = 0; h < resized.rows; h++) {
    // NOTE: Accessing through ptr() is faster than at()
    auto * const debug_image_ptr = debug_image.ptr<cv::Vec3b>(h);
    auto * const output_image_ptr = output_image.ptr<uchar>(h);
    const int * const segmented_image_ptr = segmented.ptr<int>(h);

    for (int w = 0; w < resized.cols; w++) {
      cv::Point2i px(w, h);
      const int key = segmented_image_ptr[w];
      if (road_keys.count(key) > 0) {
        output_image_ptr[w] = 255;
        if (key == target_class)
          debug_image_ptr[w] = cv::Vec3b(30, 255, 255);
        else
          debug_image_ptr[w] = cv::Vec3b(10, 255, 255);
      } else {
        debug_image_ptr[w] = random_hsv(key);
      }
    }
  }
  cv::cvtColor(debug_image, debug_image, cv::COLOR_HSV2BGR);
  cv::resize(output_image, output_image, image.size(), 0, 0, cv::INTER_NEAREST);
  cv::resize(debug_image, debug_image, image.size(), 0, 0, cv::INTER_NEAREST);

  common::publish_image(*pub_mask_image_, output_image, msg.header.stamp);

  draw_and_publish_image(image, debug_image, msg.header.stamp);
  RCLCPP_INFO_STREAM(get_logger(), "total processing time: " << stop_watch.toc() * 1000 << "[ms]");
}

void GraphSegment::draw_and_publish_image(
  const cv::Mat & raw_image, const cv::Mat & debug_image, const rclcpp::Time & stamp)
{
  cv::Mat show_image;
  cv::addWeighted(raw_image, 0.5, debug_image, 0.8, 1.0, show_image);
  const cv::Size size = debug_image.size();

  // Draw target rectangle
  {
    const int w = target_candidate_box_width_;
    const float r = target_height_ratio_;
    cv::Point2i target(size.width / 2, static_cast<int>(static_cast<float>(size.height) * r));
    cv::Rect2i rect(target + cv::Point2i(-w, -w), target + cv::Point2i(w, w));
    cv::rectangle(show_image, rect, cv::Scalar::all(0), 2);
  }

  common::publish_image(*pub_debug_image_, show_image, stamp);
}

}  // namespace yabloc::graph_segment

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yabloc::graph_segment::GraphSegment)
