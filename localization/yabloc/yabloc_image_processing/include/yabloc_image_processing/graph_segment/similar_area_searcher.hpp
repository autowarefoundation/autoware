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

#ifndef YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__SIMILAR_AREA_SEARCHER_HPP_
#define YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__SIMILAR_AREA_SEARCHER_HPP_

#include <Eigen/Core>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/logger.hpp>

#include <set>

namespace yabloc::graph_segment
{
// This class takes a region-segmented image and an index referring to one of those
// regions, and returns region with the same color as the specified region. For each region, it
// computes the RGB histogram and determines the similarity of regions based on the distance between
// their histograms.
//
// The class only has one parameter `similarity_score_threshold`, which is the threshold of
// similarity used to determine if two histograms are considered a match. It takes a
// value between 0 and 1. A value closer to 0 is more likely to be being considered a match.
class SimilarAreaSearcher
{
public:
  explicit SimilarAreaSearcher(float similarity_score_threshold)
  : similarity_score_threshold_(similarity_score_threshold),
    logger_(rclcpp::get_logger("similar_area_searcher"))
  {
  }

  std::set<int> search(
    const cv::Mat & rgb_image, const cv::Mat & segmented, int best_road_like_class);

private:
  const float similarity_score_threshold_;
  rclcpp::Logger logger_;
};
}  // namespace yabloc::graph_segment

#endif  // YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__SIMILAR_AREA_SEARCHER_HPP_
