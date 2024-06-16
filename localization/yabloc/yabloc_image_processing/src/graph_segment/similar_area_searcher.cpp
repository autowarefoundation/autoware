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

#include "yabloc_image_processing/graph_segment/similar_area_searcher.hpp"

#include "yabloc_image_processing/graph_segment/histogram.hpp"

#include <rclcpp/logging.hpp>

#include <queue>

namespace yabloc::graph_segment
{
struct KeyAndArea
{
  KeyAndArea(int key, int count) : key(key), count(count) {}
  int key;
  int count;
};

std::set<int> SimilarAreaSearcher::search(
  const cv::Mat & rgb_image, const cv::Mat & segmented, int best_road_like_class)
{
  std::unordered_map<int, Histogram> histogram_map;
  std::unordered_map<int, int> count_map;

  for (int h = 0; h < rgb_image.rows; h++) {
    const int * seg_ptr = segmented.ptr<int>(h);
    const auto * rgb_ptr = rgb_image.ptr<cv::Vec3b>(h);

    for (int w = 0; w < rgb_image.cols; w++) {
      int key = seg_ptr[w];
      cv::Vec3b rgb = rgb_ptr[w];
      if (count_map.count(key) == 0) {
        count_map[key] = 1;
        histogram_map[key].add(rgb);
      } else {
        count_map[key]++;
        histogram_map[key].add(rgb);
      }
    }
  }

  auto compare = [](KeyAndArea a, KeyAndArea b) { return a.count < b.count; };
  std::priority_queue<KeyAndArea, std::vector<KeyAndArea>, decltype(compare)> key_queue{compare};
  for (auto [key, count] : count_map) {
    key_queue.push({key, count});
  }

  const Eigen::MatrixXf ref_histogram = histogram_map.at(best_road_like_class).eval();

  std::stringstream debug_ss;
  debug_ss << "histogram equality ";

  int index = 0;
  std::set<int> acceptable_keys;
  while (!key_queue.empty()) {
    KeyAndArea key = key_queue.top();
    key_queue.pop();

    Eigen::MatrixXf query = histogram_map.at(key.key).eval();
    float score = Histogram::eval_histogram_intersection(ref_histogram, query);
    debug_ss << " " << score;

    if (score > similarity_score_threshold_) acceptable_keys.insert(key.key);
    if (++index > 10) break;
  }
  RCLCPP_INFO_STREAM(logger_, debug_ss.str());

  return acceptable_keys;
}
}  // namespace yabloc::graph_segment
