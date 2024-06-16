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

#ifndef YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__HISTOGRAM_HPP_
#define YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__HISTOGRAM_HPP_

#include <Eigen/Core>
#include <opencv4/opencv2/core.hpp>

#include <algorithm>

namespace yabloc::graph_segment
{
struct Histogram
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit Histogram(int bin = 10) : bin(bin) { data = Eigen::MatrixXf::Zero(3, bin); }

  [[nodiscard]] Eigen::MatrixXf eval() const
  {
    float sum = data.sum();
    if (sum < 1e-6f) throw std::runtime_error("invalid division");
    return data / sum;
  }

  void add(const cv::Vec3b & rgb)
  {
    for (int ch = 0; ch < 3; ++ch) {
      int index = std::clamp(
        static_cast<int>(static_cast<float>(rgb[ch]) * static_cast<float>(bin) / 255.f), 0,
        bin - 1);
      data(ch, index) += 1.0f;
    }
  }
  const int bin;
  Eigen::MatrixXf data;

  static float eval_histogram_intersection(const Eigen::MatrixXf & a, const Eigen::MatrixXf & b)
  {
    float score = 0;
    for (int c = 0; c < a.cols(); c++) {
      for (int r = 0; r < a.rows(); r++) {
        score += std::min(a(r, c), b(r, c));
      }
    }
    return score;
  };

  static float eval_bhattacharyya_coeff(const Eigen::MatrixXf & a, const Eigen::MatrixXf & b)
  {
    float score = 0;
    for (int c = 0; c < a.cols(); c++) {
      for (int r = 0; r < a.rows(); r++) {
        score += std::sqrt(a(r, c) * b(r, c));
      }
    }
    return score;
  };
};

}  // namespace yabloc::graph_segment

#endif  // YABLOC_IMAGE_PROCESSING__GRAPH_SEGMENT__HISTOGRAM_HPP_
