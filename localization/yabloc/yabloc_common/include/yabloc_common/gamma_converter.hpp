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

#ifndef YABLOC_COMMON__GAMMA_CONVERTER_HPP_
#define YABLOC_COMMON__GAMMA_CONVERTER_HPP_
#include <opencv4/opencv2/core.hpp>

namespace yabloc::common
{
class GammaConverter
{
public:
  explicit GammaConverter(float gamma = 1.0f) { reset(gamma); }

  void reset(float gamma)
  {
    lut_ = cv::Mat(1, 256, CV_8UC1);
    for (int i = 0; i < 256; i++) {
      lut_.at<uchar>(0, i) = 256 * std::pow(i / 256.f, gamma);
    }
  }

  cv::Mat operator()(const cv::Mat & src_image) const
  {
    cv::Mat dst_image;
    cv::LUT(src_image, lut_, dst_image);
    return dst_image;
  }

private:
  cv::Mat lut_;
};
}  // namespace yabloc::common

#endif  // YABLOC_COMMON__GAMMA_CONVERTER_HPP_
