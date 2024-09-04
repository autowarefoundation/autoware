// Copyright 2024 TIER IV, Inc.
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

#include "perception_utils/run_length_encoder.hpp"

namespace perception_utils
{

std::vector<std::pair<uint8_t, int>> runLengthEncoder(const cv::Mat & image)
{
  std::vector<std::pair<uint8_t, int>> compressed_data;
  const int rows = image.rows;
  const int cols = image.cols;
  compressed_data.emplace_back(image.at<uint8_t>(0, 0), 0);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      uint8_t current_value = image.at<uint8_t>(i, j);
      if (compressed_data.back().first == current_value) {
        ++compressed_data.back().second;
      } else {
        compressed_data.emplace_back(current_value, 1);
      }
    }
  }
  return compressed_data;
}

cv::Mat runLengthDecoder(const std::vector<uint8_t> & rle_data, const int rows, const int cols)
{
  cv::Mat mask(rows, cols, CV_8UC1, cv::Scalar(0));
  int idx = 0;
  int step = sizeof(uint8_t) + sizeof(int);
  for (size_t i = 0; i < rle_data.size(); i += step) {
    uint8_t value;
    int length;
    std::memcpy(&value, &rle_data[i], sizeof(uint8_t));
    std::memcpy(
      &length, &rle_data[i + 1],
      sizeof(
        int));  // under the condition that we know rle_data[i] only consume 1 element of the vector
    for (int j = 0; j < length; ++j) {
      int row_idx = static_cast<int>(idx / cols);
      int col_idx = static_cast<int>(idx % cols);
      mask.at<uint8_t>(row_idx, col_idx) = value;
      idx++;
      if (idx > rows * cols) {
        break;
      }
    }
  }
  return mask;
}

}  // namespace perception_utils
