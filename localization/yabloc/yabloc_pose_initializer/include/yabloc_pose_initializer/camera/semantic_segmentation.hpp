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

#ifndef YABLOC_POSE_INITIALIZER__CAMERA__SEMANTIC_SEGMENTATION_HPP_
#define YABLOC_POSE_INITIALIZER__CAMERA__SEMANTIC_SEGMENTATION_HPP_
#include <opencv2/core.hpp>
#include <rclcpp/logger.hpp>

#include <iostream>
#include <memory>
#include <string>

class SemanticSegmentation
{
public:
  explicit SemanticSegmentation(const std::string & model_path);

  cv::Mat inference(const cv::Mat & image, double score_threshold = 0.5);

  static void print_error_message(const rclcpp::Logger & logger);

private:
  static cv::Mat make_blob(const cv::Mat & image);

  static cv::Mat convert_blob_to_image(const cv::Mat & blob);

  static cv::Mat normalize(const cv::Mat & mask, double score_threshold = 0.5);

  static cv::Mat draw_overlay(const cv::Mat & image, const cv::Mat & segmentation);

  struct Impl;
  std::shared_ptr<Impl> impl_{nullptr};
};

#endif  // YABLOC_POSE_INITIALIZER__CAMERA__SEMANTIC_SEGMENTATION_HPP_
