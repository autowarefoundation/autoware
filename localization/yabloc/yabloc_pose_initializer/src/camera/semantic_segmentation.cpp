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

#include "yabloc_pose_initializer/camera/semantic_segmentation.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <iostream>
#include <vector>

struct SemanticSegmentation::Impl
{
  cv::dnn::Net net;
};

SemanticSegmentation::SemanticSegmentation(const std::string & model_path)
{
  impl_ = std::make_shared<Impl>();
  impl_->net = cv::dnn::readNet(model_path);
  impl_->net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  impl_->net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

cv::Mat SemanticSegmentation::make_blob(const cv::Mat & image)
{
  double scale = 1.0;
  cv::Size size = cv::Size(896, 512);
  cv::Scalar mean = cv::Scalar(0, 0, 0);
  bool swap = true;
  bool crop = false;
  return cv::dnn::blobFromImage(image, scale, size, mean, swap, crop, CV_32F);
}

cv::Mat SemanticSegmentation::convert_blob_to_image(const cv::Mat & blob)
{
  if (blob.size.dims() != 4) {
    throw std::runtime_error("blob has invalid size");
  }
  const int channels = blob.size[1];
  const int height = blob.size[2];
  const int width = blob.size[3];
  cv::Mat image = cv::Mat(height, width, CV_32FC4);

  for (int h = 0; h < height; ++h) {
    for (int w = 0; w < width; ++w) {
      cv::Vec4f vec4f;
      for (int c = 0; c < channels; ++c) {
        vec4f[c] = blob.at<float>(c * height * width + h * width + w);
      }
      image.at<cv::Vec4f>(h, w) = vec4f;
    }
  }
  return image;
}

cv::Mat SemanticSegmentation::inference(const cv::Mat & image, double score_threshold)
{
  cv::Mat blob = make_blob(image);
  impl_->net.setInput(blob);
  std::vector<std::string> output_layers = impl_->net.getUnconnectedOutLayersNames();
  std::vector<cv::Mat> masks;
  impl_->net.forward(masks, output_layers);

  cv::Mat mask = masks[0];
  cv::Mat output = convert_blob_to_image(mask);

  cv::resize(output, output, cv::Size(image.cols, image.rows), 0, 0, cv::INTER_LINEAR);

  return normalize(output, score_threshold);
}

cv::Mat SemanticSegmentation::normalize(const cv::Mat & mask, double score_threshold)
{
  std::vector<cv::Mat> masks;
  cv::split(mask, masks);
  std::vector<cv::Mat> bin_masks;

  for (size_t i = 1; i < masks.size(); ++i) {
    cv::Mat bin_mask;
    cv::threshold(masks[i], bin_mask, score_threshold, 255, cv::THRESH_BINARY_INV);
    bin_mask.convertTo(bin_mask, CV_8UC1);
    bin_masks.push_back(255 - bin_mask);
  }

  cv::Mat result;
  cv::merge(bin_masks, result);
  return result;
}

cv::Mat SemanticSegmentation::draw_overlay(const cv::Mat & image, const cv::Mat & segmentation)
{
  cv::Mat overlay_image = image.clone();
  return overlay_image * 0.5 + segmentation * 0.5;
}

void SemanticSegmentation::print_error_message(const rclcpp::Logger & logger)
{
  const std::string ERROR_MESSAGE =
    R"(The yabloc_pose_initializer is not working correctly because the DNN model has not been downloaded correctly.
Please check the README of yabloc_pose_initializer to know how download models.)";

  std::istringstream stream(ERROR_MESSAGE);
  std::string line;
  while (std::getline(stream, line)) {
    RCLCPP_ERROR_STREAM(logger, line);
  }
}
