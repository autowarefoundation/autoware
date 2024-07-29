// Copyright 2020 Tier IV, Inc.
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

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "autoware/image_transport_decompressor/image_transport_decompressor.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor_msgs/image_encodings.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::image_preprocessor
{
ImageTransportDecompressor::ImageTransportDecompressor(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("image_transport_decompressor", node_options),
  encoding_(declare_parameter<std::string>("encoding"))
{
  compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "~/input/compressed_image", rclcpp::SensorDataQoS(),
    std::bind(&ImageTransportDecompressor::onCompressedImage, this, std::placeholders::_1));
  raw_image_pub_ =
    create_publisher<sensor_msgs::msg::Image>("~/output/raw_image", rclcpp::SensorDataQoS());
}

void ImageTransportDecompressor::onCompressedImage(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg)
{
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  // Copy message header
  cv_ptr->header = input_compressed_image_msg->header;

  // Decode color/mono image
  try {
    cv_ptr->image = cv::imdecode(cv::Mat(input_compressed_image_msg->data), cv::IMREAD_COLOR);

    // Assign image encoding string
    const size_t split_pos = input_compressed_image_msg->format.find(';');
    if (split_pos == std::string::npos) {
      // Older version of compressed_image_transport does not signal image format
      switch (cv_ptr->image.channels()) {
        case 1:
          cv_ptr->encoding = sensor_msgs::image_encodings::MONO8;
          break;
        case 3:
          cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
          break;
        default:
          RCLCPP_ERROR(
            get_logger(), "Unsupported number of channels: %i", cv_ptr->image.channels());
          break;
      }
    } else {
      std::string image_encoding;
      if (encoding_ == std::string("default")) {
        image_encoding = input_compressed_image_msg->format.substr(0, split_pos);
      } else if (encoding_ == std::string("rgb8")) {
        image_encoding = "rgb8";
      } else if (encoding_ == std::string("bgr8")) {
        image_encoding = "bgr8";
      } else {
        image_encoding = input_compressed_image_msg->format.substr(0, split_pos);
      }

      cv_ptr->encoding = image_encoding;

      if (sensor_msgs::image_encodings::isColor(image_encoding)) {
        std::string compressed_encoding = input_compressed_image_msg->format.substr(split_pos);
        bool compressed_bgr_image =
          (compressed_encoding.find("compressed bgr") != std::string::npos);

        // Revert color transformation
        if (compressed_bgr_image) {
          // if necessary convert colors from bgr to rgb
          if (
            (image_encoding == sensor_msgs::image_encodings::RGB8) ||
            (image_encoding == sensor_msgs::image_encodings::RGB16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGB);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
            (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2RGBA);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
            (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2BGRA);
          }
        } else {
          // if necessary convert colors from rgb to bgr
          if (
            (image_encoding == sensor_msgs::image_encodings::BGR8) ||
            (image_encoding == sensor_msgs::image_encodings::BGR16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGR);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::BGRA8) ||
            (image_encoding == sensor_msgs::image_encodings::BGRA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2BGRA);
          }

          if (
            (image_encoding == sensor_msgs::image_encodings::RGBA8) ||
            (image_encoding == sensor_msgs::image_encodings::RGBA16)) {
            cv::cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2RGBA);
          }
        }
      }
    }
  } catch (cv::Exception & e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
  }

  size_t rows = cv_ptr->image.rows;
  size_t cols = cv_ptr->image.cols;

  if ((rows > 0) && (cols > 0)) {
    // Publish message to user callback
    auto image_ptr = std::make_unique<sensor_msgs::msg::Image>(*cv_ptr->toImageMsg());
    raw_image_pub_->publish(std::move(image_ptr));
  }
}
}  // namespace autoware::image_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::image_preprocessor::ImageTransportDecompressor)
