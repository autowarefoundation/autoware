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

#ifndef IMAGE_TRANSPORT_DECOMPRESSOR__IMAGE_TRANSPORT_DECOMPRESSOR_HPP_
#define IMAGE_TRANSPORT_DECOMPRESSOR__IMAGE_TRANSPORT_DECOMPRESSOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <memory>
#include <string>
#include <utility>

namespace image_preprocessor
{
class ImageTransportDecompressor : public rclcpp::Node
{
public:
  explicit ImageTransportDecompressor(const rclcpp::NodeOptions & node_options);

private:
  void onCompressedImage(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr input_compressed_image_msg);

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_image_pub_;
  std::string encoding_;
};

}  // namespace image_preprocessor

#endif  // IMAGE_TRANSPORT_DECOMPRESSOR__IMAGE_TRANSPORT_DECOMPRESSOR_HPP_
