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

#ifndef YABLOC_COMMON__CV_DECOMPRESS_HPP_
#define YABLOC_COMMON__CV_DECOMPRESS_HPP_

#include <opencv4/opencv2/core.hpp>

#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>

namespace yabloc::common
{
cv::Mat decompress_to_cv_mat(const sensor_msgs::msg::Image & img);

sensor_msgs::msg::Image::ConstSharedPtr decompress_to_ros_msg(
  const sensor_msgs::msg::CompressedImage & compressed_img, const std::string & encoding = "bgr8");

cv::Mat decompress_to_cv_mat(const sensor_msgs::msg::CompressedImage & compressed_img);

}  // namespace yabloc::common

#endif  // YABLOC_COMMON__CV_DECOMPRESS_HPP_
