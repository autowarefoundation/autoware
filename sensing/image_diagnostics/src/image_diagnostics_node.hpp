// Copyright 2022 TIER IV, Inc.
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

#ifndef IMAGE_DIAGNOSTICS_NODE_HPP_
#define IMAGE_DIAGNOSTICS_NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_debug_msgs/msg/int32_stamped.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <string>
#include <unordered_map>

namespace autoware::image_diagnostics
{
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;

struct ImageParam
{
  int num_of_regions_normal = 0;
  int num_of_regions_dark = 0;
  int num_of_regions_blockage = 0;
  int num_of_regions_low_visibility = 0;
  int num_of_regions_backlight = 0;

  int image_resize_height = 480;
  int diagnostic_status = -1;
  int number_block_horizontal = 5;
  int number_block_vertical = 5;

  int dark_regions_num_warn_thresh = 10;
  int blockage_region_num_warn_thresh = 3;
  int lowVis_region_num_warn_thresh = 2;
  int backlight_region_num_warn_thresh = 2;

  int dark_regions_num_error_thresh = 20;
  int blockage_region_num_error_thresh = 5;
  int lowVis_region_num_error_thresh = 4;
  int backlight_region_num_error_thresh = 3;

  float blockage_ratio_thresh = 0.9f;
  int blockage_intensity_thresh = 10;
  float blockage_freq_ratio_thresh = 0.3f;

  int dark_intensity_thresh = 10;
  float lowVis_freq_thresh = 4.0f;
  int backlight_intensity_thresh = 230;
};

enum Image_State : uint8_t { NORMAL = 0, DARK, BLOCKAGE, LOW_VIS, BACKLIGHT };
std::unordered_map<std::string, cv::Scalar> state_color_map_ = {
  {"NORMAL", cv::Scalar(100, 100, 100)},  {"DARK", cv::Scalar(0, 0, 0)},
  {"BLOCKAGE", cv::Scalar(0, 0, 200)},    {"LOW_VIS", cv::Scalar(0, 200, 200)},
  {"BACKLIGHT", cv::Scalar(200, 0, 200)}, {"BORDER", cv::Scalar(255, 255, 255)}};

class ImageDiagNode : public rclcpp::Node
{
private:
  void ImageChecker(const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg);
  void shiftImage(cv::Mat & img);
  void onImageDiagChecker(DiagnosticStatusWrapper & stat);
  ImageParam params_;
  Updater updater_{this};

public:
  explicit ImageDiagNode(const rclcpp::NodeOptions & node_options);

protected:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  image_transport::Publisher block_diag_image_pub_;
  image_transport::Publisher dft_image_pub_;
  image_transport::Publisher gray_image_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32MultiArrayStamped>::SharedPtr average_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Int32Stamped>::SharedPtr image_state_pub_;
};

}  // namespace autoware::image_diagnostics

#endif  // IMAGE_DIAGNOSTICS_NODE_HPP_
