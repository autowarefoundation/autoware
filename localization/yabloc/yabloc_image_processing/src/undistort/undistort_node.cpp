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

#include <autoware/universe_utils/system/stop_watch.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yabloc_common/cv_decompress.hpp>
#include <yabloc_common/pub_sub.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif

#include <optional>

namespace yabloc::undistort
{
class UndistortNode : public rclcpp::Node
{
public:
  using CompressedImage = sensor_msgs::msg::CompressedImage;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Image = sensor_msgs::msg::Image;

  explicit UndistortNode(const rclcpp::NodeOptions & options)
  : Node("undistort", options),
    OUTPUT_WIDTH_(static_cast<int>(declare_parameter<int>("width"))),
    OVERRIDE_FRAME_ID_(declare_parameter<std::string>("override_frame_id"))
  {
    using std::placeholders::_1;

    rclcpp::QoS qos{10};
    if (declare_parameter<bool>("use_sensor_qos")) {
      qos = rclcpp::QoS(10).durability_volatile().best_effort();
    }

    sub_image_ = create_subscription<Image>(
      "~/input/image_raw", qos, std::bind(&UndistortNode::on_image, this, _1));
    sub_compressed_image_ = create_subscription<CompressedImage>(
      "~/input/image_raw/compressed", qos,
      std::bind(&UndistortNode::on_compressed_image, this, _1));
    sub_info_ = create_subscription<CameraInfo>(
      "~/input/camera_info", qos, std::bind(&UndistortNode::on_info, this, _1));

    pub_info_ = create_publisher<CameraInfo>("~/output/resized_info", 10);
    pub_image_ = create_publisher<Image>("~/output/resized_image", 10);
  }

private:
  const int OUTPUT_WIDTH_;
  const std::string OVERRIDE_FRAME_ID_;

  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CompressedImage>::SharedPtr sub_compressed_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<CameraInfo>::SharedPtr pub_info_;
  std::optional<CameraInfo> info_{std::nullopt};
  std::optional<CameraInfo> scaled_info_{std::nullopt};

  cv::Mat undistort_map_x_, undistort_map_y_;

  void make_remap_lut()
  {
    if (!info_.has_value()) return;
    cv::Mat k = cv::Mat(cv::Size(3, 3), CV_64FC1, info_->k.data());
    cv::Mat d = cv::Mat(cv::Size(5, 1), CV_64FC1, info_->d.data());
    cv::Size size(static_cast<int>(info_->width), static_cast<int>(info_->height));

    cv::Size new_size = size;
    if (OUTPUT_WIDTH_ > 0)
      // set new_size along with aspect ratio
      new_size = cv::Size(
        OUTPUT_WIDTH_, static_cast<int>(
                         static_cast<float>(OUTPUT_WIDTH_) *
                         (static_cast<float>(size.height) / static_cast<float>(size.width))));

    cv::Mat new_k = cv::getOptimalNewCameraMatrix(k, d, size, 0, new_size);

    cv::initUndistortRectifyMap(
      k, d, cv::Mat(), new_k, new_size, CV_32FC1, undistort_map_x_, undistort_map_y_);

    scaled_info_ = sensor_msgs::msg::CameraInfo{};
    scaled_info_->k.at(0) = new_k.at<double>(0, 0);
    scaled_info_->k.at(2) = new_k.at<double>(0, 2);
    scaled_info_->k.at(4) = new_k.at<double>(1, 1);
    scaled_info_->k.at(5) = new_k.at<double>(1, 2);
    scaled_info_->k.at(8) = 1;
    scaled_info_->d.resize(5);
    scaled_info_->width = new_size.width;
    scaled_info_->height = new_size.height;
  }

  void remap_and_publish(const cv::Mat & image, const std_msgs::msg::Header & header)
  {
    cv::Mat undistorted_image;
    cv::remap(image, undistorted_image, undistort_map_x_, undistort_map_y_, cv::INTER_LINEAR);

    // Publish CameraInfo
    {
      scaled_info_->header = info_->header;
      if (!OVERRIDE_FRAME_ID_.empty()) scaled_info_->header.frame_id = OVERRIDE_FRAME_ID_;
      pub_info_->publish(scaled_info_.value());
    }

    // Publish Image
    {
      cv_bridge::CvImage bridge;
      bridge.header.stamp = header.stamp;
      if (!OVERRIDE_FRAME_ID_.empty())
        bridge.header.frame_id = OVERRIDE_FRAME_ID_;
      else
        bridge.header.frame_id = header.frame_id;
      bridge.encoding = "bgr8";
      bridge.image = undistorted_image;
      pub_image_->publish(*bridge.toImageMsg());
    }
  }

  void on_image(const Image & msg)
  {
    if (!info_.has_value()) {
      return;
    }
    if (undistort_map_x_.empty()) {
      make_remap_lut();
    }

    // To remove redundant decompression, deactivate compressed image subscriber
    if (sub_compressed_image_) {
      sub_compressed_image_.reset();
    }

    autoware::universe_utils::StopWatch stop_watch;
    remap_and_publish(common::decompress_to_cv_mat(msg), msg.header);
    RCLCPP_INFO_STREAM(get_logger(), "image undistort: " << stop_watch.toc() << "[ms]");
  }

  void on_compressed_image(const CompressedImage & msg)
  {
    if (!info_.has_value()) {
      return;
    }
    if (undistort_map_x_.empty()) {
      make_remap_lut();
    }

    autoware::universe_utils::StopWatch stop_watch;
    remap_and_publish(common::decompress_to_cv_mat(msg), msg.header);
    RCLCPP_INFO_STREAM(get_logger(), "image undistort: " << stop_watch.toc() << "[ms]");
  }

  void on_info(const CameraInfo & msg) { info_ = msg; }
};
}  // namespace yabloc::undistort

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(yabloc::undistort::UndistortNode)
