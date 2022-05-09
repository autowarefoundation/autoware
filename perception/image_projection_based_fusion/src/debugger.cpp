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

#include "image_projection_based_fusion/debugger.hpp"

#include <cv_bridge/cv_bridge.h>

namespace
{

void drawRoiOnImage(
  const cv::Mat & image, const sensor_msgs::msg::RegionOfInterest & roi, const int width,
  const int height, const cv::Scalar & color)
{
  const auto left = std::max(0, static_cast<int>(roi.x_offset));
  const auto top = std::max(0, static_cast<int>(roi.y_offset));
  const auto right = std::min(static_cast<int>(roi.x_offset + roi.width), width);
  const auto bottom = std::min(static_cast<int>(roi.y_offset + roi.height), height);
  cv::rectangle(image, cv::Point(left, top), cv::Point(right, bottom), color, 2);
}

}  // namespace

namespace image_projection_based_fusion
{
Debugger::Debugger(
  rclcpp::Node * node_ptr, const std::size_t image_num, const std::size_t image_buffer_size)
: node_ptr_(node_ptr)
{
  image_buffers_.resize(image_num);
  image_buffer_size_ = image_buffer_size;
  for (std::size_t img_i = 0; img_i < image_num; ++img_i) {
    auto sub = image_transport::create_subscription(
      node_ptr, "input/image_raw" + std::to_string(img_i),
      std::bind(&Debugger::imageCallback, this, std::placeholders::_1, img_i), "raw",
      rmw_qos_profile_sensor_data);
    image_subs_.push_back(sub);

    std::vector<std::string> node_params = {"format", "jpeg_quality", "png_level"};
    for (const auto & param : node_params) {
      if (node_ptr->has_parameter(param)) {
        node_ptr->undeclare_parameter(param);
      }
    }

    auto pub =
      image_transport::create_publisher(node_ptr, "output/image_raw" + std::to_string(img_i));
    image_pubs_.push_back(pub);
    image_buffers_.at(img_i).set_capacity(image_buffer_size_);
  }
}

void Debugger::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr input_image_msg, const std::size_t image_id)
{
  image_buffers_.at(image_id).push_front(input_image_msg);
}

void Debugger::clear()
{
  image_rois_.clear();
  obstacle_rois_.clear();
  obstacle_points_.clear();
}

void Debugger::publishImage(const std::size_t image_id, const rclcpp::Time & stamp)
{
  const boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> & image_buffer =
    image_buffers_.at(image_id);
  const image_transport::Publisher & image_pub = image_pubs_.at(image_id);

  for (std::size_t i = 0; i < image_buffer.size(); ++i) {
    if (image_buffer.at(i)->header.stamp != stamp) {
      continue;
    }

    auto cv_ptr = cv_bridge::toCvCopy(image_buffer.at(i), image_buffer.at(i)->encoding);

    for (const auto & point : obstacle_points_) {
      cv::circle(
        cv_ptr->image, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())), 2,
        cv::Scalar(255, 255, 255), 3, 4);
    }
    for (const auto & roi : obstacle_rois_) {
      drawRoiOnImage(
        cv_ptr->image, roi, image_buffer.at(i)->width, image_buffer.at(i)->height,
        cv::Scalar(255, 0, 0));
    }
    // TODO(yukke42): show iou_score on image
    for (const auto & roi : image_rois_) {
      drawRoiOnImage(
        cv_ptr->image, roi, image_buffer.at(i)->width, image_buffer.at(i)->height,
        cv::Scalar(0, 0, 255));
    }

    image_pub.publish(cv_ptr->toImageMsg());
    break;
  }
}

}  // namespace image_projection_based_fusion
