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

#include "autoware/image_projection_based_fusion/debugger.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

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

namespace autoware::image_projection_based_fusion
{
Debugger::Debugger(
  rclcpp::Node * node_ptr, const std::size_t image_num, const std::size_t image_buffer_size,
  std::vector<std::string> input_camera_topics)
: node_ptr_(node_ptr), input_camera_topics_{input_camera_topics}
{
  image_buffers_.resize(image_num);
  image_buffer_size_ = image_buffer_size;
  for (std::size_t img_i = 0; img_i < image_num; ++img_i) {
    auto sub = image_transport::create_subscription(
      node_ptr, input_camera_topics.at(img_i),
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
      image_transport::create_publisher(node_ptr, "~/debug/image_raw" + std::to_string(img_i));
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
  max_iou_for_image_rois_.clear();
}

void Debugger::publishImage(const std::size_t image_id, const rclcpp::Time & stamp)
{
  const boost::circular_buffer<sensor_msgs::msg::Image::ConstSharedPtr> & image_buffer =
    image_buffers_.at(image_id);
  const image_transport::Publisher & image_pub = image_pubs_.at(image_id);
  const bool draw_iou_score =
    max_iou_for_image_rois_.size() > 0 && max_iou_for_image_rois_.size() == image_rois_.size();

  for (std::size_t i = 0; i < image_buffer.size(); ++i) {
    if (image_buffer.at(i)->header.stamp != stamp) {
      continue;
    }

    auto cv_ptr = cv_bridge::toCvCopy(image_buffer.at(i), image_buffer.at(i)->encoding);
    // draw obstacle points
    for (const auto & point : obstacle_points_) {
      cv::circle(
        cv_ptr->image, cv::Point(static_cast<int>(point.x()), static_cast<int>(point.y())), 2,
        cv::Scalar(255, 255, 255), 3, 4);
    }

    // draw rois
    const int img_height = static_cast<int>(image_buffer.at(i)->height);
    const int img_width = static_cast<int>(image_buffer.at(i)->width);
    for (const auto & roi : obstacle_rois_) {
      drawRoiOnImage(cv_ptr->image, roi, img_width, img_height, cv::Scalar(255, 0, 0));  // blue
    }
    for (const auto & roi : image_rois_) {
      drawRoiOnImage(cv_ptr->image, roi, img_width, img_height, cv::Scalar(0, 0, 255));  // red
    }

    // show iou_score on image
    if (draw_iou_score) {
      for (auto roi_index = 0; roi_index < static_cast<int>(image_rois_.size()); ++roi_index) {
        std::stringstream stream;
        stream << std::fixed << std::setprecision(2) << max_iou_for_image_rois_.at(roi_index);
        std::string iou_score = stream.str();

        // set text position
        int baseline = 3;
        cv::Size textSize = cv::getTextSize(iou_score, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        const int text_height = static_cast<int>(textSize.height);
        const int text_width = static_cast<int>(textSize.width);
        int x = image_rois_.at(roi_index).x_offset;
        int y = image_rois_.at(roi_index).y_offset;  // offset for text
        if (y < 0 + text_height)
          y = text_height;  // if roi is on the top of image, put text on lower left of roi
        if (y > img_height - text_height)
          y = img_height -
              text_height;  // if roi is on the bottom of image, put text on upper left of roi
        if (x > img_width - text_width)
          x = img_width - text_width;  // if roi is on the right of image, put text on left of roi
        if (x < 0) x = 0;              // if roi is on the left of image, put text on right of roi

        // choice color by iou score
        // cv::Scalar color = max_iou_for_image_rois_.at(i) > 0.5 ? cv::Scalar(0, 255, 0) :
        // cv::Scalar(0, 0, 255);
        cv::Scalar color = cv::Scalar(0, 0, 255);  // red

        cv::rectangle(
          cv_ptr->image, cv::Point(x, y - textSize.height - baseline),
          cv::Point(x + textSize.width, y), cv::Scalar(255, 255, 255),
          cv::FILLED);  // white background
        cv::putText(
          cv_ptr->image, iou_score, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
          cv::LINE_AA);  // text
      }
    }

    image_pub.publish(cv_ptr->toImageMsg());
    break;
  }
}

}  // namespace autoware::image_projection_based_fusion
