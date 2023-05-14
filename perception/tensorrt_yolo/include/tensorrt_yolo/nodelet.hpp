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

#ifndef TENSORRT_YOLO__NODELET_HPP_
#define TENSORRT_YOLO__NODELET_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trt_yolo.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace object_recognition
{
class TensorrtYoloNodelet : public rclcpp::Node
{
public:
  explicit TensorrtYoloNodelet(const rclcpp::NodeOptions & options);
  void connectCb();
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  bool readLabelFile(const std::string & filepath, std::vector<std::string> * labels);

private:
  std::mutex connect_mutex_;

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;

  image_transport::Subscriber image_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  yolo::Config yolo_config_;

  int gpu_device_id_;

  std::vector<std::string> labels_;
  std::unique_ptr<float[]> out_scores_;
  std::unique_ptr<float[]> out_boxes_;
  std::unique_ptr<float[]> out_classes_;
  std::unique_ptr<yolo::Net> net_ptr_;
};

}  // namespace object_recognition

#endif  // TENSORRT_YOLO__NODELET_HPP_
