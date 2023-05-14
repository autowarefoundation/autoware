// Copyright 2022 Tier IV, Inc.
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

#ifndef TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_
#define TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tensorrt_yolox/tensorrt_yolox.hpp>

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
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace tensorrt_yolox
{
using LabelMap = std::map<int, std::string>;

class TrtYoloXNode : public rclcpp::Node
{
public:
  explicit TrtYoloXNode(const rclcpp::NodeOptions & node_options);

private:
  void onConnect();
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  bool readLabelFile(const std::string & label_path);
  void replaceLabelMap();

  image_transport::Publisher image_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr objects_pub_;

  image_transport::Subscriber image_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  LabelMap label_map_;
  std::unique_ptr<tensorrt_yolox::TrtYoloX> trt_yolox_;
};

}  // namespace tensorrt_yolox

#endif  // TENSORRT_YOLOX__TENSORRT_YOLOX_NODE_HPP_
