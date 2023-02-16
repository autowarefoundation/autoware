// Copyright 2023 Tier IV, Inc.
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

#include <rclcpp/rclcpp.hpp>

#if ENABLE_GPU
#include <traffic_light_classifier/cnn_classifier.hpp>
#endif

#include <traffic_light_classifier/color_classifier.hpp>
#include <traffic_light_classifier/nodelet.hpp>

#include <memory>
#include <string>

namespace
{
std::string toString(const uint8_t state)
{
  if (state == autoware_auto_perception_msgs::msg::TrafficLight::RED) {
    return "red";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::AMBER) {
    return "yellow";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::GREEN) {
    return "green";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::WHITE) {
    return "white";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::CIRCLE) {
    return "circle";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::LEFT_ARROW) {
    return "left";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::RIGHT_ARROW) {
    return "right";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::UP_ARROW) {
    return "straight";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::DOWN_ARROW) {
    return "down";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::DOWN_LEFT_ARROW) {
    return "down_left";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::DOWN_RIGHT_ARROW) {
    return "down_right";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::CROSS) {
    return "cross";
  } else if (state == autoware_auto_perception_msgs::msg::TrafficLight::UNKNOWN) {
    return "unknown";
  } else {
    return "";
  }
}
}  // namespace

namespace traffic_light
{
class SingleImageDebugInferenceNode : public rclcpp::Node
{
public:
  explicit SingleImageDebugInferenceNode(const rclcpp::NodeOptions & node_options)
  : Node("single_image_debug_inference", node_options)
  {
    const auto image_path = declare_parameter("image_path", "");

    int classifier_type = this->declare_parameter(
      "classifier_type",
      static_cast<int>(TrafficLightClassifierNodelet::ClassifierType::HSVFilter));
    if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::HSVFilter) {
      classifier_ptr_ = std::make_unique<ColorClassifier>(this);
    } else if (classifier_type == TrafficLightClassifierNodelet::ClassifierType::CNN) {
#if ENABLE_GPU
      classifier_ptr_ = std::make_unique<CNNClassifier>(this);
#else
      RCLCPP_ERROR(get_logger(), "please install CUDA, CUDNN and TensorRT to use cnn classifier");
#endif
    }

    image_ = cv::imread(image_path);
    if (image_.empty()) {
      RCLCPP_ERROR(get_logger(), "image is empty");
      return;
    }
    cv::namedWindow("inference image", cv::WINDOW_NORMAL);
    cv::setMouseCallback("inference image", SingleImageDebugInferenceNode::onMouse, this);

    cv::imshow("inference image", image_);

    // loop until q character is pressed
    while (cv::waitKey(0) != 113) {
    }
    cv::destroyAllWindows();
    rclcpp::shutdown();
  }

private:
  static void onMouse(int event, int x, int y, int flags, void * param)
  {
    SingleImageDebugInferenceNode * node = static_cast<SingleImageDebugInferenceNode *>(param);
    if (node) {
      node->inferWithCrop(event, x, y, flags);
    }
  }

  void inferWithCrop(int action, int x, int y, [[maybe_unused]] int flags)
  {
    if (action == cv::EVENT_LBUTTONDOWN) {
      top_left_corner_ = cv::Point(x, y);
    } else if (action == cv::EVENT_LBUTTONUP) {
      bottom_right_corner_ = cv::Point(x, y);
      cv::Mat tmp = image_.clone();
      cv::Mat crop = image_(cv::Rect{top_left_corner_, bottom_right_corner_}).clone();
      if (crop.empty()) {
        RCLCPP_ERROR(get_logger(), "crop image is empty");
        return;
      }
      cv::cvtColor(crop, crop, cv::COLOR_BGR2RGB);
      autoware_auto_perception_msgs::msg::TrafficSignal traffic_signal;
      if (!classifier_ptr_->getTrafficSignal(crop, traffic_signal)) {
        RCLCPP_ERROR(get_logger(), "failed to classify image");
        return;
      }
      cv::Scalar color;
      cv::Scalar text_color;
      for (const auto & light : traffic_signal.lights) {
        auto color_str = toString(light.color);
        auto shape_str = toString(light.shape);
        auto confidence_str = std::to_string(light.confidence);
        if (shape_str == "circle") {
          if (color_str == "red") {
            color = cv::Scalar(0, 0, 255);
          } else if (color_str == "green") {
            color = cv::Scalar(0, 255, 0);
          } else if (color_str == "yellow") {
            color = cv::Scalar(0, 255, 255);
          } else if (color_str == "white") {
            color = cv::Scalar(0, 0, 0);
          } else {
            color = cv::Scalar(255, 255, 255);
          }
        }
        RCLCPP_INFO_STREAM(get_logger(), color_str << " " << shape_str << " " << confidence_str);
      }
      cv::rectangle(tmp, top_left_corner_, bottom_right_corner_, color, 2, 8);
      cv::imshow("inference image", tmp);
    }
  }

  cv::Point top_left_corner_;
  cv::Point bottom_right_corner_;
  cv::Mat image_;
  std::unique_ptr<ClassifierInterface> classifier_ptr_;
};
}  // namespace traffic_light

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(traffic_light::SingleImageDebugInferenceNode)
