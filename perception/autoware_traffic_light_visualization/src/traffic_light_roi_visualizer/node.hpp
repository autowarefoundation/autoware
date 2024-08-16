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
#ifndef TRAFFIC_LIGHT_ROI_VISUALIZER__NODE_HPP_
#define TRAFFIC_LIGHT_ROI_VISUALIZER__NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // for ROS 2 Jazzy or newer
#else
#include <cv_bridge/cv_bridge.h>  // for ROS 2 Humble or older
#endif
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc/imgproc_c.h>

#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
struct ClassificationResult
{
  float prob = 0.0;
  std::string label;
};

/**
 * @brief A struct to represent parsed traffic light shape information.
 */
struct TrafficLightShapeInfo
{
  cv::Scalar color;                 //!< Color associated with "circle".
  std::vector<std::string> shapes;  //!< Shape names.
};

class TrafficLightRoiVisualizerNode : public rclcpp::Node
{
public:
  explicit TrafficLightRoiVisualizerNode(const rclcpp::NodeOptions & options);
  void connectCb();

  void imageRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr &
      input_traffic_signals_msg);

  void imageRoughRoiCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & input_image_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_roi_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & input_tl_rough_roi_msg,
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr &
      input_traffic_signals_msg);

private:
  std::map<int, std::string> state2label_{
    // color
    {tier4_perception_msgs::msg::TrafficLightElement::RED, "red"},
    {tier4_perception_msgs::msg::TrafficLightElement::AMBER, "yellow"},
    {tier4_perception_msgs::msg::TrafficLightElement::GREEN, "green"},
    {tier4_perception_msgs::msg::TrafficLightElement::WHITE, "white"},
    // shape
    {tier4_perception_msgs::msg::TrafficLightElement::CIRCLE, "circle"},
    {tier4_perception_msgs::msg::TrafficLightElement::LEFT_ARROW, "left"},
    {tier4_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW, "right"},
    {tier4_perception_msgs::msg::TrafficLightElement::UP_ARROW, "straight"},
    {tier4_perception_msgs::msg::TrafficLightElement::DOWN_ARROW, "down"},
    {tier4_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW, "straight_left"},
    {tier4_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW, "straight_right"},
    {tier4_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW, "down_left"},
    {tier4_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW, "down_right"},
    {tier4_perception_msgs::msg::TrafficLightElement::CROSS, "cross"},
    // other
    {tier4_perception_msgs::msg::TrafficLightElement::UNKNOWN, "unknown"},
  };

  /**
   * @brief Return RGB color from color string associated with "circle".
   * @param color Color string.
   * @return RGB color.
   */
  static cv::Scalar strToColor(const std::string & color)
  {
    if (color == "red") {
      return {254, 149, 149};
    } else if (color == "yellow") {
      return {254, 250, 149};
    } else if (color == "green") {
      return {149, 254, 161};
    } else {
      return {250, 250, 250};
    }
  }

  /**
   * @brief Extract color and shape names from label.
   * @param label String formatted as `<Color0>-<Shape0>,<Color1>-<Shape1>,...,<ColorN>-<ShapeN>`.
   * @return Extracted information includes a color associated with "circle" and shape names.
   */
  static TrafficLightShapeInfo extractShapeInfo(const std::string & label)
  {
    cv::Scalar color{255, 255, 255};
    std::vector<std::string> shapes;

    std::stringstream ss(label);
    std::string segment;
    while (std::getline(ss, segment, ',')) {
      size_t hyphen_pos = segment.find('-');
      if (hyphen_pos != std::string::npos) {
        auto shape = segment.substr(hyphen_pos + 1);
        if (shape == "circle") {
          const auto color_str = segment.substr(0, hyphen_pos);
          color = strToColor(color_str);
        }
        shapes.emplace_back(shape);
      }
    }
    return {color, shapes};
  }

  bool createRect(
    cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoi & tl_roi,
    const cv::Scalar & color);

  bool createRect(
    cv::Mat & image, const tier4_perception_msgs::msg::TrafficLightRoi & tl_roi,
    const ClassificationResult & result);

  bool getClassificationResult(
    int id, const tier4_perception_msgs::msg::TrafficLightArray & traffic_signals,
    ClassificationResult & result);

  bool getRoiFromId(
    int id, const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & rois,
    tier4_perception_msgs::msg::TrafficLightRoi & correspond_roi);

  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray> roi_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightRoiArray> rough_roi_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::TrafficLightArray> traffic_signals_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr simple_image_pub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficLightRoiArray,
    tier4_perception_msgs::msg::TrafficLightArray>
    SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::TrafficLightRoiArray,
    tier4_perception_msgs::msg::TrafficLightRoiArray, tier4_perception_msgs::msg::TrafficLightArray>
    SyncPolicyWithRoughRoi;
  typedef message_filters::Synchronizer<SyncPolicyWithRoughRoi> SyncWithRoughRoi;
  std::shared_ptr<SyncWithRoughRoi> sync_with_rough_roi_;

  bool enable_fine_detection_;
  bool use_image_transport_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_ROI_VISUALIZER__NODE_HPP_
