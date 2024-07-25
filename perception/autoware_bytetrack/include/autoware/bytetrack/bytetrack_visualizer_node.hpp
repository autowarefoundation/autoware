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

#ifndef AUTOWARE__BYTETRACK__BYTETRACK_VISUALIZER_NODE_HPP_
#define AUTOWARE__BYTETRACK__BYTETRACK_VISUALIZER_NODE_HPP_

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_perception_msgs/msg/dynamic_object_array.hpp>

#include <boost/uuid/uuid.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::bytetrack
{
// A helper class to generate bright color instance
class ColorMapper
{
public:
  const size_t kColorNum = 512;
  ColorMapper()
  {
    // generate bright color map
    cv::Mat src = cv::Mat::zeros(cv::Size(kColorNum, 1), CV_8UC1);
    for (size_t i = 0; i < kColorNum; i++) {
      src.at<unsigned char>(0, i) = i;
    }
    cv::applyColorMap(src, color_table_, cv::COLORMAP_HSV);
  }

  cv::Scalar operator()(size_t idx)
  {
    if (kColorNum <= idx) {
      throw std::runtime_error("idx should be between [0, 255]");
    }
    return color_table_.at<cv::Vec3b>(0, idx);
  }

protected:
  cv::Mat color_table_;
};

class ByteTrackVisualizerNode : public rclcpp::Node
{
public:
  explicit ByteTrackVisualizerNode(const rclcpp::NodeOptions & node_options);
  ~ByteTrackVisualizerNode();

protected:
  void on_timer();
  bool get_topic_qos(const std::string & query_topic, rclcpp::QoS & qos);

  void callback(
    const sensor_msgs::msg::Image::SharedPtr & image_msg,
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr & rect_msg,
    const tier4_perception_msgs::msg::DynamicObjectArray::SharedPtr & uuid_msg);
  void draw(
    cv::Mat & image, const std::vector<cv::Rect> & bboxes,
    const std::vector<boost::uuids::uuid> & uuids);

  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::DetectedObjectsWithFeature> rect_sub_;
  message_filters::Subscriber<tier4_perception_msgs::msg::DynamicObjectArray> uuid_sub_;

  using ExactTimeSyncPolicy = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image, tier4_perception_msgs::msg::DetectedObjectsWithFeature,
    tier4_perception_msgs::msg::DynamicObjectArray>;
  using ExactTimeSync = message_filters::Synchronizer<ExactTimeSyncPolicy>;
  std::shared_ptr<ExactTimeSync> sync_ptr_;

  image_transport::Publisher image_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool use_raw_;
  ColorMapper color_map_;
};
}  // namespace autoware::bytetrack

#endif  // AUTOWARE__BYTETRACK__BYTETRACK_VISUALIZER_NODE_HPP_
