
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

#ifndef RADAR_OBJECT_FUSION_TO_DETECTED_OBJECT__RADAR_OBJECT_FUSION_TO_DETECTED_OBJECT_NODE_HPP_
#define RADAR_OBJECT_FUSION_TO_DETECTED_OBJECT__RADAR_OBJECT_FUSION_TO_DETECTED_OBJECT_NODE_HPP_

#include "radar_fusion_to_detected_object.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_perception_msgs/msg/detected_objects.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_fusion_to_detected_object
{
using autoware_auto_perception_msgs::msg::DetectedObject;
using autoware_auto_perception_msgs::msg::DetectedObjects;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjects;

class RadarObjectFusionToDetectedObjectNode : public rclcpp::Node
{
public:
  explicit RadarObjectFusionToDetectedObjectNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
  };

private:
  // Subscriber
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_object_{};
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_radar_{};

  // Callback
  void onDetectedObjects(const DetectedObjects::ConstSharedPtr msg);
  void onRadarObjects(const TrackedObjects::ConstSharedPtr msg);

  // Data Buffer
  DetectedObjects::ConstSharedPtr detected_objects_{};
  TrackedObjects::ConstSharedPtr radar_objects_{};

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  bool isDataReady();
  void onTimer();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  RadarFusionToDetectedObject::Input input_{};
  RadarFusionToDetectedObject::Output output_{};
  RadarFusionToDetectedObject::Param core_param_{};
  std::unique_ptr<RadarFusionToDetectedObject> radar_fusion_to_detected_object_{};

  // Lapper
  RadarFusionToDetectedObject::RadarInput setRadarInput(
    const TrackedObject & radar_object, const std_msgs::msg::Header & header_);
};

}  // namespace radar_fusion_to_detected_object

#endif  // RADAR_OBJECT_FUSION_TO_DETECTED_OBJECT__RADAR_OBJECT_FUSION_TO_DETECTED_OBJECT_NODE_HPP_
