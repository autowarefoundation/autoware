
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

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
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
  message_filters::Subscriber<DetectedObjects> sub_object_{};
  message_filters::Subscriber<TrackedObjects> sub_radar_{};

  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<DetectedObjects, TrackedObjects>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  typename std::shared_ptr<Sync> sync_ptr_;

  // Callback
  void onData(
    const DetectedObjects::ConstSharedPtr object_msg,
    const TrackedObjects::ConstSharedPtr radar_msg);
  bool isDataReady();

  // Data Buffer
  DetectedObjects::ConstSharedPtr detected_objects_{};
  TrackedObjects::ConstSharedPtr radar_objects_{};

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_debug_low_confidence_objects_{};

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
