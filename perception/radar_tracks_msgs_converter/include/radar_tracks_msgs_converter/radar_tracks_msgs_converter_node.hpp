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

#ifndef RADAR_TRACKS_MSGS_CONVERTER__RADAR_TRACKS_MSGS_CONVERTER_NODE_HPP_
#define RADAR_TRACKS_MSGS_CONVERTER__RADAR_TRACKS_MSGS_CONVERTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/shape.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace radar_tracks_msgs_converter
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjectKinematics;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using nav_msgs::msg::Odometry;
using radar_msgs::msg::RadarTracks;

class RadarTracksMsgsConverterNode : public rclcpp::Node
{
public:
  explicit RadarTracksMsgsConverterNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double update_rate_hz{};
    std::string new_frame_id{};
    bool use_twist_compensation{};
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarTracks>::SharedPtr sub_radar_{};
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_{};
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;

  // Callback
  void onRadarTracks(const RadarTracks::ConstSharedPtr msg);
  void onTwist(const Odometry::ConstSharedPtr msg);

  // Data Buffer
  RadarTracks::ConstSharedPtr radar_data_{};
  Odometry::ConstSharedPtr odometry_data_{};
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  // Publisher
  rclcpp::Publisher<TrackedObjects>::SharedPtr pub_radar_{};

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
  TrackedObjects convertRadarTrackToTrackedObjects();
  uint8_t convertClassification(const uint16_t classification);
};

}  // namespace radar_tracks_msgs_converter

#endif  // RADAR_TRACKS_MSGS_CONVERTER__RADAR_TRACKS_MSGS_CONVERTER_NODE_HPP_
