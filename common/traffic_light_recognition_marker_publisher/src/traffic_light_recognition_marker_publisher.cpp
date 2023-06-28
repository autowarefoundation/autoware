// Copyright 2021 Tier IV, Inc.
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

#include "traffic_light_recognition_marker_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

TrafficLightRecognitionMarkerPublisher::TrafficLightRecognitionMarkerPublisher(
  const rclcpp::NodeOptions & options)
: Node("traffic_light_recognition_marker_publisher", options)
{
  using std::placeholders::_1;

  sub_map_ptr_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightRecognitionMarkerPublisher::onMap, this, _1));
  sub_tlr_ptr_ = this->create_subscription<autoware_auto_perception_msgs::msg::TrafficSignalArray>(
    "~/input/traffic_signals", rclcpp::QoS{1},
    std::bind(&TrafficLightRecognitionMarkerPublisher::onTrafficSignalArray, this, _1));
  pub_marker_ptr_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/output/marker", rclcpp::QoS{1});
}

void TrafficLightRecognitionMarkerPublisher::onMap(const HADMapBin::ConstSharedPtr msg_ptr)
{
  is_map_ready_ = false;
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg_ptr, viz_lanelet_map);
  lanelet::ConstLanelets lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  std::vector<lanelet::AutowareTrafficLightConstPtr> tl_regulatory_elements =
    lanelet::utils::query::autowareTrafficLights(lanelets);
  for (const auto & tl : tl_regulatory_elements) {
    const auto tl_lights = tl->trafficLights();
    for (const auto & tl_light : tl_lights) {
      if (tl_light.isLineString()) {
        lanelet::ConstLineString3d tl_linestring =
          static_cast<lanelet::ConstLineString3d>(tl_light);
        int32_t tl_id = static_cast<int32_t>(tl_light.id());
        Pose tl_pose;
        tl_pose.position.x = (tl_linestring.front().x() + tl_linestring.back().x()) / 2;
        tl_pose.position.y = (tl_linestring.front().y() + tl_linestring.back().y()) / 2;
        tl_pose.position.z = tl_linestring.front().z() + 1.0;
        tl_position_map_[tl_id] = tl_pose;
      }
    }
  }
  is_map_ready_ = true;
}

void TrafficLightRecognitionMarkerPublisher::onTrafficSignalArray(
  const TrafficSignalArray::ConstSharedPtr msg_ptr)
{
  if (!is_map_ready_) {
    return;
  }
  MarkerArray markers;
  marker_id = 0;
  for (const auto & tl : msg_ptr->signals) {
    if (tl_position_map_.count(tl.map_primitive_id) == 0) continue;
    auto tl_position = tl_position_map_[tl.map_primitive_id];
    for (const auto tl_light : tl.lights) {
      const auto marker = getTrafficLightMarker(tl_position, tl_light.color, tl_light.shape);
      markers.markers.emplace_back(marker);
      marker_id++;
      tl_position.position.z += 0.5;
    }
  }
  pub_marker_ptr_->publish(markers);
}

visualization_msgs::msg::Marker TrafficLightRecognitionMarkerPublisher::getTrafficLightMarker(
  const Pose & tl_pose, const uint8_t tl_color, const uint8_t tl_shape)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = rclcpp::Time();
  marker.ns = "traffic_light_color";
  marker.id = marker_id;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = tl_pose;
  marker.color = getTrafficLightColor(tl_color, tl_shape);
  marker.scale.z = 0.5;
  marker.frame_locked = false;
  marker.text = getTrafficLightString(tl_color, tl_shape);

  return marker;
}

std::string TrafficLightRecognitionMarkerPublisher::getTrafficLightString(
  const uint8_t tl_color, const uint8_t tl_shape)
{
  if (tl_shape == TrafficLight::LEFT_ARROW) return "LEFT_ARROW";
  if (tl_shape == TrafficLight::RIGHT_ARROW) return "RIGHT_ARROW";
  if (tl_shape == TrafficLight::UP_ARROW) return "UP_ARROW";
  if (tl_shape == TrafficLight::DOWN_ARROW) return "DOWN_ARROW";
  if (tl_shape == TrafficLight::DOWN_LEFT_ARROW) return "DOWN_LEFT_ARROW";
  if (tl_shape == TrafficLight::DOWN_RIGHT_ARROW) return "DOWN_RIGHT_ARROW";

  if (tl_color == TrafficLight::RED) return "RED";
  if (tl_color == TrafficLight::GREEN) return "GREEN";
  if (tl_color == TrafficLight::AMBER) return "AMBER";

  if (tl_shape == TrafficLight::UNKNOWN) return "UNKNOWN";

  return "UNKNOWN";
}

std_msgs::msg::ColorRGBA TrafficLightRecognitionMarkerPublisher::getTrafficLightColor(
  const uint8_t tl_color, const uint8_t tl_shape)
{
  std_msgs::msg::ColorRGBA c;
  c.r = 1.0f;
  c.g = 1.0f;
  c.b = 1.0f;
  c.a = 0.999;

  if (
    tl_shape == TrafficLight::LEFT_ARROW || tl_shape == TrafficLight::RIGHT_ARROW ||
    tl_shape == TrafficLight::UP_ARROW || tl_shape == TrafficLight::DOWN_ARROW ||
    tl_shape == TrafficLight::DOWN_LEFT_ARROW || tl_shape == TrafficLight::DOWN_RIGHT_ARROW) {
    return c;  // white
  }

  if (tl_color == TrafficLight::RED) {
    c.r = 1.0f;
    c.g = 0.0f;
    c.b = 0.0f;
    return c;  // red
  }
  if (tl_color == TrafficLight::GREEN) {
    c.r = 0.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    return c;  // green
  }
  if (tl_color == TrafficLight::AMBER) {
    c.r = 1.0f;
    c.g = 1.0f;
    c.b = 0.0f;
    return c;  // amber
  }

  return c;  // white
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrafficLightRecognitionMarkerPublisher)
