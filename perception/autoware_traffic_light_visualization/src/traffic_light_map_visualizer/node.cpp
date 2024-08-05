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

#include "node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>

#include <string>
#include <vector>

namespace
{
[[maybe_unused]] void setColor(
  const double r, const double g, const double b, const double a, std_msgs::msg::ColorRGBA & cl)
{
  cl.r = r;
  cl.g = g;
  cl.b = b;
  cl.a = a;
}

bool isAttributeValue(
  const lanelet::ConstPoint3d p, const std::string & attr_str, const std::string & value_str)
{
  lanelet::Attribute attr = p.attribute(attr_str);
  if (attr.value().compare(value_str) == 0) {
    return true;
  }
  return false;
}

void lightAsMarker(
  const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & node_logging,
  lanelet::ConstPoint3d p, visualization_msgs::msg::Marker * marker, const std::string & ns,
  const rclcpp::Time & current_time)
{
  if (marker == nullptr) {
    RCLCPP_ERROR_STREAM(node_logging->get_logger(), __FUNCTION__ << ": marker is null pointer!");
    return;
  }

  marker->header.frame_id = "map";
  marker->header.stamp = current_time;
  marker->frame_locked = true;
  marker->ns = ns;
  marker->id = p.id();
  marker->lifetime = rclcpp::Duration::from_seconds(0.2);
  marker->type = visualization_msgs::msg::Marker::SPHERE;
  marker->pose.position.x = p.x();
  marker->pose.position.y = p.y();
  marker->pose.position.z = p.z();
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;

  float s = 0.3;

  marker->scale.x = s;
  marker->scale.y = s;
  marker->scale.z = s;

  marker->color.r = 0.0f;
  marker->color.g = 0.0f;
  marker->color.b = 0.0f;
  marker->color.a = 0.999f;

  if (isAttributeValue(p, "color", "red")) {
    marker->color.r = 1.0f;
    marker->color.g = 0.0f;
    marker->color.b = 0.0f;
  } else if (isAttributeValue(p, "color", "green")) {
    marker->color.r = 0.0f;
    marker->color.g = 1.0f;
    marker->color.b = 0.0f;
  } else if (isAttributeValue(p, "color", "yellow")) {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 0.0f;
  } else {
    marker->color.r = 1.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
  }
}
}  // namespace

namespace autoware::traffic_light
{
TrafficLightMapVisualizerNode::TrafficLightMapVisualizerNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_map_visualizer_node", node_options)
{
  using std::placeholders::_1;

  light_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/traffic_light", 1);
  tl_state_sub_ = create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/input/tl_state", 1,
    std::bind(&TrafficLightMapVisualizerNode::trafficSignalsCallback, this, _1));
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightMapVisualizerNode::binMapCallback, this, _1));
}
void TrafficLightMapVisualizerNode::trafficSignalsCallback(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr input_traffic_signals)
{
  visualization_msgs::msg::MarkerArray output_msg;
  const auto current_time = now();

#if 0
  for (auto tli = aw_tl_reg_elems_.begin(); tli != aw_tl_reg_elems_.end(); tli++) {
    for (const auto & lsp : (*tli)->trafficLights()) {
      if (lsp.isLineString()) { // traffic lights can either polygons or
                                // linestrings
        lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);
        for (const auto & input_traffic_signal : input_traffic_signals->signals) {
          if (ls.id() != input_traffic_signal.map_primitive_id) {
            continue;
          }
          visualization_msgs::msg::Marker marker;
          std_msgs::msg::ColorRGBA color;
          setColor(1.0f, 1.0f, 1.0f, 0.999f, color);
          lanelet::visualization::initTrafficLightTriangleMarker(&marker, "traffic_light_triangle");
          lanelet::visualization::pushTrafficLightTriangleMarker(&marker, ls, color);
          marker.header.frame_id = "map";
          marker.header.stamp = current_time;
          marker.frame_locked = true;
          marker.lifetime = rclcpp::Duration::from_seconds(0.2);
          output_msg.markers.push_back(marker);
        }
      }
    }
  }
#endif

  for (auto tli = aw_tl_reg_elems_.begin(); tli != aw_tl_reg_elems_.end(); tli++) {
    for (auto ls : (*tli)->lightBulbs()) {
      if (!ls.hasAttribute("traffic_light_id")) {
        continue;
      }
      for (const auto & input_traffic_signal : input_traffic_signals->traffic_light_groups) {
        if ((*tli)->id() == input_traffic_signal.traffic_light_group_id) {
          // if (isAttributeValue(ls, "traffic_light_id", input_traffic_signal.map_primitive_id)) {
          for (auto pt : ls) {
            if (!pt.hasAttribute("color")) {
              continue;
            }

            for (const auto & elem : input_traffic_signal.elements) {
              visualization_msgs::msg::Marker marker;
              if (
                isAttributeValue(pt, "color", "red") &&
                elem.color == autoware_perception_msgs::msg::TrafficLightElement::RED) {
                lightAsMarker(
                  get_node_logging_interface(), pt, &marker, "traffic_light", current_time);
              } else if (  // NOLINT
                isAttributeValue(pt, "color", "green") &&
                elem.color == autoware_perception_msgs::msg::TrafficLightElement::GREEN) {
                lightAsMarker(
                  get_node_logging_interface(), pt, &marker, "traffic_light", current_time);

              } else if (  // NOLINT
                isAttributeValue(pt, "color", "yellow") &&
                elem.color == autoware_perception_msgs::msg::TrafficLightElement::AMBER) {
                lightAsMarker(
                  get_node_logging_interface(), pt, &marker, "traffic_light", current_time);
              } else {
                continue;
              }
              output_msg.markers.push_back(marker);
            }
          }
        }
      }
    }
  }

  light_marker_pub_->publish(output_msg);
}

void TrafficLightMapVisualizerNode::binMapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_map_msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*input_map_msg, viz_lanelet_map);
  RCLCPP_DEBUG(get_logger(), "Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  aw_tl_reg_elems_ = lanelet::utils::query::autowareTrafficLights(all_lanelets);
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightMapVisualizerNode)
