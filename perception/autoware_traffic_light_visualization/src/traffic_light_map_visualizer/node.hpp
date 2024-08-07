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

#ifndef TRAFFIC_LIGHT_MAP_VISUALIZER__NODE_HPP_
#define TRAFFIC_LIGHT_MAP_VISUALIZER__NODE_HPP_

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
class TrafficLightMapVisualizerNode : public rclcpp::Node
{
public:
  explicit TrafficLightMapVisualizerNode(const rclcpp::NodeOptions & node_options);
  ~TrafficLightMapVisualizerNode() = default;
  void trafficSignalsCallback(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr
      input_traffic_signals);
  void binMapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_map_msg);

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr light_marker_pub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    tl_state_sub_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr vector_map_sub_;

  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MAP_VISUALIZER__NODE_HPP_
