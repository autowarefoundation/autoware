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

#include "heatmap_visualizer/heatmap_visualizer_node.hpp"

#include "heatmap_visualizer/utils.hpp"
#include "perception_utils/perception_utils.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <memory>

namespace heatmap_visualizer
{
HeatmapVisualizerNode::HeatmapVisualizerNode(const rclcpp::NodeOptions & node_options)
: Node("heatmap_visualizer", node_options), frame_count_(0)
{
  total_frame_ = declare_parameter("frame_count", 50);
  map_frame_ = declare_parameter("map_frame", "base_link");
  map_length_ = declare_parameter("map_length", 200.0);
  map_resolution_ = declare_parameter("map_resolution", 0.8);
  use_confidence_ = declare_parameter("use_confidence", false);
  class_names_ = declare_parameter("class_names", class_names_);
  rename_car_to_truck_and_bus_ = declare_parameter("rename_car_to_truck_and_bus", false);

  width_ = static_cast<uint32_t>(map_length_ / map_resolution_);
  height_ = static_cast<uint32_t>(map_length_ / map_resolution_);

  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "/heatmap_visualizer/input/objects", 1,
    std::bind(&HeatmapVisualizerNode::detectedObjectsCallback, this, std::placeholders::_1));

  for (std::string & key : class_names_) {
    nav_msgs::msg::OccupancyGrid heatmap;
    heatmap.header.frame_id = map_frame_;
    heatmap.info.width = width_;
    heatmap.info.height = height_;
    heatmap.info.resolution = map_resolution_;
    heatmap.info.origin.position.x = -map_length_ / 2;
    heatmap.info.origin.position.y = -map_length_ / 2;
    heatmap.data.resize(width_ * height_, 0);

    std::vector<float> buffer;
    buffer.resize(width_ * height_, 0);

    uint8_t label = getSemanticType(key);
    bool is_car_like_vehicle = perception_utils::isCarLikeVehicle(label);
    if ((!rename_car_to_truck_and_bus_) && (is_car_like_vehicle)) {
      uint8_t car_label = getSemanticType("CAR");
      heatmaps_.insert(std::make_pair(car_label, heatmap));
      data_buffers_.insert(std::make_pair(car_label, buffer));

      heatmap_pubs_.insert(std::make_pair(
        car_label, create_publisher<nav_msgs::msg::OccupancyGrid>("~/output/heatmap/CAR", 10.0)));
    } else {
      heatmaps_.insert(std::make_pair(label, heatmap));
      data_buffers_.insert(std::make_pair(label, buffer));

      heatmap_pubs_.insert(std::make_pair(
        label, create_publisher<nav_msgs::msg::OccupancyGrid>("~/output/heatmap/" + key, 10.0)));
    }
  }
}

void HeatmapVisualizerNode::detectedObjectsCallback(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg)
{
  frame_count_ += 1;
  for (const auto & obj : msg->objects) {
    uint8_t label = obj.classification[0].label;
    bool is_car_like_vehicle = perception_utils::isCarLikeVehicle(label);
    if ((!rename_car_to_truck_and_bus_) && (is_car_like_vehicle)) {
      label = getSemanticType("CAR");
    }
    // Set value to data buffer
    setHeatmapToBuffer(obj, heatmaps_.at(label), &data_buffers_.at(label), use_confidence_);
  }
  // Publish messages if frame_count_ == total_frame_
  if (frame_count_ == total_frame_) {
    for (auto & map : heatmaps_) {
      setHeatmapToOccupancyGrid(data_buffers_.at(map.first), &map.second);
      heatmap_pubs_.at(map.first)->publish(map.second);
    }
    // Reset frame count
    frame_count_ = 0;
  }
}

}  // namespace heatmap_visualizer

RCLCPP_COMPONENTS_REGISTER_NODE(heatmap_visualizer::HeatmapVisualizerNode)
