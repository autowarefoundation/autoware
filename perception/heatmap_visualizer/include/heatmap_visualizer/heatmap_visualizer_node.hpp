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

#ifndef HEATMAP_VISUALIZER__HEATMAP_VISUALIZER_NODE_HPP_
#define HEATMAP_VISUALIZER__HEATMAP_VISUALIZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <map>
#include <string>
#include <vector>

namespace heatmap_visualizer
{
class HeatmapVisualizerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Heatmap Visualizer Node object
   *
   * @param node_options
   */
  explicit HeatmapVisualizerNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Callback function
   *
   * @param msg
   */
  void detectedObjectsCallback(
    const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr msg);

  // Subscriber
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::DetectedObjects>::SharedPtr objects_sub_;

  // Publishers
  std::map<uint8_t, rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr> heatmap_pubs_;

  std::map<uint8_t, nav_msgs::msg::OccupancyGrid> heatmaps_;

  uint32_t frame_count_;

  // ROS params
  uint32_t total_frame_;
  std::string target_frame_;
  std::string src_frame_;
  std::string map_frame_;
  float map_length_;
  float map_resolution_;
  bool use_confidence_;
  std::vector<std::string> class_names_{"UNKNOWN", "CAR",     "TRUCK",     "BUS",
                                        "TRAILER", "BICYCLE", "MOTORBIKE", "PEDESTRIAN"};
  bool rename_car_to_truck_and_bus_;

  // Number of width and height cells
  uint32_t width_;
  uint32_t height_;
  std::map<uint8_t, std::vector<float>> data_buffers_;
};  // class HeatmapVisualizerNode

}  // namespace heatmap_visualizer

#endif  // HEATMAP_VISUALIZER__HEATMAP_VISUALIZER_NODE_HPP_
