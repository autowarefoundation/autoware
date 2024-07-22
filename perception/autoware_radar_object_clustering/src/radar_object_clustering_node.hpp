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

#ifndef RADAR_OBJECT_CLUSTERING_NODE_HPP_
#define RADAR_OBJECT_CLUSTERING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace autoware::radar_object_clustering
{
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;

class RadarObjectClusteringNode : public rclcpp::Node
{
public:
  explicit RadarObjectClusteringNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    double angle_threshold{};
    double distance_threshold{};
    double velocity_threshold{};
    bool is_fixed_label{};
    std::string fixed_label{};
    bool is_fixed_size{};
    double size_x{};
    double size_y{};
    double size_z{};
  };

private:
  // Subscriber
  rclcpp::Subscription<DetectedObjects>::SharedPtr sub_objects_{};

  // Callback
  void onObjects(const DetectedObjects::ConstSharedPtr msg);

  // Publisher
  rclcpp::Publisher<DetectedObjects>::SharedPtr pub_objects_{};

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  // Core
  bool isSameObject(const DetectedObject & object_1, const DetectedObject & object_2);
};

}  // namespace autoware::radar_object_clustering

#endif  // RADAR_OBJECT_CLUSTERING_NODE_HPP_
