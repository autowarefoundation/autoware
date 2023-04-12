// Copyright 2020-2023 TIER IV, Inc.
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

#ifndef LIDAR_APOLLO_INSTANCE_SEGMENTATION__DEBUGGER_HPP_
#define LIDAR_APOLLO_INSTANCE_SEGMENTATION__DEBUGGER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

namespace lidar_apollo_instance_segmentation
{
class Debugger
{
public:
  explicit Debugger(rclcpp::Node * node);
  ~Debugger() {}
  void publishColoredPointCloud(
    const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr instance_pointcloud_pub_;
};
}  // namespace lidar_apollo_instance_segmentation

#endif  // LIDAR_APOLLO_INSTANCE_SEGMENTATION__DEBUGGER_HPP_
