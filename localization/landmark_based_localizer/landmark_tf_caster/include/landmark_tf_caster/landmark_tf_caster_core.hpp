// Copyright 2023 Autoware Foundation
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

#ifndef LANDMARK_TF_CASTER__LANDMARK_TF_CASTER_CORE_HPP_
#define LANDMARK_TF_CASTER__LANDMARK_TF_CASTER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/primitives/Polygon.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

class LandmarkTfCaster : public rclcpp::Node
{
public:
  LandmarkTfCaster();

private:
  void map_bin_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr & msg);
  void publish_tf(const lanelet::Polygon3d & poly);

  // Parameters
  double volume_threshold_;

  // tf
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  // Subscribers
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_bin_sub_;
};

#endif  // LANDMARK_TF_CASTER__LANDMARK_TF_CASTER_CORE_HPP_
