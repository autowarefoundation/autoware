// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher/particle.hpp"

#include "localization_util/util_func.hpp"

void push_debug_markers(
  visualization_msgs::msg::MarkerArray & marker_array, const builtin_interfaces::msg::Time & stamp,
  const std::string & map_frame_, const Particle & particle, const size_t i)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.id = static_cast<int32_t>(i);
  marker.lifetime = rclcpp::Duration::from_seconds(10.0);  // 10.0 is the lifetime in seconds.

  marker.ns = "initial_pose_transform_probability_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = exchange_color_crc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_iteration_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = exchange_color_crc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_index_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = exchange_color_crc(static_cast<double>(i) / 100.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_transform_probability_color_marker";
  marker.pose = particle.result_pose;
  marker.color = exchange_color_crc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_iteration_color_marker";
  marker.pose = particle.result_pose;
  marker.color = exchange_color_crc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_index_color_marker";
  marker.pose = particle.result_pose;
  marker.color = exchange_color_crc(static_cast<double>(i) / 100.0);
  marker_array.markers.push_back(marker);
}
