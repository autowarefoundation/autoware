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

#include "yabloc_particle_filter/common/visualize.hpp"

#include <yabloc_common/color.hpp>

namespace yabloc::modularized_particle_filter
{
ParticleVisualizer::ParticleVisualizer(rclcpp::Node & node)
{
  pub_marker_array_ = node.create_publisher<MarkerArray>("~/debug/particles_marker_array", 10);
}

void ParticleVisualizer::publish(const ParticleArray & msg)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto minmax_weight = std::minmax_element(
    msg.particles.begin(), msg.particles.end(),
    [](const Particle & a, const Particle & b) -> bool { return a.weight < b.weight; });

  float min = minmax_weight.first->weight;
  float max = minmax_weight.second->weight;
  max = std::max(max, min + 1e-7f);
  auto boundWeight = [min, max](float raw) -> float { return (raw - min) / (max - min); };

  int id = 0;
  for (const Particle & p : msg.particles) {
    visualization_msgs::msg::Marker marker;
    marker.frame_locked = true;
    marker.header.frame_id = "map";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color = common::color_scale::rainbow(boundWeight(p.weight));
    marker.pose.orientation = p.pose.orientation;
    marker.pose.position.x = p.pose.position.x;
    marker.pose.position.y = p.pose.position.y;
    marker.pose.position.z = p.pose.position.z;
    marker_array.markers.push_back(marker);
  }

  pub_marker_array_->publish(marker_array);
}
}  // namespace yabloc::modularized_particle_filter
