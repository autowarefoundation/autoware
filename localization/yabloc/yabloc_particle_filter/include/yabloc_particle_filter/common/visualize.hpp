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

#ifndef YABLOC_PARTICLE_FILTER__COMMON__VISUALIZE_HPP_
#define YABLOC_PARTICLE_FILTER__COMMON__VISUALIZE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <yabloc_particle_filter/msg/particle_array.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace yabloc
{
namespace modularized_particle_filter
{
class ParticleVisualizer
{
public:
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Particle = yabloc_particle_filter::msg::Particle;
  using ParticleArray = yabloc_particle_filter::msg::ParticleArray;

  explicit ParticleVisualizer(rclcpp::Node & node);
  void publish(const ParticleArray & msg);

private:
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_array_;
  std_msgs::msg::ColorRGBA compute_color(float value);
};
}  // namespace modularized_particle_filter
}  // namespace yabloc

#endif  // YABLOC_PARTICLE_FILTER__COMMON__VISUALIZE_HPP_
