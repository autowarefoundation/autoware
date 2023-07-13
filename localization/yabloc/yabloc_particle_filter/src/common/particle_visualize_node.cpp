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

#include "yabloc_common/color.hpp"

#include <rclcpp/rclcpp.hpp>
#include <yabloc_particle_filter/msg/particle_array.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace yabloc::modularized_particle_filter
{
class ParticleVisualize : public rclcpp::Node
{
public:
  using Particle = yabloc_particle_filter::msg::Particle;
  using ParticleArray = yabloc_particle_filter::msg::ParticleArray;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  ParticleVisualize() : Node("particle_visualize")
  {
    using std::placeholders::_1;
    // Subscriber
    sub_particles_ = this->create_subscription<ParticleArray>(
      "/particle_array", 10, std::bind(&ParticleVisualize::on_particles, this, _1));

    // Publisher
    pub_marker_array = this->create_publisher<MarkerArray>("/marker_array", 10);
  }

private:
  rclcpp::Subscription<ParticleArray>::SharedPtr sub_particles_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_array;

  std_msgs::msg::ColorRGBA compute_color(float value) const
  {
    float r = 1.0f, g = 1.0f, b = 1.0f;
    // clang-format off
    value = std::clamp(value, 0.0f, 1.0f);
    if (value < 0.25f) {
      r = 0; g = 4 * (value);
    } else if (value < 0.5f) {
      r = 0; b = 1 + 4 * (0.25f - value);
    } else if (value < 0.75f) {
      r = 4 * (value - 0.5f); b = 0;
    } else {
      g = 1 + 4 * (0.75f - value); b = 0;
    }
    // clang-format on

    std_msgs::msg::ColorRGBA rgba;
    rgba.r = r;
    rgba.g = g;
    rgba.b = b;
    rgba.a = 1.0f;
    return rgba;
  }

  void on_particles(const ParticleArray & msg)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    auto minmax_weight = std::minmax_element(
      msg.particles.begin(), msg.particles.end(),
      [](const Particle & a, const Particle & b) -> bool { return a.weight < b.weight; });

    float min = minmax_weight.first->weight;
    float max = minmax_weight.second->weight;
    max = std::max(max, min + 1e-7f);
    auto bound_weight = [min, max](float raw) -> float { return (raw - min) / (max - min); };

    RCLCPP_INFO_STREAM(get_logger(), "min: " << min << " max: " << max);
    int id = 0;
    for (const Particle & p : msg.particles) {
      Marker marker;
      marker.frame_locked = true;
      marker.header.frame_id = "map";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.scale.x = 0.3;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;

      marker.color = common::color_scale::rainbow(bound_weight(p.weight));
      marker.pose.orientation = p.pose.orientation;
      marker.pose.position.x = p.pose.position.x;
      marker.pose.position.y = p.pose.position.y;
      marker.pose.position.z = p.pose.position.z;
      marker_array.markers.push_back(marker);
    }
    pub_marker_array->publish(marker_array);
  }
};
}  // namespace yabloc::modularized_particle_filter

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<yabloc::modularized_particle_filter::ParticleVisualize>());
  rclcpp::shutdown();
  return 0;
}
