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

#include "yabloc_particle_filter/correction/correction_util.hpp"

namespace yabloc::modularized_particle_filter
{
std::optional<yabloc_particle_filter::msg::ParticleArray> find_synced_particles(
  boost::circular_buffer<yabloc_particle_filter::msg::ParticleArray> circular_buffer,
  rclcpp::Time time)
{
  for (int i{1}; i < static_cast<int>(circular_buffer.size()); i++) {
    if (rclcpp::Time(circular_buffer[i].header.stamp) < time) {
      return circular_buffer[i];
    }
  }
  if (0 < static_cast<int>(circular_buffer.size())) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("modularized_particle_filter.correction_util"),
      "the sensor data is too old: "
        << rclcpp::Time(circular_buffer[static_cast<int>(circular_buffer.size()) - 1].header.stamp)
               .seconds() -
             time.seconds());
  }
  return std::nullopt;
}
}  // namespace yabloc::modularized_particle_filter
