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

#ifndef YABLOC_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
#define YABLOC_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "yabloc_particle_filter/msg/particle_array.hpp"

#include <boost/circular_buffer.hpp>

#include <optional>

namespace yabloc
{
namespace modularized_particle_filter
{
std::optional<yabloc_particle_filter::msg::ParticleArray> find_synced_particles(
  boost::circular_buffer<yabloc_particle_filter::msg::ParticleArray> circular_buffer,
  rclcpp::Time time);
}
}  // namespace yabloc

#endif  // YABLOC_PARTICLE_FILTER__CORRECTION__CORRECTION_UTIL_HPP_
