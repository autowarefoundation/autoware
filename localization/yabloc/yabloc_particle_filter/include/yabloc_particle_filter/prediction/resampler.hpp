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

#ifndef YABLOC_PARTICLE_FILTER__PREDICTION__RESAMPLER_HPP_
#define YABLOC_PARTICLE_FILTER__PREDICTION__RESAMPLER_HPP_

#include "yabloc_particle_filter/msg/particle_array.hpp"
#include "yabloc_particle_filter/prediction/resampling_history.hpp"

#include <rclcpp/logger.hpp>

namespace yabloc::modularized_particle_filter
{
class resampling_skip_exception : public std::runtime_error
{
public:
  explicit resampling_skip_exception(const char * message) : runtime_error(message) {}
};

class RetroactiveResampler
{
public:
  using Particle = yabloc_particle_filter::msg::Particle;
  using ParticleArray = yabloc_particle_filter::msg::ParticleArray;

  RetroactiveResampler(int number_of_particles, int max_history_num);

  ParticleArray add_weight_retroactively(
    const ParticleArray & predicted_particles, const ParticleArray & weighted_particles);

  ParticleArray resample(const ParticleArray & predicted_particles);

private:
  // Number of updates to keep resampling history.
  // Resampling records prior to this will not be kept.
  const int max_history_num_;
  // Number of particles to be managed.
  const int number_of_particles_;
  // ROS logger
  rclcpp::Logger logger_;
  // This is handled like ring buffer.
  // It keeps track of which particles each particle has transformed into at each resampling.
  ResamplingHistory resampling_history_;
  // Indicates how many times the particles were resampled.
  int latest_resampling_generation_;

  // Random generator from 0 to 1
  double random_from_01_uniformly() const;
  // Check the sanity of the particles obtained from the particle corrector.
  bool check_weighted_particles_validity(const ParticleArray & weighted_particles) const;
};
}  // namespace yabloc::modularized_particle_filter

#endif  // YABLOC_PARTICLE_FILTER__PREDICTION__RESAMPLER_HPP_
