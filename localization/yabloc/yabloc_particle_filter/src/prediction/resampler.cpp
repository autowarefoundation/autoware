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

#include "yabloc_particle_filter/prediction/resampler.hpp"

#include <rclcpp/rclcpp.hpp>

#include <boost/range/adaptor/indexed.hpp>

#include <cmath>
#include <numeric>
#include <random>

namespace yabloc::modularized_particle_filter
{
RetroactiveResampler::RetroactiveResampler(int number_of_particles, int max_history_num)
: max_history_num_(max_history_num),
  number_of_particles_(number_of_particles),
  logger_(rclcpp::get_logger("modularized_particle_filter.retroactive_resampler")),
  resampling_history_(max_history_num_, number_of_particles)
{
  latest_resampling_generation_ = 0;
}

bool RetroactiveResampler::check_weighted_particles_validity(
  const ParticleArray & weighted_particles) const
{
  if (static_cast<int>(weighted_particles.particles.size()) != number_of_particles_) {
    return false;
  }

  // invalid generation
  if (weighted_particles.id < 0) {
    RCLCPP_ERROR_STREAM(logger_, "invalid generation id");
    return false;
  }

  // future data
  if (weighted_particles.id > latest_resampling_generation_) {
    RCLCPP_ERROR_STREAM(logger_, "future generation id");
    return false;
  }

  // not too old data
  if (!(weighted_particles.id > latest_resampling_generation_ - max_history_num_)) {
    RCLCPP_WARN_STREAM(logger_, "too old generation id");
    return false;
  }
  return true;
}

RetroactiveResampler::ParticleArray RetroactiveResampler::add_weight_retroactively(
  const ParticleArray & predicted_particles, const ParticleArray & weighted_particles)
{
  if (!check_weighted_particles_validity(weighted_particles)) {
    RCLCPP_ERROR_STREAM(logger_, "weighted_particles has invalid data");
    throw resampling_skip_exception("weighted_particles has invalid data");
  }

  // Initialize corresponding index lookup table
  // The m-th address has the m-th particle's parent index
  std::vector<int> index_table(weighted_particles.particles.size());
  std::iota(index_table.begin(), index_table.end(), 0);

  // Lookup corresponding indices
  for (int generation = latest_resampling_generation_; generation > weighted_particles.id;
       generation--) {
    for (int m = 0; m < number_of_particles_; m++) {
      index_table[m] = resampling_history_[generation][index_table[m]];
    }
  }

  ParticleArray weighted_particles_updated = predicted_particles;

  // Add weights to current particles
  float sum_weight = 0;
  for (auto && it : weighted_particles_updated.particles | boost::adaptors::indexed()) {
    it.value().weight *= weighted_particles.particles[index_table[it.index()]].weight;
    sum_weight += it.value().weight;
  }

  // Normalize all weight
  for (auto & particle : weighted_particles_updated.particles) {
    particle.weight /= sum_weight;
  }

  return weighted_particles_updated;
}

RetroactiveResampler::ParticleArray RetroactiveResampler::resample(
  const ParticleArray & predicted_particles)
{
  ParticleArray resampled_particles{predicted_particles};
  latest_resampling_generation_++;
  resampled_particles.id = latest_resampling_generation_;

  // Summation of current weights
  const double sum_weight = std::accumulate(
    predicted_particles.particles.begin(), predicted_particles.particles.end(), 0.0,
    [](double weight, const Particle & ps) { return weight + ps.weight; });
  // Inverse of the summation of current weight
  const double sum_weight_inv = 1.0 / sum_weight;
  // Inverse of the number of particle
  const double num_of_particles_inv = 1.0 / static_cast<double>(number_of_particles_);
  // A residual term for a random selection of particle sampling thresholds.
  // This can range from 0 to 1/(num_of_particles)
  const double weight_threshold_residual = random_from_01_uniformly() * num_of_particles_inv;

  if (!std::isfinite(sum_weight_inv)) {
    RCLCPP_ERROR_STREAM(logger_, "The inverse of the sum of the weights is not a valid value");
    throw std::runtime_error("weighted_particles has invalid data");
  }

  auto n_th_normalized_weight = [&](int index) -> double {
    return predicted_particles.particles.at(index).weight * sum_weight_inv;
  };

  //
  int predicted_particle_index = 0;
  double accumulated_normalized_weights = n_th_normalized_weight(0);
  // Here, 'm' means resampled_particle_index
  for (int m = 0; m < number_of_particles_; m++) {
    const double m_th_weight_threshold = m * num_of_particles_inv + weight_threshold_residual;

    // Accumulate predicted particles' weight until it exceeds the threshold
    // (If the previous weights are sufficiently large, this accumulation can be skipped over
    // several resampled particles. It means that a probable particle will make many successors.)
    while (accumulated_normalized_weights < m_th_weight_threshold) {
      predicted_particle_index++;
      accumulated_normalized_weights += n_th_normalized_weight(predicted_particle_index);
    }
    // Copy particle to resampled variable
    resampled_particles.particles[m] = predicted_particles.particles[predicted_particle_index];
    // Reset weight uniformly
    resampled_particles.particles[m].weight = num_of_particles_inv;
    // Make history
    resampling_history_[latest_resampling_generation_][m] = predicted_particle_index;
  }

  // NOTE: This check wastes the computation time
  if (!resampling_history_.check_history_validity()) {
    RCLCPP_ERROR_STREAM(logger_, "resampling_history may be broken");
    throw std::runtime_error("resampling_history may be broken");
  }

  return resampled_particles;
}

double RetroactiveResampler::random_from_01_uniformly() const
{
  static std::default_random_engine engine(0);
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  return dist(engine);
}

}  // namespace yabloc::modularized_particle_filter
