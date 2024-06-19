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

#include <rclcpp/time.hpp>

#include <gtest/gtest.h>

namespace mpf = yabloc::modularized_particle_filter;
using Particle = yabloc_particle_filter::msg::Particle;
using ParticleArray = yabloc_particle_filter::msg::ParticleArray;

constexpr int particle_count = 10;
constexpr int history_size = 10;

TEST(ResamplerTestSuite, outOfHistory)
{
  mpf::RetroactiveResampler resampler(particle_count, history_size);
  ParticleArray predicted;
  ParticleArray weighted;

  predicted.header.stamp = rclcpp::Time(0);
  predicted.id = 0;
  weighted.id = 0;
  predicted.particles.resize(particle_count);
  weighted.particles.resize(particle_count);
  for (auto & p : predicted.particles) p.weight = 1;
  for (auto & p : weighted.particles) p.weight = 1;

  // Invalid generation
  {
    weighted.id = -1;
    bool caught = false;
    try {
      resampler.add_weight_retroactively(predicted, weighted);
    } catch (...) {
      caught = true;
    }
    EXPECT_TRUE(caught);
  }

  // Future generation
  {
    weighted.id = 1;
    bool caught = false;
    try {
      resampler.add_weight_retroactively(predicted, weighted);
    } catch (...) {
      caught = true;
    }
    EXPECT_TRUE(caught);
  }

  // Iterate resampling to fill all history
  for (int t = 0; t < history_size; ++t) {
    auto resampled = resampler.resample(predicted);
    EXPECT_EQ(resampled.id, t + 1);
    predicted = resampled;
  }

  // Too old generation
  {
    weighted.id = 0;
    bool caught = false;
    try {
      resampler.add_weight_retroactively(predicted, weighted);
    } catch (...) {
      caught = true;
    }
    EXPECT_TRUE(caught);
  }
}

TEST(ResamplerTestSuite, simpleResampling)
{
  mpf::RetroactiveResampler resampler(particle_count, history_size);

  ParticleArray predicted;
  predicted.header.stamp = rclcpp::Time(0);
  predicted.particles.resize(particle_count);
  predicted.id = 0;
  for (int i = 0; i < particle_count; ++i) predicted.particles.at(i).weight = 1;

  ParticleArray weighted;
  weighted.particles.resize(particle_count);

  // Update by uniform distribution
  {
    // Weight
    weighted.id = 0;
    for (int i = 0; i < particle_count; ++i) weighted.particles.at(i).weight = 0.5;
    ParticleArray array1 = resampler.add_weight_retroactively(predicted, weighted);

    // All weights must be equal
    for (const auto & p : array1.particles) EXPECT_NEAR(p.weight, 1.0 / particle_count, 1e-3);

    // Resample
    predicted = array1;
    auto resampled = resampler.resample(predicted);
    predicted = resampled;
    EXPECT_EQ(predicted.id, 1);
  }

  // Update half and half
  {
    // Weight
    weighted.id = 0;
    for (int i = 0; i < particle_count; ++i) {
      auto & p = predicted.particles.at(i);
      auto & q = weighted.particles.at(i);
      if (i < particle_count / 2) {
        p.pose.position.x = 1;
        q.weight = 2.0;
      } else {
        p.pose.position.x = -1;
        q.weight = 1.0;
      }
    }
    ParticleArray array1 = resampler.add_weight_retroactively(predicted, weighted);

    // All weight must match with following expectation
    for (int i = 0; i < particle_count; ++i) {
      const auto & p = array1.particles.at(i);
      if (i < particle_count / 2) {
        EXPECT_NEAR(p.weight, 2.0 / 1.5 / particle_count, 1e-3f);
      } else {
        EXPECT_NEAR(p.weight, 1.0 / 1.5 / particle_count, 1e-3f);
      }
    }

    // Resample
    predicted = array1;
    auto resampled = resampler.resample(predicted);
    predicted = resampled;
    EXPECT_EQ(predicted.id, 2);

    int centroid = std::accumulate(
      predicted.particles.begin(), predicted.particles.end(), 0,
      [](int sum, const auto & p) { return sum + static_cast<int>(p.pose.position.x); });
    EXPECT_GT(centroid, 0);
  }
}

TEST(ResamplerTestSuite, resamplingWithRetrogression)
{
  mpf::RetroactiveResampler resampler(particle_count, history_size);

  ParticleArray predicted;
  predicted.header.stamp = rclcpp::Time(0);
  predicted.particles.resize(particle_count);
  predicted.id = 0;

  for (int i = 0; i < particle_count; ++i) {
    auto & p = predicted.particles.at(i);
    p.weight = 1.0;
    if (i < particle_count / 2)
      p.pose.position.x = 1;
    else
      p.pose.position.x = -1;
  }

  // Fill all history with biased weighted particles
  for (int p = 0; p < history_size; ++p) {
    auto resampled = resampler.resample(predicted);
    predicted = resampled;
    EXPECT_EQ(predicted.id, p + 1);
  }

  // Update by ancient measurement
  {
    double before_centroid = std::accumulate(
      predicted.particles.begin(), predicted.particles.end(), 0.0,
      [](double sum, const auto & p) { return sum + p.pose.position.x * p.weight; });

    // Weight
    ParticleArray weighted;
    weighted.particles.resize(particle_count);
    weighted.id = 1;  // ancient generation id
    for (int i = 0; i < particle_count; ++i) {
      auto & q = weighted.particles.at(i);
      if (i < particle_count / 2) {
        q.weight = 2.0;
      } else {
        q.weight = 1.0;
      }
    }

    predicted = resampler.add_weight_retroactively(predicted, weighted);

    double after_centroid = std::accumulate(
      predicted.particles.begin(), predicted.particles.end(), 0.0,
      [](double sum, const auto & p) { return sum + p.pose.position.x * p.weight; });

    EXPECT_TRUE(after_centroid > before_centroid);
  }
}
