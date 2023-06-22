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

#ifndef YABLOC_PARTICLE_FILTER__COMMON__PREDICTION_UTIL_HPP_
#define YABLOC_PARTICLE_FILTER__COMMON__PREDICTION_UTIL_HPP_

#include <Eigen/SVD>

#include <numeric>
#include <optional>
#include <random>
#include <vector>

namespace yabloc
{
namespace modularized_particle_filter::util
{
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());

Eigen::Vector2d nrand_2d(const Eigen::Matrix2d cov)
{
  Eigen::JacobiSVD<Eigen::Matrix2d> svd;
  svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector2d std = svd.singularValues();
  std = std.cwiseMax(0.01);

  std::normal_distribution<> dist(0.0, 1.0);
  Eigen::Vector2d xy;
  xy.x() = std::sqrt(std.x()) * dist(engine);
  xy.y() = std::sqrt(std.y()) * dist(engine);
  return svd.matrixU() * xy;
}

template <typename T = float>
T nrand(T std)
{
  std::normal_distribution<T> dist(0.0, std);
  return dist(engine);
}

double normalize_radian(const double rad, const double min_rad = -M_PI)
{
  const auto max_rad = min_rad + 2 * M_PI;

  const auto value = std::fmod(rad, 2 * M_PI);

  if (min_rad <= value && value <= max_rad) {
    return value;
  }

  return value - std::copysign(2 * M_PI, value);
}

}  // namespace modularized_particle_filter::util
}  // namespace yabloc
#endif  // YABLOC_PARTICLE_FILTER__COMMON__PREDICTION_UTIL_HPP_
