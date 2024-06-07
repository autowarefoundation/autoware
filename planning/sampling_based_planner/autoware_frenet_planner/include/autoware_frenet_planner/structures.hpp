// Copyright 2023 Tier IV, Inc.
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

#ifndef AUTOWARE_FRENET_PLANNER__STRUCTURES_HPP_
#define AUTOWARE_FRENET_PLANNER__STRUCTURES_HPP_

#include "autoware_frenet_planner/polynomials.hpp"

#include <autoware_sampler_common/structures.hpp>

#include <optional>
#include <vector>

namespace autoware::frenet_planner
{

struct FrenetState
{
  autoware::sampler_common::FrenetPoint position = {0, 0};
  double lateral_velocity{};
  double longitudinal_velocity{};
  double lateral_acceleration{};
  double longitudinal_acceleration{};
};

struct Path : autoware::sampler_common::Path
{
  std::vector<autoware::sampler_common::FrenetPoint> frenet_points{};
  std::optional<Polynomial> lateral_polynomial{};

  Path() = default;
  explicit Path(const autoware::sampler_common::Path & path) : autoware::sampler_common::Path(path)
  {
  }

  void clear() override
  {
    autoware::sampler_common::Path::clear();
    frenet_points.clear();
    lateral_polynomial.reset();
  }

  void reserve(const size_t size) override
  {
    autoware::sampler_common::Path::reserve(size);
    frenet_points.reserve(size);
  }

  [[nodiscard]] Path extend(const Path & path) const
  {
    Path extended_traj(autoware::sampler_common::Path::extend(path));
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), frenet_points.begin(), frenet_points.end());
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), path.frenet_points.begin(), path.frenet_points.end());
    // TODO(Maxime CLEMENT): direct copy from the 2nd trajectory. might need to be improved
    extended_traj.lateral_polynomial = path.lateral_polynomial;
    return extended_traj;
  }

  [[nodiscard]] Path * subset(const size_t from_idx, const size_t to_idx) const override
  {
    auto * subpath = new Path(*autoware::sampler_common::Path::subset(from_idx, to_idx));
    assert(to_idx >= from_idx);
    subpath->reserve(to_idx - from_idx);

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };
    copy_subset(frenet_points, subpath->frenet_points);
    return subpath;
  };
};

struct SamplingParameter
{
  double target_duration{};
  FrenetState target_state;
};

struct Trajectory : autoware::sampler_common::Trajectory
{
  std::vector<autoware::sampler_common::FrenetPoint> frenet_points{};
  std::optional<Polynomial> lateral_polynomial{};
  std::optional<Polynomial> longitudinal_polynomial{};
  SamplingParameter sampling_parameter;

  Trajectory() = default;
  explicit Trajectory(const autoware::sampler_common::Trajectory & traj)
  : autoware::sampler_common::Trajectory(traj)
  {
  }

  void clear() override
  {
    autoware::sampler_common::Trajectory::clear();
    frenet_points.clear();
    lateral_polynomial.reset();
    longitudinal_polynomial.reset();
  }

  void reserve(const size_t size) override
  {
    autoware::sampler_common::Trajectory::reserve(size);
    frenet_points.reserve(size);
  }

  [[nodiscard]] Trajectory extend(const Trajectory & traj) const
  {
    Trajectory extended_traj(autoware::sampler_common::Trajectory::extend(traj));
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), frenet_points.begin(), frenet_points.end());
    extended_traj.frenet_points.insert(
      extended_traj.frenet_points.end(), traj.frenet_points.begin(), traj.frenet_points.end());
    // TODO(Maxime CLEMENT): direct copy from the 2nd trajectory. might need to be improved
    extended_traj.lateral_polynomial = traj.lateral_polynomial;
    extended_traj.longitudinal_polynomial = traj.longitudinal_polynomial;
    return extended_traj;
  }

  [[nodiscard]] Trajectory * subset(const size_t from_idx, const size_t to_idx) const override
  {
    auto * sub_trajectory =
      new Trajectory(*autoware::sampler_common::Trajectory::subset(from_idx, to_idx));
    assert(to_idx >= from_idx);
    sub_trajectory->reserve(to_idx - from_idx);

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };
    copy_subset(frenet_points, sub_trajectory->frenet_points);
    return sub_trajectory;
  };
};

inline std::ostream & operator<<(std::ostream & os, const SamplingParameter & sp)
{
  const auto & s = sp.target_state;
  return os << "["
            << "T=" << sp.target_duration << ", s=" << s.position.s << ", d=" << s.position.d
            << ", s'=" << s.longitudinal_velocity << ", d'=" << s.lateral_velocity
            << ", s\"=" << s.longitudinal_acceleration << ", d\"=" << s.lateral_acceleration << "]";
}
struct SamplingParameters
{
  std::vector<SamplingParameter> parameters;
  double resolution;
};
}  // namespace autoware::frenet_planner

#endif  // AUTOWARE_FRENET_PLANNER__STRUCTURES_HPP_
