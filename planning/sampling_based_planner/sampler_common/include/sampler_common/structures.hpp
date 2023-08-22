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

#ifndef SAMPLER_COMMON__STRUCTURES_HPP_
#define SAMPLER_COMMON__STRUCTURES_HPP_

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include <eigen3/Eigen/Core>
#include <interpolation/linear_interpolation.hpp>

#include <boost/geometry/core/cs.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <algorithm>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace sampler_common
{
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::MultiPolygon2d;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

/// @brief data about constraint check results of a given path
struct ConstraintResults
{
  bool collision = true;
  bool curvature = true;
  bool drivable_area = true;

  [[nodiscard]] bool isValid() const { return collision && curvature && drivable_area; }

  void clear() { collision = curvature = drivable_area = true; }
};
struct FrenetPoint
{
  FrenetPoint(double s_, double d_) : s(s_), d(d_) {}
  double s = 0.0;
  double d = 0.0;
};

struct State
{
  Point2d pose{};
  FrenetPoint frenet{0.0, 0.0};
  double curvature{};
  double heading{};
};

struct Configuration : State
{
  double velocity{};
  double acceleration{};
};

/// @brief Path
struct Path
{
  std::vector<Point2d> points{};
  std::vector<double> curvatures{};
  std::vector<double> yaws{};
  std::vector<double> lengths{};
  ConstraintResults constraint_results{};
  double cost{};
  std::string tag{};  // string tag used for debugging

  Path() = default;
  virtual ~Path() = default;

  virtual void clear()
  {
    points.clear();
    curvatures.clear();
    yaws.clear();
    lengths.clear();
    constraint_results.clear();
    tag = "";
    cost = 0.0;
  }

  virtual void reserve(const size_t size)
  {
    points.reserve(size);
    curvatures.reserve(size);
    yaws.reserve(size);
    lengths.reserve(size);
  }

  [[nodiscard]] Path extend(const Path & path) const
  {
    Path extended_path;
    auto offset = 0l;
    auto length_offset = 0.0;
    if (!points.empty() && !path.points.empty()) {
      if (
        points.back().x() == path.points.front().x() &&
        points.back().y() == path.points.front().y()) {
        offset = 1l;
      }
      length_offset = std::hypot(
        path.points[offset].x() - points.back().x(), path.points[offset].y() - points.back().y());
    }
    const auto ext = [&](auto & dest, const auto & first, const auto & second) {
      dest.insert(dest.end(), first.begin(), first.end());
      dest.insert(dest.end(), std::next(second.begin(), offset), second.end());
    };
    ext(extended_path.points, points, path.points);
    ext(extended_path.curvatures, curvatures, path.curvatures);
    ext(extended_path.yaws, yaws, path.yaws);
    extended_path.lengths.insert(extended_path.lengths.end(), lengths.begin(), lengths.end());
    const auto last_base_length = lengths.empty() ? 0.0 : lengths.back() + length_offset;
    for (size_t i = offset; i < path.lengths.size(); ++i)
      extended_path.lengths.push_back(last_base_length + path.lengths[i]);
    extended_path.cost = path.cost;
    extended_path.constraint_results = path.constraint_results;
    extended_path.tag = path.tag;
    return extended_path;
  }

  // Return a pointer to allow overriding classes to return the appropriate type
  // Without pointer we are stuck with returning a Path
  [[nodiscard]] virtual Path * subset(const size_t from_idx, const size_t to_idx) const
  {
    auto * subpath = new Path();
    subpath->reserve(to_idx - from_idx);

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };
    copy_subset(points, subpath->points);
    copy_subset(curvatures, subpath->curvatures);
    copy_subset(yaws, subpath->yaws);
    copy_subset(lengths, subpath->lengths);
    return subpath;
  };
};

struct Trajectory : Path
{
  std::vector<double> longitudinal_velocities{};
  std::vector<double> longitudinal_accelerations{};
  std::vector<double> lateral_velocities{};
  std::vector<double> lateral_accelerations{};
  std::vector<double> jerks{};
  std::vector<double> times{};

  Trajectory() = default;
  ~Trajectory() override = default;
  explicit Trajectory(const Path & path) : Path(path) {}

  void clear() override
  {
    Path::clear();
    longitudinal_velocities.clear();
    longitudinal_accelerations.clear();
    lateral_velocities.clear();
    lateral_accelerations.clear();
    jerks.clear();
    times.clear();
  }

  void reserve(const size_t size) override
  {
    Path::reserve(size);
    longitudinal_velocities.reserve(size);
    longitudinal_accelerations.reserve(size);
    lateral_velocities.reserve(size);
    lateral_accelerations.reserve(size);
    jerks.reserve(size);
    times.reserve(size);
  }

  [[nodiscard]] Trajectory extend(const Trajectory & traj) const
  {
    Trajectory extended_traj(Path::extend(traj));
    auto offset = 0l;
    auto time_offset = 0.0;
    // TODO(Maxime): remove these checks
    if (!points.empty() && !traj.points.empty()) {
      if (
        points.back().x() == traj.points.front().x() &&
        points.back().y() == traj.points.front().y())
        offset = 1l;
      const auto ds = std::hypot(
        traj.points[offset].x() - points.back().x(), traj.points[offset].y() - points.back().y());
      const auto v = std::max(
        std::max(longitudinal_velocities.back(), traj.longitudinal_velocities[offset]), 0.1);
      time_offset = std::abs(ds / v);
    }
    const auto ext = [&](auto & dest, const auto & first, const auto & second) {
      dest.insert(dest.end(), first.begin(), first.end());
      dest.insert(dest.end(), std::next(second.begin(), offset), second.end());
    };
    ext(
      extended_traj.longitudinal_velocities, longitudinal_velocities, traj.longitudinal_velocities);
    ext(
      extended_traj.longitudinal_accelerations, longitudinal_accelerations,
      traj.longitudinal_accelerations);
    ext(extended_traj.lateral_velocities, lateral_velocities, traj.lateral_velocities);
    ext(extended_traj.lateral_accelerations, lateral_accelerations, traj.lateral_accelerations);
    ext(extended_traj.jerks, jerks, traj.jerks);
    extended_traj.times.insert(extended_traj.times.end(), times.begin(), times.end());
    const auto last_base_time = times.empty() ? 0.0 : times.back() + time_offset;
    for (size_t i = offset; i < traj.times.size(); ++i)
      extended_traj.times.push_back(last_base_time + traj.times[i]);
    return extended_traj;
  }

  [[nodiscard]] Trajectory * subset(const size_t from_idx, const size_t to_idx) const override
  {
    auto * sub_trajectory = new Trajectory(*Path::subset(from_idx, to_idx));

    const auto copy_subset = [&](const auto & from, auto & to) {
      to.insert(to.end(), std::next(from.begin(), from_idx), std::next(from.begin(), to_idx));
    };

    copy_subset(longitudinal_velocities, sub_trajectory->longitudinal_velocities);
    copy_subset(longitudinal_accelerations, sub_trajectory->longitudinal_accelerations);
    copy_subset(lateral_velocities, sub_trajectory->lateral_velocities);
    copy_subset(lateral_accelerations, sub_trajectory->lateral_accelerations);
    copy_subset(jerks, sub_trajectory->jerks);
    copy_subset(times, sub_trajectory->times);
    return sub_trajectory;
  }

  [[nodiscard]] Trajectory resample(const double fixed_interval) const
  {
    Trajectory t;
    if (lengths.size() < 2 || fixed_interval <= 0.0) return *this;

    const auto new_size =
      static_cast<size_t>((lengths.back() - lengths.front()) / fixed_interval) + 1;
    t.times.reserve(new_size);
    t.lengths.reserve(new_size);
    for (auto i = 0lu; i < new_size; ++i)
      t.lengths.push_back(lengths.front() + static_cast<double>(i) * fixed_interval);
    t.times = interpolation::lerp(lengths, times, t.lengths);
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(points.size());
    ys.reserve(points.size());
    for (const auto & p : points) {
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
    const auto new_xs = interpolation::lerp(times, xs, t.times);
    const auto new_ys = interpolation::lerp(times, ys, t.times);
    for (auto i = 0lu; i < new_xs.size(); ++i) t.points.emplace_back(new_xs[i], new_ys[i]);
    t.curvatures = interpolation::lerp(times, curvatures, t.times);
    t.jerks = interpolation::lerp(times, jerks, t.times);
    t.yaws = interpolation::lerp(times, yaws, t.times);
    t.longitudinal_velocities = interpolation::lerp(times, longitudinal_velocities, t.times);
    t.longitudinal_accelerations = interpolation::lerp(times, longitudinal_accelerations, t.times);
    t.lateral_velocities = interpolation::lerp(times, lateral_velocities, t.times);
    t.lateral_accelerations = interpolation::lerp(times, lateral_accelerations, t.times);
    t.constraint_results = constraint_results;
    t.cost = cost;
    return t;
  }

  [[nodiscard]] Trajectory resampleTimeFromZero(const double fixed_interval) const
  {
    Trajectory t;
    if (times.size() < 2 || times.back() < 0.0 || fixed_interval <= 0.0) return *this;

    const auto min_time = 0;
    const auto max_time = times.back();
    const auto new_size = static_cast<size_t>((max_time - min_time) / fixed_interval) + 1;
    t.times.reserve(new_size);
    t.lengths.reserve(new_size);
    for (auto i = 0lu; i < new_size; ++i)
      t.times.push_back(static_cast<double>(i) * fixed_interval);
    t.lengths = interpolation::lerp(times, lengths, t.times);
    std::vector<double> xs;
    std::vector<double> ys;
    xs.reserve(points.size());
    ys.reserve(points.size());
    for (const auto & p : points) {
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
    const auto new_xs = interpolation::lerp(times, xs, t.times);
    const auto new_ys = interpolation::lerp(times, ys, t.times);
    for (auto i = 0lu; i < new_xs.size(); ++i) t.points.emplace_back(new_xs[i], new_ys[i]);
    t.curvatures = interpolation::lerp(times, curvatures, t.times);
    t.jerks = interpolation::lerp(times, jerks, t.times);
    t.yaws = interpolation::lerp(times, yaws, t.times);
    t.longitudinal_velocities = interpolation::lerp(times, longitudinal_velocities, t.times);
    t.longitudinal_accelerations = interpolation::lerp(times, longitudinal_accelerations, t.times);
    t.lateral_velocities = interpolation::lerp(times, lateral_velocities, t.times);
    t.lateral_accelerations = interpolation::lerp(times, lateral_accelerations, t.times);
    t.constraint_results = constraint_results;
    t.cost = cost;
    return t;
  }
};

struct DynamicObstacle
{
  std::vector<Polygon2d> footprint_per_time;
  double time_step;  // [s] time step between each footprint
};

struct Constraints
{
  struct
  {
    double lateral_deviation_weight;
    double length_weight;
    double curvature_weight;
  } soft{};
  struct
  {
    double min_curvature;
    double max_curvature;
  } hard{};
  LinearRing2d ego_footprint;
  double ego_width;
  double ego_length;
  MultiPolygon2d obstacle_polygons;
  MultiPolygon2d drivable_polygons;
  std::vector<DynamicObstacle> dynamic_obstacles;
};

struct ReusableTrajectory
{
  Trajectory trajectory;                 // base trajectory
  Configuration planning_configuration;  // planning configuration at the end of the trajectory
};

}  // namespace sampler_common

#endif  // SAMPLER_COMMON__STRUCTURES_HPP_
