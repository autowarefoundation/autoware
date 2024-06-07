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

#include "autoware_sampler_common/constraints/footprint.hpp"

#include "autoware_sampler_common/structures.hpp"

#include <eigen3/Eigen/Core>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/correct.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace autoware::sampler_common::constraints
{

namespace
{
const auto to_eigen = [](const Point2d & p) { return Eigen::Vector2d(p.x(), p.y()); };
}  // namespace

MultiPoint2d buildFootprintPoints(const Path & path, const Constraints & constraints)
{
  MultiPoint2d footprint;

  footprint.reserve(path.points.size() * 4);
  for (auto i = 0UL; i < path.points.size(); ++i) {
    const Eigen::Vector2d p = to_eigen(path.points[i]);
    const double heading = path.yaws[i];
    Eigen::Matrix2d rotation;
    rotation << std::cos(heading), -std::sin(heading), std::sin(heading), std::cos(heading);
    for (const auto & fp : constraints.ego_footprint) {
      const Eigen::Vector2d fp_point = p + rotation * fp;
      footprint.emplace_back(fp_point.x(), fp_point.y());
    }
  }
  return footprint;
}
}  // namespace autoware::sampler_common::constraints
