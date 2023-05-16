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

#ifndef SAMPLER_COMMON__CONSTRAINTS__MAP_CONSTRAINTS_HPP_
#define SAMPLER_COMMON__CONSTRAINTS__MAP_CONSTRAINTS_HPP_

#include "sampler_common/structures.hpp"

#include <boost/geometry/algorithms/within.hpp>

#include <vector>

namespace sampler_common::constraints
{
struct MapConstraints
{
  MapConstraints(
    const Point2d & ego_pose, const lanelet::LaneletMapConstPtr & map_ptr,
    const std::vector<lanelet::Ids> route_ids)
  {
  }
  /// @brief check the given path and return the corresponding cost
  /// \return cost of the path. If negative then the path is invalid.
  double check(const Path & path)
  {
    double cost = 0.0;
    for (const auto & p : path.points) {
      bool drivable = false;
      for (size_t i = 0; i < drivable_polygons.size(); ++i) {
        if (boost::geometry::within(p, drivable_polygons[i])) {
          drivable = true;
          cost += polygon_costs[i];
          break;
        }
      }
      if (!drivable) {
        cost = -1;
        break;
      }
    }
    return cost;
  }

  MultiPolygon2d drivable_polygons;
  std::vector<double> polygon_costs;
};
}  // namespace sampler_common::constraints

#endif  // SAMPLER_COMMON__CONSTRAINTS__MAP_CONSTRAINTS_HPP_
