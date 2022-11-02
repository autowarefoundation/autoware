// Copyright 2022 TIER IV, Inc.
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

#include "obstacle_velocity_limiter/obstacles.hpp"

#include <obstacle_velocity_limiter/distance.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/detail/distance/interface.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <algorithm>
#include <limits>

namespace obstacle_velocity_limiter
{
namespace bg = boost::geometry;

std::optional<double> distanceToClosestCollision(
  const linestring_t & projection, const polygon_t & footprint,
  const CollisionChecker & collision_checker, const ProjectionParameters & params)
{
  std::optional<double> distance;
  if (projection.empty()) return distance;
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & obs_point : collision_checker.intersections(footprint)) {
    if (params.distance_method == ProjectionParameters::EXACT) {
      if (params.model == ProjectionParameters::PARTICLE) {
        const auto euclidian_dist = bg::distance(obs_point, projection.front());
        const auto collision_heading = std::atan2(
          obs_point.y() - projection.front().y(), obs_point.x() - projection.front().x());
        const auto angle = params.heading - collision_heading;
        const auto long_dist = std::abs(std::cos(angle)) * euclidian_dist;
        min_dist = std::min(min_dist, long_dist);
      } else {  // BICYCLE model with curved projection
        min_dist = std::min(min_dist, arcDistance(projection.front(), params.heading, obs_point));
      }
    } else {  // APPROXIMATION
      const auto euclidian_dist = bg::distance(obs_point, projection.front());
      min_dist = std::min(min_dist, euclidian_dist);
    }
  }
  if (min_dist != std::numeric_limits<double>::max()) distance = min_dist;
  return distance;
}

double arcDistance(const point_t & origin, const double heading, const point_t & target)
{
  // Circle passing through the origin and the target such that origin+heading is tangent
  const auto d_normal = point_t{-std::sin(heading), std::cos(heading)};
  const auto midpoint = (origin + target) / 2;
  const auto mid_to_target = target - midpoint;
  const auto circle_center =
    origin + (midpoint - origin).dot(mid_to_target) / (mid_to_target.dot(d_normal)) * d_normal;
  const auto squared_radius = (circle_center - origin).squaredNorm();
  // Arc distance from angle between origin and target on the origin
  const auto angle =
    std::acos((2 * squared_radius - (origin - target).squaredNorm()) / (2 * squared_radius));
  return std::sqrt(squared_radius) * angle;
}
}  // namespace obstacle_velocity_limiter
