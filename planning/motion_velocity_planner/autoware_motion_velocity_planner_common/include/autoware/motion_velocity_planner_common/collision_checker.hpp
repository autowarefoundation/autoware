// Copyright 2024 Autoware Foundation
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

#ifndef AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__COLLISION_CHECKER_HPP_
#define AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__COLLISION_CHECKER_HPP_

#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/envelope.hpp>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner
{
namespace bgi = boost::geometry::index;
using RtreeNode = std::pair<autoware::universe_utils::Box2d, size_t>;
using Rtree = bgi::rtree<RtreeNode, bgi::rstar<16>>;

/// @brief collision as a trajectory index and corresponding collision points
struct Collision
{
  size_t trajectory_index{};
  autoware::universe_utils::MultiPoint2d collision_points;
};

/// @brief collision checker for a trajectory as a sequence of 2D footprints
class CollisionChecker
{
  const autoware::universe_utils::MultiPolygon2d trajectory_footprints_;
  std::shared_ptr<Rtree> rtree_;

public:
  /// @brief construct the collisions checker
  /// @param trajectory_footprints footprints of the trajectory to be used for collision checking
  /// @details internally, the rtree is built with the packing algorithm
  explicit CollisionChecker(autoware::universe_utils::MultiPolygon2d trajectory_footprints);

  /// @brief efficiently calculate collisions with a geometry
  /// @tparam Geometry boost geometry type
  /// @param geometry geometry to check for collisions
  /// @return collisions between the trajectory footprints and the input geometry
  template <class Geometry>
  [[nodiscard]] std::vector<Collision> get_collisions(const Geometry & geometry) const;

  /// @brief direct access to the rtree storing the polygon footprints
  /// @return rtree of the polygon footprints
  [[nodiscard]] std::shared_ptr<const Rtree> get_rtree() const { return rtree_; }

  /// @brief get the size of the trajectory used by this collision checker
  [[nodiscard]] size_t trajectory_size() const { return trajectory_footprints_.size(); }
};
}  // namespace autoware::motion_velocity_planner

#endif  // AUTOWARE__MOTION_VELOCITY_PLANNER_COMMON__COLLISION_CHECKER_HPP_
