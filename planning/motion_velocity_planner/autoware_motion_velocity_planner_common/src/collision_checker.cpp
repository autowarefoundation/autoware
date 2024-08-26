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

#include "autoware/motion_velocity_planner_common/collision_checker.hpp"

namespace autoware::motion_velocity_planner
{
CollisionChecker::CollisionChecker(autoware::universe_utils::MultiPolygon2d trajectory_footprints)
: trajectory_footprints_(std::move(trajectory_footprints))
{
  std::vector<RtreeNode> nodes;
  nodes.reserve(trajectory_footprints_.size());
  for (auto i = 0UL; i < trajectory_footprints_.size(); ++i) {
    nodes.emplace_back(
      boost::geometry::return_envelope<autoware::universe_utils::Box2d>(trajectory_footprints_[i]),
      i);
  }
  rtree_ = std::make_shared<Rtree>(nodes.begin(), nodes.end());
}

template <class Geometry>
std::vector<Collision> CollisionChecker::get_collisions(const Geometry & geometry) const
{
  std::vector<Collision> collisions;
  std::vector<RtreeNode> approximate_results;
  autoware::universe_utils::MultiPoint2d intersections;
  ;
  rtree_->query(bgi::intersects(geometry), std::back_inserter(approximate_results));
  for (const auto & result : approximate_results) {
    intersections.clear();
    boost::geometry::intersection(trajectory_footprints_[result.second], geometry, intersections);
    if (!intersections.empty()) {
      Collision c;
      c.trajectory_index = result.second;
      c.collision_points = intersections;
      collisions.push_back(c);
    }
  }
  return collisions;
}

template std::vector<Collision> CollisionChecker::get_collisions<autoware::universe_utils::Point2d>(
  const autoware::universe_utils::Point2d & geometry) const;
template std::vector<Collision> CollisionChecker::get_collisions<autoware::universe_utils::Line2d>(
  const autoware::universe_utils::Line2d & geometry) const;
template std::vector<Collision>
CollisionChecker::get_collisions<autoware::universe_utils::MultiPolygon2d>(
  const autoware::universe_utils::MultiPolygon2d & geometry) const;

// @warn Multi geometries usually lead to very inefficient queries
template std::vector<Collision>
CollisionChecker::get_collisions<autoware::universe_utils::MultiPoint2d>(
  const autoware::universe_utils::MultiPoint2d & geometry) const;
template std::vector<Collision>
CollisionChecker::get_collisions<autoware::universe_utils::MultiLineString2d>(
  const autoware::universe_utils::MultiLineString2d & geometry) const;
template std::vector<Collision> CollisionChecker::get_collisions<
  autoware::universe_utils::Polygon2d>(const autoware::universe_utils::Polygon2d & geometry) const;
}  // namespace autoware::motion_velocity_planner
