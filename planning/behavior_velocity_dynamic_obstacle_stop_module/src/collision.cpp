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

#include "collision.hpp"

#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <boost/geometry.hpp>

#include <optional>
#include <vector>

namespace behavior_velocity_planner::dynamic_obstacle_stop
{

std::optional<geometry_msgs::msg::Point> find_earliest_collision(
  const EgoData & ego_data,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
  const tier4_autoware_utils::MultiPolygon2d & obstacle_forward_footprints, DebugData & debug_data)
{
  auto earliest_idx = ego_data.path_footprints.size();
  auto earliest_arc_length = motion_utils::calcArcLength(ego_data.path.points);
  std::optional<geometry_msgs::msg::Point> earliest_collision_point;
  debug_data.collisions.clear();
  std::vector<BoxIndexPair> rough_collisions;
  for (auto obstacle_idx = 0UL; obstacle_idx < objects.size(); ++obstacle_idx) {
    rough_collisions.clear();
    const auto & obstacle_pose = objects[obstacle_idx].kinematics.initial_pose_with_covariance.pose;
    const auto & obstacle_footprint = obstacle_forward_footprints[obstacle_idx];
    ego_data.rtree.query(
      boost::geometry::index::intersects(obstacle_footprint), std::back_inserter(rough_collisions));
    for (const auto & rough_collision : rough_collisions) {
      const auto path_idx = rough_collision.second;
      const auto & ego_footprint = ego_data.path_footprints[path_idx];
      const auto & ego_pose = ego_data.path.points[ego_data.first_path_idx + path_idx].point.pose;
      const auto angle_diff = tier4_autoware_utils::normalizeRadian(
        tf2::getYaw(ego_pose.orientation) - tf2::getYaw(obstacle_pose.orientation));
      if (path_idx <= earliest_idx && std::abs(angle_diff) > (M_PI_2 + M_PI_4)) {
        tier4_autoware_utils::MultiPoint2d collision_points;
        boost::geometry::intersection(
          obstacle_footprint.outer(), ego_footprint.outer(), collision_points);
        earliest_idx = path_idx;
        for (const auto & coll_p : collision_points) {
          auto p = geometry_msgs::msg::Point().set__x(coll_p.x()).set__y(coll_p.y());
          const auto arc_length =
            motion_utils::calcSignedArcLength(ego_data.path.points, ego_data.first_path_idx, p);
          if (arc_length < earliest_arc_length) {
            earliest_arc_length = arc_length;
            debug_data.collisions = {obstacle_footprint, ego_footprint};
            earliest_collision_point = p;
          }
        }
      }
    }
  }
  return earliest_collision_point;
}

}  // namespace behavior_velocity_planner::dynamic_obstacle_stop
