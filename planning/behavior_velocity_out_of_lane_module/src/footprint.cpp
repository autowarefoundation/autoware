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

#include "footprint.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <tf2/utils.h>

#include <vector>

namespace behavior_velocity_planner::out_of_lane
{
tier4_autoware_utils::Polygon2d make_base_footprint(
  const PlannerParam & p, const bool ignore_offset)
{
  tier4_autoware_utils::Polygon2d base_footprint;
  const auto front_offset = ignore_offset ? 0.0 : p.extra_front_offset;
  const auto rear_offset = ignore_offset ? 0.0 : p.extra_rear_offset;
  const auto right_offset = ignore_offset ? 0.0 : p.extra_right_offset;
  const auto left_offset = ignore_offset ? 0.0 : p.extra_left_offset;
  base_footprint.outer() = {
    {p.front_offset + front_offset, p.left_offset + left_offset},
    {p.front_offset + front_offset, p.right_offset - right_offset},
    {p.rear_offset - rear_offset, p.right_offset - right_offset},
    {p.rear_offset - rear_offset, p.left_offset + left_offset}};
  return base_footprint;
}

lanelet::BasicPolygon2d project_to_pose(
  const tier4_autoware_utils::Polygon2d & base_footprint, const geometry_msgs::msg::Pose & pose)
{
  const auto angle = tf2::getYaw(pose.orientation);
  const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
  lanelet::BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + pose.position.x, p.y() + pose.position.y);
  return footprint;
}

std::vector<lanelet::BasicPolygon2d> calculate_path_footprints(
  const EgoData & ego_data, const PlannerParam & params)
{
  const auto base_footprint = make_base_footprint(params);
  std::vector<lanelet::BasicPolygon2d> path_footprints;
  path_footprints.reserve(ego_data.path.points.size());
  for (auto i = ego_data.first_path_idx; i < ego_data.path.points.size(); ++i) {
    const auto & path_pose = ego_data.path.points[i].point.pose;
    const auto angle = tf2::getYaw(path_pose.orientation);
    const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
    lanelet::BasicPolygon2d footprint;
    for (const auto & p : rotated_footprint.outer())
      footprint.emplace_back(p.x() + path_pose.position.x, p.y() + path_pose.position.y);
    path_footprints.push_back(footprint);
  }
  return path_footprints;
}

lanelet::BasicPolygon2d calculate_current_ego_footprint(
  const EgoData & ego_data, const PlannerParam & params, const bool ignore_offset)
{
  const auto base_footprint = make_base_footprint(params, ignore_offset);
  const auto angle = tf2::getYaw(ego_data.pose.orientation);
  const auto rotated_footprint = tier4_autoware_utils::rotatePolygon(base_footprint, angle);
  lanelet::BasicPolygon2d footprint;
  for (const auto & p : rotated_footprint.outer())
    footprint.emplace_back(p.x() + ego_data.pose.position.x, p.y() + ego_data.pose.position.y);
  return footprint;
}
}  // namespace behavior_velocity_planner::out_of_lane
