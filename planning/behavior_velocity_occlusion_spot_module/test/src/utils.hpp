// Copyright 2021 Tier IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "occlusion_spot_utils.hpp"

#include <grid_map_core/GridMap.hpp>

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <vector>

namespace test
{

inline autoware_auto_planning_msgs::msg::PathWithLaneId generatePath(
  double x0, double y0, double x, double y, int nb_points)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId path{};
  double x_step = (x - x0) / (nb_points - 1);
  double y_step = (y - y0) / (nb_points - 1);
  for (int i = 0; i < nb_points; ++i) {
    autoware_auto_planning_msgs::msg::PathPointWithLaneId point{};
    point.point.pose.position.x = x0 + x_step * i;
    point.point.pose.position.y = y0 + y_step * i;
    point.point.pose.position.z = 0.0;
    path.points.push_back(point);
  }
  return path;
}

// /!\ columns and rows in the GridMap are "inverted" (x -> rows, y -> columns)
inline grid_map::GridMap generateGrid(int w, int h, double res)
{
  grid_map::GridMap grid{};
  grid_map::Length length(w * res, h * res);
  grid.setGeometry(length, res, grid_map::Position(length.x() / 2.0, length.y() / 2.0));
  grid.add("layer", behavior_velocity_planner::grid_utils::occlusion_cost_value::FREE_SPACE);
  return grid;
}

using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
inline void generatePossibleCollisions(
  std::vector<PossibleCollisionInfo> & possible_collisions, double x0, double y0, double x,
  double y, int nb_cols)
{
  using behavior_velocity_planner::occlusion_spot_utils::ObstacleInfo;
  using behavior_velocity_planner::occlusion_spot_utils::PossibleCollisionInfo;
  const double lon = 0.0;  // assume col_x = intersection_x
  const double lat = -1.0;
  const double velocity = 1.0;
  /**
   * @brief representation of a possible collision between ego and some obstacle
   *                                      ^
   *                                      |
   * Ego ---------collision----------intersection-------> path
   *                                      |
   *             ------------------       |
   *            |     Vehicle      |   obstacle
   *             ------------------
   */
  double x_step = (x - x0) / (nb_cols - 1);
  double y_step = (y - y0) / (nb_cols - 1);
  for (int i = 0; i < nb_cols; ++i) {
    // collision
    ObstacleInfo obstacle_info;
    obstacle_info.position.x = x0 + x_step * i;
    obstacle_info.position.y = y0 + y_step * i + lat;
    obstacle_info.max_velocity = velocity;

    // intersection
    geometry_msgs::msg::Pose intersection_pose{};
    intersection_pose.position.x = x0 + x_step * i + lon;
    intersection_pose.position.x = y0 + y_step * i;

    // collision path point
    autoware_auto_planning_msgs::msg::PathPoint collision_with_margin{};
    collision_with_margin.pose.position.x = x0 + x_step * i + lon;
    collision_with_margin.pose.position.y = y0 + y_step * i;

    lanelet::ArcCoordinates arc;
    arc.length = obstacle_info.position.x;
    arc.distance = obstacle_info.position.y;

    PossibleCollisionInfo col(obstacle_info, collision_with_margin, intersection_pose, arc);
    possible_collisions.emplace_back(col);
  }
}

}  // namespace test

#endif  // UTILS_HPP_
