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

#include <grid_map_core/GridMap.hpp>
#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>

#include <autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <utility>
#include <vector>

namespace test
{
// Default grid parameter such that UNKNOWN cells have a value of 50
const behavior_velocity_planner::grid_utils::GridParam grid_param = {10, 51};
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using ConstPair = const std::pair<double, double>;

/* lanelet
        0 1 2 3 4 5
      0 x
      1   x
      2     x
      3       x
      4         x
      5           x
      */
inline lanelet::ConstLanelet createLanelet(
  ConstPair & start = {0, 0}, ConstPair & end = {5, 5}, int nb_points = 6)
{
  const double interval =
    std::hypot(end.first - start.first, end.second - start.second) / (nb_points - 1);
  const double norm_x = (end.first - start.first) / interval;
  const double norm_y = (end.second - start.second) / interval;
  lanelet::Points3d path_points;
  for (int i = 0; i < nb_points; i++) {
    const double x = start.first + norm_x * i;
    const double y = start.second + norm_y * i;
    path_points.emplace_back(lanelet::InvalId, x, y, 0);
  }
  lanelet::LineString3d centerline(lanelet::InvalId, path_points);
  lanelet::Lanelet path_lanelet(lanelet::InvalId);
  path_lanelet.setCenterline(centerline);
  return lanelet::ConstLanelet(path_lanelet);
}

/* Horizontal lanelet
        0 1 2 3 4 5 6
      0 x x x x x x x
      1
      2 x x x x x x x
      3
      4
      5
      */
inline lanelet::ConstLanelet horizontalLanelet(
  std::pair<double, double> origin = {0, 0}, double width = 2.0, double length = 6.0,
  int nb_points = 2)
{
  lanelet::Lanelet l;
  lanelet::Points3d line;
  for (double x = origin.first; x <= origin.first + length; x += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, x, origin.second));
  }
  l.setLeftBound(lanelet::LineString3d(0, line));
  line.clear();
  for (double x = origin.second; x <= origin.first + length; x += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, x, origin.second + width));
  }
  l.setRightBound(lanelet::LineString3d(1, line));
  return lanelet::ConstLanelet(l);
}
/* Vertical lanelet
        0 1 2 3 4 5 6
      0 x   x
      1 x   x
      2 x   x
      3 x   x
      4 x   x
      5 x   x
      */
inline lanelet::ConstLanelet verticalLanelet(
  std::pair<double, double> origin = {0, 0}, double width = 2.0, double length = 5.0,
  int nb_points = 2)
{
  lanelet::Lanelet l;
  lanelet::Points3d line;
  for (double y = origin.second; y <= origin.second + length; y += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, origin.first, y));
  }
  l.setLeftBound(lanelet::LineString3d(0, line));
  line.clear();
  for (double y = origin.second; y <= origin.second + length; y += length / (nb_points - 1)) {
    line.emplace_back(lanelet::ConstPoint2d(0, origin.first + width, y));
  }
  l.setRightBound(lanelet::LineString3d(1, line));
  return lanelet::ConstLanelet(l);
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
inline void addConstantVelocity(
  autoware_auto_planning_msgs::msg::PathWithLaneId & trajectory, double velocity)
{
  for (auto & p : trajectory.points) {
    p.point.longitudinal_velocity_mps = velocity;
  }
}
inline autoware_auto_perception_msgs::msg::PredictedObject generatePredictedObject(double x)
{
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  obj.shape.dimensions.x = 5.0;
  obj.shape.dimensions.y = 2.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = x;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0;
  obj.classification.push_back(autoware_auto_perception_msgs::msg::ObjectClassification{});
  obj.classification.at(0).label = autoware_auto_perception_msgs::msg::ObjectClassification::CAR;
  return obj;
}
inline geometry_msgs::msg::Pose generatePose(double x)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = 0.0;
  p.position.z = 1.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  p.orientation = tf2::toMsg(q);
  return p;
}

}  // namespace test

#endif  // UTILS_HPP_
