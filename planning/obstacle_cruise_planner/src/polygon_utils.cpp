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

#include "obstacle_cruise_planner/polygon_utils.hpp"

#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"

namespace
{
namespace bg = boost::geometry;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & base_step_pose, const geometry_msgs::msg::Pose & next_step_pose,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width)
{
  Polygon2d polygon;

  const double longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + expand_width;
  const double rear_overhang = vehicle_info.rear_overhang_m;

  {  // base step
    appendPointToPolygon(
      polygon, tier4_autoware_utils::calcOffsetPose(base_step_pose, longitudinal_offset, width, 0.0)
                 .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(base_step_pose, longitudinal_offset, -width, 0.0)
        .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(base_step_pose, -rear_overhang, -width, 0.0).position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(base_step_pose, -rear_overhang, width, 0.0).position);
  }

  {  // next step
    appendPointToPolygon(
      polygon, tier4_autoware_utils::calcOffsetPose(next_step_pose, longitudinal_offset, width, 0.0)
                 .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(next_step_pose, longitudinal_offset, -width, 0.0)
        .position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(next_step_pose, -rear_overhang, -width, 0.0).position);
    appendPointToPolygon(
      polygon,
      tier4_autoware_utils::calcOffsetPose(next_step_pose, -rear_overhang, width, 0.0).position);
  }

  polygon = tier4_autoware_utils::isClockwise(polygon)
              ? polygon
              : tier4_autoware_utils::inverseClockwise(polygon);

  Polygon2d hull_polygon;
  bg::convex_hull(polygon, hull_polygon);

  return hull_polygon;
}
}  // namespace

namespace polygon_utils
{
boost::optional<size_t> getFirstCollisionIndex(
  const std::vector<Polygon2d> & traj_polygons, const Polygon2d & obj_polygon,
  std::vector<geometry_msgs::msg::Point> & collision_geom_points)
{
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    std::deque<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          geometry_msgs::msg::Point collision_geom_point;
          collision_geom_point.x = collision_point.x();
          collision_geom_point.y = collision_point.y();
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      return i;
    }
  }

  return {};
}

boost::optional<size_t> getFirstNonCollisionIndex(
  const std::vector<Polygon2d> & traj_polygons,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape, const size_t start_idx)
{
  constexpr double epsilon = 1e-3;

  size_t latest_collision_idx = start_idx;
  for (const auto & path_point : predicted_path.path) {
    const auto obj_polygon = tier4_autoware_utils::toPolygon2d(path_point, shape);
    for (size_t i = start_idx; i < traj_polygons.size(); ++i) {
      const double dist = bg::distance(traj_polygons.at(i), obj_polygon);
      if (dist <= epsilon) {
        latest_collision_idx = i;
        break;
      }
      if (i == traj_polygons.size() - 1) {
        return latest_collision_idx;
      }
    }
  }
  return {};
}

boost::optional<size_t> willCollideWithSurroundObstacle(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const std::vector<Polygon2d> & traj_polygons,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape, const double max_dist,
  const double ego_obstacle_overlap_time_threshold,
  const double max_prediction_time_for_collision_check,
  std::vector<geometry_msgs::msg::Point> & collision_geom_points)
{
  constexpr double epsilon = 1e-3;

  bool is_found = false;
  size_t start_predicted_path_idx = 0;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    const auto & path_point = predicted_path.path.at(i);
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      return {};
    }

    for (size_t j = 0; j < traj.points.size(); ++j) {
      const auto & traj_point = traj.points.at(j);
      const double approximated_dist =
        tier4_autoware_utils::calcDistance2d(path_point.position, traj_point.pose.position);
      if (approximated_dist > max_dist) {
        continue;
      }

      const auto & traj_polygon = traj_polygons.at(j);
      const auto obj_polygon = tier4_autoware_utils::toPolygon2d(path_point, shape);
      const double dist = bg::distance(traj_polygon, obj_polygon);

      if (dist < epsilon) {
        if (!is_found) {
          // calculate collision point by polygon collision
          std::deque<Polygon2d> collision_polygons;
          boost::geometry::intersection(traj_polygon, obj_polygon, collision_polygons);

          bool has_collision = false;
          for (const auto & collision_polygon : collision_polygons) {
            if (boost::geometry::area(collision_polygon) > 0.0) {
              has_collision = true;

              for (const auto & collision_point : collision_polygon.outer()) {
                geometry_msgs::msg::Point collision_geom_point;
                collision_geom_point.x = collision_point.x();
                collision_geom_point.y = collision_point.y();
                collision_geom_points.push_back(collision_geom_point);
              }
            }
          }

          if (has_collision) {
            start_predicted_path_idx = i;
            is_found = true;
          }
        } else {
          const double overlap_time = (static_cast<double>(i) - start_predicted_path_idx) *
                                      rclcpp::Duration(predicted_path.time_step).seconds();
          if (ego_obstacle_overlap_time_threshold < overlap_time) {
            return j;
          }
        }
      } else {
        is_found = false;
      }
    }
  }

  collision_geom_points.clear();
  return {};
}

std::vector<Polygon2d> createOneStepPolygons(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width)
{
  std::vector<Polygon2d> polygons;

  for (size_t i = 0; i < traj.points.size(); ++i) {
    const auto polygon = [&]() {
      if (i == 0) {
        return createOneStepPolygon(
          traj.points.at(i).pose, traj.points.at(i).pose, vehicle_info, expand_width);
      }
      return createOneStepPolygon(
        traj.points.at(i - 1).pose, traj.points.at(i).pose, vehicle_info, expand_width);
    }();

    polygons.push_back(polygon);
  }
  return polygons;
}
}  // namespace polygon_utils
