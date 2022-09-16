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

#include "motion_utils/trajectory/trajectory.hpp"
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
boost::optional<size_t> getCollisionIndex(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const std::vector<Polygon2d> & traj_polygons, const geometry_msgs::msg::PoseStamped & obj_pose,
  const autoware_auto_perception_msgs::msg::Shape & shape,
  std::vector<geometry_msgs::msg::PointStamped> & collision_geom_points, const double max_dist)
{
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(obj_pose.pose, shape);
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist =
      tier4_autoware_utils::calcDistance2d(traj.points.at(i).pose, obj_pose.pose);
    if (approximated_dist > max_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          geometry_msgs::msg::PointStamped collision_geom_point;
          collision_geom_point.header = obj_pose.header;
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
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

std::vector<geometry_msgs::msg::PointStamped> getCollisionPoints(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const std::vector<Polygon2d> & traj_polygons, const std_msgs::msg::Header & obj_header,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape, const rclcpp::Time & current_time,
  const double vehicle_max_longitudinal_offset, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_dist,
  const double max_prediction_time_for_collision_check)
{
  std::vector<geometry_msgs::msg::PointStamped> collision_points;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      break;
    }

    const auto object_time =
      rclcpp::Time(obj_header.stamp) + rclcpp::Duration(predicted_path.time_step) * i;
    // Ignore past position
    if ((object_time - current_time).seconds() < 0.0) {
      continue;
    }

    geometry_msgs::msg::PoseStamped obj_pose;
    obj_pose.header.frame_id = obj_header.frame_id;
    obj_pose.header.stamp = object_time;
    obj_pose.pose = predicted_path.path.at(i);

    std::vector<geometry_msgs::msg::PointStamped> current_collision_points;
    const auto collision_idx =
      getCollisionIndex(traj, traj_polygons, obj_pose, shape, current_collision_points, max_dist);
    if (collision_idx) {
      const auto nearest_collision_point = calcNearestCollisionPoint(
        *collision_idx, current_collision_points, traj, vehicle_max_longitudinal_offset,
        is_driving_forward);
      collision_points.push_back(nearest_collision_point);
      collision_index.push_back(*collision_idx);
    }
  }

  return collision_points;
}

std::vector<geometry_msgs::msg::PointStamped> willCollideWithSurroundObstacle(
  const autoware_auto_planning_msgs::msg::Trajectory & traj,
  const std::vector<Polygon2d> & traj_polygons, const std_msgs::msg::Header & obj_header,
  const autoware_auto_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_auto_perception_msgs::msg::Shape & shape, const rclcpp::Time & current_time,
  const double max_dist, const double ego_obstacle_overlap_time_threshold,
  const double max_prediction_time_for_collision_check, std::vector<size_t> & collision_index,
  const double vehicle_max_longitudinal_offset, const bool is_driving_forward)
{
  const auto collision_points = getCollisionPoints(
    traj, traj_polygons, obj_header, predicted_path, shape, current_time,
    vehicle_max_longitudinal_offset, is_driving_forward, collision_index, max_dist,
    max_prediction_time_for_collision_check);

  if (collision_points.empty()) {
    return {};
  }

  const double overlap_time = (rclcpp::Time(collision_points.back().header.stamp) -
                               rclcpp::Time(collision_points.front().header.stamp))
                                .seconds();
  if (overlap_time < ego_obstacle_overlap_time_threshold) {
    return {};
  }

  return collision_points;
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

geometry_msgs::msg::PointStamped calcNearestCollisionPoint(
  const size_t & first_within_idx,
  const std::vector<geometry_msgs::msg::PointStamped> & collision_points,
  const autoware_auto_planning_msgs::msg::Trajectory & decimated_traj,
  const double vehicle_max_longitudinal_offset, const bool is_driving_forward)
{
  std::vector<geometry_msgs::msg::Point> segment_points(2);
  if (first_within_idx == 0) {
    const auto & traj_front_pose = decimated_traj.points.at(0).pose;
    const auto front_pos = tier4_autoware_utils::calcOffsetPose(
                             traj_front_pose, vehicle_max_longitudinal_offset, 0.0, 0.0)
                             .position;
    if (is_driving_forward) {
      segment_points.at(0) = traj_front_pose.position;
      segment_points.at(1) = front_pos;
    } else {
      segment_points.at(0) = front_pos;
      segment_points.at(1) = traj_front_pose.position;
    }
  } else {
    const size_t seg_idx = first_within_idx - 1;
    segment_points.at(0) = decimated_traj.points.at(seg_idx).pose.position;
    segment_points.at(1) = decimated_traj.points.at(seg_idx + 1).pose.position;
  }

  size_t min_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t cp_idx = 0; cp_idx < collision_points.size(); ++cp_idx) {
    const auto & collision_point = collision_points.at(cp_idx);
    const double dist =
      motion_utils::calcLongitudinalOffsetToSegment(segment_points, 0, collision_point.point);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = cp_idx;
    }
  }

  return collision_points.at(min_idx);
}
}  // namespace polygon_utils
