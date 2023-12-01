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
#include "tier4_autoware_utils/geometry/geometry.hpp"

namespace
{
void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point(geom_point.x, geom_point.y);
  bg::append(polygon.outer(), point);
}

geometry_msgs::msg::Point calcOffsetPosition(
  const geometry_msgs::msg::Pose & pose, const double offset_x, const double offset_y)
{
  return tier4_autoware_utils::calcOffsetPose(pose, offset_x, offset_y, 0.0).position;
}

PointWithStamp calcNearestCollisionPoint(
  const size_t first_within_idx, const std::vector<PointWithStamp> & collision_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points, const bool is_driving_forward)
{
  const size_t prev_idx = first_within_idx == 0 ? first_within_idx : first_within_idx - 1;
  const size_t next_idx = first_within_idx == 0 ? first_within_idx + 1 : first_within_idx;

  std::vector<geometry_msgs::msg::Pose> segment_points{
    decimated_traj_points.at(prev_idx).pose, decimated_traj_points.at(next_idx).pose};
  if (!is_driving_forward) {
    std::reverse(segment_points.begin(), segment_points.end());
  }

  std::vector<double> dist_vec;
  for (const auto & collision_point : collision_points) {
    const double dist =
      motion_utils::calcLongitudinalOffsetToSegment(segment_points, 0, collision_point.point);
    dist_vec.push_back(dist);
  }

  const size_t min_idx = std::min_element(dist_vec.begin(), dist_vec.end()) - dist_vec.begin();
  return collision_points.at(min_idx);
}

// NOTE: max_lat_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::optional<std::pair<size_t, std::vector<PointWithStamp>>> getCollisionIndex(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const geometry_msgs::msg::Pose & object_pose, const rclcpp::Time & object_time,
  const Shape & object_shape, const double max_lat_dist = std::numeric_limits<double>::max())
{
  const auto obj_polygon = tier4_autoware_utils::toPolygon2d(object_pose, object_shape);
  for (size_t i = 0; i < traj_polygons.size(); ++i) {
    const double approximated_dist =
      tier4_autoware_utils::calcDistance2d(traj_points.at(i).pose, object_pose);
    if (approximated_dist > max_lat_dist) {
      continue;
    }

    std::vector<Polygon2d> collision_polygons;
    boost::geometry::intersection(traj_polygons.at(i), obj_polygon, collision_polygons);

    std::vector<PointWithStamp> collision_geom_points;
    bool has_collision = false;
    for (const auto & collision_polygon : collision_polygons) {
      if (boost::geometry::area(collision_polygon) > 0.0) {
        has_collision = true;

        for (const auto & collision_point : collision_polygon.outer()) {
          PointWithStamp collision_geom_point;
          collision_geom_point.stamp = object_time;
          collision_geom_point.point.x = collision_point.x();
          collision_geom_point.point.y = collision_point.y();
          collision_geom_point.point.z = traj_points.at(i).pose.position.z;
          collision_geom_points.push_back(collision_geom_point);
        }
      }
    }

    if (has_collision) {
      const auto collision_info =
        std::pair<size_t, std::vector<PointWithStamp>>{i, collision_geom_points};
      return collision_info;
    }
  }

  return std::nullopt;
}
}  // namespace

namespace polygon_utils
{
Polygon2d createOneStepPolygon(
  const std::vector<geometry_msgs::msg::Pose> & last_poses,
  const std::vector<geometry_msgs::msg::Pose> & current_poses,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double lat_margin)
{
  Polygon2d polygon;

  const double base_to_front = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + lat_margin;
  const double base_to_rear = vehicle_info.rear_overhang_m;

  for (auto & pose : last_poses) {
    appendPointToPolygon(polygon, calcOffsetPosition(pose, base_to_front, width));
    appendPointToPolygon(polygon, calcOffsetPosition(pose, base_to_front, -width));
    appendPointToPolygon(polygon, calcOffsetPosition(pose, -base_to_rear, -width));
    appendPointToPolygon(polygon, calcOffsetPosition(pose, -base_to_rear, width));
  }
  for (auto & pose : current_poses) {
    appendPointToPolygon(polygon, calcOffsetPosition(pose, base_to_front, width));
    appendPointToPolygon(polygon, calcOffsetPosition(pose, base_to_front, -width));
    appendPointToPolygon(polygon, calcOffsetPosition(pose, -base_to_rear, -width));
    appendPointToPolygon(polygon, calcOffsetPosition(pose, -base_to_rear, width));
  }

  bg::correct(polygon);

  Polygon2d hull_polygon;
  bg::convex_hull(polygon, hull_polygon);

  return hull_polygon;
}

std::optional<std::pair<geometry_msgs::msg::Point, double>> getCollisionPoint(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const Obstacle & obstacle, const bool is_driving_forward,
  const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const auto collision_info =
    getCollisionIndex(traj_points, traj_polygons, obstacle.pose, obstacle.stamp, obstacle.shape);
  if (!collision_info) {
    return std::nullopt;
  }

  const double x_diff_to_bumper = is_driving_forward ? vehicle_info.max_longitudinal_offset_m
                                                     : vehicle_info.min_longitudinal_offset_m;
  const auto bumper_pose = tier4_autoware_utils::calcOffsetPose(
    traj_points.at(collision_info->first).pose, x_diff_to_bumper, 0.0, 0.0);

  std::optional<double> max_collision_length = std::nullopt;
  std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;
  for (const auto & poly_vertex : collision_info->second) {
    const double dist_from_bumper =
      std::abs(tier4_autoware_utils::inverseTransformPoint(poly_vertex.point, bumper_pose).x);

    if (!max_collision_length.has_value() || dist_from_bumper > *max_collision_length) {
      max_collision_length = dist_from_bumper;
      max_collision_point = poly_vertex.point;
    }
  }
  return std::make_pair(
    *max_collision_point, motion_utils::calcSignedArcLength(traj_points, 0, collision_info->first) -
                            *max_collision_length);
}

// NOTE: max_lat_dist is used for efficient calculation to suppress boost::geometry's polygon
// calculation.
std::vector<PointWithStamp> getCollisionPoints(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const rclcpp::Time & obstacle_stamp, const PredictedPath & predicted_path, const Shape & shape,
  const rclcpp::Time & current_time, const bool is_driving_forward,
  std::vector<size_t> & collision_index, const double max_lat_dist,
  const double max_prediction_time_for_collision_check)
{
  std::vector<PointWithStamp> collision_points;
  for (size_t i = 0; i < predicted_path.path.size(); ++i) {
    if (
      max_prediction_time_for_collision_check <
      rclcpp::Duration(predicted_path.time_step).seconds() * static_cast<double>(i)) {
      break;
    }

    const auto object_time =
      rclcpp::Time(obstacle_stamp) + rclcpp::Duration(predicted_path.time_step) * i;
    // Ignore past position
    if ((object_time - current_time).seconds() < 0.0) {
      continue;
    }

    const auto collision_info = getCollisionIndex(
      traj_points, traj_polygons, predicted_path.path.at(i), object_time, shape, max_lat_dist);
    if (collision_info) {
      const auto nearest_collision_point = calcNearestCollisionPoint(
        collision_info->first, collision_info->second, traj_points, is_driving_forward);
      collision_points.push_back(nearest_collision_point);
      collision_index.push_back(collision_info->first);
    }
  }

  return collision_points;
}

}  // namespace polygon_utils
