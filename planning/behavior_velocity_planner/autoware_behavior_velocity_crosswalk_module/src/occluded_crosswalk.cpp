// Copyright 2024 Tier IV, Inc.
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

#include "occluded_crosswalk.hpp"

#include "autoware/behavior_velocity_crosswalk_module/util.hpp"

#include <autoware_grid_map_utils/polygon_iterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <algorithm>
#include <vector>

namespace autoware::behavior_velocity_planner
{
bool is_occluded(
  const grid_map::GridMap & grid_map, const int min_nb_of_cells, const grid_map::Index idx,
  const autoware::behavior_velocity_planner::CrosswalkModule::PlannerParam & params)
{
  grid_map::Index idx_offset;
  for (idx_offset.x() = 0; idx_offset.x() < min_nb_of_cells; ++idx_offset.x()) {
    for (idx_offset.y() = 0; idx_offset.y() < min_nb_of_cells; ++idx_offset.y()) {
      const auto index = idx + idx_offset;
      if ((index < grid_map.getSize()).all()) {
        const auto cell_value = grid_map.at("layer", index);
        if (
          cell_value < params.occlusion_free_space_max ||
          cell_value > params.occlusion_occupied_min)
          return false;
      }
    }
  }
  return true;
}

lanelet::BasicPoint2d interpolate_point(
  const lanelet::BasicSegment2d & segment, const double extra_distance)
{
  const auto direction_vector = (segment.second - segment.first).normalized();
  return segment.second + extra_distance * direction_vector;
}

std::vector<lanelet::BasicPolygon2d> calculate_detection_areas(
  const lanelet::ConstLanelet & crosswalk_lanelet, const lanelet::BasicPoint2d & crosswalk_origin,
  const double detection_range)
{
  std::vector<lanelet::BasicPolygon2d> detection_areas = {
    crosswalk_lanelet.polygon2d().basicPolygon()};
  const std::vector<std::function<lanelet::BasicSegment2d(lanelet::ConstLineString2d)>>
    segment_getters = {
      [](const auto & ls) -> lanelet::BasicSegment2d {
        return {ls[1].basicPoint2d(), ls[0].basicPoint2d()};
      },
      [](const auto & ls) -> lanelet::BasicSegment2d {
        return {ls[ls.size() - 2].basicPoint2d(), ls[ls.size() - 1].basicPoint2d()};
      }};
  if (
    crosswalk_lanelet.centerline2d().size() > 1 && crosswalk_lanelet.leftBound2d().size() > 1 &&
    crosswalk_lanelet.rightBound2d().size() > 1) {
    for (const auto & segment_getter : segment_getters) {
      const auto center_segment = segment_getter(crosswalk_lanelet.centerline2d());
      const auto left_segment = segment_getter(crosswalk_lanelet.leftBound2d());
      const auto right_segment = segment_getter(crosswalk_lanelet.rightBound2d());
      const auto dist = lanelet::geometry::distance2d(center_segment.second, crosswalk_origin);
      if (dist < detection_range) {
        const auto target_left = interpolate_point(left_segment, detection_range - dist);
        const auto target_right = interpolate_point(right_segment, detection_range - dist);
        detection_areas.push_back(
          {left_segment.second, target_left, target_right, right_segment.second});
      }
    }
  }
  return detection_areas;
}

std::vector<autoware_perception_msgs::msg::PredictedObject> select_and_inflate_objects(
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects,
  const std::vector<double> & velocity_thresholds, const double inflate_size)
{
  std::vector<autoware_perception_msgs::msg::PredictedObject> selected_objects;
  for (const auto & o : objects) {
    const auto vel_threshold = velocity_thresholds[o.classification.front().label];
    if (o.kinematics.initial_twist_with_covariance.twist.linear.x >= vel_threshold) {
      auto selected_object = o;
      selected_object.shape.dimensions.x += inflate_size;
      selected_object.shape.dimensions.y += inflate_size;
      selected_objects.push_back(selected_object);
    }
  }
  return selected_objects;
}

void clear_occlusions_behind_objects(
  grid_map::GridMap & grid_map,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects)
{
  const auto angle_cmp = [&](const auto & p1, const auto & p2) {
    const auto d1 = p1 - grid_map.getPosition();
    const auto d2 = p2 - grid_map.getPosition();
    return std::atan2(d1.y(), d1.x()) < std::atan2(d2.y(), d2.x());
  };
  const lanelet::BasicPoint2d grid_map_position = grid_map.getPosition();
  const auto range = grid_map.getLength().maxCoeff() / 2.0;
  for (auto object : objects) {
    const lanelet::BasicPoint2d object_position = {
      object.kinematics.initial_pose_with_covariance.pose.position.x,
      object.kinematics.initial_pose_with_covariance.pose.position.y};
    if (lanelet::geometry::distance2d(grid_map_position, object_position) < range) {
      lanelet::BasicPoints2d edge_points;
      const auto object_polygon = autoware::universe_utils::toPolygon2d(object);
      for (const auto & edge_point : object_polygon.outer()) edge_points.push_back(edge_point);
      std::sort(edge_points.begin(), edge_points.end(), angle_cmp);
      // points.push_back(interpolate_point({object_position, edge_point}, 10.0 * range));
      grid_map::Polygon polygon_to_clear;
      polygon_to_clear.addVertex(edge_points.front());
      polygon_to_clear.addVertex(
        interpolate_point({grid_map_position, edge_points.front()}, 10.0 * range));
      polygon_to_clear.addVertex(
        interpolate_point({grid_map_position, edge_points.back()}, 10.0 * range));
      polygon_to_clear.addVertex(edge_points.back());
      for (autoware::grid_map_utils::PolygonIterator it(grid_map, polygon_to_clear);
           !it.isPastEnd(); ++it)
        grid_map.at("layer", *it) = 0;
    }
  }
}

bool is_crosswalk_occluded(
  const lanelet::ConstLanelet & crosswalk_lanelet,
  const nav_msgs::msg::OccupancyGrid & occupancy_grid,
  const geometry_msgs::msg::Point & path_intersection, const double detection_range,
  const std::vector<autoware_perception_msgs::msg::PredictedObject> & dynamic_objects,
  const autoware::behavior_velocity_planner::CrosswalkModule::PlannerParam & params)
{
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromOccupancyGrid(occupancy_grid, "layer", grid_map);

  if (params.occlusion_ignore_behind_predicted_objects) {
    const auto objects = select_and_inflate_objects(
      dynamic_objects, params.occlusion_ignore_velocity_thresholds,
      params.occlusion_extra_objects_size);
    clear_occlusions_behind_objects(grid_map, objects);
  }
  const auto min_nb_of_cells = std::ceil(params.occlusion_min_size / grid_map.getResolution());
  for (const auto & detection_area : calculate_detection_areas(
         crosswalk_lanelet, {path_intersection.x, path_intersection.y}, detection_range)) {
    grid_map::Polygon poly;
    for (const auto & p : detection_area) poly.addVertex(grid_map::Position(p.x(), p.y()));
    for (autoware::grid_map_utils::PolygonIterator iter(grid_map, poly); !iter.isPastEnd(); ++iter)
      if (is_occluded(grid_map, min_nb_of_cells, *iter, params)) return true;
  }
  return false;
}

double calculate_detection_range(
  const double occluded_object_velocity, const double dist_ego_to_crosswalk,
  const double ego_velocity)
{
  constexpr double min_ego_velocity = 1.0;
  const auto time_to_crosswalk = dist_ego_to_crosswalk / std::max(min_ego_velocity, ego_velocity);
  return time_to_crosswalk > 0.0 ? time_to_crosswalk / occluded_object_velocity : 20.0;
}
}  // namespace autoware::behavior_velocity_planner
