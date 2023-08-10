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

#include "behavior_path_planner/utils/drivable_area_expansion/footprints.hpp"

#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/assign.hpp>
#include <boost/geometry.hpp>

#include <tf2/utils.h>

namespace drivable_area_expansion
{
polygon_t translatePolygon(const polygon_t & polygon, const double x, const double y)
{
  polygon_t translated_polygon;
  const boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translation(x, y);
  boost::geometry::transform(polygon, translated_polygon, translation);
  return translated_polygon;
}

polygon_t createFootprint(const geometry_msgs::msg::Pose & pose, const polygon_t base_footprint)
{
  const auto angle = tf2::getYaw(pose.orientation);
  return translatePolygon(rotatePolygon(base_footprint, angle), pose.position.x, pose.position.y);
}

multi_polygon_t createObjectFootprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params)
{
  multi_polygon_t footprints;
  if (params.avoid_dynamic_objects) {
    for (const auto & object : objects.objects) {
      const auto front = object.shape.dimensions.x / 2 + params.dynamic_objects_extra_front_offset;
      const auto rear = -object.shape.dimensions.x / 2 - params.dynamic_objects_extra_rear_offset;
      const auto left = object.shape.dimensions.y / 2 + params.dynamic_objects_extra_left_offset;
      const auto right = -object.shape.dimensions.y / 2 - params.dynamic_objects_extra_right_offset;
      polygon_t base_footprint;
      base_footprint.outer() = {
        point_t{front, left}, point_t{front, right}, point_t{rear, right}, point_t{rear, left},
        point_t{front, left}};
      for (const auto & path : object.kinematics.predicted_paths)
        for (const auto & pose : path.path)
          footprints.push_back(createFootprint(pose, base_footprint));
    }
  }
  return footprints;
}

multi_polygon_t createPathFootprints(
  const PathWithLaneId & path, const DrivableAreaExpansionParameters & params)
{
  const auto left = params.ego_left_offset + params.ego_extra_left_offset;
  const auto right = params.ego_right_offset - params.ego_extra_right_offset;
  const auto rear = params.ego_rear_offset - params.ego_extra_rear_offset;
  const auto front = params.ego_front_offset + params.ego_extra_front_offset;
  polygon_t base_footprint;
  base_footprint.outer() = {
    point_t{front, left}, point_t{front, right}, point_t{rear, right}, point_t{rear, left},
    point_t{front, left}};
  multi_polygon_t footprints;
  // skip the last footprint as its orientation is usually wrong
  footprints.reserve(path.points.size() - 1);
  double arc_length = 0.0;
  for (auto it = path.points.begin(); std::next(it) != path.points.end(); ++it) {
    footprints.push_back(createFootprint(it->point.pose, base_footprint));
    if (params.max_path_arc_length > 0.0) {
      arc_length += tier4_autoware_utils::calcDistance2d(it->point.pose, std::next(it)->point.pose);
      if (arc_length > params.max_path_arc_length) break;
    }
  }
  return footprints;
}
}  // namespace drivable_area_expansion
