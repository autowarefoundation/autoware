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

#include "behavior_path_planner_common/utils/drivable_area_expansion/footprints.hpp"

#include "behavior_path_planner_common/utils/drivable_area_expansion/parameters.hpp"

#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <boost/geometry/strategies/transform/matrix_transformers.hpp>

#include <tf2/utils.h>

namespace drivable_area_expansion
{
Polygon2d translate_polygon(const Polygon2d & polygon, const double x, const double y)
{
  Polygon2d translated_polygon;
  const boost::geometry::strategy::transform::translate_transformer<double, 2, 2> translation(x, y);
  boost::geometry::transform(polygon, translated_polygon, translation);
  return translated_polygon;
}

Polygon2d create_footprint(const geometry_msgs::msg::Pose & pose, const Polygon2d base_footprint)
{
  const auto angle = tf2::getYaw(pose.orientation);
  return translate_polygon(
    tier4_autoware_utils::rotatePolygon(base_footprint, angle), pose.position.x, pose.position.y);
}

MultiPolygon2d create_object_footprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params)
{
  MultiPolygon2d footprints;
  if (params.avoid_dynamic_objects) {
    for (const auto & object : objects.objects) {
      const auto front = object.shape.dimensions.x / 2 + params.dynamic_objects_extra_front_offset;
      const auto rear = -object.shape.dimensions.x / 2 - params.dynamic_objects_extra_rear_offset;
      const auto left = object.shape.dimensions.y / 2 + params.dynamic_objects_extra_left_offset;
      const auto right = -object.shape.dimensions.y / 2 - params.dynamic_objects_extra_right_offset;
      Polygon2d base_footprint;
      base_footprint.outer() = {
        Point2d{front, left}, Point2d{front, right}, Point2d{rear, right}, Point2d{rear, left},
        Point2d{front, left}};
      for (const auto & path : object.kinematics.predicted_paths)
        for (const auto & pose : path.path)
          footprints.push_back(create_footprint(pose, base_footprint));
    }
  }
  return footprints;
}
}  // namespace drivable_area_expansion
