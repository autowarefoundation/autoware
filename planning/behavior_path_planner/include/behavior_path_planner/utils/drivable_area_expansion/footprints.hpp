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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__FOOTPRINTS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__FOOTPRINTS_HPP_

#include "behavior_path_planner/utils/drivable_area_expansion/parameters.hpp"
#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"

#include <tier4_autoware_utils/ros/transform_listener.hpp>

#include "autoware_auto_planning_msgs/msg/detail/path_point__struct.hpp"
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace drivable_area_expansion
{
/// @brief translate a polygon by some (x,y) vector
/// @param[in] polygon input polygon
/// @param[in] x translation distance on the x axis
/// @param[in] y translation distance on the y axis
/// @return translated polygon
polygon_t translatePolygon(const polygon_t & polygon, const double x, const double y);

/// @brief create the footprint of a pose and its base footprint
/// @param[in] pose the origin pose of the footprint
/// @param[in] base_footprint the base axis-aligned footprint
/// @return footprint polygon
polygon_t createFootprint(const geometry_msgs::msg::Pose & pose, const polygon_t base_footprint);

/// @brief create footprints of the predicted paths of an object
/// @param [in] objects objects from which to create polygons
/// @param[in] params expansion parameters containing extra offsets to add to the dynamic objects
/// @return footprint polygons of the object's predicted paths
multi_polygon_t createObjectFootprints(
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const DrivableAreaExpansionParameters & params);

/// @brief create the footprint polygon from a path
/// @param[in] path the path for which to create a footprint
/// @param[in] params expansion parameters defining how to create the footprint
/// @return footprint polygons of the path
multi_polygon_t createPathFootprints(
  const std::vector<PathPointWithLaneId> & path, const DrivableAreaExpansionParameters & params);
}  // namespace drivable_area_expansion
#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__FOOTPRINTS_HPP_
