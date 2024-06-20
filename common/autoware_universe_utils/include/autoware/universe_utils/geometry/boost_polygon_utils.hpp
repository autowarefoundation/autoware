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

#ifndef AUTOWARE__UNIVERSE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
#define AUTOWARE__UNIVERSE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <vector>

namespace autoware::universe_utils
{
bool isClockwise(const Polygon2d & polygon);
Polygon2d inverseClockwise(const Polygon2d & polygon);
geometry_msgs::msg::Polygon rotatePolygon(
  const geometry_msgs::msg::Polygon & polygon, const double & angle);
/// @brief rotate a polygon by some angle around the origin
/// @param[in] polygon input polygon
/// @param[in] angle angle of rotation [rad]
/// @return rotated polygon
Polygon2d rotatePolygon(const Polygon2d & polygon, const double angle);
Polygon2d toPolygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);
Polygon2d toPolygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);
Polygon2d toPolygon2d(const autoware_perception_msgs::msg::DetectedObject & object);
Polygon2d toPolygon2d(const autoware_perception_msgs::msg::TrackedObject & object);
Polygon2d toPolygon2d(const autoware_perception_msgs::msg::PredictedObject & object);
Polygon2d toFootprint(
  const geometry_msgs::msg::Pose & base_link_pose, const double base_to_front,
  const double base_to_rear, const double width);
double getArea(const autoware_perception_msgs::msg::Shape & shape);
Polygon2d expandPolygon(const Polygon2d & input_polygon, const double offset);
}  // namespace autoware::universe_utils

#endif  // AUTOWARE__UNIVERSE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
