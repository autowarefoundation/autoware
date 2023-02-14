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

#ifndef TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
#define TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/tracked_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry.hpp>
#include <boost/optional.hpp>

#include <vector>

namespace tier4_autoware_utils
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
  const geometry_msgs::msg::Pose & pose, const autoware_auto_perception_msgs::msg::Shape & shape);
Polygon2d toPolygon2d(const autoware_auto_perception_msgs::msg::DetectedObject & object);
Polygon2d toPolygon2d(const autoware_auto_perception_msgs::msg::TrackedObject & object);
Polygon2d toPolygon2d(const autoware_auto_perception_msgs::msg::PredictedObject & object);
double getArea(const autoware_auto_perception_msgs::msg::Shape & shape);

}  // namespace tier4_autoware_utils

#endif  // TIER4_AUTOWARE_UTILS__GEOMETRY__BOOST_POLYGON_UTILS_HPP_
