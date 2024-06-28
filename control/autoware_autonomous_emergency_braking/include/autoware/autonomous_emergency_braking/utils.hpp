// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__UTILS_HPP_
#define AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__UTILS_HPP_

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <tf2/utils.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <vector>

namespace autoware::motion::control::autonomous_emergency_braking::utils
{
using autoware::universe_utils::Polygon2d;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;

/**
 * @brief Apply a transform to a predicted object
 * @param input the predicted object
 * @param transform_stamped the tf2 transform
 */
PredictedObject transformObjectFrame(
  const PredictedObject & input, geometry_msgs::msg::TransformStamped & transform_stamped);

/**
 * @brief Get the predicted objects polygon as a geometry polygon
 * @param current_pose the predicted object's pose
 * @param obj_shape the object's shape
 */
Polygon2d convertPolygonObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_perception_msgs::msg::Shape & obj_shape);

/**
 * @brief Get the predicted objects cylindrical shape as a geometry polygon
 * @param current_pose the predicted object's pose
 * @param obj_shape the object's shape
 */
Polygon2d convertCylindricalObjectToGeometryPolygon(
  const Pose & current_pose, const autoware_perception_msgs::msg::Shape & obj_shape);

/**
 * @brief Get the predicted objects bounding box shape as a geometry polygon
 * @param current_pose the predicted object's pose
 * @param obj_shape the object's shape
 */
Polygon2d convertBoundingBoxObjectToGeometryPolygon(
  const Pose & current_pose, const double & base_to_front, const double & base_to_rear,
  const double & base_to_width);

/**
 * @brief Get the predicted object's shape as a geometry polygon
 * @param obj the object
 */
Polygon2d convertObjToPolygon(const PredictedObject & obj);
}  // namespace autoware::motion::control::autonomous_emergency_braking::utils

#endif  // AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__UTILS_HPP_
