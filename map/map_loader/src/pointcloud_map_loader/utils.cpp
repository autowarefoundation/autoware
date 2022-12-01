// Copyright 2022 The Autoware Contributors
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

#include "utils.hpp"

#include <fmt/format.h>

bool sphereAndBoxOverlapExists(
  const geometry_msgs::msg::Point center, const double radius, const pcl::PointXYZ box_min_point,
  const pcl::PointXYZ box_max_point)
{
  // Collision detection with x-axis plane
  if (
    box_min_point.x - radius <= center.x && center.x <= box_max_point.x + radius &&
    box_min_point.y <= center.y && center.y <= box_max_point.y && box_min_point.z <= center.z &&
    center.z <= box_max_point.z) {
    return true;
  }

  // Collision detection with y-axis plane
  if (
    box_min_point.x <= center.x && center.x <= box_max_point.x &&
    box_min_point.y - radius <= center.y && center.y <= box_max_point.y + radius &&
    box_min_point.z <= center.z && center.z <= box_max_point.z) {
    return true;
  }

  // Collision detection with z-axis plane
  if (
    box_min_point.x <= center.x && center.x <= box_max_point.x && box_min_point.y <= center.y &&
    center.y <= box_max_point.y && box_min_point.z - radius <= center.z &&
    center.z <= box_max_point.z + radius) {
    return true;
  }

  // Collision detection with box edges
  const double dx0 = center.x - box_min_point.x;
  const double dx1 = center.x - box_max_point.x;
  const double dy0 = center.y - box_min_point.y;
  const double dy1 = center.y - box_max_point.y;
  const double dz0 = center.z - box_min_point.z;
  const double dz1 = center.z - box_max_point.z;
  if (
    std::hypot(dx0, dy0, dz0) <= radius || std::hypot(dx1, dy0, dz0) <= radius ||
    std::hypot(dx0, dy1, dz0) <= radius || std::hypot(dx0, dy0, dz1) <= radius ||
    std::hypot(dx0, dy1, dz1) <= radius || std::hypot(dx1, dy0, dz1) <= radius ||
    std::hypot(dx1, dy1, dz0) <= radius || std::hypot(dx1, dy1, dz1) <= radius) {
    return true;
  }
  return false;
}

bool isGridWithinQueriedArea(
  const autoware_map_msgs::msg::AreaInfo area, const PCDFileMetadata metadata)
{
  // Currently, the area load only supports spherical area
  geometry_msgs::msg::Point center = area.center;
  double radius = area.radius;
  bool res = sphereAndBoxOverlapExists(center, radius, metadata.min, metadata.max);
  return res;
}
