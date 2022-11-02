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

#ifndef OBSTACLE_VELOCITY_LIMITER__TYPES_HPP_
#define OBSTACLE_VELOCITY_LIMITER__TYPES_HPP_

#include "tier4_autoware_utils/geometry/boost_geometry.hpp"

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace obstacle_velocity_limiter
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::OccupancyGrid;
using PointCloud = sensor_msgs::msg::PointCloud2;
using Float = decltype(TrajectoryPoint::longitudinal_velocity_mps);

using point_t = tier4_autoware_utils::Point2d;
using multipoint_t = tier4_autoware_utils::MultiPoint2d;
using polygon_t = tier4_autoware_utils::Polygon2d;
using multipolygon_t = tier4_autoware_utils::MultiPolygon2d;
using segment_t = tier4_autoware_utils::Segment2d;
using linestring_t = tier4_autoware_utils::LineString2d;
using multilinestring_t = tier4_autoware_utils::MultiLineString2d;

struct ObstacleMasks
{
  polygon_t positive_mask;        // discard obstacles outside of this polygon
  multipolygon_t negative_masks;  // discard obstacles inside of these polygons
};

}  // namespace obstacle_velocity_limiter
#endif  // OBSTACLE_VELOCITY_LIMITER__TYPES_HPP_
