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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "autoware/universe_utils/geometry/boost_geometry.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::OccupancyGrid;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

using TrajectoryPoints = std::vector<TrajectoryPoint>;
using point_t = autoware::universe_utils::Point2d;
using multipoint_t = autoware::universe_utils::MultiPoint2d;
using polygon_t = autoware::universe_utils::Polygon2d;
using multi_polygon_t = autoware::universe_utils::MultiPolygon2d;
using segment_t = autoware::universe_utils::Segment2d;
using linestring_t = autoware::universe_utils::LineString2d;
using multi_linestring_t = autoware::universe_utils::MultiLineString2d;

struct ObstacleMasks
{
  polygon_t positive_mask;         // discard obstacles outside of this polygon
  multi_polygon_t negative_masks;  // discard obstacles inside of these polygons
};

}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
#endif  // TYPES_HPP_
