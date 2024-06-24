// Copyright 2022 Tier IV, Inc.
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
#ifndef TYPE_ALIAS_HPP_
#define TYPE_ALIAS_HPP_

#include "autoware/route_handler/route_handler.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include "autoware_map_msgs/msg/lanelet_map_bin.hpp"
#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace autoware::static_centerline_generator
{
using autoware::route_handler::RouteHandler;
using autoware::universe_utils::LinearRing2d;
using autoware::universe_utils::LineString2d;
using autoware::universe_utils::Point2d;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PathPoint;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using tier4_planning_msgs::msg::PathWithLaneId;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
}  // namespace autoware::static_centerline_generator

#endif  // TYPE_ALIAS_HPP_
