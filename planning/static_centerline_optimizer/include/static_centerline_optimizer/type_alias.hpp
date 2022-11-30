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
#ifndef STATIC_CENTERLINE_OPTIMIZER__TYPE_ALIAS_HPP_
#define STATIC_CENTERLINE_OPTIMIZER__TYPE_ALIAS_HPP_

#include "route_handler/route_handler.hpp"

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "autoware_planning_msgs/msg/lanelet_route.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace static_centerline_optimizer
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathPoint;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using autoware_planning_msgs::msg::LaneletRoute;
using route_handler::RouteHandler;
using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::LineString2d;
using tier4_autoware_utils::Point2d;
using visualization_msgs::msg::MarkerArray;
}  // namespace static_centerline_optimizer

#endif  // STATIC_CENTERLINE_OPTIMIZER__TYPE_ALIAS_HPP_
