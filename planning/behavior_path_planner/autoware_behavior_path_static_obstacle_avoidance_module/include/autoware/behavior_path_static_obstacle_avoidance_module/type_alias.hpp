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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace autoware::behavior_path_planner
{
// auto msgs
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedPath;
using tier4_planning_msgs::msg::PathWithLaneId;

// ROS 2 general msgs
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// tier4 msgs
using tier4_planning_msgs::msg::AvoidanceDebugMsg;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_rtc_msgs::msg::State;

// tier4 utils functions
using autoware_universe_utils::appendMarkerArray;
using autoware_universe_utils::calcDistance2d;
using autoware_universe_utils::calcLateralDeviation;
using autoware_universe_utils::calcOffsetPose;
using autoware_universe_utils::calcYawDeviation;
using autoware_universe_utils::createDefaultMarker;
using autoware_universe_utils::createMarkerColor;
using autoware_universe_utils::createMarkerScale;
using autoware_universe_utils::createPoint;
using autoware_universe_utils::createQuaternionFromRPY;
using autoware_universe_utils::getPoint;
using autoware_universe_utils::getPose;
using autoware_universe_utils::Point2d;
using autoware_universe_utils::Polygon2d;
using autoware_universe_utils::pose2transform;
using autoware_universe_utils::toHexString;
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
