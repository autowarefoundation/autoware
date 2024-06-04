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

#ifndef AUTOWARE_BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
#define AUTOWARE_BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_

#include <motion_utils/distance/distance.hpp>
#include <motion_utils/trajectory/path_with_lane_id.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace behavior_path_planner
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
using tier4_autoware_utils::appendMarkerArray;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::calcYawDeviation;
using tier4_autoware_utils::createDefaultMarker;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::createMarkerScale;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternionFromRPY;
using tier4_autoware_utils::getPoint;
using tier4_autoware_utils::getPose;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using tier4_autoware_utils::pose2transform;
using tier4_autoware_utils::toHexString;
}  // namespace behavior_path_planner

#endif  // AUTOWARE_BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__TYPE_ALIAS_HPP_
