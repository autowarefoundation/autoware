// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/scene_module/pull_over/util.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <boost/geometry/algorithms/dispatch/distance.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

using motion_utils::calcSignedArcLength;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::inverseTransformPoint;

namespace behavior_path_planner
{
namespace pull_over_utils
{
PathWithLaneId combineReferencePath(const PathWithLaneId path1, const PathWithLaneId path2)
{
  PathWithLaneId path;
  path.points.insert(path.points.end(), path1.points.begin(), path1.points.end());

  // skip overlapping point
  path.points.insert(path.points.end(), next(path2.points.begin()), path2.points.end());

  return path;
}

lanelet::ConstLanelets getPullOverLanes(const RouteHandler & route_handler)
{
  lanelet::ConstLanelets pull_over_lanes;
  lanelet::ConstLanelet target_shoulder_lane;
  const Pose goal_pose = route_handler.getGoalPose();

  if (route_handler.getPullOverTarget(
        route_handler.getShoulderLanelets(), goal_pose, &target_shoulder_lane)) {
    constexpr double pull_over_lane_length = 200;
    pull_over_lanes = route_handler.getShoulderLaneletSequence(
      target_shoulder_lane, goal_pose, pull_over_lane_length, pull_over_lane_length);
  }
  return pull_over_lanes;
}

bool hasEnoughDistanceToParkingStart(
  const PathWithLaneId & path, const Pose & current_pose, const Pose & start_pose,
  const double current_vel, const double maximum_deceleration, const double decide_path_distance,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold)
{
  const size_t ego_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  const size_t start_segment_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, start_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);
  const double dist_to_start_pose = calcSignedArcLength(
    path.points, current_pose.position, ego_segment_idx, start_pose.position, start_segment_idx);

  // const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;
  const double current_to_stop_distance = std::pow(current_vel, 2) / maximum_deceleration / 2;

  // once stopped, it cannot start again if start_pose is close.
  // so need enough distance to restart
  constexpr double eps_vel = 0.01;
  // dist to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  const double dist_to_restart = decide_path_distance / 2;
  if (std::abs(current_vel) < eps_vel && dist_to_start_pose < dist_to_restart) {
    return false;
  }

  return dist_to_start_pose > current_to_stop_distance;
}

Marker createPullOverAreaMarker(
  const Pose & start_pose, const Pose & end_pose, const int32_t id,
  const std_msgs::msg::Header & header, const double base_link2front, const double base_link2rear,
  const double vehicle_width, const std_msgs::msg::ColorRGBA & color)
{
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerScale;
  using tier4_autoware_utils::createPoint;

  Marker marker = createDefaultMarker(
    header.frame_id, header.stamp, "pull_over_area", id,
    visualization_msgs::msg::Marker::LINE_STRIP, createMarkerScale(0.1, 0.0, 0.0), color);

  auto p_left_front = calcOffsetPose(end_pose, base_link2front, vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_left_front.x, p_left_front.y, p_left_front.z));

  auto p_right_front = calcOffsetPose(end_pose, base_link2front, -vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_right_front.x, p_right_front.y, p_right_front.z));

  auto p_right_back = calcOffsetPose(start_pose, -base_link2rear, -vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_right_back.x, p_right_back.y, p_right_back.z));

  auto p_left_back = calcOffsetPose(start_pose, -base_link2rear, vehicle_width / 2, 0).position;
  marker.points.push_back(createPoint(p_left_back.x, p_left_back.y, p_left_back.z));

  marker.points.push_back(createPoint(p_left_front.x, p_left_front.y, p_left_front.z));

  return marker;
}

}  // namespace pull_over_utils
}  // namespace behavior_path_planner
