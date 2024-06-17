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

#ifndef AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
#define AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
#include <autoware/route_handler/route_handler.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <vector>

namespace autoware_planning_test_manager::utils
{
using autoware::route_handler::RouteHandler;
using autoware_planning_msgs::msg::LaneletRoute;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;

Pose createPoseFromLaneID(const lanelet::Id & lane_id)
{
  auto map_bin_msg = autoware::test_utils::makeMapBinMsg();
  // create route_handler
  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_bin_msg);

  // get middle idx of the lanelet
  const auto lanelet = route_handler->getLaneletsFromId(lane_id);
  const auto center_line = lanelet.centerline();
  const size_t middle_point_idx = std::floor(center_line.size() / 2.0);

  // get middle position of the lanelet
  geometry_msgs::msg::Point middle_pos;
  middle_pos.x = center_line[middle_point_idx].x();
  middle_pos.y = center_line[middle_point_idx].y();

  // get next middle position of the lanelet
  geometry_msgs::msg::Point next_middle_pos;
  next_middle_pos.x = center_line[middle_point_idx + 1].x();
  next_middle_pos.y = center_line[middle_point_idx + 1].y();

  // calculate middle pose
  geometry_msgs::msg::Pose middle_pose;
  middle_pose.position = middle_pos;
  const double yaw = tier4_autoware_utils::calcAzimuthAngle(middle_pos, next_middle_pos);
  middle_pose.orientation = tier4_autoware_utils::createQuaternionFromYaw(yaw);

  return middle_pose;
}

// Function to create a route from given start and goal lanelet ids
// start pose and goal pose are set to the middle of the lanelet
LaneletRoute makeBehaviorRouteFromLaneId(const int & start_lane_id, const int & goal_lane_id)
{
  LaneletRoute route;
  route.header.frame_id = "map";
  auto start_pose = createPoseFromLaneID(start_lane_id);
  auto goal_pose = createPoseFromLaneID(goal_lane_id);
  route.start_pose = start_pose;
  route.goal_pose = goal_pose;

  auto map_bin_msg = autoware::test_utils::makeMapBinMsg();
  // create route_handler
  auto route_handler = std::make_shared<RouteHandler>();
  route_handler->setMap(map_bin_msg);

  LaneletRoute route_msg;
  RouteSections route_sections;
  lanelet::ConstLanelets all_route_lanelets;

  // Plan the path between checkpoints (start and goal poses)
  lanelet::ConstLanelets path_lanelets;
  if (!route_handler->planPathLaneletsBetweenCheckpoints(start_pose, goal_pose, &path_lanelets)) {
    return route_msg;
  }

  // Add all path_lanelets to all_route_lanelets
  for (const auto & lane : path_lanelets) {
    all_route_lanelets.push_back(lane);
  }
  // create local route sections
  route_handler->setRouteLanelets(path_lanelets);
  const auto local_route_sections = route_handler->createMapSegments(path_lanelets);
  route_sections =
    autoware::test_utils::combineConsecutiveRouteSections(route_sections, local_route_sections);
  for (const auto & route_section : route_sections) {
    for (const auto & primitive : route_section.primitives) {
      std::cerr << "primitive: " << primitive.id << std::endl;
    }
    std::cerr << "preferred_primitive id : " << route_section.preferred_primitive.id << std::endl;
  }
  route_handler->setRouteLanelets(all_route_lanelets);
  route.segments = route_sections;

  route.allow_modification = false;
  return route;
}

Odometry makeInitialPoseFromLaneId(const lanelet::Id & lane_id)
{
  Odometry current_odometry;
  current_odometry.pose.pose = createPoseFromLaneID(lane_id);
  current_odometry.header.frame_id = "map";

  return current_odometry;
}

}  // namespace autoware_planning_test_manager::utils
#endif  // AUTOWARE_PLANNING_TEST_MANAGER__AUTOWARE_PLANNING_TEST_MANAGER_UTILS_HPP_
