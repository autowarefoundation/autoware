// Copyright 2020 Tier IV, Inc.
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

#include "awapi_awiv_adapter/awapi_obstacle_avoidance_state_publisher.hpp"

#include "tier4_auto_msgs_converter/tier4_auto_msgs_converter.hpp"

namespace autoware_api
{
AutowareIvObstacleAvoidanceStatePublisher::AutowareIvObstacleAvoidanceStatePublisher(
  rclcpp::Node & node)
: logger_(node.get_logger().get_child("awapi_awiv_obstacle_avoidance_state_publisher")),
  clock_(node.get_clock())
{
  // publisher
  pub_state_ = node.create_publisher<tier4_api_msgs::msg::ObstacleAvoidanceStatus>(
    "output/obstacle_avoid_status", 1);
}

void AutowareIvObstacleAvoidanceStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  tier4_api_msgs::msg::ObstacleAvoidanceStatus status;

  // input header
  status.header.frame_id = "base_link";
  status.header.stamp = clock_->now();

  // get all info
  getObstacleAvoidReadyInfo(aw_info.obstacle_avoid_ready_ptr, &status);
  getCandidatePathInfo(aw_info.obstacle_avoid_candidate_ptr, &status);

  // publish info
  pub_state_->publish(status);
}

void AutowareIvObstacleAvoidanceStatePublisher::getObstacleAvoidReadyInfo(
  const tier4_planning_msgs::msg::IsAvoidancePossible::ConstSharedPtr & ready_ptr,
  tier4_api_msgs::msg::ObstacleAvoidanceStatus * status)
{
  if (!ready_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "obstacle_avoidance_ready is nullptr");
    return;
  }

  status->obstacle_avoidance_ready = ready_ptr->is_avoidance_possible;
}

void AutowareIvObstacleAvoidanceStatePublisher::getCandidatePathInfo(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr & path_ptr,
  tier4_api_msgs::msg::ObstacleAvoidanceStatus * status)
{
  if (!path_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "obstacle_avoidance_candidate_path is nullptr");
    return;
  }

  using tier4_auto_msgs_converter::convert;
  status->candidate_path = convert(*path_ptr);
}

}  // namespace autoware_api
