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

#include "awapi_awiv_adapter/awapi_lane_change_state_publisher.hpp"

#include "autoware_iv_auto_msgs_converter/autoware_iv_auto_msgs_converter.hpp"

namespace autoware_api
{
AutowareIvLaneChangeStatePublisher::AutowareIvLaneChangeStatePublisher(rclcpp::Node & node)
: logger_(node.get_logger().get_child("awapi_awiv_lane_change_state_publisher")),
  clock_(node.get_clock())
{
  // publisher
  pub_state_ =
    node.create_publisher<tier4_api_msgs::msg::LaneChangeStatus>("output/lane_change_status", 1);
}

void AutowareIvLaneChangeStatePublisher::statePublisher(const AutowareInfo & aw_info)
{
  tier4_api_msgs::msg::LaneChangeStatus status;

  // input header
  status.header.frame_id = "base_link";
  status.header.stamp = clock_->now();

  // get all info
  getLaneChangeAvailableInfo(aw_info.lane_change_available_ptr, &status);
  getLaneChangeReadyInfo(aw_info.lane_change_ready_ptr, &status);
  getCandidatePathInfo(aw_info.lane_change_candidate_ptr, &status);

  // publish info
  pub_state_->publish(status);
}

void AutowareIvLaneChangeStatePublisher::getLaneChangeAvailableInfo(
  const tier4_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr & available_ptr,
  tier4_api_msgs::msg::LaneChangeStatus * status)
{
  if (!available_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "lane change available is nullptr");
    return;
  }

  // get lane change available info
  status->force_lane_change_available = available_ptr->status;
}

void AutowareIvLaneChangeStatePublisher::getLaneChangeReadyInfo(
  const tier4_planning_msgs::msg::LaneChangeStatus::ConstSharedPtr & ready_ptr,
  tier4_api_msgs::msg::LaneChangeStatus * status)
{
  if (!ready_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 5000 /* ms */, "lane change ready is nullptr");
    return;
  }

  // get lane change available info
  status->lane_change_ready = ready_ptr->status;
}

void AutowareIvLaneChangeStatePublisher::getCandidatePathInfo(
  const autoware_auto_planning_msgs::msg::Path::ConstSharedPtr & path_ptr,
  tier4_api_msgs::msg::LaneChangeStatus * status)
{
  if (!path_ptr) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "lane_change_candidate_path is nullptr");
    return;
  }

  using autoware_iv_auto_msgs_converter::convert;
  status->candidate_path = convert(*path_ptr);
}

}  // namespace autoware_api
