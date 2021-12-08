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

#include "awapi_awiv_adapter/awapi_stop_reason_aggregator.hpp"

#include <memory>
#include <vector>

namespace autoware_api
{
AutowareIvStopReasonAggregator::AutowareIvStopReasonAggregator(
  rclcpp::Node & node, const double timeout, const double thresh_dist_to_stop_pose)
: logger_(node.get_logger().get_child("awapi_awiv_stop_reason_aggregator")),
  clock_(node.get_clock()),
  timeout_(timeout),
  thresh_dist_to_stop_pose_(thresh_dist_to_stop_pose)
{
}

autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr
AutowareIvStopReasonAggregator::updateStopReasonArray(
  const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr,
  const AutowareInfo & aw_info)
{
  applyUpdate(msg_ptr, aw_info);
  applyTimeOut();
  return makeStopReasonArray(aw_info);
}

void AutowareIvStopReasonAggregator::applyUpdate(
  const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr,
  [[maybe_unused]] const AutowareInfo & aw_info)
{
  /* remove old stop_reason that matches reason with received msg */
  // make reason-matching msg list

  std::vector<size_t> remove_idx;
  if (!stop_reason_array_vec_.empty()) {
    for (int i = stop_reason_array_vec_.size() - 1; i >= 0; i--) {
      if (checkMatchingReason(msg_ptr, stop_reason_array_vec_.at(i))) {
        remove_idx.emplace_back(i);
      }
    }
  }

  // remove reason matching msg
  for (const auto idx : remove_idx) {
    stop_reason_array_vec_.erase(stop_reason_array_vec_.begin() + idx);
  }

  // add new reason msg
  stop_reason_array_vec_.emplace_back(*msg_ptr);
}

bool AutowareIvStopReasonAggregator::checkMatchingReason(
  const autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_stop_reason_array,
  const autoware_planning_msgs::msg::StopReasonArray & stop_reason_array)
{
  for (const auto & msg_stop_reason : msg_stop_reason_array->stop_reasons) {
    for (const auto & stop_reason : stop_reason_array.stop_reasons) {
      if (msg_stop_reason.reason == stop_reason.reason) {
        // find matching reason
        return true;
      }
    }
  }
  // cannot find matching reason
  return false;
}

void AutowareIvStopReasonAggregator::applyTimeOut()
{
  const auto current_time = clock_->now();

  // make timeout-msg list
  std::vector<size_t> remove_idx;
  if (!stop_reason_array_vec_.empty()) {
    for (int i = stop_reason_array_vec_.size() - 1; i >= 0; i--) {
      if (
        (current_time - rclcpp::Time(stop_reason_array_vec_.at(i).header.stamp)).seconds() >
        timeout_) {
        remove_idx.emplace_back(i);
      }
    }
  }
  // remove timeout-msg
  for (const auto idx : remove_idx) {
    stop_reason_array_vec_.erase(stop_reason_array_vec_.begin() + idx);
  }
}

void AutowareIvStopReasonAggregator::appendStopReasonToArray(
  const autoware_planning_msgs::msg::StopReason & stop_reason,
  autoware_planning_msgs::msg::StopReasonArray * stop_reason_array, const AutowareInfo & aw_info)
{
  // calculate dist_to_stop_pose
  const auto stop_reason_with_dist = inputStopDistToStopReason(stop_reason, aw_info);

  // cut stop reason
  const auto near_stop_reason = getNearStopReason(stop_reason_with_dist, aw_info);

  // if stop factors is empty, not append
  if (near_stop_reason.stop_factors.empty()) {
    return;
  }

  // if already exists same reason msg in stop_reason_array_msg, append stop_factors to there
  for (auto & base_stop_reasons : stop_reason_array->stop_reasons) {
    if (base_stop_reasons.reason == near_stop_reason.reason) {
      base_stop_reasons.stop_factors.insert(
        base_stop_reasons.stop_factors.end(), near_stop_reason.stop_factors.begin(),
        near_stop_reason.stop_factors.end());
      return;
    }
  }

  // if not exist same reason msg, append new stop reason
  stop_reason_array->stop_reasons.emplace_back(near_stop_reason);
}

autoware_planning_msgs::msg::StopReasonArray::ConstSharedPtr
AutowareIvStopReasonAggregator::makeStopReasonArray(const AutowareInfo & aw_info)
{
  autoware_planning_msgs::msg::StopReasonArray stop_reason_array_msg;
  // input header
  stop_reason_array_msg.header.frame_id = "map";
  stop_reason_array_msg.header.stamp = clock_->now();

  // input stop reason
  for (const auto & stop_reason_array : stop_reason_array_vec_) {
    for (const auto & stop_reason : stop_reason_array.stop_reasons) {
      appendStopReasonToArray(stop_reason, &stop_reason_array_msg, aw_info);
    }
  }
  return std::make_shared<autoware_planning_msgs::msg::StopReasonArray>(stop_reason_array_msg);
}

autoware_planning_msgs::msg::StopReason AutowareIvStopReasonAggregator::inputStopDistToStopReason(
  const autoware_planning_msgs::msg::StopReason & stop_reason, const AutowareInfo & aw_info)
{
  if (!aw_info.autoware_planning_traj_ptr || !aw_info.current_pose_ptr) {
    // pass through all stop reason
    return stop_reason;
  }

  auto stop_reason_with_dist = stop_reason;
  for (auto & stop_factor : stop_reason_with_dist.stop_factors) {
    const auto & trajectory = *aw_info.autoware_planning_traj_ptr;
    const auto & current_pose = aw_info.current_pose_ptr->pose;
    stop_factor.dist_to_stop_pose =
      planning_util::calcDistanceAlongTrajectory(trajectory, current_pose, stop_factor.stop_pose);
  }
  return stop_reason_with_dist;
}

autoware_planning_msgs::msg::StopReason AutowareIvStopReasonAggregator::getNearStopReason(
  const autoware_planning_msgs::msg::StopReason & stop_reason, const AutowareInfo & aw_info)
{
  if (!aw_info.autoware_planning_traj_ptr || !aw_info.current_pose_ptr) {
    // pass through all stop reason
    return stop_reason;
  }

  autoware_planning_msgs::msg::StopReason near_stop_reason;
  near_stop_reason.reason = stop_reason.reason;
  for (const auto & stop_factor : stop_reason.stop_factors) {
    if (stop_factor.dist_to_stop_pose < thresh_dist_to_stop_pose_) {
      // append only near stop factor
      near_stop_reason.stop_factors.emplace_back(stop_factor);
    }
  }
  return near_stop_reason;
}

}  // namespace autoware_api
