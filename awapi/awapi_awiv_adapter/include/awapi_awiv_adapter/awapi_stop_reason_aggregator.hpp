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

#ifndef AWAPI_AWIV_ADAPTER__AWAPI_STOP_REASON_AGGREGATOR_HPP_
#define AWAPI_AWIV_ADAPTER__AWAPI_STOP_REASON_AGGREGATOR_HPP_

#include "awapi_awiv_adapter/awapi_autoware_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <vector>

namespace autoware_api
{
class AutowareIvStopReasonAggregator
{
public:
  AutowareIvStopReasonAggregator(
    rclcpp::Node & node, const double timeout, const double thresh_dist_to_stop_pose);
  tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr updateStopReasonArray(
    const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr,
    const AutowareInfo & aw_info);

private:
  void applyUpdate(
    const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_ptr,
    const AutowareInfo & aw_info);
  bool checkMatchingReason(
    const tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr & msg_stop_reason_array,
    const tier4_planning_msgs::msg::StopReasonArray & stop_reason_array);
  void applyTimeOut();
  void appendStopReasonToArray(
    const tier4_planning_msgs::msg::StopReason & stop_reason,
    tier4_planning_msgs::msg::StopReasonArray * stop_reason_array, const AutowareInfo & aw_info);
  tier4_planning_msgs::msg::StopReasonArray::ConstSharedPtr makeStopReasonArray(
    const AutowareInfo & aw_info);
  tier4_planning_msgs::msg::StopReason inputStopDistToStopReason(
    const tier4_planning_msgs::msg::StopReason & stop_reason, const AutowareInfo & aw_info);
  double calcStopDistToStopFactor(
    const tier4_planning_msgs::msg::StopFactor & stop_factor, const AutowareInfo & aw_info);
  tier4_planning_msgs::msg::StopReason getNearStopReason(
    const tier4_planning_msgs::msg::StopReason & stop_reason, const AutowareInfo & aw_info);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  double timeout_;
  double thresh_dist_to_stop_pose_;
  std::vector<tier4_planning_msgs::msg::StopReasonArray> stop_reason_array_vec_;
};

}  // namespace autoware_api

#endif  // AWAPI_AWIV_ADAPTER__AWAPI_STOP_REASON_AGGREGATOR_HPP_
