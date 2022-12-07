// Copyright 2022 TIER IV, Inc.
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

#ifndef COMPATIBILITY__AUTOWARE_STATE_HPP_
#define COMPATIBILITY__AUTOWARE_STATE_HPP_

#include <autoware_ad_api_specs/localization.hpp>
#include <autoware_ad_api_specs/operation_mode.hpp>
#include <autoware_ad_api_specs/routing.hpp>

#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tier4_system_msgs/msg/mode_change_available.hpp>

#include <vector>

// This file should be included after messages.
#include "../utils/types.hpp"

namespace default_ad_api
{

class AutowareStateNode : public rclcpp::Node
{
public:
  explicit AutowareStateNode(const rclcpp::NodeOptions & options);

private:
  using ModeChangeAvailable = tier4_system_msgs::msg::ModeChangeAvailable;
  rclcpp::TimerBase::SharedPtr timer_;
  // emergency
  Sub<autoware_ad_api::localization::InitializationState> sub_localization_;
  Sub<autoware_ad_api::routing::RouteState> sub_routing_;
  Sub<autoware_ad_api::operation_mode::OperationModeState> sub_operation_mode_;

  using AutowareState = autoware_auto_system_msgs::msg::AutowareState;
  using LocalizationState = autoware_ad_api::localization::InitializationState::Message;
  using RoutingState = autoware_ad_api::routing::RouteState::Message;
  using OperationModeState = autoware_ad_api::operation_mode::OperationModeState::Message;
  using Trigger = std_srvs::srv::Trigger;
  std::vector<bool> component_states_;
  std::vector<rclcpp::Subscription<ModeChangeAvailable>::SharedPtr> sub_component_states_;
  rclcpp::Publisher<AutowareState>::SharedPtr pub_autoware_state_;
  rclcpp::Service<Trigger>::SharedPtr srv_autoware_shutdown_;

  enum class LaunchState { Initializing, Running, Finalizing };
  LaunchState launch_state_;
  LocalizationState localization_state_;
  RoutingState routing_state_;
  OperationModeState operation_mode_state_;

  void on_timer();
  void on_localization(const LocalizationState::ConstSharedPtr msg);
  void on_routing(const RoutingState::ConstSharedPtr msg);
  void on_operation_mode(const OperationModeState::ConstSharedPtr msg);
  void on_shutdown(const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr);
};

}  // namespace default_ad_api

#endif  // COMPATIBILITY__AUTOWARE_STATE_HPP_
