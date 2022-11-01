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

#ifndef MOTION_HPP_
#define MOTION_HPP_

#include <autoware_ad_api_specs/motion.hpp>
#include <component_interface_specs/control.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <component_interface_utils/status.hpp>
#include <motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class MotionNode : public rclcpp::Node
{
public:
  explicit MotionNode(const rclcpp::NodeOptions & options);

private:
  motion_utils::VehicleStopChecker vehicle_stop_checker_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Srv<autoware_ad_api::motion::AcceptStart> srv_accept_;
  Pub<autoware_ad_api::motion::State> pub_state_;
  Cli<control_interface::SetPause> cli_set_pause_;
  Sub<control_interface::IsPaused> sub_is_paused_;
  Sub<control_interface::IsStartRequested> sub_is_start_requested_;

  enum class State { Unknown, Pausing, Paused, Starting, Resuming, Resumed, Moving };
  State state_;
  std::optional<bool> is_paused_;
  std::optional<bool> is_start_requested_;

  double stop_check_duration_;
  bool require_accept_start_;
  bool is_calling_set_pause_;

  void update_state();
  void change_state(const State state);
  void change_pause(bool pause);
  void on_timer();
  void on_is_paused(const control_interface::IsPaused::Message::ConstSharedPtr msg);
  void on_is_start_requested(
    const control_interface::IsStartRequested::Message::ConstSharedPtr msg);
  void on_accept(
    const autoware_ad_api::motion::AcceptStart::Service::Request::SharedPtr req,
    const autoware_ad_api::motion::AcceptStart::Service::Response::SharedPtr res);
};

}  // namespace default_ad_api

#endif  // MOTION_HPP_
