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

#ifndef ROUTING_HPP_
#define ROUTING_HPP_

#include <autoware_ad_api_specs/routing.hpp>
#include <component_interface_specs/planning.hpp>
#include <component_interface_specs/system.hpp>
#include <component_interface_utils/status.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace default_ad_api
{

class RoutingNode : public rclcpp::Node
{
public:
  explicit RoutingNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  Pub<autoware_ad_api::routing::RouteState> pub_state_;
  Pub<autoware_ad_api::routing::Route> pub_route_;
  Srv<autoware_ad_api::routing::SetRoutePoints> srv_set_route_points_;
  Srv<autoware_ad_api::routing::SetRoute> srv_set_route_;
  Srv<autoware_ad_api::routing::ChangeRoutePoints> srv_change_route_points_;
  Srv<autoware_ad_api::routing::ChangeRoute> srv_change_route_;
  Srv<autoware_ad_api::routing::ClearRoute> srv_clear_route_;
  Sub<planning_interface::RouteState> sub_state_;
  Sub<planning_interface::Route> sub_route_;
  Cli<planning_interface::SetRoutePoints> cli_set_route_points_;
  Cli<planning_interface::SetRoute> cli_set_route_;
  Cli<planning_interface::ChangeRoutePoints> cli_change_route_points_;
  Cli<planning_interface::ChangeRoute> cli_change_route_;
  Cli<planning_interface::ClearRoute> cli_clear_route_;
  Cli<system_interface::ChangeOperationMode> cli_operation_mode_;
  Sub<system_interface::OperationModeState> sub_operation_mode_;
  bool is_auto_mode = false;

  using OperationModeState = system_interface::OperationModeState;
  using State = planning_interface::RouteState;
  using Route = planning_interface::Route;
  void change_stop_mode();
  void on_operation_mode(const OperationModeState::Message::ConstSharedPtr msg);
  void on_state(const State::Message::ConstSharedPtr msg);
  void on_route(const Route::Message::ConstSharedPtr msg);
  void on_clear_route(
    const autoware_ad_api::routing::ClearRoute::Service::Request::SharedPtr req,
    const autoware_ad_api::routing::ClearRoute::Service::Response::SharedPtr res);
};

}  // namespace default_ad_api

#endif  // ROUTING_HPP_
