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

#ifndef ROUTING_ADAPTOR_HPP_
#define ROUTING_ADAPTOR_HPP_

#include <autoware_ad_api_specs/routing.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace ad_api_adaptors
{

class RoutingAdaptor : public rclcpp::Node
{
public:
  RoutingAdaptor();

private:
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetRoutePoints = autoware_ad_api::routing::SetRoutePoints;
  using ClearRoute = autoware_ad_api::routing::ClearRoute;
  using RouteState = autoware_ad_api::routing::RouteState;
  component_interface_utils::Client<SetRoutePoints>::SharedPtr cli_route_;
  component_interface_utils::Client<ClearRoute>::SharedPtr cli_clear_;
  component_interface_utils::Subscription<RouteState>::SharedPtr sub_state_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_waypoint_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool calling_service_ = false;
  int request_timing_control_ = 0;
  SetRoutePoints::Service::Request::SharedPtr route_;
  RouteState::Message::_state_type state_;

  void on_timer();
  void on_goal(const PoseStamped::ConstSharedPtr pose);
  void on_waypoint(const PoseStamped::ConstSharedPtr pose);
};

}  // namespace ad_api_adaptors

#endif  // ROUTING_ADAPTOR_HPP_
