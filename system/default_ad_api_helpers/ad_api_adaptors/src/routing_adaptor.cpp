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

#include "routing_adaptor.hpp"

#include <memory>

namespace ad_api_adaptors
{

RoutingAdaptor::RoutingAdaptor() : Node("routing_adaptor")
{
  using std::placeholders::_1;

  sub_fixed_goal_ = create_subscription<PoseStamped>(
    "~/input/fixed_goal", 3, std::bind(&RoutingAdaptor::on_fixed_goal, this, _1));
  sub_rough_goal_ = create_subscription<PoseStamped>(
    "~/input/rough_goal", 3, std::bind(&RoutingAdaptor::on_rough_goal, this, _1));
  sub_reroute_ = create_subscription<PoseStamped>(
    "~/input/reroute", 3, std::bind(&RoutingAdaptor::on_reroute, this, _1));
  sub_waypoint_ = create_subscription<PoseStamped>(
    "~/input/waypoint", 10, std::bind(&RoutingAdaptor::on_waypoint, this, _1));

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_cli(cli_reroute_);
  adaptor.init_cli(cli_route_);
  adaptor.init_cli(cli_clear_);
  adaptor.init_sub(
    sub_state_, [this](const RouteState::Message::ConstSharedPtr msg) { state_ = msg->state; });

  const auto rate = rclcpp::Rate(5.0);
  timer_ = rclcpp::create_timer(
    this, get_clock(), rate.period(), std::bind(&RoutingAdaptor::on_timer, this));

  state_ = RouteState::Message::UNKNOWN;
  route_ = std::make_shared<SetRoutePoints::Service::Request>();
}

void RoutingAdaptor::on_timer()
{
  // Wait a moment to combine consecutive goals and checkpoints into a single request.
  // This value is rate dependent and set the wait time for merging.
  constexpr int delay_count = 3;  // 0.4 seconds (rate * (value - 1))
  if (0 < request_timing_control_ && request_timing_control_ < delay_count) {
    ++request_timing_control_;
  }
  if (request_timing_control_ != delay_count) {
    return;
  }

  if (!calling_service_) {
    if (state_ != RouteState::Message::UNSET) {
      const auto request = std::make_shared<ClearRoute::Service::Request>();
      calling_service_ = true;
      cli_clear_->async_send_request(request, [this](auto) { calling_service_ = false; });
    } else {
      request_timing_control_ = 0;
      calling_service_ = true;
      cli_route_->async_send_request(route_, [this](auto) { calling_service_ = false; });
    }
  }
}

void RoutingAdaptor::on_fixed_goal(const PoseStamped::ConstSharedPtr pose)
{
  request_timing_control_ = 1;
  route_->header = pose->header;
  route_->goal = pose->pose;
  route_->waypoints.clear();
  route_->option.allow_goal_modification = false;
}

void RoutingAdaptor::on_rough_goal(const PoseStamped::ConstSharedPtr pose)
{
  request_timing_control_ = 1;
  route_->header = pose->header;
  route_->goal = pose->pose;
  route_->waypoints.clear();
  route_->option.allow_goal_modification = true;
}

void RoutingAdaptor::on_waypoint(const PoseStamped::ConstSharedPtr pose)
{
  if (route_->header.frame_id != pose->header.frame_id) {
    RCLCPP_ERROR_STREAM(get_logger(), "The waypoint frame does not match the goal.");
    return;
  }
  request_timing_control_ = 1;
  route_->waypoints.push_back(pose->pose);
}

void RoutingAdaptor::on_reroute(const PoseStamped::ConstSharedPtr pose)
{
  const auto route = std::make_shared<SetRoutePoints::Service::Request>();
  route->header = pose->header;
  route->goal = pose->pose;
  cli_reroute_->async_send_request(route);
}

}  // namespace ad_api_adaptors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ad_api_adaptors::RoutingAdaptor>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
