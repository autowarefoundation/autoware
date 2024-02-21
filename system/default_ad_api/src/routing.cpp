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

#include "routing.hpp"

#include "utils/route_conversion.hpp"

#include <memory>

namespace
{

using autoware_adapi_v1_msgs::msg::ResponseStatus;

template <class InterfaceT>
ResponseStatus route_already_set()
{
  ResponseStatus status;
  status.success = false;
  status.code = InterfaceT::Service::Response::ERROR_INVALID_STATE;
  status.message = "The route is already set.";
  return status;
}

template <class InterfaceT>
ResponseStatus route_is_not_set()
{
  ResponseStatus status;
  status.success = false;
  status.code = InterfaceT::Service::Response::ERROR_INVALID_STATE;
  status.message = "The route is not set yet.";
  return status;
}

}  // namespace

namespace default_ad_api
{

RoutingNode::RoutingNode(const rclcpp::NodeOptions & options) : Node("routing", options)
{
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_pub(pub_state_);
  adaptor.init_pub(pub_route_);
  adaptor.init_sub(sub_state_, this, &RoutingNode::on_state);
  adaptor.init_sub(sub_route_, this, &RoutingNode::on_route);
  adaptor.init_cli(cli_clear_route_, group_cli_);
  adaptor.init_cli(cli_set_waypoint_route_, group_cli_);
  adaptor.init_cli(cli_set_lanelet_route_, group_cli_);
  adaptor.init_srv(srv_clear_route_, this, &RoutingNode::on_clear_route);
  adaptor.init_srv(srv_set_route_, this, &RoutingNode::on_set_route);
  adaptor.init_srv(srv_set_route_points_, this, &RoutingNode::on_set_route_points);
  adaptor.init_srv(srv_change_route_, this, &RoutingNode::on_change_route);
  adaptor.init_srv(srv_change_route_points_, this, &RoutingNode::on_change_route_points);

  adaptor.init_cli(cli_operation_mode_, group_cli_);
  adaptor.init_sub(sub_operation_mode_, this, &RoutingNode::on_operation_mode);

  is_auto_mode_ = false;
  state_.state = State::Message::UNKNOWN;
}

void RoutingNode::change_stop_mode()
{
  using OperationModeRequest = system_interface::ChangeOperationMode::Service::Request;
  if (is_auto_mode_) {
    const auto req = std::make_shared<OperationModeRequest>();
    req->mode = OperationModeRequest::STOP;
    cli_operation_mode_->async_send_request(req);
  }
}

void RoutingNode::on_operation_mode(const OperationModeState::Message::ConstSharedPtr msg)
{
  is_auto_mode_ = msg->mode == OperationModeState::Message::AUTONOMOUS;
}

void RoutingNode::on_state(const State::Message::ConstSharedPtr msg)
{
  // TODO(Takagi, Isamu): Add adapi initializing state.
  // Represent initializing state by not publishing the topic for now.
  if (msg->state == State::Message::INITIALIZING) {
    return;
  }

  state_ = *msg;
  pub_state_->publish(conversion::convert_state(*msg));

  // Change operation mode to stop when the vehicle arrives.
  if (msg->state == State::Message::ARRIVED) {
    change_stop_mode();
  }

  // TODO(Takagi, Isamu): Remove when the mission planner supports an empty route.
  if (msg->state == State::Message::UNSET) {
    pub_route_->publish(conversion::create_empty_route(msg->stamp));
  }
}

void RoutingNode::on_route(const Route::Message::ConstSharedPtr msg)
{
  pub_route_->publish(conversion::convert_route(*msg));
}

void RoutingNode::on_clear_route(
  const autoware_ad_api::routing::ClearRoute::Service::Request::SharedPtr req,
  const autoware_ad_api::routing::ClearRoute::Service::Response::SharedPtr res)
{
  change_stop_mode();
  res->status = conversion::convert_call(cli_clear_route_, req);
}

void RoutingNode::on_set_route_points(
  const autoware_ad_api::routing::SetRoutePoints::Service::Request::SharedPtr req,
  const autoware_ad_api::routing::SetRoutePoints::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::UNSET) {
    res->status = route_already_set<autoware_ad_api::routing::SetRoutePoints>();
    return;
  }
  res->status = conversion::convert_call(cli_set_waypoint_route_, req);
}

void RoutingNode::on_set_route(
  const autoware_ad_api::routing::SetRoute::Service::Request::SharedPtr req,
  const autoware_ad_api::routing::SetRoute::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::UNSET) {
    res->status = route_already_set<autoware_ad_api::routing::SetRoute>();
    return;
  }
  res->status = conversion::convert_call(cli_set_lanelet_route_, req);
}

void RoutingNode::on_change_route_points(
  const autoware_ad_api::routing::SetRoutePoints::Service::Request::SharedPtr req,
  const autoware_ad_api::routing::SetRoutePoints::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::SET) {
    res->status = route_is_not_set<autoware_ad_api::routing::SetRoutePoints>();
    return;
  }
  res->status = conversion::convert_call(cli_set_waypoint_route_, req);
}

void RoutingNode::on_change_route(
  const autoware_ad_api::routing::SetRoute::Service::Request::SharedPtr req,
  const autoware_ad_api::routing::SetRoute::Service::Response::SharedPtr res)
{
  if (state_.state != State::Message::SET) {
    res->status = route_is_not_set<autoware_ad_api::routing::SetRoute>();
    return;
  }
  res->status = conversion::convert_call(cli_set_lanelet_route_, req);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::RoutingNode)
