// Copyright 2024 The Autoware Contributors
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

#include "route_selector.hpp"

#include "service_utils.hpp"

#include <array>
#include <memory>
#include <random>

namespace autoware::mission_planner::uuid
{

std::array<uint8_t, 16> generate_random_id()
{
  static std::independent_bits_engine<std::mt19937, 8, uint8_t> engine(std::random_device{}());
  std::array<uint8_t, 16> id;
  std::generate(id.begin(), id.end(), std::ref(engine));
  return id;
}

UUID generate_if_empty(const UUID & uuid)
{
  constexpr std::array<uint8_t, 16> zero_uuid = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  UUID result;
  result.uuid = (uuid.uuid == zero_uuid) ? generate_random_id() : uuid.uuid;
  return result;
}

}  // namespace autoware::mission_planner::uuid

namespace autoware::mission_planner
{

RouteInterface::RouteInterface(rclcpp::Clock::SharedPtr clock)
{
  clock_ = clock;
  state_.state = RouteState::UNKNOWN;
}

RouteState::_state_type RouteInterface::get_state() const
{
  return state_.state;
}

std::optional<LaneletRoute> RouteInterface::get_route() const
{
  return route_;
}

void RouteInterface::change_route()
{
  route_ = std::nullopt;
}

void RouteInterface::change_state(RouteState::_state_type state)
{
  state_.stamp = clock_->now();
  state_.state = state;
  pub_state_->publish(state_);
}

void RouteInterface::update_state(const RouteState & state)
{
  state_ = state;
  pub_state_->publish(state_);
}

void RouteInterface::update_route(const LaneletRoute & route)
{
  route_ = route;
  pub_route_->publish(route);
}

RouteSelector::RouteSelector(const rclcpp::NodeOptions & options)
: Node("route_selector", options), main_(get_clock()), mrm_(get_clock())
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto service_qos = rmw_qos_profile_services_default;
  const auto durable_qos = rclcpp::QoS(1).transient_local();

  // Init main route interface.
  main_.srv_clear_route = create_service<ClearRoute>(
    "~/main/clear_route",
    service_utils::handle_exception(&RouteSelector::on_clear_route_main, this));
  main_.srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/main/set_waypoint_route",
    service_utils::handle_exception(&RouteSelector::on_set_waypoint_route_main, this));
  main_.srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/main/set_lanelet_route",
    service_utils::handle_exception(&RouteSelector::on_set_lanelet_route_main, this));
  main_.pub_state_ = create_publisher<RouteState>("~/main/state", durable_qos);
  main_.pub_route_ = create_publisher<LaneletRoute>("~/main/route", durable_qos);

  // Init mrm route interface.
  mrm_.srv_clear_route = create_service<ClearRoute>(
    "~/mrm/clear_route", service_utils::handle_exception(&RouteSelector::on_clear_route_mrm, this));
  mrm_.srv_set_waypoint_route = create_service<SetWaypointRoute>(
    "~/mrm/set_waypoint_route",
    service_utils::handle_exception(&RouteSelector::on_set_waypoint_route_mrm, this));
  mrm_.srv_set_lanelet_route = create_service<SetLaneletRoute>(
    "~/mrm/set_lanelet_route",
    service_utils::handle_exception(&RouteSelector::on_set_lanelet_route_mrm, this));
  mrm_.pub_state_ = create_publisher<RouteState>("~/mrm/state", durable_qos);
  mrm_.pub_route_ = create_publisher<LaneletRoute>("~/mrm/route", durable_qos);

  // Init mission planner interface.
  group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cli_clear_route_ = create_client<ClearRoute>("~/planner/clear_route", service_qos, group_);
  cli_set_lanelet_route_ =
    create_client<SetLaneletRoute>("~/planner/set_lanelet_route", service_qos, group_);
  cli_set_waypoint_route_ =
    create_client<SetWaypointRoute>("~/planner/set_waypoint_route", service_qos, group_);
  sub_state_ = create_subscription<RouteState>(
    "~/planner/state", durable_qos, std::bind(&RouteSelector::on_state, this, _1));
  sub_route_ = create_subscription<LaneletRoute>(
    "~/planner/route", durable_qos, std::bind(&RouteSelector::on_route, this, _1));

  // Set initial state.
  main_.change_state(RouteState::INITIALIZING);
  mrm_.change_state(RouteState::INITIALIZING);
  initialized_ = false;
  mrm_operating_ = false;
  main_request_ = std::monostate{};
}

void RouteSelector::on_state(const RouteState::ConstSharedPtr msg)
{
  if (msg->state == RouteState::UNSET && !initialized_) {
    main_.change_state(RouteState::UNSET);
    mrm_.change_state(RouteState::UNSET);
    initialized_ = true;
  }

  (mrm_operating_ ? mrm_ : main_).update_state(*msg);
}

void RouteSelector::on_route(const LaneletRoute::ConstSharedPtr msg)
{
  (mrm_operating_ ? mrm_ : main_).update_route(*msg);
}

void RouteSelector::on_clear_route_main(
  ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res)
{
  // Save the request and clear old route to resume from MRM.
  main_request_ = std::monostate{};
  main_.change_route();

  // During MRM, only change the state.
  if (mrm_operating_) {
    main_.change_state(RouteState::UNSET);
    res->status.success = true;
    return;
  }

  // Forward the request if not in MRM.
  res->status = service_utils::sync_call(cli_clear_route_, req);
}

void RouteSelector::on_set_waypoint_route_main(
  SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res)
{
  // Save the request and clear old route to resume from MRM.
  req->uuid = uuid::generate_if_empty(req->uuid);
  main_request_ = req;
  main_.change_route();

  // During MRM, only change the state.
  if (mrm_operating_) {
    main_.change_state(RouteState::INTERRUPTED);
    res->status.success = true;
    return;
  }

  // Forward the request if not in MRM.
  res->status = service_utils::sync_call(cli_set_waypoint_route_, req);
}

void RouteSelector::on_set_lanelet_route_main(
  SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res)
{
  // Save the request and clear old route to resume from MRM.
  req->uuid = uuid::generate_if_empty(req->uuid);
  main_request_ = req;
  main_.change_route();

  // During MRM, only change the state.
  if (mrm_operating_) {
    main_.change_state(RouteState::INTERRUPTED);
    res->status.success = true;
    return;
  }

  // Forward the request if not in MRM.
  res->status = service_utils::sync_call(cli_set_lanelet_route_, req);
}

void RouteSelector::on_clear_route_mrm(
  ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res)
{
  res->status = resume_main_route(req);

  if (res->status.success) {
    mrm_operating_ = false;
    mrm_.change_state(RouteState::UNSET);
  }
}

void RouteSelector::on_set_waypoint_route_mrm(
  SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res)
{
  req->uuid = uuid::generate_if_empty(req->uuid);
  res->status = service_utils::sync_call(cli_set_waypoint_route_, req);

  if (res->status.success) {
    mrm_operating_ = true;
    if (main_.get_state() != RouteState::UNSET) {
      main_.change_state(RouteState::INTERRUPTED);
    }
  }
}

void RouteSelector::on_set_lanelet_route_mrm(
  SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res)
{
  req->uuid = uuid::generate_if_empty(req->uuid);
  res->status = service_utils::sync_call(cli_set_lanelet_route_, req);

  if (res->status.success) {
    mrm_operating_ = true;
    if (main_.get_state() != RouteState::UNSET) {
      main_.change_state(RouteState::INTERRUPTED);
    }
  }
}

ResponseStatus RouteSelector::resume_main_route(ClearRoute::Request::SharedPtr req)
{
  const auto create_lanelet_request = [](const LaneletRoute & route) {
    // NOTE: The start_pose.is not included in the request.
    const auto r = std::make_shared<SetLaneletRoute::Request>();
    r->header = route.header;
    r->goal_pose = route.goal_pose;
    r->segments = route.segments;
    r->uuid = route.uuid;
    r->allow_modification = route.allow_modification;
    return r;
  };

  const auto create_goal_request = [](const auto & request) {
    const auto r = std::make_shared<SetWaypointRoute::Request>();
    r->header = request->header;
    r->goal_pose = request->goal_pose;
    r->uuid = request->uuid;
    r->allow_modification = request->allow_modification;
    return r;
  };

  // Clear the route if there is no request for the main route.
  if (std::holds_alternative<std::monostate>(main_request_)) {
    return service_utils::sync_call(cli_clear_route_, req);
  }

  // Attempt to resume the main route if there is a planned route.
  if (const auto route = main_.get_route()) {
    const auto r = create_lanelet_request(route.value());
    const auto status = service_utils::sync_call(cli_set_lanelet_route_, r);
    if (status.success) return status;
  }

  // If resuming fails, replan main route using the goal.
  // NOTE: Clear the waypoints to avoid returning. Remove this once resuming is supported.
  if (const auto request = std::get_if<WaypointRequest>(&main_request_)) {
    const auto r = create_goal_request(*request);
    return service_utils::sync_call(cli_set_waypoint_route_, r);
  }
  if (const auto request = std::get_if<LaneletRequest>(&main_request_)) {
    const auto r = create_goal_request(*request);
    return service_utils::sync_call(cli_set_waypoint_route_, r);
  }
  throw std::logic_error("route_selector: unknown main route request");
}

}  // namespace autoware::mission_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mission_planner::RouteSelector)
