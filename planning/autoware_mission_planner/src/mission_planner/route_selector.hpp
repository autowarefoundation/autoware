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

#ifndef MISSION_PLANNER__ROUTE_SELECTOR_HPP_
#define MISSION_PLANNER__ROUTE_SELECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_planning_msgs/msg/route_state.hpp>
#include <tier4_planning_msgs/srv/clear_route.hpp>
#include <tier4_planning_msgs/srv/set_lanelet_route.hpp>
#include <tier4_planning_msgs/srv/set_waypoint_route.hpp>

#include <optional>
#include <variant>

namespace autoware::mission_planner
{

using autoware_common_msgs::msg::ResponseStatus;
using autoware_planning_msgs::msg::LaneletRoute;
using tier4_planning_msgs::msg::RouteState;
using tier4_planning_msgs::srv::ClearRoute;
using tier4_planning_msgs::srv::SetLaneletRoute;
using tier4_planning_msgs::srv::SetWaypointRoute;
using unique_identifier_msgs::msg::UUID;

class RouteInterface
{
public:
  explicit RouteInterface(rclcpp::Clock::SharedPtr clock);
  RouteState::_state_type get_state() const;
  std::optional<LaneletRoute> get_route() const;
  void change_route();
  void change_state(RouteState::_state_type state);
  void update_state(const RouteState & state);
  void update_route(const LaneletRoute & route);

  rclcpp::Service<ClearRoute>::SharedPtr srv_clear_route;
  rclcpp::Service<SetLaneletRoute>::SharedPtr srv_set_lanelet_route;
  rclcpp::Service<SetWaypointRoute>::SharedPtr srv_set_waypoint_route;
  rclcpp::Publisher<RouteState>::SharedPtr pub_state_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr pub_route_;

private:
  RouteState state_;
  std::optional<LaneletRoute> route_;
  rclcpp::Clock::SharedPtr clock_;
};

class RouteSelector : public rclcpp::Node
{
public:
  explicit RouteSelector(const rclcpp::NodeOptions & options);

private:
  using WaypointRequest = SetWaypointRoute::Request::SharedPtr;
  using LaneletRequest = SetLaneletRoute::Request::SharedPtr;

  RouteInterface main_;
  RouteInterface mrm_;

  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Client<ClearRoute>::SharedPtr cli_clear_route_;
  rclcpp::Client<SetWaypointRoute>::SharedPtr cli_set_waypoint_route_;
  rclcpp::Client<SetLaneletRoute>::SharedPtr cli_set_lanelet_route_;
  rclcpp::Subscription<RouteState>::SharedPtr sub_state_;
  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;

  bool initialized_;
  bool mrm_operating_;
  std::variant<std::monostate, WaypointRequest, LaneletRequest> main_request_;

  void on_state(const RouteState::ConstSharedPtr msg);
  void on_route(const LaneletRoute::ConstSharedPtr msg);

  void on_clear_route_main(ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res);
  void on_set_waypoint_route_main(
    SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res);
  void on_set_lanelet_route_main(
    SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res);

  void on_clear_route_mrm(ClearRoute::Request::SharedPtr req, ClearRoute::Response::SharedPtr res);
  void on_set_waypoint_route_mrm(
    SetWaypointRoute::Request::SharedPtr req, SetWaypointRoute::Response::SharedPtr res);
  void on_set_lanelet_route_mrm(
    SetLaneletRoute::Request::SharedPtr req, SetLaneletRoute::Response::SharedPtr res);

  ResponseStatus resume_main_route(ClearRoute::Request::SharedPtr req);
};

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__ROUTE_SELECTOR_HPP_
