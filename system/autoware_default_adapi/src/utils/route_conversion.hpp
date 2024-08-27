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

#ifndef UTILS__ROUTE_CONVERSION_HPP_
#define UTILS__ROUTE_CONVERSION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/route.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_planning_msgs/msg/route_state.hpp>
#include <tier4_planning_msgs/srv/clear_route.hpp>
#include <tier4_planning_msgs/srv/set_lanelet_route.hpp>
#include <tier4_planning_msgs/srv/set_waypoint_route.hpp>

namespace autoware::default_adapi::conversion
{

using ExternalRoute = autoware_adapi_v1_msgs::msg::Route;
using InternalRoute = autoware_planning_msgs::msg::LaneletRoute;
ExternalRoute create_empty_route(const rclcpp::Time & stamp);
ExternalRoute convert_route(const InternalRoute & internal);

using ExternalState = autoware_adapi_v1_msgs::msg::RouteState;
using InternalState = tier4_planning_msgs::msg::RouteState;
ExternalState convert_state(const InternalState & internal);

using ExternalClearRequest = autoware_adapi_v1_msgs::srv::ClearRoute::Request::SharedPtr;
using InternalClearRequest = tier4_planning_msgs::srv::ClearRoute::Request::SharedPtr;
InternalClearRequest convert_request(const ExternalClearRequest & external);

using ExternalLaneletRequest = autoware_adapi_v1_msgs::srv::SetRoute::Request::SharedPtr;
using InternalLaneletRequest = tier4_planning_msgs::srv::SetLaneletRoute::Request::SharedPtr;
InternalLaneletRequest convert_request(const ExternalLaneletRequest & external);

using ExternalWaypointRequest = autoware_adapi_v1_msgs::srv::SetRoutePoints::Request::SharedPtr;
using InternalWaypointRequest = tier4_planning_msgs::srv::SetWaypointRoute::Request::SharedPtr;
InternalWaypointRequest convert_request(const ExternalWaypointRequest & external);

using ExternalResponse = autoware_adapi_v1_msgs::msg::ResponseStatus;
using InternalResponse = autoware_common_msgs::msg::ResponseStatus;
ExternalResponse convert_response(const InternalResponse & internal);

template <class ClientT, class RequestT>
ExternalResponse convert_call(ClientT & client, RequestT & req)
{
  return convert_response(client->call(convert_request(req))->status);
}

}  // namespace autoware::default_adapi::conversion

#endif  // UTILS__ROUTE_CONVERSION_HPP_
