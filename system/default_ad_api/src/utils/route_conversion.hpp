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
#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>

namespace default_ad_api::conversion
{

using ExternalRoute = autoware_adapi_v1_msgs::msg::Route;
using InternalRoute = autoware_planning_msgs::msg::LaneletRoute;

ExternalRoute create_empty_route(const rclcpp::Time & stamp);
ExternalRoute convert_route(const InternalRoute & internal);

}  // namespace default_ad_api::conversion

#endif  // UTILS__ROUTE_CONVERSION_HPP_
