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

#ifndef LANELET2_EXTENSION__UTILITY__ROUTE_CHECKER_HPP_
#define LANELET2_EXTENSION__UTILITY__ROUTE_CHECKER_HPP_

#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

namespace lanelet::utils::route
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_planning_msgs::msg::HADMapRoute;

bool isRouteValid(const HADMapRoute route, const lanelet::LaneletMapPtr lanelet_map_ptr_);
}  // namespace lanelet::utils::route

#endif  // LANELET2_EXTENSION__UTILITY__ROUTE_CHECKER_HPP_
