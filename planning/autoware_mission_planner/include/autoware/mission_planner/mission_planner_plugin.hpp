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

#ifndef AUTOWARE__MISSION_PLANNER__MISSION_PLANNER_PLUGIN_HPP_
#define AUTOWARE__MISSION_PLANNER__MISSION_PLANNER_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace autoware::mission_planner
{

class PlannerPlugin
{
public:
  using RoutePoints = std::vector<geometry_msgs::msg::Pose>;
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  virtual ~PlannerPlugin() = default;
  virtual void initialize(rclcpp::Node * node) = 0;
  virtual void initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg) = 0;
  virtual bool ready() const = 0;
  virtual LaneletRoute plan(const RoutePoints & points) = 0;
  virtual MarkerArray visualize(const LaneletRoute & route) const = 0;
  virtual void updateRoute(const LaneletRoute & route) = 0;
  virtual void clearRoute() = 0;
};

}  // namespace autoware::mission_planner

#endif  // AUTOWARE__MISSION_PLANNER__MISSION_PLANNER_PLUGIN_HPP_
