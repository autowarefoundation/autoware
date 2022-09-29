// Copyright 2019 Autoware Foundation
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

#ifndef MISSION_PLANNER__MISSION_PLANNER_LANELET2_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_LANELET2_HPP_

#include <string>
#include <vector>

// ROS
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include "mission_planner/mission_planner_interface.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace mission_planner
{
using RouteSections = std::vector<autoware_auto_mapping_msgs::msg::HADMapSegment>;
class MissionPlannerLanelet2 : public MissionPlannerInterface
{
public:
  explicit MissionPlannerLanelet2(const rclcpp::NodeOptions & node_options);

private:
  bool is_graph_ready_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  route_handler::RouteHandler route_handler_;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_subscriber_;

  void map_callback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  bool is_goal_valid(const geometry_msgs::msg::Pose & goal_pose) const;
  geometry_msgs::msg::Pose refine_goal_height(
    const RouteSections & route_sections, const geometry_msgs::msg::Pose & goal_pose) const;

  // virtual functions
  bool is_routing_graph_ready() const override;
  autoware_auto_planning_msgs::msg::HADMapRoute plan_route(
    const std::vector<geometry_msgs::msg::Pose> & check_points) override;
  void visualize_route(const autoware_auto_planning_msgs::msg::HADMapRoute & route) const override;
};
}  // namespace mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_LANELET2_HPP_
