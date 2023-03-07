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

#ifndef LANELET2_PLUGINS__DEFAULT_PLANNER_HPP_
#define LANELET2_PLUGINS__DEFAULT_PLANNER_HPP_

#include <mission_planner/mission_planner_plugin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <vector>

namespace mission_planner::lanelet2
{

struct DefaultPlannerParameters
{
  double goal_angle_threshold_deg;
  bool enable_correct_goal_pose;
};

class DefaultPlanner : public mission_planner::PlannerPlugin
{
public:
  void initialize(rclcpp::Node * node) override;
  void initialize(rclcpp::Node * node, const HADMapBin::ConstSharedPtr msg) override;
  bool ready() const override;
  LaneletRoute plan(const RoutePoints & points) override;
  MarkerArray visualize(const LaneletRoute & route) const override;
  MarkerArray visualize_debug_footprint(tier4_autoware_utils::LinearRing2d goal_footprint_) const;
  vehicle_info_util::VehicleInfo vehicle_info_;

private:
  using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
  using Pose = geometry_msgs::msg::Pose;
  bool is_graph_ready_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  lanelet::ConstLanelets shoulder_lanelets_;
  route_handler::RouteHandler route_handler_;

  DefaultPlannerParameters param_;

  rclcpp::Node * node_;
  rclcpp::Subscription<HADMapBin>::SharedPtr map_subscriber_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_goal_footprint_marker_;

  void initialize_common(rclcpp::Node * node);
  void map_callback(const HADMapBin::ConstSharedPtr msg);
  bool check_goal_footprint(
    const lanelet::ConstLanelet & current_lanelet,
    const lanelet::ConstLanelet & combined_prev_lanelet,
    const tier4_autoware_utils::Polygon2d & goal_footprint, double & next_lane_length,
    const double search_margin = 2.0);
  bool is_goal_valid(const geometry_msgs::msg::Pose & goal, lanelet::ConstLanelets path_lanelets);
  Pose refine_goal_height(const Pose & goal, const RouteSections & route_sections);
};

}  // namespace mission_planner::lanelet2

#endif  // LANELET2_PLUGINS__DEFAULT_PLANNER_HPP_
