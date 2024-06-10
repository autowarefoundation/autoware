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

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <mission_planner/mission_planner_plugin.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <vector>

namespace autoware::mission_planner::lanelet2
{

struct DefaultPlannerParameters
{
  double goal_angle_threshold_deg;
  bool enable_correct_goal_pose;
  bool consider_no_drivable_lanes;
  bool check_footprint_inside_lanes;
};

class DefaultPlanner : public mission_planner::PlannerPlugin
{
public:
  void initialize(rclcpp::Node * node) override;
  void initialize(rclcpp::Node * node, const LaneletMapBin::ConstSharedPtr msg) override;
  bool ready() const override;
  LaneletRoute plan(const RoutePoints & points) override;
  void updateRoute(const PlannerPlugin::LaneletRoute & route) override;
  void clearRoute() override;
  MarkerArray visualize(const LaneletRoute & route) const override;
  MarkerArray visualize_debug_footprint(tier4_autoware_utils::LinearRing2d goal_footprint_) const;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

private:
  using RouteSections = std::vector<autoware_planning_msgs::msg::LaneletSegment>;
  using Pose = geometry_msgs::msg::Pose;
  bool is_graph_ready_;
  route_handler::RouteHandler route_handler_;

  DefaultPlannerParameters param_;

  rclcpp::Node * node_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_subscriber_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_goal_footprint_marker_;

  void initialize_common(rclcpp::Node * node);
  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  /**
   * @brief check if the goal_footprint is within the combined lanelet of route_lanelets plus the
   * succeeding lanelets around the goal
   * @attention this function will terminate when the accumulated search length from the initial
   * current_lanelet exceeds max_longitudinal_offset_m + search_margin, so under normal assumptions
   * (i.e. the map is composed of finite elements of practically normal sized lanelets), it is
   * assured to terminate
   * @param current_lanelet the start lanelet to begin recursive query
   * @param combined_prev_lanelet initial entire route_lanelets plus the small consecutive lanelets
   * around the goal during the query
   * @param next_lane_length the accumulated total length from the start lanelet of the search to
   * the lanelet of current goal query
   */
  bool check_goal_footprint_inside_lanes(
    const lanelet::ConstLanelet & current_lanelet,
    const lanelet::ConstLanelet & combined_prev_lanelet,
    const tier4_autoware_utils::Polygon2d & goal_footprint, double & next_lane_length,
    const double search_margin = 2.0);

  /**
   * @brief return true if (1)the goal is in parking area or (2)the goal is on the lanes and the
   * footprint around the goal does not overlap the lanes
   */
  bool is_goal_valid(const geometry_msgs::msg::Pose & goal, lanelet::ConstLanelets path_lanelets);

  /**
   * @brief project the specified goal pose onto the goal lanelet(the last preferred lanelet of
   * route_sections) and return the z-aligned goal position
   */
  Pose refine_goal_height(const Pose & goal, const RouteSections & route_sections);
};

}  // namespace autoware::mission_planner::lanelet2

#endif  // LANELET2_PLUGINS__DEFAULT_PLANNER_HPP_
