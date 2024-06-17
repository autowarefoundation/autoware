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

#ifndef REMAINING_DISTANCE_TIME_CALCULATOR_NODE_HPP_
#define REMAINING_DISTANCE_TIME_CALCULATOR_NODE_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_msgs/msg/mission_remaining_distance_time.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <memory>

namespace autoware::remaining_distance_time_calculator
{

struct NodeParam
{
  double update_rate{0.0};
};

class RemainingDistanceTimeCalculatorNode : public rclcpp::Node
{
public:
  explicit RemainingDistanceTimeCalculatorNode(const rclcpp::NodeOptions & options);

private:
  using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
  using HADMapBin = autoware_map_msgs::msg::LaneletMapBin;
  using Odometry = nav_msgs::msg::Odometry;
  using VelocityLimit = tier4_planning_msgs::msg::VelocityLimit;
  using MissionRemainingDistanceTime = autoware_internal_msgs::msg::MissionRemainingDistanceTime;

  rclcpp::Subscription<LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_planning_velocity_;

  rclcpp::Publisher<MissionRemainingDistanceTime>::SharedPtr pub_mission_remaining_distance_time_;

  rclcpp::TimerBase::SharedPtr timer_;

  route_handler::RouteHandler route_handler_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::ConstLanelets road_lanelets_;
  bool is_graph_ready_;

  // Data Buffer
  geometry_msgs::msg::Pose current_vehicle_pose_;
  geometry_msgs::msg::Vector3 current_vehicle_velocity_;
  geometry_msgs::msg::Pose goal_pose_;
  bool has_received_route_;
  double velocity_limit_;

  double remaining_distance_;
  double remaining_time_;

  // Parameter
  NodeParam node_param_;

  // Callbacks
  void on_timer();
  void on_odometry(const Odometry::ConstSharedPtr & msg);
  void on_route(const LaneletRoute::ConstSharedPtr & msg);
  void on_map(const HADMapBin::ConstSharedPtr & msg);
  void on_velocity_limit(const VelocityLimit::ConstSharedPtr & msg);

  /**
   * @brief calculate mission remaining distance
   */
  void calculate_remaining_distance();

  /**
   * @brief calculate mission remaining time
   */
  void calculate_remaining_time();

  /**
   * @brief publish mission remaining distance and time
   */
  void publish_mission_remaining_distance_time();
};
}  // namespace autoware::remaining_distance_time_calculator
#endif  // REMAINING_DISTANCE_TIME_CALCULATOR_NODE_HPP_
