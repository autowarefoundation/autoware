// Copyright 2020 Tier IV, Inc.
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

#ifndef SCENARIO_SELECTOR__SCENARIO_SELECTOR_NODE_HPP_
#define SCENARIO_SELECTOR__SCENARIO_SELECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <route_handler/route_handler.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>

class ScenarioSelectorNode : public rclcpp::Node
{
public:
  explicit ScenarioSelectorNode(const rclcpp::NodeOptions & node_options);

  void onMap(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  void onRoute(const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onParkingState(const std_msgs::msg::Bool::ConstSharedPtr msg);

  bool isDataReady();
  void onTimer();
  void onLaneDrivingTrajectory(
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onParkingTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void publishTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg);

  void updateCurrentScenario();
  std::string selectScenarioByPosition();
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr getScenarioTrajectory(
    const std::string & scenario);

private:
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_lanelet_map_;
  rclcpp::Subscription<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_lane_driving_trajectory_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_parking_trajectory_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_parking_state_;
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<tier4_planning_msgs::msg::Scenario>::SharedPtr pub_scenario_;

  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr lane_driving_trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr parking_trajectory_;
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route_;
  nav_msgs::msg::Odometry::ConstSharedPtr current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;

  std::string current_scenario_;
  std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  std::shared_ptr<route_handler::RouteHandler> route_handler_;

  // Parameters
  double update_rate_;
  double th_max_message_delay_sec_;
  double th_arrived_distance_m_;
  double th_stopped_time_sec_;
  double th_stopped_velocity_mps_;
  bool is_parking_completed_;
};

#endif  // SCENARIO_SELECTOR__SCENARIO_SELECTOR_NODE_HPP_
