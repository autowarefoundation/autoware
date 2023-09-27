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

#ifndef MISSION_PLANNER__MISSION_PLANNER_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_HPP_

#include "arrival_checker.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <component_interface_specs/planning.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <mission_planner/mission_planner_interface.hpp>
#include <mission_planner/mission_planner_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace mission_planner
{

using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
using LaneletRoute = autoware_planning_msgs::msg::LaneletRoute;
using LaneletPrimitive = autoware_planning_msgs::msg::LaneletPrimitive;
using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using ClearRoute = planning_interface::ClearRoute;
using SetRoutePoints = planning_interface::SetRoutePoints;
using SetRoute = planning_interface::SetRoute;
using ChangeRoutePoints = planning_interface::ChangeRoutePoints;
using ChangeRoute = planning_interface::ChangeRoute;
using Route = planning_interface::Route;
using NormalRoute = planning_interface::NormalRoute;
using MrmRoute = planning_interface::MrmRoute;
using RouteState = planning_interface::RouteState;
using Odometry = nav_msgs::msg::Odometry;
using RerouteAvailability = tier4_planning_msgs::msg::RerouteAvailability;

class MissionPlanner : public rclcpp::Node
{
public:
  explicit MissionPlanner(const rclcpp::NodeOptions & options);

private:
  ArrivalChecker arrival_checker_;
  pluginlib::ClassLoader<PlannerPlugin> plugin_loader_;
  std::shared_ptr<PlannerPlugin> planner_;
  std::string map_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  PoseStamped transform_pose(const PoseStamped & input);

  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<RerouteAvailability>::SharedPtr sub_reroute_availability_;
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_modified_goal_;

  Odometry::ConstSharedPtr odometry_;
  HADMapBin::ConstSharedPtr map_ptr_;
  RerouteAvailability::ConstSharedPtr reroute_availability_;
  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_map(const HADMapBin::ConstSharedPtr msg);
  void on_reroute_availability(const RerouteAvailability::ConstSharedPtr msg);
  void on_modified_goal(const PoseWithUuidStamped::ConstSharedPtr msg);

  rclcpp::TimerBase::SharedPtr data_check_timer_;
  void checkInitialization();

  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  void clear_route();
  void clear_mrm_route();
  void change_route(const LaneletRoute & route);
  void change_mrm_route(const LaneletRoute & route);
  LaneletRoute create_route(const SetRoute::Service::Request::SharedPtr req);
  LaneletRoute create_route(const SetRoutePoints::Service::Request::SharedPtr req);
  LaneletRoute create_route(
    const std_msgs::msg::Header & header,
    const std::vector<autoware_adapi_v1_msgs::msg::RouteSegment> & route_segments,
    const geometry_msgs::msg::Pose & goal_pose, const bool allow_goal_modification);
  LaneletRoute create_route(
    const std_msgs::msg::Header & header, const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const geometry_msgs::msg::Pose & goal_pose, const bool allow_goal_modification);

  RouteState::Message state_;
  component_interface_utils::Publisher<RouteState>::SharedPtr pub_state_;
  component_interface_utils::Publisher<Route>::SharedPtr pub_route_;
  component_interface_utils::Publisher<NormalRoute>::SharedPtr pub_normal_route_;
  component_interface_utils::Publisher<MrmRoute>::SharedPtr pub_mrm_route_;
  void change_state(RouteState::Message::_state_type state);

  component_interface_utils::Service<ClearRoute>::SharedPtr srv_clear_route_;
  component_interface_utils::Service<SetRoute>::SharedPtr srv_set_route_;
  component_interface_utils::Service<SetRoutePoints>::SharedPtr srv_set_route_points_;
  component_interface_utils::Service<ChangeRoute>::SharedPtr srv_change_route_;
  component_interface_utils::Service<ChangeRoutePoints>::SharedPtr srv_change_route_points_;
  void on_clear_route(
    const ClearRoute::Service::Request::SharedPtr req,
    const ClearRoute::Service::Response::SharedPtr res);
  void on_set_route(
    const SetRoute::Service::Request::SharedPtr req,
    const SetRoute::Service::Response::SharedPtr res);
  void on_set_route_points(
    const SetRoutePoints::Service::Request::SharedPtr req,
    const SetRoutePoints::Service::Response::SharedPtr res);

  component_interface_utils::Service<SetMrmRoute>::SharedPtr srv_set_mrm_route_;
  component_interface_utils::Service<ClearMrmRoute>::SharedPtr srv_clear_mrm_route_;
  void on_set_mrm_route(
    const SetMrmRoute::Service::Request::SharedPtr req,
    const SetMrmRoute::Service::Response::SharedPtr res);
  void on_clear_mrm_route(
    const ClearMrmRoute::Service::Request::SharedPtr req,
    const ClearMrmRoute::Service::Response::SharedPtr res);

  void on_change_route(
    const SetRoute::Service::Request::SharedPtr req,
    const SetRoute::Service::Response::SharedPtr res);
  void on_change_route_points(
    const SetRoutePoints::Service::Request::SharedPtr req,
    const SetRoutePoints::Service::Response::SharedPtr res);

  double reroute_time_threshold_{10.0};
  double minimum_reroute_length_{30.0};
  bool check_reroute_safety(const LaneletRoute & original_route, const LaneletRoute & target_route);

  std::shared_ptr<LaneletRoute> normal_route_{nullptr};
  std::shared_ptr<LaneletRoute> mrm_route_{nullptr};

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
