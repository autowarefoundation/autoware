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

#include <mission_planner/mission_planner_plugin.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>
#include <tier4_autoware_utils/ros/logger_level_configure.hpp>

#include <autoware_adapi_v1_msgs/srv/set_route.hpp>
#include <autoware_adapi_v1_msgs/srv/set_route_points.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>
#include <tier4_planning_msgs/msg/route_state.hpp>
#include <tier4_planning_msgs/srv/clear_route.hpp>
#include <tier4_planning_msgs/srv/set_lanelet_route.hpp>
#include <tier4_planning_msgs/srv/set_waypoint_route.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::mission_planner
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Odometry;
using std_msgs::msg::Header;
using tier4_planning_msgs::msg::RerouteAvailability;
using tier4_planning_msgs::msg::RouteState;
using tier4_planning_msgs::srv::ClearRoute;
using tier4_planning_msgs::srv::SetLaneletRoute;
using tier4_planning_msgs::srv::SetWaypointRoute;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;

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
  Pose transform_pose(const Pose & pose, const Header & header);

  rclcpp::Service<ClearRoute>::SharedPtr srv_clear_route;
  rclcpp::Service<SetLaneletRoute>::SharedPtr srv_set_lanelet_route;
  rclcpp::Service<SetWaypointRoute>::SharedPtr srv_set_waypoint_route;
  rclcpp::Publisher<RouteState>::SharedPtr pub_state_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr pub_route_;

  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_modified_goal_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_vector_map_;
  rclcpp::Subscription<RerouteAvailability>::SharedPtr sub_reroute_availability_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  Odometry::ConstSharedPtr odometry_;
  LaneletMapBin::ConstSharedPtr map_ptr_;
  RerouteAvailability::ConstSharedPtr reroute_availability_;
  RouteState state_;
  LaneletRoute::ConstSharedPtr current_route_;
  lanelet::LaneletMapPtr lanelet_map_ptr_{nullptr};

  void on_odometry(const Odometry::ConstSharedPtr msg);
  void on_map(const LaneletMapBin::ConstSharedPtr msg);
  void on_reroute_availability(const RerouteAvailability::ConstSharedPtr msg);
  void on_modified_goal(const PoseWithUuidStamped::ConstSharedPtr msg);

  void on_clear_route(
    const ClearRoute::Request::SharedPtr req, const ClearRoute::Response::SharedPtr res);
  void on_set_lanelet_route(
    const SetLaneletRoute::Request::SharedPtr req, const SetLaneletRoute::Response::SharedPtr res);
  void on_set_waypoint_route(
    const SetWaypointRoute::Request::SharedPtr req,
    const SetWaypointRoute::Response::SharedPtr res);

  void change_state(RouteState::_state_type state);
  void change_route();
  void change_route(const LaneletRoute & route);
  void cancel_route();
  LaneletRoute create_route(const SetLaneletRoute::Request & req);
  LaneletRoute create_route(const SetWaypointRoute::Request & req);
  LaneletRoute create_route(const PoseWithUuidStamped & msg);
  LaneletRoute create_route(
    const Header & header, const std::vector<LaneletSegment> & segments, const Pose & goal_pose,
    const UUID & uuid, const bool allow_goal_modification);
  LaneletRoute create_route(
    const Header & header, const std::vector<Pose> & waypoints, const Pose & goal_pose,
    const UUID & uuid, const bool allow_goal_modification);

  void publish_pose_log(const Pose & pose, const std::string & pose_type);

  rclcpp::TimerBase::SharedPtr data_check_timer_;
  void check_initialization();

  double reroute_time_threshold_;
  double minimum_reroute_length_;
  bool check_reroute_safety(const LaneletRoute & original_route, const LaneletRoute & target_route);

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};

}  // namespace autoware::mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
