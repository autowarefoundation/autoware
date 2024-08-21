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

/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
#define AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_

#include "autoware/universe_utils/ros/logger_level_configure.hpp"

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planning_algorithms/rrtstar.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <autoware/route_handler/route_handler.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace autoware::freespace_planner
{
using autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using autoware::freespace_planning_algorithms::AstarParam;
using autoware::freespace_planning_algorithms::AstarSearch;
using autoware::freespace_planning_algorithms::PlannerCommonParam;
using autoware::freespace_planning_algorithms::RRTStar;
using autoware::freespace_planning_algorithms::RRTStarParam;
using autoware::freespace_planning_algorithms::VehicleShape;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using tier4_planning_msgs::msg::Scenario;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;  // collision margin [m]
  double th_obstacle_time_sec;
  double vehicle_shape_margin_m;
  bool replan_when_obstacle_found;
  bool replan_when_course_out;
};

class FreespacePlannerNode : public rclcpp::Node
{
public:
  explicit FreespacePlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // ros
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_pose_array_pub_;
  rclcpp::Publisher<PoseArray>::SharedPtr debug_partial_pose_array_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr parking_state_pub_;

  rclcpp::Subscription<LaneletRoute>::SharedPtr route_sub_;

  autoware::universe_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_sub_{
    this, "~/input/occupancy_grid"};
  autoware::universe_utils::InterProcessPollingSubscriber<Scenario> scenario_sub_{
    this, "~/input/scenario"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    Odometry, autoware::universe_utils::polling_policy::All>
    odom_sub_{this, "~/input/odometry", rclcpp::QoS{100}};

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;

  Trajectory trajectory_;
  Trajectory partial_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  bool is_completed_ = false;
  bool reset_in_progress_ = false;
  bool is_new_parking_cycle_ = true;
  boost::optional<rclcpp::Time> obs_found_time_;

  LaneletRoute::ConstSharedPtr route_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  Scenario::ConstSharedPtr scenario_;
  Odometry::ConstSharedPtr odom_;
  std::shared_ptr<autoware::route_handler::RouteHandler> route_handler_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);

  void updateData();
  bool isDataReady();

  void onTimer();

  void reset();
  bool isPlanRequired();
  void planTrajectory();
  void updateTargetIndex();
  void initializePlanningAlgorithm();

  bool checkCurrentTrajectoryCollision();

  TransformStamped getTransform(const std::string & from, const std::string & to);

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace autoware::freespace_planner

#endif  // AUTOWARE__FREESPACE_PLANNER__FREESPACE_PLANNER_NODE_HPP_
