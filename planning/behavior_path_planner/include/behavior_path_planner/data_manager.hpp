// Copyright 2021 Tier IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__DATA_MANAGER_HPP_
#define BEHAVIOR_PATH_PLANNER__DATA_MANAGER_HPP_

#include "behavior_path_planner/parameters.hpp"

#include <rclcpp/rclcpp.hpp>
#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <memory>
#include <string>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using route_handler::RouteHandler;
struct BoolStamped
{
  explicit BoolStamped(bool in_data) : data(in_data) {}
  bool data{false};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct ModuleNameStamped
{
  std::string module_name = "NONE";
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct Approval
{
  BoolStamped is_approved{false};
  ModuleNameStamped is_force_approved{};
};

struct DrivableLanes
{
  lanelet::ConstLanelet right_lane;
  lanelet::ConstLanelet left_lane;
  lanelet::ConstLanelets middle_lanes;
};

struct PlannerData
{
  PoseStamped::ConstSharedPtr self_pose{};
  Odometry::ConstSharedPtr self_odometry{};
  AccelWithCovarianceStamped::ConstSharedPtr self_acceleration{};
  PredictedObjects::ConstSharedPtr dynamic_object{};
  OccupancyGrid::ConstSharedPtr occupancy_grid{};
  PathWithLaneId::SharedPtr reference_path{std::make_shared<PathWithLaneId>()};
  PathWithLaneId::SharedPtr prev_output_path{std::make_shared<PathWithLaneId>()};
  lanelet::ConstLanelets current_lanes{};
  std::shared_ptr<RouteHandler> route_handler{std::make_shared<RouteHandler>()};
  BehaviorPathPlannerParameters parameters{};
  Approval approval{};
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__DATA_MANAGER_HPP_
