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

#ifndef OBSTACLE_STOP_PLANNER__PLANNER_DATA_HPP_
#define OBSTACLE_STOP_PLANNER__PLANNER_DATA_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>

namespace motion_planning
{

using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::Pose;

using autoware_auto_planning_msgs::msg::TrajectoryPoint;

struct StopPoint
{
  TrajectoryPoint point{};

  size_t index;
};

struct SlowDownSection
{
  TrajectoryPoint start_point{};

  TrajectoryPoint end_point{};

  size_t slow_down_start_idx;

  size_t slow_down_end_idx;

  double velocity;
};

struct NodeParam
{
  // set True, slow down for obstacle beside the path
  bool enable_slow_down;

  // max velocity [m/s]
  double max_velocity;

  // keep slow down or stop state if obstacle vanished [s]
  double hunting_threshold;

  // dist threshold for ego's nearest index
  double ego_nearest_dist_threshold;

  // yaw threshold for ego's nearest index
  double ego_nearest_yaw_threshold;
};

struct StopParam
{
  // ==============================
  // params for longitudinal margin
  // ==============================

  // margin between obstacle and the ego's front [m]
  double max_longitudinal_margin;

  // margin between obstacle and the ego's front [m]
  // if any other stop point is inserted within max_longitudinal_margin.
  double min_longitudinal_margin;

  // ==================================
  // params for obstacle detection area
  // ==================================

  // lateral margin between the ego's footprint and
  // the boundary of the detection area for collision obstacles [m]
  // if any obstacles exist within the detection area, this module plans to stop
  // before the obstacle.
  double lateral_margin;

  // =================================
  // params for trajectory pre-process
  // =================================

  // trajectory extend distance [m]
  double extend_distance;

  // step length for pointcloud search range [m]
  double step_length;

  // ======
  // others
  // ======

  // search radius for obstacle point cloud [m]
  double stop_search_radius;

  // keep stopping if the ego is in this margin [m]
  double hold_stop_margin_distance;
};

struct SlowDownParam
{
  // =================
  // params for margin
  // =================

  // margin between obstacle and the ego's front [m]
  double longitudinal_forward_margin;

  // margin between obstacle and the ego's rear [m]
  double longitudinal_backward_margin;

  // OPTIONAL (use this param if consider_constraints is True)
  // minimum distance between obstacle and the ego's front when slow down margin is relaxed [m]
  double min_longitudinal_forward_margin;

  // OPTIONAL (use this param if consider_constraints is True)
  // fineness param for relaxing slow down margin [m]
  double longitudinal_margin_span;

  // ==================================
  // params for obstacle detection area
  // ==================================

  // lateral margin between the ego's footprint and the boundary of the detection area for slow down
  // obstacles [m]
  double lateral_margin;

  // ===================
  // params for velocity
  // ===================

  // OPTIONAL (use this param if consider_constraints is False)
  // maximum velocity fow slow down section [m/s]
  double max_slow_down_velocity;

  // OPTIONAL (use this param if consider_constraints is False)
  // minimum velocity for slow down section [m/s]
  double min_slow_down_velocity;

  // OPTIONAL (use this param if consider_constraints is True)
  // target slow down velocity [m/s]
  double slow_down_velocity;

  // OPTIONAL (use this param if consider_constraints is True)
  // velocity threshold whether the vehicle complete deceleration [m/s]
  double velocity_threshold_decel_complete;

  // OPTIONAL (use this param if consider_constraints is True)
  // acceleration threshold whether the vehicle complete deceleration [m/ss]
  double acceleration_threshold_decel_complete;

  // ===================================
  // params for deceleration constraints
  // ===================================

  // OPTIONAL (use this param if consider_constraints is True)
  // min jerk limit for mild stop [m/sss]
  double normal_min_jerk;

  // OPTIONAL (use this param if consider_constraints is True)
  // min deceleration limit for mild stop [m/ss]
  double normal_min_acc;

  // OPTIONAL (use this param if consider_constraints is True)
  // min jerk limit [m/sss]
  double limit_min_jerk;

  // OPTIONAL (use this param if consider_constraints is True)
  // min deceleration limit [m/ss]
  double limit_min_acc;

  // OPTIONAL (use this param if consider_constraints is True)
  // min slow down jerk constraint [m/sss]
  double slow_down_min_jerk;

  // OPTIONAL (use this param if consider_constraints is True)
  // init jerk used for deceleration planning [m/sss]
  double jerk_start;

  // OPTIONAL (use this param if consider_constraints is True)
  // fineness param for planning deceleration jerk [m/sss]
  double jerk_span;

  // ======
  // others
  // ======

  // set "True", decel point is planned under jerk/dec constraints
  bool consider_constraints;

  // search radius for slow down obstacle point cloud [m]
  double slow_down_search_radius;
};

struct PlannerData
{
  DiagnosticStatus stop_reason_diag{};

  Pose current_pose{};

  pcl::PointXYZ nearest_collision_point;

  pcl::PointXYZ nearest_slow_down_point;

  pcl::PointXYZ lateral_nearest_slow_down_point;

  rclcpp::Time nearest_collision_point_time{};

  double lateral_deviation{0.0};

  size_t trajectory_trim_index{};

  size_t decimate_trajectory_collision_index{};

  size_t decimate_trajectory_slow_down_index{};

  // key: decimate index, value: original index
  std::map<size_t, size_t> decimate_trajectory_index_map{};

  bool found_collision_points{false};

  bool found_slow_down_points{false};

  bool stop_require{false};

  bool slow_down_require{false};

  bool enable_adaptive_cruise{false};
};

}  // namespace motion_planning

#endif  // OBSTACLE_STOP_PLANNER__PLANNER_DATA_HPP_
