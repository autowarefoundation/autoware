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
#ifndef AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DATA_STRUCTS_HPP_
#define AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DATA_STRUCTS_HPP_

#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"

#include <autoware/behavior_path_planner_common/parameters.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>
#include <interpolation/linear_interpolation.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner::lane_change
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using route_handler::Direction;
using route_handler::RouteHandler;
using utils::path_safety_checker::ExtendedPredictedObjects;

struct LateralAccelerationMap
{
  std::vector<double> base_vel;
  std::vector<double> base_min_acc;
  std::vector<double> base_max_acc;

  void add(const double velocity, const double min_acc, const double max_acc)
  {
    if (base_vel.size() != base_min_acc.size() || base_vel.size() != base_max_acc.size()) {
      return;
    }

    size_t idx = 0;
    for (size_t i = 0; i < base_vel.size(); ++i) {
      if (velocity < base_vel.at(i)) {
        break;
      }
      idx = i + 1;
    }

    base_vel.insert(base_vel.begin() + idx, velocity);
    base_min_acc.insert(base_min_acc.begin() + idx, min_acc);
    base_max_acc.insert(base_max_acc.begin() + idx, max_acc);
  }

  std::pair<double, double> find(const double velocity) const
  {
    if (!base_vel.empty() && velocity < base_vel.front()) {
      return std::make_pair(base_min_acc.front(), base_max_acc.front());
    }
    if (!base_vel.empty() && velocity > base_vel.back()) {
      return std::make_pair(base_min_acc.back(), base_max_acc.back());
    }

    const double min_acc = interpolation::lerp(base_vel, base_min_acc, velocity);
    const double max_acc = interpolation::lerp(base_vel, base_max_acc, velocity);

    return std::make_pair(min_acc, max_acc);
  }
};

struct CancelParameters
{
  bool enable_on_prepare_phase{true};
  bool enable_on_lane_changing_phase{false};
  double delta_time{1.0};
  double duration{5.0};
  double max_lateral_jerk{10.0};
  double overhang_tolerance{0.0};

  // unsafe_hysteresis_threshold will be compare with the number of detected unsafe instance. If the
  // number of unsafe exceeds unsafe_hysteresis_threshold, the lane change will be cancelled or
  // aborted.
  int unsafe_hysteresis_threshold{2};
};

struct Parameters
{
  // trajectory generation
  double backward_lane_length{200.0};
  double prediction_time_resolution{0.5};
  int longitudinal_acc_sampling_num{10};
  int lateral_acc_sampling_num{10};

  // lane change parameters
  double backward_length_buffer_for_end_of_lane{0.0};
  double backward_length_buffer_for_blocking_object{0.0};
  double lane_changing_lateral_jerk{0.5};
  double minimum_lane_changing_velocity{5.6};
  double lane_change_prepare_duration{4.0};
  LateralAccelerationMap lane_change_lat_acc_map;

  // parked vehicle
  double object_check_min_road_shoulder_width{0.5};
  double object_shiftable_ratio_threshold{0.6};

  // turn signal
  double min_length_for_turn_signal_activation{10.0};
  double length_ratio_for_turn_signal_deactivation{0.8};

  // acceleration data
  double min_longitudinal_acc{-1.0};
  double max_longitudinal_acc{1.0};

  // collision check
  bool enable_collision_check_for_prepare_phase_in_general_lanes{false};
  bool enable_collision_check_for_prepare_phase_in_intersection{true};
  bool enable_collision_check_for_prepare_phase_in_turns{true};
  double prepare_segment_ignore_object_velocity_thresh{0.1};
  bool check_objects_on_current_lanes{true};
  bool check_objects_on_other_lanes{true};
  bool use_all_predicted_path{false};
  double lane_expansion_left_offset{0.0};
  double lane_expansion_right_offset{0.0};

  // regulatory elements
  bool regulate_on_crosswalk{false};
  bool regulate_on_intersection{false};
  bool regulate_on_traffic_light{false};

  // ego vehicle stuck detection
  double stop_velocity_threshold{0.1};
  double stop_time_threshold{3.0};

  // true by default for all objects
  utils::path_safety_checker::ObjectTypesToCheck object_types_to_check;

  // safety check
  bool allow_loose_check_for_cancel{true};
  double collision_check_yaw_diff_threshold{3.1416};
  utils::path_safety_checker::RSSparams rss_params{};
  utils::path_safety_checker::RSSparams rss_params_for_parked{};
  utils::path_safety_checker::RSSparams rss_params_for_abort{};
  utils::path_safety_checker::RSSparams rss_params_for_stuck{};

  // abort
  CancelParameters cancel{};

  // finish judge parameter
  double lane_change_finish_judge_buffer{3.0};
  double finish_judge_lateral_threshold{0.2};
  double finish_judge_lateral_angle_deviation{autoware::universe_utils::deg2rad(3.0)};

  // debug marker
  bool publish_debug_marker{false};
};

enum class States {
  Normal = 0,
  Cancel,
  Abort,
  Stop,
};

struct PhaseInfo
{
  double prepare{0.0};
  double lane_changing{0.0};

  [[nodiscard]] double sum() const { return prepare + lane_changing; }

  PhaseInfo(const double _prepare, const double _lane_changing)
  : prepare(_prepare), lane_changing(_lane_changing)
  {
  }
};

struct Lanes
{
  bool current_lane_in_goal_section{false};
  lanelet::ConstLanelets current;
  lanelet::ConstLanelets target;
  std::vector<lanelet::ConstLanelets> preceding_target;
};

struct Info
{
  PhaseInfo longitudinal_acceleration{0.0, 0.0};
  PhaseInfo velocity{0.0, 0.0};
  PhaseInfo duration{0.0, 0.0};
  PhaseInfo length{0.0, 0.0};

  Pose lane_changing_start{};
  Pose lane_changing_end{};

  ShiftLine shift_line{};

  double lateral_acceleration{0.0};
  double terminal_lane_changing_velocity{0.0};
};

struct LanesObjects
{
  ExtendedPredictedObjects current_lane{};
  ExtendedPredictedObjects target_lane{};
  ExtendedPredictedObjects other_lane{};
};

enum class ModuleType {
  NORMAL = 0,
  EXTERNAL_REQUEST,
  AVOIDANCE_BY_LANE_CHANGE,
};

struct PathSafetyStatus
{
  bool is_safe{true};
  bool is_object_coming_from_rear{false};
};

struct LanesPolygon
{
  std::optional<lanelet::BasicPolygon2d> current;
  std::optional<lanelet::BasicPolygon2d> target;
  std::optional<lanelet::BasicPolygon2d> expanded_target;
  lanelet::BasicPolygon2d target_neighbor;
  std::vector<lanelet::BasicPolygon2d> preceding_target;
};

using RouteHandlerPtr = std::shared_ptr<RouteHandler>;
using BppParamPtr = std::shared_ptr<BehaviorPathPlannerParameters>;
using LCParamPtr = std::shared_ptr<Parameters>;
using LanesPtr = std::shared_ptr<Lanes>;
using LanesPolygonPtr = std::shared_ptr<LanesPolygon>;

struct CommonData
{
  RouteHandlerPtr route_handler_ptr;
  Odometry::ConstSharedPtr self_odometry_ptr;
  BppParamPtr bpp_param_ptr;
  LCParamPtr lc_param_ptr;
  LanesPtr lanes_ptr;
  LanesPolygonPtr lanes_polygon_ptr;
  ModuleType lc_type;
  Direction direction;

  [[nodiscard]] Pose get_ego_pose() const { return self_odometry_ptr->pose.pose; }

  [[nodiscard]] Twist get_ego_twist() const { return self_odometry_ptr->twist.twist; }

  [[nodiscard]] double get_ego_speed(bool use_norm = false) const
  {
    if (!use_norm) {
      return get_ego_twist().linear.x;
    }

    const auto x = get_ego_twist().linear.x;
    const auto y = get_ego_twist().linear.y;
    return std::hypot(x, y);
  }
};
using CommonDataPtr = std::shared_ptr<CommonData>;
}  // namespace autoware::behavior_path_planner::lane_change

namespace autoware::behavior_path_planner
{
using LaneChangeModuleType = lane_change::ModuleType;
using LaneChangeParameters = lane_change::Parameters;
using LaneChangeStates = lane_change::States;
using LaneChangePhaseInfo = lane_change::PhaseInfo;
using LaneChangeInfo = lane_change::Info;
using LaneChangeLanesFilteredObjects = lane_change::LanesObjects;
using LateralAccelerationMap = lane_change::LateralAccelerationMap;
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_LANE_CHANGE_MODULE__UTILS__DATA_STRUCTS_HPP_
