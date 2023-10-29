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

#ifndef BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_

#include <interpolation/linear_interpolation.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <utility>
#include <vector>

struct ModuleConfigParameters
{
  bool enable_module{false};
  bool enable_rtc{false};
  bool enable_simultaneous_execution_as_approved_module{false};
  bool enable_simultaneous_execution_as_candidate_module{false};
  bool keep_last{false};
  uint8_t priority{0};
  uint8_t max_module_size{0};
};

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

struct BehaviorPathPlannerParameters
{
  bool verbose;
  size_t max_iteration_num{100};

  ModuleConfigParameters config_avoidance;
  ModuleConfigParameters config_avoidance_by_lc;
  ModuleConfigParameters config_dynamic_avoidance;
  ModuleConfigParameters config_start_planner;
  ModuleConfigParameters config_goal_planner;
  ModuleConfigParameters config_side_shift;
  ModuleConfigParameters config_lane_change_left;
  ModuleConfigParameters config_lane_change_right;
  ModuleConfigParameters config_ext_request_lane_change_left;
  ModuleConfigParameters config_ext_request_lane_change_right;

  double backward_path_length;
  double forward_path_length;
  double backward_length_buffer_for_end_of_lane;
  double backward_length_buffer_for_blocking_object;
  double backward_length_buffer_for_end_of_pull_over;
  double backward_length_buffer_for_end_of_pull_out;

  // common parameters
  double min_acc;
  double max_acc;

  // lane change parameters
  double lane_changing_lateral_jerk{0.5};
  double minimum_lane_changing_velocity{5.6};
  double lane_change_prepare_duration{4.0};
  double lane_change_finish_judge_buffer{3.0};
  LateralAccelerationMap lane_change_lat_acc_map;

  double minimum_pull_over_length;
  double minimum_pull_out_length;
  double drivable_area_resolution;

  double refine_goal_search_radius_range;

  double turn_signal_intersection_search_distance;
  double turn_signal_intersection_angle_threshold_deg;
  double turn_signal_search_time;
  double turn_signal_minimum_search_distance;
  double turn_signal_shift_length_threshold;
  bool turn_signal_on_swerving;

  bool enable_akima_spline_first;
  bool enable_cog_on_centerline;
  double input_path_interval;
  double output_path_interval;

  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;

  // vehicle info
  vehicle_info_util::VehicleInfo vehicle_info;
  double wheel_base;
  double front_overhang;
  double rear_overhang;
  double vehicle_width;
  double vehicle_length;
  double wheel_tread;
  double left_over_hang;
  double right_over_hang;
  double base_link2front;
  double base_link2rear;

  // maximum drivable area visualization
  bool visualize_maximum_drivable_area;
};

#endif  // BEHAVIOR_PATH_PLANNER__PARAMETERS_HPP_
