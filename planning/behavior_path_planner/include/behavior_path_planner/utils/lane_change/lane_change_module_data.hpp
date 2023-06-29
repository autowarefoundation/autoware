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
#ifndef BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_

#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "lanelet2_core/geometry/Lanelet.h"

#include "autoware_auto_planning_msgs/msg/path_point_with_lane_id.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{

struct LaneChangeCancelParameters
{
  bool enable_on_prepare_phase{true};
  bool enable_on_lane_changing_phase{false};
  double delta_time{1.0};
  double duration{5.0};
  double max_lateral_jerk{10.0};
  double overhang_tolerance{0.0};
};
struct LaneChangeParameters
{
  // trajectory generation
  double backward_lane_length{200.0};
  double prediction_time_resolution{0.5};
  int longitudinal_acc_sampling_num{10};
  int lateral_acc_sampling_num{10};

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
  bool enable_prepare_segment_collision_check{true};
  double prepare_segment_ignore_object_velocity_thresh{0.1};
  bool use_predicted_path_outside_lanelet{false};
  bool use_all_predicted_path{false};

  // true by default
  bool check_car{true};         // check object car
  bool check_truck{true};       // check object truck
  bool check_bus{true};         // check object bus
  bool check_trailer{true};     // check object trailer
  bool check_unknown{true};     // check object unknown
  bool check_bicycle{true};     // check object bicycle
  bool check_motorcycle{true};  // check object motorbike
  bool check_pedestrian{true};  // check object pedestrian

  // abort
  LaneChangeCancelParameters cancel;

  double finish_judge_lateral_threshold{0.2};

  // debug marker
  bool publish_debug_marker{false};
};

enum class LaneChangeStates {
  Normal = 0,
  Cancel,
  Abort,
  Stop,
};

struct LaneChangePhaseInfo
{
  double prepare{0.0};
  double lane_changing{0.0};

  [[nodiscard]] double sum() const { return prepare + lane_changing; }

  LaneChangePhaseInfo(const double _prepare, const double _lane_changing)
  : prepare(_prepare), lane_changing(_lane_changing)
  {
  }
};

struct LaneChangeTargetObjectIndices
{
  std::vector<size_t> current_lane{};
  std::vector<size_t> target_lane{};
  std::vector<size_t> other_lane{};
};

enum class LaneChangeModuleType {
  NORMAL = 0,
  EXTERNAL_REQUEST,
  AVOIDANCE_BY_LANE_CHANGE,
};

struct AvoidanceByLCParameters
{
  std::shared_ptr<AvoidanceParameters> avoidance{};
  std::shared_ptr<LaneChangeParameters> lane_change{};

  // execute if the target object number is larger than this param.
  size_t execute_object_num{1};

  // execute only when the target object longitudinal distance is larger than this param.
  double execute_object_longitudinal_margin{0.0};

  // execute only when lane change end point is before the object.
  bool execute_only_when_lane_change_finish_before_object{false};
};
}  // namespace behavior_path_planner

namespace behavior_path_planner::data::lane_change
{
struct PathSafetyStatus
{
  bool is_safe{true};
  bool is_object_coming_from_rear{false};
};
}  // namespace behavior_path_planner::data::lane_change

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__LANE_CHANGE_MODULE_DATA_HPP_
