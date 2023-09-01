// Copyright 2023 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__PARAMETERS_HPP_

#include "behavior_path_planner/utils/drivable_area_expansion/types.hpp"

#include <rclcpp/node.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <string>
#include <vector>

namespace drivable_area_expansion
{

struct DrivableAreaExpansionParameters
{
  static constexpr auto DRIVABLE_AREA_RIGHT_BOUND_OFFSET_PARAM = "drivable_area_right_bound_offset";
  static constexpr auto DRIVABLE_AREA_LEFT_BOUND_OFFSET_PARAM = "drivable_area_left_bound_offset";
  static constexpr auto DRIVABLE_AREA_TYPES_TO_SKIP_PARAM = "drivable_area_types_to_skip";
  static constexpr auto ENABLED_PARAM = "dynamic_expansion.enabled";
  static constexpr auto EGO_EXTRA_OFFSET_FRONT =
    "dynamic_expansion.ego.extra_footprint_offset.front";
  static constexpr auto EGO_EXTRA_OFFSET_REAR = "dynamic_expansion.ego.extra_footprint_offset.rear";
  static constexpr auto EGO_EXTRA_OFFSET_LEFT = "dynamic_expansion.ego.extra_footprint_offset.left";
  static constexpr auto EGO_EXTRA_OFFSET_RIGHT =
    "dynamic_expansion.ego.extra_footprint_offset.right";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_FRONT =
    "dynamic_expansion.dynamic_objects.extra_footprint_offset.front";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_REAR =
    "dynamic_expansion.dynamic_objects.extra_footprint_offset.rear";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_LEFT =
    "dynamic_expansion.dynamic_objects.extra_footprint_offset.left";
  static constexpr auto DYN_OBJECTS_EXTRA_OFFSET_RIGHT =
    "dynamic_expansion.dynamic_objects.extra_footprint_offset.right";
  static constexpr auto EXPANSION_METHOD_PARAM = "dynamic_expansion.expansion.method";
  static constexpr auto MAX_EXP_DIST_PARAM = "dynamic_expansion.expansion.max_distance";
  static constexpr auto RESAMPLE_INTERVAL_PARAM =
    "dynamic_expansion.path_preprocessing.resample_interval";
  static constexpr auto MAX_PATH_ARC_LENGTH_PARAM =
    "dynamic_expansion.path_preprocessing.max_arc_length";
  static constexpr auto EXTRA_ARC_LENGTH_PARAM = "dynamic_expansion.expansion.extra_arc_length";
  static constexpr auto AVOID_DYN_OBJECTS_PARAM = "dynamic_expansion.dynamic_objects.avoid";
  static constexpr auto AVOID_LINESTRING_TYPES_PARAM = "dynamic_expansion.avoid_linestring.types";
  static constexpr auto AVOID_LINESTRING_DIST_PARAM = "dynamic_expansion.avoid_linestring.distance";
  static constexpr auto COMPENSATE_PARAM = "dynamic_expansion.avoid_linestring.compensate.enable";
  static constexpr auto EXTRA_COMPENSATE_PARAM =
    "dynamic_expansion.avoid_linestring.compensate.extra_distance";

  double drivable_area_right_bound_offset;
  double drivable_area_left_bound_offset;
  std::vector<std::string> drivable_area_types_to_skip;
  bool enabled = false;
  std::string expansion_method{};
  double avoid_linestring_dist{};
  double ego_left_offset{};
  double ego_right_offset{};
  double ego_rear_offset{};
  double ego_front_offset{};
  double ego_extra_left_offset{};
  double ego_extra_right_offset{};
  double ego_extra_rear_offset{};
  double ego_extra_front_offset{};
  double dynamic_objects_extra_left_offset{};
  double dynamic_objects_extra_right_offset{};
  double dynamic_objects_extra_rear_offset{};
  double dynamic_objects_extra_front_offset{};
  double max_expansion_distance{};
  double max_path_arc_length{};
  double resample_interval{};
  double extra_arc_length{};
  bool avoid_dynamic_objects{};
  std::vector<std::string> avoid_linestring_types{};
  bool compensate_uncrossable_lines = false;
  double compensate_extra_dist{};

  DrivableAreaExpansionParameters() = default;
  explicit DrivableAreaExpansionParameters(rclcpp::Node & node) { init(node); }

  void init(rclcpp::Node & node)
  {
    drivable_area_right_bound_offset =
      node.declare_parameter<double>(DRIVABLE_AREA_RIGHT_BOUND_OFFSET_PARAM);
    drivable_area_left_bound_offset =
      node.declare_parameter<double>(DRIVABLE_AREA_LEFT_BOUND_OFFSET_PARAM);
    drivable_area_types_to_skip =
      node.declare_parameter<std::vector<std::string>>(DRIVABLE_AREA_TYPES_TO_SKIP_PARAM);
    enabled = node.declare_parameter<bool>(ENABLED_PARAM);
    max_expansion_distance = node.declare_parameter<double>(MAX_EXP_DIST_PARAM);
    max_path_arc_length = node.declare_parameter<double>(MAX_PATH_ARC_LENGTH_PARAM);
    resample_interval = node.declare_parameter<double>(RESAMPLE_INTERVAL_PARAM);
    ego_extra_front_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_FRONT);
    ego_extra_rear_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_REAR);
    ego_extra_left_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_LEFT);
    ego_extra_right_offset = node.declare_parameter<double>(EGO_EXTRA_OFFSET_RIGHT);
    dynamic_objects_extra_front_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_FRONT);
    dynamic_objects_extra_rear_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_REAR);
    dynamic_objects_extra_left_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_LEFT);
    dynamic_objects_extra_right_offset =
      node.declare_parameter<double>(DYN_OBJECTS_EXTRA_OFFSET_RIGHT);
    avoid_linestring_types =
      node.declare_parameter<std::vector<std::string>>(AVOID_LINESTRING_TYPES_PARAM);
    avoid_dynamic_objects = node.declare_parameter<bool>(AVOID_DYN_OBJECTS_PARAM);
    avoid_linestring_dist = node.declare_parameter<double>(AVOID_LINESTRING_DIST_PARAM);
    extra_arc_length = node.declare_parameter<double>(EXTRA_ARC_LENGTH_PARAM);
    compensate_uncrossable_lines = node.declare_parameter<bool>(COMPENSATE_PARAM);
    compensate_extra_dist = node.declare_parameter<double>(EXTRA_COMPENSATE_PARAM);
    expansion_method = node.declare_parameter<std::string>(EXPANSION_METHOD_PARAM);

    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
    ego_left_offset = vehicle_info.max_lateral_offset_m;
    ego_right_offset = vehicle_info.min_lateral_offset_m;
    ego_rear_offset = vehicle_info.min_longitudinal_offset_m;
    ego_front_offset = vehicle_info.max_longitudinal_offset_m;
  }
};

}  // namespace drivable_area_expansion
#endif  // BEHAVIOR_PATH_PLANNER__UTILS__DRIVABLE_AREA_EXPANSION__PARAMETERS_HPP_
