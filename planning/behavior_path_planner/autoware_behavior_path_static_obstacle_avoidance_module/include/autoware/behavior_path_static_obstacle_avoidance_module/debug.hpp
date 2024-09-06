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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__DEBUG_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__DEBUG_HPP_

#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/type_alias.hpp"

#include <memory>
#include <string>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{

using autoware::behavior_path_planner::AvoidanceParameters;
using autoware::behavior_path_planner::AvoidancePlanningData;
using autoware::behavior_path_planner::AvoidLineArray;
using autoware::behavior_path_planner::DebugData;
using autoware::behavior_path_planner::ObjectDataArray;
using autoware::behavior_path_planner::ObjectInfo;
using autoware::behavior_path_planner::PathShifter;
using autoware::behavior_path_planner::ShiftLineArray;

MarkerArray createAvoidLineMarkerArray(
  const AvoidLineArray & shift_lines, std::string && ns, const float & r, const float & g,
  const float & b, const double & w);

MarkerArray createTargetObjectsMarkerArray(const ObjectDataArray & objects, const std::string & ns);

MarkerArray createOtherObjectsMarkerArray(
  const ObjectDataArray & objects, const ObjectInfo & info, const bool verbose);

MarkerArray createAmbiguousObjectsMarkerArray(
  const ObjectDataArray & objects, const Pose & ego_pose, const std::string & policy);

MarkerArray createStopTargetObjectMarkerArray(const AvoidancePlanningData & data);

MarkerArray createDebugMarkerArray(
  const BehaviorModuleOutput & output, const AvoidancePlanningData & data,
  const PathShifter & shifter, const DebugData & debug,
  const std::shared_ptr<AvoidanceParameters> & parameters);
}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance

std::string toStrInfo(const autoware::behavior_path_planner::ShiftLineArray & sl_arr);

std::string toStrInfo(const autoware::behavior_path_planner::AvoidLineArray & ap_arr);

std::string toStrInfo(const autoware::behavior_path_planner::ShiftLine & sl);

std::string toStrInfo(const autoware::behavior_path_planner::AvoidLine & ap);

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__DEBUG_HPP_
