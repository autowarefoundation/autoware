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

#ifndef BEHAVIOR_PATH_AVOIDANCE_MODULE__DEBUG_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_MODULE__DEBUG_HPP_

#include "behavior_path_avoidance_module/data_structs.hpp"
#include "behavior_path_avoidance_module/type_alias.hpp"

#include <string>

namespace behavior_path_planner::utils::avoidance
{

using behavior_path_planner::AvoidancePlanningData;
using behavior_path_planner::AvoidLineArray;
using behavior_path_planner::DebugData;
using behavior_path_planner::ObjectDataArray;
using behavior_path_planner::ObjectInfo;
using behavior_path_planner::PathShifter;
using behavior_path_planner::ShiftLineArray;

MarkerArray createEgoStatusMarkerArray(
  const AvoidancePlanningData & data, const Pose & p_ego, std::string && ns);

MarkerArray createAvoidLineMarkerArray(
  const AvoidLineArray & shift_lines, std::string && ns, const float & r, const float & g,
  const float & b, const double & w);

MarkerArray createPredictedVehiclePositions(const PathWithLaneId & path, std::string && ns);

MarkerArray createTargetObjectsMarkerArray(const ObjectDataArray & objects, const std::string & ns);

MarkerArray createOtherObjectsMarkerArray(const ObjectDataArray & objects, const ObjectInfo & info);

MarkerArray createDebugMarkerArray(
  const AvoidancePlanningData & data, const PathShifter & shifter, const DebugData & debug);
}  // namespace behavior_path_planner::utils::avoidance

std::string toStrInfo(const behavior_path_planner::ShiftLineArray & sl_arr);

std::string toStrInfo(const behavior_path_planner::AvoidLineArray & ap_arr);

std::string toStrInfo(const behavior_path_planner::ShiftLine & sl);

std::string toStrInfo(const behavior_path_planner::AvoidLine & ap);

#endif  // BEHAVIOR_PATH_AVOIDANCE_MODULE__DEBUG_HPP_
