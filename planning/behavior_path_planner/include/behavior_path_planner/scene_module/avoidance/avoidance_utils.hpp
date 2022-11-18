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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_UTILS_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_UTILS_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module_data.hpp"

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using behavior_path_planner::PlannerData;
bool isOnRight(const ObjectData & obj);

double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin);

bool isSameDirectionShift(const bool & is_object_on_right, const double & shift_length);

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc);

ShiftedPath toShiftedPath(const PathWithLaneId & path);

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points);

std::vector<size_t> concatParentIds(
  const std::vector<size_t> & ids1, const std::vector<size_t> & ids2);

double lerpShiftLengthOnArc(double arc, const AvoidLine & al);

void clipByMinStartIdx(const AvoidLineArray & shift_lines, PathWithLaneId & path);

void fillLongitudinalAndLengthByClosestFootprint(
  const PathWithLaneId & path, const PredictedObject & object, const Point & ego_pos,
  ObjectData & obj);

void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj);

double calcOverhangDistance(
  const ObjectData & object_data, const Pose & base_pose, Point & overhang_pose);

double calcEnvelopeOverhangDistance(
  const ObjectData & object_data, const Pose & base_pose, Point & overhang_pose);

void setEndData(
  AvoidLine & al, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist);

void setStartData(
  AvoidLine & al, const double start_shift_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist);

std::string getUuidStr(const ObjectData & obj);

std::vector<std::string> getUuidStr(const ObjectDataArray & objs);

Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer);
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__AVOIDANCE__AVOIDANCE_UTILS_HPP_
