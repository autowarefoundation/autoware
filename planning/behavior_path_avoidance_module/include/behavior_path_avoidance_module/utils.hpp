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

#ifndef BEHAVIOR_PATH_AVOIDANCE_MODULE__UTILS_HPP_
#define BEHAVIOR_PATH_AVOIDANCE_MODULE__UTILS_HPP_

#include "behavior_path_avoidance_module/data_structs.hpp"
#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace behavior_path_planner::utils::avoidance
{
using behavior_path_planner::PlannerData;
using behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityAndPolygonStamped;
using behavior_path_planner::utils::path_safety_checker::PoseWithVelocityStamped;
using behavior_path_planner::utils::path_safety_checker::PredictedPathWithPolygon;

bool isOnRight(const ObjectData & obj);

double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin);

bool isShiftNecessary(const bool & is_object_on_right, const double & shift_length);

bool isSameDirectionShift(const bool & is_object_on_right, const double & shift_length);

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc);

ShiftedPath toShiftedPath(const PathWithLaneId & path);

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points);

std::vector<UUID> concatParentIds(const std::vector<UUID> & ids1, const std::vector<UUID> & ids2);

std::vector<UUID> calcParentIds(const AvoidLineArray & lines1, const AvoidLine & lines2);

double lerpShiftLengthOnArc(double arc, const AvoidLine & al);

void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj);

double calcEnvelopeOverhangDistance(
  const ObjectData & object_data, const PathWithLaneId & path, Point & overhang_pose);

void setEndData(
  AvoidLine & al, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist);

void setStartData(
  AvoidLine & al, const double start_shift_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist);

Polygon2d createEnvelopePolygon(
  const Polygon2d & object_polygon, const Pose & closest_pose, const double envelope_buffer);

Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer);

std::vector<DrivableAreaInfo::Obstacle> generateObstaclePolygonsForDrivableArea(
  const ObjectDataArray & objects, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double vehicle_width);

double getLongitudinalVelocity(const Pose & p_ref, const Pose & p_target, const double v);

bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets);

lanelet::ConstLanelets getAdjacentLane(
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters, const bool is_right_shift);

lanelet::ConstLanelets getTargetLanelets(
  const std::shared_ptr<const PlannerData> & planner_data, lanelet::ConstLanelets & route_lanelets,
  const double left_offset, const double right_offset);

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data);

lanelet::ConstLanelets getExtendLanes(
  const lanelet::ConstLanelets & lanelets, const Pose & ego_pose,
  const std::shared_ptr<const PlannerData> & planner_data);

void insertDecelPoint(
  const Point & p_src, const double offset, const double velocity, PathWithLaneId & path,
  std::optional<Pose> & p_out);

void fillObjectEnvelopePolygon(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const Pose & closest_pose,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void fillObjectMovingTime(
  ObjectData & object_data, ObjectDataArray & stopped_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void fillAvoidanceNecessity(
  ObjectData & object_data, const ObjectDataArray & registered_objects, const double vehicle_width,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void fillObjectStoppableJudge(
  ObjectData & object_data, const ObjectDataArray & registered_objects,
  const double feasible_stop_distance, const std::shared_ptr<AvoidanceParameters> & parameters);

void fillInitialPose(ObjectData & object_data, ObjectDataArray & detected_objects);

void updateRegisteredObject(
  ObjectDataArray & registered_objects, const ObjectDataArray & now_objects,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void compensateDetectionLost(
  const ObjectDataArray & registered_objects, ObjectDataArray & now_objects,
  ObjectDataArray & other_objects);

void filterTargetObjects(
  ObjectDataArray & objects, AvoidancePlanningData & data, DebugData & debug,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters);

double getRoadShoulderDistance(
  ObjectData & object, const AvoidancePlanningData & data,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters);

void fillAdditionalInfoFromPoint(const AvoidancePlanningData & data, AvoidLineArray & lines);

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLine & line);

void fillAdditionalInfoFromLongitudinal(
  const AvoidancePlanningData & data, AvoidOutlines & outlines);

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLineArray & lines);

AvoidLine fillAdditionalInfo(const AvoidancePlanningData & data, const AvoidLine & line);

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines);

std::vector<ExtendedPredictedObject> getSafetyCheckTargetObjects(
  const AvoidancePlanningData & data, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters, const bool is_right_shift,
  DebugData & debug);

std::pair<PredictedObjects, PredictedObjects> separateObjectsByPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data,
  const AvoidancePlanningData & data, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double object_check_forward_distance, const bool is_running, DebugData & debug);

DrivableLanes generateExpandDrivableLanes(
  const lanelet::ConstLanelet & lanelet, const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters);

double calcDistanceToReturnDeadLine(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters);

double calcDistanceToAvoidStartLine(
  const lanelet::ConstLanelets & lanelets, const PathWithLaneId & path,
  const std::shared_ptr<const PlannerData> & planner_data,
  const std::shared_ptr<AvoidanceParameters> & parameters);

TurnSignalInfo calcTurnSignalInfo(
  const ShiftedPath & path, const ShiftLine & shift_line, const double current_shift_length,
  const AvoidancePlanningData & data, const std::shared_ptr<const PlannerData> & planner_data);
}  // namespace behavior_path_planner::utils::avoidance

#endif  // BEHAVIOR_PATH_AVOIDANCE_MODULE__UTILS_HPP_
