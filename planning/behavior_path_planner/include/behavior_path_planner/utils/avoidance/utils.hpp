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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__UTILS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__UTILS_HPP_

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"

#include <memory>
#include <string>
#include <vector>

namespace behavior_path_planner::utils::avoidance
{
using behavior_path_planner::PlannerData;

bool isOnRight(const ObjectData & obj);

bool isVehicleTypeObject(const ObjectData & object);

bool isWithinCrosswalk(
  const ObjectData & object,
  const std::shared_ptr<const lanelet::routing::RoutingGraphContainer> & overall_graphs);

bool isTargetObjectType(
  const PredictedObject & object, const std::shared_ptr<AvoidanceParameters> & parameters);

double calcShiftLength(
  const bool & is_object_on_right, const double & overhang_dist, const double & avoid_margin);

bool isShiftNecessary(const bool & is_object_on_right, const double & shift_length);

bool isSameDirectionShift(const bool & is_object_on_right, const double & shift_length);

size_t findPathIndexFromArclength(
  const std::vector<double> & path_arclength_arr, const double target_arc);

ShiftedPath toShiftedPath(const PathWithLaneId & path);

ShiftLineArray toShiftLineArray(const AvoidLineArray & avoid_points);

std::vector<size_t> concatParentIds(
  const std::vector<size_t> & ids1, const std::vector<size_t> & ids2);

std::vector<size_t> calcParentIds(const AvoidLineArray & lines1, const AvoidLine & lines2);

double lerpShiftLengthOnArc(double arc, const AvoidLine & al);

void fillLongitudinalAndLengthByClosestEnvelopeFootprint(
  const PathWithLaneId & path, const Point & ego_pos, ObjectData & obj);

double calcEnvelopeOverhangDistance(
  const ObjectData & object_data, const Pose & base_pose, Point & overhang_pose);

void setEndData(
  AvoidLine & al, const double length, const geometry_msgs::msg::Pose & end, const size_t end_idx,
  const double end_dist);

void setStartData(
  AvoidLine & al, const double start_shift_length, const geometry_msgs::msg::Pose & start,
  const size_t start_idx, const double start_dist);

Polygon2d createEnvelopePolygon(
  const ObjectData & object_data, const Pose & closest_pose, const double envelope_buffer);

std::vector<DrivableAreaInfo::Obstacle> generateObstaclePolygonsForDrivableArea(
  const ObjectDataArray & objects, const std::shared_ptr<AvoidanceParameters> & parameters,
  const double vehicle_width);

double getLongitudinalVelocity(const Pose & p_ref, const Pose & p_target, const double v);

bool isCentroidWithinLanelets(
  const PredictedObject & object, const lanelet::ConstLanelets & target_lanelets);

lanelet::ConstLanelets getTargetLanelets(
  const std::shared_ptr<const PlannerData> & planner_data, lanelet::ConstLanelets & route_lanelets,
  const double left_offset, const double right_offset);

lanelet::ConstLanelets getCurrentLanesFromPath(
  const PathWithLaneId & path, const std::shared_ptr<const PlannerData> & planner_data);

void insertDecelPoint(
  const Point & p_src, const double offset, const double velocity, PathWithLaneId & path,
  boost::optional<Pose> & p_out);

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

double extendToRoadShoulderDistanceWithPolygon(
  const std::shared_ptr<route_handler::RouteHandler> & rh,
  const lanelet::ConstLineString3d & target_line, const double to_road_shoulder_distance,
  const lanelet::ConstLanelet & overhang_lanelet, const geometry_msgs::msg::Point & overhang_pos,
  const lanelet::BasicPoint3d & overhang_basic_pose, const bool use_hatched_road_markings,
  const bool use_intersection_areas);

void fillAdditionalInfoFromPoint(const AvoidancePlanningData & data, AvoidLineArray & lines);

void fillAdditionalInfoFromLongitudinal(const AvoidancePlanningData & data, AvoidLineArray & lines);

AvoidLine fillAdditionalInfo(const AvoidancePlanningData & data, const AvoidLine & line);

AvoidLineArray combineRawShiftLinesWithUniqueCheck(
  const AvoidLineArray & base_lines, const AvoidLineArray & added_lines);
}  // namespace behavior_path_planner::utils::avoidance

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__UTILS_HPP_
