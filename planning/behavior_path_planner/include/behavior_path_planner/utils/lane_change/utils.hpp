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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__UTILS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__UTILS_HPP_

#include "behavior_path_planner/marker_util/lane_change/debug.hpp"
#include "behavior_path_planner/parameters.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_path.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <route_handler/route_handler.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <lanelet2_core/primitives/Primitive.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace behavior_path_planner::utils::lane_change
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::PredictedPath;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using marker_utils::CollisionCheckDebug;
using route_handler::Direction;
using tier4_autoware_utils::Polygon2d;

double calcLaneChangeResampleInterval(
  const double lane_changing_length, const double lane_changing_velocity);

std::vector<int64_t> replaceWithSortedIds(
  const std::vector<int64_t> & original_lane_ids,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids);

std::vector<std::vector<int64_t>> getSortedLaneIds(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const double rough_shift_length);

PathWithLaneId combineReferencePath(const PathWithLaneId & path1, const PathWithLaneId & path2);

bool isPathInLanelets(
  const PathWithLaneId & path, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets);

double calcLaneChangingLength(
  const double lane_changing_velocity, const double shift_length,
  const BehaviorPathPlannerParameters & common_parameter);

std::optional<LaneChangePath> constructCandidatePath(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_segment,
  const PathWithLaneId & target_lane_reference_path, const ShiftLine & shift_line,
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const std::vector<std::vector<int64_t>> & sorted_lane_ids, const double acceleration,
  const LaneChangePhaseInfo lane_change_length, const LaneChangePhaseInfo lane_change_velocity,
  const BehaviorPathPlannerParameters & common_parameter,
  const LaneChangeParameters & lane_change_param);

bool getLaneChangePaths(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const lanelet::ConstLanelets & target_lanelets, const Pose & pose, const Twist & twist,
  const PredictedObjects::ConstSharedPtr dynamic_objects,
  const BehaviorPathPlannerParameters & common_parameter, const LaneChangeParameters & parameter,
  const double check_length, LaneChangePaths * candidate_paths,
  std::unordered_map<std::string, CollisionCheckDebug> * debug_data);

bool getLaneChangePaths(
  const PathWithLaneId & original_path, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & original_lanelets, const lanelet::ConstLanelets & target_lanelets,
  const Pose & pose, const Twist & twist, const PredictedObjects::ConstSharedPtr dynamic_objects,
  const BehaviorPathPlannerParameters & common_parameter, const LaneChangeParameters & parameter,
  const double check_length, const Direction direction, LaneChangePaths * candidate_paths,
  std::unordered_map<std::string, CollisionCheckDebug> * debug_data);

bool isLaneChangePathSafe(
  const LaneChangePath & lane_change_path, const PredictedObjects::ConstSharedPtr dynamic_objects,
  const LaneChangeTargetObjectIndices & dynamic_object_indices, const Pose & current_pose,
  const Twist & current_twist, const BehaviorPathPlannerParameters & common_parameter,
  const behavior_path_planner::LaneChangeParameters & lane_change_parameter,
  const double front_decel, const double rear_decel, Pose & ego_pose_before_collision,
  std::unordered_map<std::string, CollisionCheckDebug> & debug_data,
  const double acceleration = 0.0);

bool hasEnoughLength(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Pose & current_pose,
  const RouteHandler & route_handler, const double minimum_lane_changing_velocity,
  const BehaviorPathPlannerParameters & common_parameters,
  const Direction direction = Direction::NONE);

ShiftLine getLaneChangingShiftLine(
  const PathWithLaneId & prepare_segment, const PathWithLaneId & target_segment,
  const PathWithLaneId & reference_path, const double shift_length);

PathWithLaneId getReferencePathFromTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanes,
  const Pose & lane_changing_start_pose, const double target_lane_length,
  const double lane_changing_length, const double forward_path_length,
  const double resample_interval, const bool is_goal_in_route);

PathWithLaneId getPrepareSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & original_lanelets,
  const double arc_length_from_current, const double backward_path_length,
  const double prepare_length, const double prepare_velocity);

PathWithLaneId getPrepareSegment(
  const PathWithLaneId & original_path, const lanelet::ConstLanelets & original_lanelets,
  const Pose & current_pose, const double backward_path_length, const double prepare_length,
  const double prepare_velocity);

PathWithLaneId getTargetSegment(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & target_lanelets,
  const double forward_path_length, const Pose & lane_changing_start_pose,
  const double target_lane_length, const double lane_changing_length,
  const double lane_changing_velocity, const double total_required_min_dist);

bool isEgoWithinOriginalLane(
  const lanelet::ConstLanelets & current_lanes, const Pose & current_pose,
  const BehaviorPathPlannerParameters & common_param);

void get_turn_signal_info(
  const LaneChangePath & lane_change_path, TurnSignalInfo * turn_signal_info);

std::vector<DrivableLanes> generateDrivableLanes(
  const std::vector<DrivableLanes> original_drivable_lanes, const RouteHandler & route_handler,
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & lane_change_lanes);

std::vector<DrivableLanes> generateDrivableLanes(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & lane_change_lanes);

std::optional<LaneChangePath> getAbortPaths(
  const std::shared_ptr<const PlannerData> & planner_data, const LaneChangePath & selected_path,
  const Pose & ego_lerp_pose_before_collision, const BehaviorPathPlannerParameters & common_param,
  const LaneChangeParameters & lane_change_param);

double getLateralShift(const LaneChangePath & path);

bool hasEnoughLengthToLaneChangeAfterAbort(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const Pose & curent_pose, const double abort_return_dist,
  const BehaviorPathPlannerParameters & common_param);

lanelet::ConstLanelets getExtendedTargetLanesForCollisionCheck(
  const RouteHandler & route_handler, const lanelet::ConstLanelet & target_lanes,
  const Pose & current_pose, const double backward_length);

LaneChangeTargetObjectIndices filterObjectIndices(
  const LaneChangePaths & lane_change_paths, const PredictedObjects & objects,
  const lanelet::ConstLanelets & target_backward_lanes, const Pose & current_pose,
  const double forward_path_length, const LaneChangeParameters & lane_change_parameter,
  const double filter_width);

bool isTargetObjectType(const PredictedObject & object, const LaneChangeParameters & parameter);

double calcLateralBufferForFiltering(const double vehicle_width, const double lateral_buffer = 0.0);

double calcLateralBufferForFiltering(const double vehicle_width, const double lateral_buffer);

std::string getStrDirection(const std::string & name, const Direction direction);

CandidateOutput assignToCandidate(
  const LaneChangePath & lane_change_path, const Point & ego_position);
boost::optional<lanelet::ConstLanelet> getLaneChangeTargetLane(
  const RouteHandler & route_handler, const lanelet::ConstLanelets & current_lanes,
  const LaneChangeModuleType type, const Direction & direction);
}  // namespace behavior_path_planner::utils::lane_change
#endif  // BEHAVIOR_PATH_PLANNER__UTILS__LANE_CHANGE__UTILS_HPP_
