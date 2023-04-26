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

#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"

#include "behavior_path_planner/utils/create_vehicle_footprint.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/pull_out/util.hpp"
#include "behavior_path_planner/utils/utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using motion_utils::calcLongitudinalOffsetPose;
using tier4_autoware_utils::calcOffsetPose;

namespace behavior_path_planner
{
#ifdef USE_OLD_ARCHITECTURE
PullOutModule::PullOutModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<PullOutParameters> & parameters)
: SceneModuleInterface{name, node, createRTCInterfaceMap(node, name, {""})},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  lane_departure_checker_ = std::make_shared<LaneDepartureChecker>();
  lane_departure_checker_->setVehicleInfo(vehicle_info_);

  // set enabled planner
  if (parameters_->enable_shift_pull_out) {
    pull_out_planners_.push_back(
      std::make_shared<ShiftPullOut>(node, *parameters, lane_departure_checker_));
  }
  if (parameters_->enable_geometric_pull_out) {
    pull_out_planners_.push_back(
      std::make_shared<GeometricPullOut>(node, *parameters, getGeometricPullOutParameters()));
  }
  if (pull_out_planners_.empty()) {
    RCLCPP_ERROR(getLogger(), "Not found enabled planner");
  }
}
#else
PullOutModule::PullOutModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<PullOutParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  lane_departure_checker_ = std::make_shared<LaneDepartureChecker>();
  lane_departure_checker_->setVehicleInfo(vehicle_info_);

  // set enabled planner
  if (parameters_->enable_shift_pull_out) {
    pull_out_planners_.push_back(
      std::make_shared<ShiftPullOut>(node, *parameters, lane_departure_checker_));
  }
  if (parameters_->enable_geometric_pull_out) {
    pull_out_planners_.push_back(
      std::make_shared<GeometricPullOut>(node, *parameters, getGeometricPullOutParameters()));
  }
  if (pull_out_planners_.empty()) {
    RCLCPP_ERROR(getLogger(), "Not found enabled planner");
  }
}
#endif

BehaviorModuleOutput PullOutModule::run()
{
  current_state_ = ModuleStatus::RUNNING;

#ifndef USE_OLD_ARCHITECTURE
  if (!isActivated()) {
    return planWaitingApproval();
  }
#endif

  return plan();
}

void PullOutModule::processOnExit()
{
  resetPathCandidate();
  resetPathReference();
}

bool PullOutModule::isExecutionRequested() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  const bool is_stopped = utils::l2Norm(planner_data_->self_odometry->twist.twist.linear) <
                          parameters_->th_arrived_distance;
  if (!is_stopped) {
    return false;
  }

  // Create vehicle footprint
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info_);
  const auto vehicle_footprint = transformVector(
    local_vehicle_footprint,
    tier4_autoware_utils::pose2transform(planner_data_->self_odometry->pose.pose));

  // Check if ego is not out of lanes
  const auto current_lanes = utils::getExtendedCurrentLanes(planner_data_);
  const auto pull_out_lanes = pull_out_utils::getPullOutLanes(planner_data_);
  auto lanes = current_lanes;
  lanes.insert(lanes.end(), pull_out_lanes.begin(), pull_out_lanes.end());
  if (LaneDepartureChecker::isOutOfLane(lanes, vehicle_footprint)) {
    return false;
  }

  // Check if any of the footprint points are in the shoulder lane
  lanelet::Lanelet closest_shoulder_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        pull_out_lanes, planner_data_->self_odometry->pose.pose, &closest_shoulder_lanelet)) {
    return false;
  }
  if (!isOverlappedWithLane(closest_shoulder_lanelet, vehicle_footprint)) {
    return false;
  }

  return true;
}

bool PullOutModule::isExecutionReady() const
{
  return true;
}

// this runs only when RUNNING
ModuleStatus PullOutModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OUT updateState");

  if (hasFinishedPullOut()) {
    current_state_ = ModuleStatus::SUCCESS;
    return current_state_;
  }

  checkBackFinished();

  return current_state_;
}

BehaviorModuleOutput PullOutModule::plan()
{
  if (isWaitingApproval()) {
    clearWaitingApproval();
    resetPathCandidate();
    resetPathReference();
    // save current_pose when approved for start_point of turn_signal for backward driving
    last_approved_pose_ = std::make_unique<Pose>(planner_data_->self_odometry->pose.pose);
  }

  BehaviorModuleOutput output;
  if (!status_.is_safe) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull out path, publish stop path");
    // the path of getCurrent() is generated by generateStopPath()
    const PathWithLaneId stop_path = getCurrentPath();
    output.path = std::make_shared<PathWithLaneId>(stop_path);
    output.drivable_area_info.drivable_lanes = status_.lanes;
    output.reference_path = getPreviousModuleOutput().reference_path;
    path_candidate_ = std::make_shared<PathWithLaneId>(stop_path);
    path_reference_ = getPreviousModuleOutput().reference_path;
    return output;
  }

  PathWithLaneId path;
  if (status_.back_finished) {
    if (hasFinishedCurrentPath()) {
      RCLCPP_INFO(getLogger(), "Increment path index");
      incrementPathIndex();
    }
    path = getCurrentPath();
  } else {
    path = status_.backward_path;
  }

  const auto target_drivable_lanes = getNonOverlappingExpandedLanes(path, status_.lanes);
  utils::generateDrivableArea(
    path, target_drivable_lanes, planner_data_->parameters.vehicle_length, planner_data_);
  output.drivable_area_info.drivable_lanes = utils::combineDrivableLanes(
    getPreviousModuleOutput().drivable_area_info.drivable_lanes, target_drivable_lanes);

  output.path = std::make_shared<PathWithLaneId>(path);
  output.drivable_area_info.drivable_lanes = status_.lanes;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = getPreviousModuleOutput().reference_path;

  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return SteeringFactor::LEFT;
    } else if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::STRAIGHT;
  });

  if (status_.back_finished) {
    const double start_distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    const double finish_distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.end_pose.position);
    updateRTCStatus(start_distance, finish_distance);
    // TODO(tkhmy) add handle status TRYING
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose},
      {start_distance, finish_distance}, SteeringFactor::PULL_OUT, steering_factor_direction,
      SteeringFactor::TURNING, "");
  } else {
    const double distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    updateRTCStatus(0.0, distance);
    // TODO(tkhmy) add handle status TRYING
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
      SteeringFactor::PULL_OUT, steering_factor_direction, SteeringFactor::TURNING, "");
  }

  setDebugData();

  return output;
}

CandidateOutput PullOutModule::planCandidate() const
{
  return CandidateOutput{};
}

std::shared_ptr<PullOutPlannerBase> PullOutModule::getCurrentPlanner() const
{
  for (const auto & planner : pull_out_planners_) {
    if (status_.planner_type == planner->getPlannerType()) {
      return planner;
    }
  }
  return nullptr;
}

PathWithLaneId PullOutModule::getFullPath() const
{
  const auto pull_out_planner = getCurrentPlanner();
  if (pull_out_planner == nullptr) {
    return PathWithLaneId{};
  }

  // combine partial pull out path
  PathWithLaneId pull_out_path;
  for (const auto & partial_path : status_.pull_out_path.partial_paths) {
    pull_out_path.points.insert(
      pull_out_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (status_.back_finished) {
    // not need backward path or finish it
    return pull_out_path;
  }

  // concat back_path and pull_out_path and
  auto full_path = status_.backward_path;
  full_path.points.insert(
    full_path.points.end(), pull_out_path.points.begin(), pull_out_path.points.end());
  return full_path;
}

BehaviorModuleOutput PullOutModule::planWaitingApproval()
{
  updatePullOutStatus();
  waitApproval();

  BehaviorModuleOutput output;
  if (!status_.is_safe) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull out path, publish stop path");
    // the path of getCurrent() is generated by generateStopPath()
    const PathWithLaneId stop_path = getCurrentPath();
    output.path = std::make_shared<PathWithLaneId>(stop_path);
    output.reference_path = getPreviousModuleOutput().reference_path;
    path_candidate_ = std::make_shared<PathWithLaneId>(stop_path);
    path_reference_ = getPreviousModuleOutput().reference_path;
    return output;
  }

  const auto current_lanes = utils::getExtendedCurrentLanes(planner_data_);
  const auto pull_out_lanes = pull_out_utils::getPullOutLanes(planner_data_);
  auto stop_path = status_.back_finished ? getCurrentPath() : status_.backward_path;
  const auto drivable_lanes =
    utils::generateDrivableLanesWithShoulderLanes(current_lanes, pull_out_lanes);
  const auto & dp = planner_data_->drivable_area_expansion_parameters;
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);
  utils::generateDrivableArea(
    stop_path, expanded_lanes, planner_data_->parameters.vehicle_length, planner_data_);
  for (auto & p : stop_path.points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }

  output.drivable_area_info.drivable_lanes = utils::combineDrivableLanes(
    getPreviousModuleOutput().drivable_area_info.drivable_lanes, expanded_lanes);

  output.path = std::make_shared<PathWithLaneId>(stop_path);
  output.drivable_area_info.drivable_lanes = status_.lanes;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = getPreviousModuleOutput().reference_path;

  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return SteeringFactor::LEFT;
    } else if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::STRAIGHT;
  });

  if (status_.back_finished) {
    const double start_distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    const double finish_distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.end_pose.position);
    updateRTCStatus(start_distance, finish_distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose},
      {start_distance, finish_distance}, SteeringFactor::PULL_OUT, steering_factor_direction,
      SteeringFactor::APPROACHING, "");
  } else {
    const double distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    updateRTCStatus(0.0, distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
      SteeringFactor::PULL_OUT, steering_factor_direction, SteeringFactor::APPROACHING, "");
  }

  setDebugData();

  return output;
}

void PullOutModule::resetStatus()
{
  PullOutStatus initial_status;
  status_ = initial_status;
}

ParallelParkingParameters PullOutModule::getGeometricPullOutParameters() const
{
  ParallelParkingParameters params{};

  params.th_arrived_distance = parameters_->th_arrived_distance;
  params.th_stopped_velocity = parameters_->th_stopped_velocity;
  params.arc_path_interval = parameters_->arc_path_interval;
  params.departing_velocity = parameters_->geometric_pull_out_velocity;
  params.departing_lane_departure_margin = parameters_->lane_departure_margin;
  params.max_steer_angle = parameters_->pull_out_max_steer_angle;

  return params;
}

void PullOutModule::incrementPathIndex()
{
  status_.current_path_idx =
    std::min(status_.current_path_idx + 1, status_.pull_out_path.partial_paths.size() - 1);
}

PathWithLaneId PullOutModule::getCurrentPath() const
{
  return status_.pull_out_path.partial_paths.at(status_.current_path_idx);
}

void PullOutModule::planWithPriorityOnEfficientPath(
  const std::vector<Pose> & start_pose_candidates, const Pose & goal_pose)
{
  status_.is_safe = false;
  status_.planner_type = PlannerType::NONE;

  // check if start pose candidates are valid
  if (start_pose_candidates.empty()) {
    return;
  }

  // plan with each planner
  for (const auto & planner : pull_out_planners_) {
    for (size_t i = 0; i < start_pose_candidates.size(); i++) {
      status_.back_finished = i == 0;
      const auto & pull_out_start_pose = start_pose_candidates.at(i);
      planner->setPlannerData(planner_data_);
      const auto pull_out_path = planner->plan(pull_out_start_pose, goal_pose);
      // not found safe path
      if (!pull_out_path) {
        continue;
      }
      // use current path if back is not needed
      if (status_.back_finished) {
        status_.is_safe = true;
        status_.pull_out_path = *pull_out_path;
        status_.pull_out_start_pose = pull_out_start_pose;
        status_.planner_type = planner->getPlannerType();
        break;
      }

      if (i == start_pose_candidates.size() - 1) continue;

      //  check next path if back is needed
      const auto & pull_out_start_pose_next = start_pose_candidates.at(i + 1);
      const auto pull_out_path_next = planner->plan(pull_out_start_pose_next, goal_pose);
      // not found safe path
      if (!pull_out_path_next) {
        continue;
      }
      status_.is_safe = true;
      status_.pull_out_path = *pull_out_path_next;
      status_.pull_out_start_pose = pull_out_start_pose_next;
      status_.planner_type = planner->getPlannerType();
      break;
    }
    if (status_.is_safe) {
      break;
    }
  }
}

// todo: common processing with planWithPriorityOnEfficientPath
void PullOutModule::planWithPriorityOnShortBackDistance(
  const std::vector<Pose> & start_pose_candidates, const Pose & goal_pose)
{
  status_.is_safe = false;
  status_.planner_type = PlannerType::NONE;

  // check if start pose candidates are valid
  if (start_pose_candidates.empty()) {
    return;
  }

  for (size_t i = 0; i < start_pose_candidates.size(); i++) {
    status_.back_finished = i == 0;
    const auto & pull_out_start_pose = start_pose_candidates.at(i);
    // plan with each planner
    for (const auto & planner : pull_out_planners_) {
      planner->setPlannerData(planner_data_);
      const auto pull_out_path = planner->plan(pull_out_start_pose, goal_pose);
      // not found safe path
      if (!pull_out_path) {
        continue;
      }
      // use current path if back is not needed
      if (status_.back_finished) {
        status_.is_safe = true;
        status_.pull_out_path = *pull_out_path;
        status_.pull_out_start_pose = pull_out_start_pose;
        status_.planner_type = planner->getPlannerType();
        break;
      }

      if (i == start_pose_candidates.size() - 1) continue;

      //  check next path if back is needed
      const auto & pull_out_start_pose_next = start_pose_candidates.at(i + 1);
      const auto pull_out_path_next = planner->plan(pull_out_start_pose_next, goal_pose);
      // not found safe path
      if (!pull_out_path_next) {
        continue;
      }
      status_.is_safe = true;
      status_.pull_out_path = *pull_out_path_next;
      status_.pull_out_start_pose = pull_out_start_pose_next;
      status_.planner_type = planner->getPlannerType();
      break;
    }
    if (status_.is_safe) {
      break;
    }
  }
}

PathWithLaneId PullOutModule::generateStopPath() const
{
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  constexpr double dummy_path_distance = 1.0;
  const auto moved_pose = calcOffsetPose(current_pose, dummy_path_distance, 0, 0);

  // convert Pose to PathPointWithLaneId with 0 velocity.
  auto toPathPointWithLaneId = [this](const Pose & pose) {
    PathPointWithLaneId p{};
    p.point.pose = pose;
    p.point.longitudinal_velocity_mps = 0.0;
    lanelet::Lanelet closest_shoulder_lanelet;
    lanelet::utils::query::getClosestLanelet(
      status_.pull_out_lanes, pose, &closest_shoulder_lanelet);
    p.lane_ids.push_back(closest_shoulder_lanelet.id());
    return p;
  };

  PathWithLaneId path{};
  path.points.push_back(toPathPointWithLaneId(current_pose));
  path.points.push_back(toPathPointWithLaneId(moved_pose));

  // generate drivable area
  const auto target_drivable_lanes = getNonOverlappingExpandedLanes(path, status_.lanes);

  // for old architecture
  utils::generateDrivableArea(
    path, target_drivable_lanes, planner_data_->parameters.vehicle_length, planner_data_);

  return path;
}

void PullOutModule::updatePullOutStatus()
{
  // if new route is received, reset status
  const bool has_received_new_route =
    last_route_received_time_ == nullptr ||
    *last_route_received_time_ != planner_data_->route_handler->getRouteHeader().stamp;
  if (has_received_new_route) {
    RCLCPP_INFO(getLogger(), "Receive new route, so reset status");
    resetStatus();
  }
  last_route_received_time_ =
    std::make_unique<rclcpp::Time>(planner_data_->route_handler->getRouteHeader().stamp);

  // skip updating if enough time has not passed for preventing chattering between back and pull_out
  if (!has_received_new_route && !status_.back_finished) {
    if (last_pull_out_start_update_time_ == nullptr) {
      last_pull_out_start_update_time_ = std::make_unique<rclcpp::Time>(clock_->now());
    }
    const auto elapsed_time = (clock_->now() - *last_pull_out_start_update_time_).seconds();
    if (elapsed_time < parameters_->backward_path_update_duration) {
      return;
    }
    last_pull_out_start_update_time_ = std::make_unique<rclcpp::Time>(clock_->now());
  }

  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & goal_pose = planner_data_->route_handler->getGoalPose();

  status_.current_lanes = utils::getExtendedCurrentLanes(planner_data_);
  status_.pull_out_lanes = pull_out_utils::getPullOutLanes(planner_data_);

  // combine road and shoulder lanes
  status_.lanes =
    utils::generateDrivableLanesWithShoulderLanes(status_.current_lanes, status_.pull_out_lanes);

  // search pull out start candidates backward
  std::vector<Pose> start_pose_candidates = searchPullOutStartPoses();

  if (parameters_->search_priority == "efficient_path") {
    planWithPriorityOnEfficientPath(start_pose_candidates, goal_pose);
  } else if (parameters_->search_priority == "short_back_distance") {
    planWithPriorityOnShortBackDistance(start_pose_candidates, goal_pose);
  } else {
    RCLCPP_ERROR(
      getLogger(),
      "search_priority should be efficient_path or short_back_distance, but %s is given.",
      parameters_->search_priority.c_str());
    throw std::domain_error("[pull_out] invalid search_priority");
  }

  if (!status_.is_safe) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull out path, generate stop path");
    status_.back_finished = true;  // no need to drive backward
    status_.pull_out_path.partial_paths.clear();
    status_.pull_out_path.partial_paths.push_back(generateStopPath());
    status_.pull_out_path.start_pose = current_pose;
    status_.pull_out_path.end_pose = current_pose;
  }

  checkBackFinished();
  if (!status_.back_finished) {
    status_.backward_path = pull_out_utils::getBackwardPath(
      *route_handler, status_.pull_out_lanes, current_pose, status_.pull_out_start_pose,
      parameters_->backward_velocity);
  }

  // Update status
  status_.lane_follow_lane_ids = utils::getIds(status_.current_lanes);
  status_.pull_out_lane_ids = utils::getIds(status_.pull_out_lanes);
}

// make this class?
std::vector<Pose> PullOutModule::searchPullOutStartPoses()
{
  std::vector<Pose> pull_out_start_pose{};

  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  // get backward shoulder path
  const auto arc_position_pose =
    lanelet::utils::getArcCoordinates(status_.pull_out_lanes, current_pose);
  const double check_distance = parameters_->max_back_distance + 30.0;  // buffer
  auto backward_shoulder_path = planner_data_->route_handler->getCenterLinePath(
    status_.pull_out_lanes, arc_position_pose.length - check_distance,
    arc_position_pose.length + check_distance);

  // lateral shift to current_pose
  const double distance_from_center_line = arc_position_pose.distance;
  for (auto & p : backward_shoulder_path.points) {
    p.point.pose = calcOffsetPose(p.point.pose, 0, distance_from_center_line, 0);
  }

  // if backward driving is disable, just refine current pose to the lanes
  if (!parameters_->enable_back) {
    const auto refined_pose =
      calcLongitudinalOffsetPose(backward_shoulder_path.points, current_pose.position, 0);
    if (refined_pose) {
      pull_out_start_pose.push_back(*refined_pose);
    }
    return pull_out_start_pose;
  }

  // check collision between footprint and object at the backed pose
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info_);
  for (double back_distance = 0.0; back_distance <= parameters_->max_back_distance;
       back_distance += parameters_->backward_search_resolution) {
    const auto backed_pose = calcLongitudinalOffsetPose(
      backward_shoulder_path.points, current_pose.position, -back_distance);
    if (!backed_pose) {
      continue;
    }

    // check the back pose is near the lane end
    const double length_to_backed_pose =
      lanelet::utils::getArcCoordinates(status_.pull_out_lanes, *backed_pose).length;
    double length_to_lane_end = 0.0;
    for (const auto & lane : status_.pull_out_lanes) {
      length_to_lane_end += lanelet::utils::getLaneletLength2d(lane);
    }
    const double distance_from_lane_end = length_to_lane_end - length_to_backed_pose;
    if (distance_from_lane_end < parameters_->ignore_distance_from_lane_end) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000,
        "the ego is too close to the lane end, so needs backward driving");
      continue;
    }

    if (utils::checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, *backed_pose, *(planner_data_->dynamic_object),
          parameters_->collision_check_margin)) {
      break;  // poses behind this has a collision, so break.
    }

    pull_out_start_pose.push_back(*backed_pose);
  }
  return pull_out_start_pose;
}

bool PullOutModule::isOverlappedWithLane(
  const lanelet::ConstLanelet & candidate_lanelet,
  const tier4_autoware_utils::LinearRing2d & vehicle_footprint)
{
  for (const auto & point : vehicle_footprint) {
    if (boost::geometry::within(point, candidate_lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

bool PullOutModule::hasFinishedPullOut() const
{
  if (!status_.back_finished) {
    return false;
  }

  const auto current_pose = planner_data_->self_odometry->pose.pose;

  // keep running until returning to the path, considering that other modules (e.g avoidance)
  // are also running at the same time.
  const double lateral_offset_to_path =
    motion_utils::calcLateralOffset(getCurrentPath().points, current_pose.position);
  constexpr double lateral_offset_threshold = 0.5;
  if (std::abs(lateral_offset_to_path) > lateral_offset_threshold) {
    return false;
  }
  const double yaw_deviation =
    motion_utils::calcYawDeviation(getCurrentPath().points, current_pose);
  constexpr double yaw_deviation_threshold = 0.5;
  if (std::abs(yaw_deviation) > yaw_deviation_threshold) {
    return false;
  }

  // check that ego has passed pull out end point
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose);
  const auto arclength_pull_out_end =
    lanelet::utils::getArcCoordinates(status_.current_lanes, status_.pull_out_path.end_pose);
  return arclength_current.length - arclength_pull_out_end.length > 0.0;
}

void PullOutModule::checkBackFinished()
{
  // check ego car is close enough to pull out start pose
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const auto distance =
    tier4_autoware_utils::calcDistance2d(current_pose, status_.pull_out_start_pose);

  const bool is_near = distance < parameters_->th_arrived_distance;
  const double ego_vel = utils::l2Norm(planner_data_->self_odometry->twist.twist.linear);
  const bool is_stopped = ego_vel < parameters_->th_stopped_velocity;

  if (!status_.back_finished && is_near && is_stopped) {
    RCLCPP_INFO(getLogger(), "back finished");
    status_.back_finished = true;

    // request pull_out approval
    waitApproval();
    removeRTCStatus();
    for (auto itr = uuid_map_.begin(); itr != uuid_map_.end(); ++itr) {
      itr->second = generateUUID();
    }
    current_state_ = ModuleStatus::SUCCESS;  // for breaking loop
  }
}

bool PullOutModule::isStopped()
{
  odometry_buffer_.push_back(planner_data_->self_odometry);
  // Delete old data in buffer
  while (rclcpp::ok()) {
    const auto time_diff = rclcpp::Time(odometry_buffer_.back()->header.stamp) -
                           rclcpp::Time(odometry_buffer_.front()->header.stamp);
    if (time_diff.seconds() < parameters_->th_stopped_time) {
      break;
    }
    odometry_buffer_.pop_front();
  }
  bool is_stopped = true;
  for (const auto & odometry : odometry_buffer_) {
    const double ego_vel = utils::l2Norm(odometry->twist.twist.linear);
    if (ego_vel > parameters_->th_stopped_velocity) {
      is_stopped = false;
      break;
    }
  }
  return is_stopped;
}

bool PullOutModule::hasFinishedCurrentPath()
{
  const auto current_path = getCurrentPath();
  const auto current_path_end = current_path.points.back();
  const auto self_pose = planner_data_->self_odometry->pose.pose;
  const bool is_near_target = tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
                              parameters_->th_arrived_distance;

  return is_near_target && isStopped();
}

TurnSignalInfo PullOutModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal{};  // output
  const auto & current_pose = planner_data_->self_odometry->pose.pose;

  // turn on hazard light when backward driving
  if (!status_.back_finished) {
    turn_signal.hazard_signal.command = HazardLightsCommand::ENABLE;
    const auto back_start_pose = isWaitingApproval() ? current_pose : *last_approved_pose_;
    turn_signal.desired_start_point = back_start_pose;
    turn_signal.required_start_point = back_start_pose;
    // pull_out start_pose is same to backward driving end_pose
    turn_signal.required_end_point = status_.pull_out_path.start_pose;
    turn_signal.desired_end_point = status_.pull_out_path.start_pose;
    return turn_signal;
  }

  // turn on right signal until passing pull_out end point
  const auto path = getFullPath();
  // pull out path does not overlap
  const double distance_from_end = motion_utils::calcSignedArcLength(
    path.points, status_.pull_out_path.end_pose.position, current_pose.position);
  if (distance_from_end < 0.0) {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  } else {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::DISABLE;
  }

  turn_signal.desired_start_point = status_.pull_out_path.start_pose;
  turn_signal.required_start_point = status_.pull_out_path.start_pose;
  turn_signal.required_end_point = status_.pull_out_path.end_pose;
  turn_signal.desired_end_point = status_.pull_out_path.end_pose;

  return turn_signal;
}

void PullOutModule::setDebugData() const
{
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;

  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  debug_marker_.markers.clear();
  add(createPoseMarkerArray(status_.pull_out_start_pose, "back_end_pose", 0, 0.9, 0.3, 0.3));
  add(createPoseMarkerArray(status_.pull_out_path.start_pose, "start_pose", 0, 0.3, 0.9, 0.3));
  add(createPoseMarkerArray(status_.pull_out_path.end_pose, "end_pose", 0, 0.9, 0.9, 0.3));
  add(createPathMarkerArray(getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
}
}  // namespace behavior_path_planner
