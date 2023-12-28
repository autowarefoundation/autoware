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

#include "behavior_path_start_planner_module/start_planner_module.hpp"

#include "behavior_path_planner_common/utils/create_vehicle_footprint.hpp"
#include "behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_start_planner_module/util.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using behavior_path_planner::utils::parking_departure::initializeCollisionCheckDebugMap;
using motion_utils::calcLongitudinalOffsetPose;
using tier4_autoware_utils::calcOffsetPose;

// set as macro so that calling function name will be printed.
// debug print is heavy. turn on only when debugging.
#define DEBUG_PRINT(...) \
  RCLCPP_DEBUG_EXPRESSION(getLogger(), parameters_->print_debug_info, __VA_ARGS__)

namespace behavior_path_planner
{
StartPlannerModule::StartPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<StartPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map},  // NOLINT
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  lane_departure_checker_ = std::make_shared<LaneDepartureChecker>();
  lane_departure_checker_->setVehicleInfo(vehicle_info_);

  // set enabled planner
  if (parameters_->enable_shift_pull_out) {
    start_planners_.push_back(
      std::make_shared<ShiftPullOut>(node, *parameters, lane_departure_checker_));
  }
  if (parameters_->enable_geometric_pull_out) {
    start_planners_.push_back(std::make_shared<GeometricPullOut>(node, *parameters));
  }
  if (start_planners_.empty()) {
    RCLCPP_ERROR(getLogger(), "Not found enabled planner");
  }

  if (parameters_->enable_freespace_planner) {
    freespace_planner_ = std::make_unique<FreespacePullOut>(node, *parameters, vehicle_info_);
    const auto freespace_planner_period_ns = rclcpp::Rate(1.0).period();
    freespace_planner_timer_cb_group_ =
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    freespace_planner_timer_ = rclcpp::create_timer(
      &node, clock_, freespace_planner_period_ns,
      std::bind(&StartPlannerModule::onFreespacePlannerTimer, this),
      freespace_planner_timer_cb_group_);
  }
}

void StartPlannerModule::onFreespacePlannerTimer()
{
  if (!planner_data_) {
    return;
  }

  if (!planner_data_->costmap) {
    return;
  }

  const bool is_new_costmap =
    (clock_->now() - planner_data_->costmap->header.stamp).seconds() < 1.0;
  if (isStuck() && is_new_costmap) {
    planFreespacePath();
  }
}

BehaviorModuleOutput StartPlannerModule::run()
{
  updateData();
  if (!isActivated()) {
    return planWaitingApproval();
  }

  return plan();
}

void StartPlannerModule::processOnEntry()
{
  initVariables();
}

void StartPlannerModule::processOnExit()
{
  initVariables();
}

void StartPlannerModule::initVariables()
{
  resetPathCandidate();
  resetPathReference();
  debug_marker_.markers.clear();
  initializeSafetyCheckParameters();
  initializeCollisionCheckDebugMap(start_planner_data_.collision_check);
}

void StartPlannerModule::updateEgoPredictedPathParams(
  std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params)
{
  ego_predicted_path_params =
    std::make_shared<EgoPredictedPathParams>(start_planner_params->ego_predicted_path_params);
}

void StartPlannerModule::updateSafetyCheckParams(
  std::shared_ptr<SafetyCheckParams> & safety_check_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params)
{
  safety_check_params =
    std::make_shared<SafetyCheckParams>(start_planner_params->safety_check_params);
}

void StartPlannerModule::updateObjectsFilteringParams(
  std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
  const std::shared_ptr<StartPlannerParameters> & start_planner_params)
{
  objects_filtering_params =
    std::make_shared<ObjectsFilteringParams>(start_planner_params->objects_filtering_params);
}

void StartPlannerModule::updateData()
{
  if (receivedNewRoute()) {
    resetStatus();
    DEBUG_PRINT("StartPlannerModule::updateData() received new route, reset status");
  }

  if (hasFinishedBackwardDriving()) {
    updateStatusAfterBackwardDriving();
    DEBUG_PRINT("StartPlannerModule::updateData() completed backward driving");
  } else {
    status_.backward_driving_complete = false;
  }

  if (requiresDynamicObjectsCollisionDetection()) {
    status_.is_safe_dynamic_objects = !hasCollisionWithDynamicObjects();
  } else {
    status_.is_safe_dynamic_objects = true;
  }
}

bool StartPlannerModule::hasFinishedBackwardDriving() const
{
  // check ego car is close enough to pull out start pose and stopped
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const auto distance =
    tier4_autoware_utils::calcDistance2d(current_pose, status_.pull_out_start_pose);

  const bool is_near = distance < parameters_->th_arrived_distance;
  const double ego_vel = utils::l2Norm(planner_data_->self_odometry->twist.twist.linear);
  const bool is_stopped = ego_vel < parameters_->th_stopped_velocity;

  const bool back_finished = !status_.driving_forward && is_near && is_stopped;
  if (back_finished) {
    RCLCPP_INFO(getLogger(), "back finished");
  }

  return back_finished;
}

bool StartPlannerModule::receivedNewRoute() const
{
  return !planner_data_->prev_route_id ||
         *planner_data_->prev_route_id != planner_data_->route_handler->getRouteUuid();
}

bool StartPlannerModule::requiresDynamicObjectsCollisionDetection() const
{
  return parameters_->safety_check_params.enable_safety_check && status_.driving_forward;
}

bool StartPlannerModule::noMovingObjectsAround() const
{
  auto dynamic_objects = *(planner_data_->dynamic_object);
  utils::path_safety_checker::filterObjectsWithinRadius(
    dynamic_objects, planner_data_->self_odometry->pose.pose.position, parameters_->search_radius);
  utils::path_safety_checker::filterObjectsByClass(
    dynamic_objects, parameters_->surround_moving_obstacles_type_to_check);
  const auto filtered_objects = utils::path_safety_checker::filterObjectsByVelocity(
    dynamic_objects, parameters_->th_moving_obstacle_velocity, false);
  if (!filtered_objects.objects.empty()) {
    DEBUG_PRINT("Moving objects exist in the safety check area");
  }
  return filtered_objects.objects.empty();
}

bool StartPlannerModule::hasCollisionWithDynamicObjects() const
{
  // TODO(Sugahara): update path, params for predicted path and so on in this function to avoid
  // mutable
  return !isSafePath();
}

bool StartPlannerModule::isExecutionRequested() const
{
  if (isModuleRunning()) {
    return true;
  }

  // Return false and do not request execution if any of the following conditions are true:
  // - The start pose is on the middle of the road.
  // - The vehicle has already arrived at the start position planner.
  // - The vehicle has reached the goal position.
  // - The vehicle is still moving.
  if (
    isCurrentPoseOnMiddleOfTheRoad() || isCloseToOriginalStartPose() || hasArrivedAtGoal() ||
    isMoving()) {
    return false;
  }

  // Check if the goal is behind the ego vehicle within the same route segment.
  if (isGoalBehindOfEgoInSameRouteSegment()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Start plan for a backward goal is not supported now");
    return false;
  }

  return true;
}

bool StartPlannerModule::isModuleRunning() const
{
  return getCurrentStatus() == ModuleStatus::RUNNING;
}

bool StartPlannerModule::isCurrentPoseOnMiddleOfTheRoad() const
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const lanelet::ConstLanelets current_lanes = utils::getCurrentLanes(planner_data_);
  const double lateral_distance_to_center_lane =
    lanelet::utils::getArcCoordinates(current_lanes, current_pose).distance;

  return std::abs(lateral_distance_to_center_lane) < parameters_->th_distance_to_middle_of_the_road;
}

bool StartPlannerModule::isCloseToOriginalStartPose() const
{
  const Pose start_pose = planner_data_->route_handler->getOriginalStartPose();
  return tier4_autoware_utils::calcDistance2d(
           start_pose.position, planner_data_->self_odometry->pose.pose.position) >
         parameters_->th_arrived_distance;
}

bool StartPlannerModule::hasArrivedAtGoal() const
{
  const Pose goal_pose = planner_data_->route_handler->getGoalPose();
  return tier4_autoware_utils::calcDistance2d(
           goal_pose.position, planner_data_->self_odometry->pose.pose.position) <
         parameters_->th_arrived_distance;
}

bool StartPlannerModule::isMoving() const
{
  return utils::l2Norm(planner_data_->self_odometry->twist.twist.linear) >=
         parameters_->th_stopped_velocity;
}

bool StartPlannerModule::isStopped()
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
  return !std::any_of(
    odometry_buffer_.begin(), odometry_buffer_.end(), [this](const auto & odometry) {
      return utils::l2Norm(odometry->twist.twist.linear) > parameters_->th_stopped_velocity;
    });
}

bool StartPlannerModule::isExecutionReady() const
{
  bool is_safe = true;
  // Evaluate safety. The situation is not safe if any of the following conditions are met:
  // 1. pull out path has not been found
  // 2. there is a moving objects around ego
  // 3. waiting for approval and there is a collision with dynamic objects
  if (!status_.found_pull_out_path) {
    is_safe = false;
  } else if (isWaitingApproval()) {
    if (!noMovingObjectsAround()) {
      is_safe = false;
    } else if (requiresDynamicObjectsCollisionDetection() && hasCollisionWithDynamicObjects()) {
      is_safe = false;
    }
  }

  if (!is_safe) {
    stop_pose_ = planner_data_->self_odometry->pose.pose;
  }

  return is_safe;
}

bool StartPlannerModule::canTransitSuccessState()
{
  return hasFinishedPullOut();
}

bool StartPlannerModule::canTransitIdleToRunningState()
{
  return isActivated() && !isWaitingApproval();
}

BehaviorModuleOutput StartPlannerModule::plan()
{
  if (isWaitingApproval()) {
    clearWaitingApproval();
    resetPathCandidate();
    resetPathReference();
  }

  BehaviorModuleOutput output;
  if (!status_.found_pull_out_path) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull out path, publish stop path");
    const auto output = generateStopOutput();
    setDebugData();  // use status updated in generateStopOutput()
    updateRTCStatus(0, 0);
    return output;
  }

  PathWithLaneId path;

  // Check if backward motion is finished
  if (status_.driving_forward || status_.backward_driving_complete) {
    // Increment path index if the current path is finished
    if (hasFinishedCurrentPath()) {
      RCLCPP_INFO(getLogger(), "Increment path index");
      incrementPathIndex();
    }

    if (!status_.is_safe_dynamic_objects && !isWaitingApproval() && !status_.stop_pose) {
      auto current_path = getCurrentPath();
      const auto stop_path =
        behavior_path_planner::utils::parking_departure::generateFeasibleStopPath(
          current_path, planner_data_, stop_pose_, parameters_->maximum_deceleration_for_stop,
          parameters_->maximum_jerk_for_stop);

      // Insert stop point in the path if needed
      if (stop_path) {
        RCLCPP_ERROR_THROTTLE(
          getLogger(), *clock_, 5000, "Insert stop point in the path because of dynamic objects");
        path = *stop_path;
        status_.prev_stop_path_after_approval = std::make_shared<PathWithLaneId>(path);
        status_.stop_pose = stop_pose_;
      } else {
        path = current_path;
      }
    } else if (!isWaitingApproval() && status_.stop_pose) {
      // Delete stop point if conditions are met
      if (status_.is_safe_dynamic_objects && isStopped()) {
        status_.stop_pose = std::nullopt;
        path = getCurrentPath();
      }
      path = *status_.prev_stop_path_after_approval;
      stop_pose_ = status_.stop_pose;
    } else {
      path = getCurrentPath();
    }
  } else {
    path = status_.backward_path;
  }

  output.path = path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  setDrivableAreaInfo(output);

  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return SteeringFactor::LEFT;
    } else if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::STRAIGHT;
  });

  if (status_.driving_forward) {
    const double start_distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    const double finish_distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.end_pose.position);
    updateRTCStatus(start_distance, finish_distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose},
      {start_distance, finish_distance}, PlanningBehavior::START_PLANNER, steering_factor_direction,
      SteeringFactor::TURNING, "");
  } else {
    const double distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    updateRTCStatus(0.0, distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
      PlanningBehavior::START_PLANNER, steering_factor_direction, SteeringFactor::TURNING, "");
  }

  setDebugData();

  return output;
}

CandidateOutput StartPlannerModule::planCandidate() const
{
  return CandidateOutput{};
}

void StartPlannerModule::initializeSafetyCheckParameters()
{
  updateEgoPredictedPathParams(ego_predicted_path_params_, parameters_);
  updateSafetyCheckParams(safety_check_params_, parameters_);
  updateObjectsFilteringParams(objects_filtering_params_, parameters_);
}

PathWithLaneId StartPlannerModule::getFullPath() const
{
  // combine partial pull out path
  PathWithLaneId pull_out_path;
  for (const auto & partial_path : status_.pull_out_path.partial_paths) {
    pull_out_path.points.insert(
      pull_out_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  if (status_.driving_forward) {
    // not need backward path or finish it
    return pull_out_path;
  }

  // concat back_path and pull_out_path and
  auto full_path = status_.backward_path;
  full_path.points.insert(
    full_path.points.end(), pull_out_path.points.begin(), pull_out_path.points.end());
  return full_path;
}

BehaviorModuleOutput StartPlannerModule::planWaitingApproval()
{
  updatePullOutStatus();

  BehaviorModuleOutput output;
  if (!status_.found_pull_out_path) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull out path, publish stop path");
    clearWaitingApproval();
    const auto output = generateStopOutput();
    setDebugData();  // use status updated in generateStopOutput()
    updateRTCStatus(0, 0);
    return output;
  }

  waitApproval();

  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  auto stop_path = status_.driving_forward ? getCurrentPath() : status_.backward_path;
  const auto drivable_lanes = generateDrivableLanes(stop_path);
  const auto & dp = planner_data_->drivable_area_expansion_parameters;
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);
  for (auto & p : stop_path.points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }

  output.path = stop_path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = std::make_unique<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  setDrivableAreaInfo(output);

  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return SteeringFactor::LEFT;
    } else if (output.turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::STRAIGHT;
  });

  if (status_.driving_forward) {
    const double start_distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    const double finish_distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.end_pose.position);
    updateRTCStatus(start_distance, finish_distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose},
      {start_distance, finish_distance}, PlanningBehavior::START_PLANNER, steering_factor_direction,
      SteeringFactor::APPROACHING, "");
  } else {
    const double distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    updateRTCStatus(0.0, distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
      PlanningBehavior::START_PLANNER, steering_factor_direction, SteeringFactor::APPROACHING, "");
  }

  setDebugData();

  return output;
}

void StartPlannerModule::resetStatus()
{
  status_ = PullOutStatus{};
}

void StartPlannerModule::incrementPathIndex()
{
  status_.current_path_idx =
    std::min(status_.current_path_idx + 1, status_.pull_out_path.partial_paths.size() - 1);
}

PathWithLaneId StartPlannerModule::getCurrentPath() const
{
  if (status_.pull_out_path.partial_paths.size() <= status_.current_path_idx) {
    return PathWithLaneId{};
  }
  return status_.pull_out_path.partial_paths.at(status_.current_path_idx);
}

void StartPlannerModule::planWithPriority(
  const std::vector<Pose> & start_pose_candidates, const Pose & refined_start_pose,
  const Pose & goal_pose, const std::string search_priority)
{
  if (start_pose_candidates.empty()) return;

  const PriorityOrder order_priority =
    determinePriorityOrder(search_priority, start_pose_candidates.size());

  for (const auto & [index, planner] : order_priority) {
    if (findPullOutPath(start_pose_candidates[index], planner, refined_start_pose, goal_pose))
      return;
  }

  updateStatusIfNoSafePathFound();
}

PriorityOrder StartPlannerModule::determinePriorityOrder(
  const std::string & search_priority, const size_t start_pose_candidates_num)
{
  PriorityOrder order_priority;
  if (search_priority == "efficient_path") {
    for (const auto & planner : start_planners_) {
      for (size_t i = 0; i < start_pose_candidates_num; i++) {
        order_priority.emplace_back(i, planner);
      }
    }
  } else if (search_priority == "short_back_distance") {
    for (size_t i = 0; i < start_pose_candidates_num; i++) {
      for (const auto & planner : start_planners_) {
        order_priority.emplace_back(i, planner);
      }
    }
  } else {
    RCLCPP_ERROR(getLogger(), "Invalid search_priority: %s", search_priority.c_str());
    throw std::domain_error("[start_planner] invalid search_priority");
  }
  return order_priority;
}

bool StartPlannerModule::findPullOutPath(
  const Pose & start_pose_candidate, const std::shared_ptr<PullOutPlannerBase> & planner,
  const Pose & refined_start_pose, const Pose & goal_pose)
{
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);
  const auto & vehicle_footprint = createVehicleFootprint(vehicle_info_);
  // extract stop objects in pull out lane for collision check
  const auto stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
    *dynamic_objects, parameters_->th_moving_object_velocity);
  const auto [pull_out_lane_stop_objects, others] =
    utils::path_safety_checker::separateObjectsByLanelets(
      stop_objects, pull_out_lanes, utils::path_safety_checker::isPolygonOverlapLanelet);

  // if start_pose_candidate is far from refined_start_pose, backward driving is necessary
  const bool backward_is_unnecessary =
    tier4_autoware_utils::calcDistance2d(start_pose_candidate, refined_start_pose) < 0.01;

  planner->setPlannerData(planner_data_);
  const auto pull_out_path = planner->plan(start_pose_candidate, goal_pose);

  // If no path is found, return false
  if (!pull_out_path) {
    return false;
  }

  // check collision
  if (utils::checkCollisionBetweenPathFootprintsAndObjects(
        vehicle_footprint, extractCollisionCheckPath(*pull_out_path), pull_out_lane_stop_objects,
        parameters_->collision_check_margin)) {
    return false;
  }

  if (backward_is_unnecessary) {
    updateStatusWithCurrentPath(*pull_out_path, start_pose_candidate, planner->getPlannerType());
    return true;
  }

  updateStatusWithNextPath(*pull_out_path, start_pose_candidate, planner->getPlannerType());

  return true;
}

PathWithLaneId StartPlannerModule::extractCollisionCheckPath(const PullOutPath & path)
{
  PathWithLaneId combined_path;
  for (const auto & partial_path : path.partial_paths) {
    combined_path.points.insert(
      combined_path.points.end(), partial_path.points.begin(), partial_path.points.end());
  }

  // calculate collision check end idx
  size_t collision_check_end_idx = 0;
  const auto collision_check_end_pose = motion_utils::calcLongitudinalOffsetPose(
    combined_path.points, path.end_pose.position, parameters_->collision_check_distance_from_end);

  if (collision_check_end_pose) {
    collision_check_end_idx =
      motion_utils::findNearestIndex(combined_path.points, collision_check_end_pose->position);
  }

  // remove the point behind of collision check end pose
  if (collision_check_end_idx + 1 < combined_path.points.size()) {
    combined_path.points.erase(
      combined_path.points.begin() + collision_check_end_idx + 1, combined_path.points.end());
  }

  return combined_path;
}

void StartPlannerModule::updateStatusWithCurrentPath(
  const behavior_path_planner::PullOutPath & path, const Pose & start_pose,
  const behavior_path_planner::PlannerType & planner_type)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  status_.driving_forward = true;
  status_.found_pull_out_path = true;
  status_.pull_out_path = path;
  status_.pull_out_start_pose = start_pose;
  status_.planner_type = planner_type;
}

void StartPlannerModule::updateStatusWithNextPath(
  const behavior_path_planner::PullOutPath & path, const Pose & start_pose,
  const behavior_path_planner::PlannerType & planner_type)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  status_.driving_forward = false;
  status_.found_pull_out_path = true;
  status_.pull_out_path = path;
  status_.pull_out_start_pose = start_pose;
  status_.planner_type = planner_type;
}

void StartPlannerModule::updateStatusIfNoSafePathFound()
{
  if (status_.planner_type != PlannerType::FREESPACE) {
    const std::lock_guard<std::mutex> lock(mutex_);
    status_.found_pull_out_path = false;
    status_.planner_type = PlannerType::NONE;
  }
}

PathWithLaneId StartPlannerModule::generateStopPath() const
{
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  constexpr double dummy_path_distance = 1.0;
  const auto moved_pose = calcOffsetPose(current_pose, dummy_path_distance, 0, 0);

  // convert Pose to PathPointWithLaneId with 0 velocity.
  auto toPathPointWithLaneId = [this](const Pose & pose) {
    PathPointWithLaneId p{};
    p.point.pose = pose;
    p.point.longitudinal_velocity_mps = 0.0;
    const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
      planner_data_,
      planner_data_->parameters.backward_path_length + parameters_->max_back_distance);
    lanelet::Lanelet closest_lanelet;
    lanelet::utils::query::getClosestLanelet(pull_out_lanes, pose, &closest_lanelet);
    p.lane_ids.push_back(closest_lanelet.id());
    return p;
  };

  PathWithLaneId path{};
  path.points.push_back(toPathPointWithLaneId(current_pose));
  path.points.push_back(toPathPointWithLaneId(moved_pose));

  return path;
}

lanelet::ConstLanelets StartPlannerModule::getPathRoadLanes(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & lanelet_layer = route_handler->getLaneletMapPtr()->laneletLayer;

  std::vector<lanelet::Id> lane_ids;
  for (const auto & p : path.points) {
    for (const auto & id : p.lane_ids) {
      if (id == lanelet::InvalId) {
        continue;
      }
      if (route_handler->isShoulderLanelet(lanelet_layer.get(id))) {
        continue;
      }
      if (std::find(lane_ids.begin(), lane_ids.end(), id) == lane_ids.end()) {
        lane_ids.push_back(id);
      }
    }
  }

  lanelet::ConstLanelets path_lanes;
  path_lanes.reserve(lane_ids.size());
  for (const auto & id : lane_ids) {
    path_lanes.push_back(lanelet_layer.get(id));
  }

  return path_lanes;
}

std::vector<DrivableLanes> StartPlannerModule::generateDrivableLanes(
  const PathWithLaneId & path) const
{
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);

  const auto path_road_lanes = getPathRoadLanes(path);
  if (!path_road_lanes.empty()) {
    lanelet::ConstLanelets shoulder_lanes;
    const auto & rh = planner_data_->route_handler;
    std::copy_if(
      pull_out_lanes.begin(), pull_out_lanes.end(), std::back_inserter(shoulder_lanes),
      [&rh](const auto & pull_out_lane) { return rh->isShoulderLanelet(pull_out_lane); });

    return utils::generateDrivableLanesWithShoulderLanes(path_road_lanes, shoulder_lanes);
  }

  // if path_road_lanes is empty, use only pull_out_lanes as drivable lanes
  std::vector<DrivableLanes> drivable_lanes;
  for (const auto & lane : pull_out_lanes) {
    DrivableLanes drivable_lane;
    drivable_lane.right_lane = lane;
    drivable_lane.left_lane = lane;
    drivable_lanes.push_back(drivable_lane);
  }
  return drivable_lanes;
}

void StartPlannerModule::updatePullOutStatus()
{
  // skip updating if enough time has not passed for preventing chattering between back and
  // start_planner
  if (!receivedNewRoute()) {
    if (!last_pull_out_start_update_time_) {
      last_pull_out_start_update_time_ = std::make_unique<rclcpp::Time>(clock_->now());
    }
    const auto elapsed_time = (clock_->now() - *last_pull_out_start_update_time_).seconds();
    if (elapsed_time < parameters_->backward_path_update_duration) {
      return;
    }
  }
  last_pull_out_start_update_time_ = std::make_unique<rclcpp::Time>(clock_->now());

  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & goal_pose = planner_data_->route_handler->getGoalPose();

  // refine start pose with pull out lanes.
  // 1) backward driving is not allowed: use refined pose just as start pose.
  // 2) backward driving is allowed: use refined pose to check if backward driving is needed.
  const PathWithLaneId start_pose_candidates_path = calcBackwardPathFromStartPose();
  const auto refined_start_pose = calcLongitudinalOffsetPose(
    start_pose_candidates_path.points, planner_data_->self_odometry->pose.pose.position, 0.0);
  if (!refined_start_pose) return;

  // search pull out start candidates backward
  const std::vector<Pose> start_pose_candidates = std::invoke([&]() -> std::vector<Pose> {
    if (parameters_->enable_back) {
      return searchPullOutStartPoseCandidates(start_pose_candidates_path);
    }
    return {*refined_start_pose};
  });

  planWithPriority(
    start_pose_candidates, *refined_start_pose, goal_pose, parameters_->search_priority);

  start_planner_data_.refined_start_pose = *refined_start_pose;
  start_planner_data_.start_pose_candidates = start_pose_candidates;
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);

  if (hasFinishedBackwardDriving()) {
    updateStatusAfterBackwardDriving();
  } else {
    status_.backward_path = start_planner_utils::getBackwardPath(
      *route_handler, pull_out_lanes, current_pose, status_.pull_out_start_pose,
      parameters_->backward_velocity);
  }
}

void StartPlannerModule::updateStatusAfterBackwardDriving()
{
  status_.driving_forward = true;
  status_.backward_driving_complete = true;
  // request start_planner approval
  waitApproval();
  // To enable approval of the forward path, the RTC status is removed.
  removeRTCStatus();
  for (auto itr = uuid_map_.begin(); itr != uuid_map_.end(); ++itr) {
    itr->second = generateUUID();
  }
}

PathWithLaneId StartPlannerModule::calcBackwardPathFromStartPose() const
{
  const Pose start_pose = planner_data_->route_handler->getOriginalStartPose();
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);

  const auto arc_position_pose = lanelet::utils::getArcCoordinates(pull_out_lanes, start_pose);

  // common buffer distance for both front and back
  static constexpr double buffer = 30.0;
  const double check_distance = parameters_->max_back_distance + buffer;

  const double start_distance = arc_position_pose.length - check_distance;
  const double end_distance = arc_position_pose.length + buffer;

  auto path =
    planner_data_->route_handler->getCenterLinePath(pull_out_lanes, start_distance, end_distance);

  // shift all path points laterally to align with the start pose
  for (auto & path_point : path.points) {
    path_point.point.pose = calcOffsetPose(path_point.point.pose, 0, arc_position_pose.distance, 0);
  }

  return path;
}

std::vector<Pose> StartPlannerModule::searchPullOutStartPoseCandidates(
  const PathWithLaneId & back_path_from_start_pose) const
{
  std::vector<Pose> pull_out_start_pose_candidates{};
  const auto start_pose = planner_data_->route_handler->getOriginalStartPose();
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info_);
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);
  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;

  const auto stop_objects_in_pull_out_lanes = filterStopObjectsInPullOutLanes(
    pull_out_lanes, start_pose.position, parameters_->th_moving_object_velocity,
    backward_path_length, std::numeric_limits<double>::max());

  // Set the maximum backward distance less than the distance from the vehicle's base_link to the
  // lane's rearmost point to prevent lane departure.
  const double current_arc_length =
    lanelet::utils::getArcCoordinates(pull_out_lanes, start_pose).length;
  const double allowed_backward_distance = std::clamp(
    current_arc_length - planner_data_->parameters.base_link2rear, 0.0,
    parameters_->max_back_distance);

  for (double back_distance = 0.0; back_distance <= allowed_backward_distance;
       back_distance += parameters_->backward_search_resolution) {
    const auto backed_pose = calcLongitudinalOffsetPose(
      back_path_from_start_pose.points, start_pose.position, -back_distance);
    if (!backed_pose) {
      continue;
    }

    const double backed_pose_arc_length =
      lanelet::utils::getArcCoordinates(pull_out_lanes, *backed_pose).length;
    const double length_to_lane_end = std::accumulate(
      std::begin(pull_out_lanes), std::end(pull_out_lanes), 0.0,
      [](double acc, const auto & lane) { return acc + lanelet::utils::getLaneletLength2d(lane); });
    const double distance_from_lane_end = length_to_lane_end - backed_pose_arc_length;
    if (distance_from_lane_end < parameters_->ignore_distance_from_lane_end) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000,
        "the ego is too close to the lane end, so needs backward driving");
      continue;
    }

    if (utils::checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, *backed_pose, stop_objects_in_pull_out_lanes,
          parameters_->collision_check_margin)) {
      break;  // poses behind this has a collision, so break.
    }

    pull_out_start_pose_candidates.push_back(*backed_pose);
  }
  return pull_out_start_pose_candidates;
}

PredictedObjects StartPlannerModule::filterStopObjectsInPullOutLanes(
  const lanelet::ConstLanelets & pull_out_lanes, const geometry_msgs::msg::Point & current_point,
  const double velocity_threshold, const double object_check_forward_distance,
  const double object_check_backward_distance) const
{
  const auto stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
    *planner_data_->dynamic_object, velocity_threshold);

  // filter for objects located in pull out lanes and moving at a speed below the threshold
  auto [stop_objects_in_pull_out_lanes, others] =
    utils::path_safety_checker::separateObjectsByLanelets(
      stop_objects, pull_out_lanes, utils::path_safety_checker::isPolygonOverlapLanelet);

  const auto path = planner_data_->route_handler->getCenterLinePath(
    pull_out_lanes, object_check_backward_distance, object_check_forward_distance);

  utils::path_safety_checker::filterObjectsByPosition(
    stop_objects_in_pull_out_lanes, path.points, current_point, object_check_forward_distance,
    object_check_backward_distance);

  return stop_objects_in_pull_out_lanes;
}

bool StartPlannerModule::hasFinishedPullOut() const
{
  if (!status_.driving_forward || !status_.found_pull_out_path) {
    return false;
  }

  const auto current_pose = planner_data_->self_odometry->pose.pose;
  if (status_.planner_type == PlannerType::FREESPACE) {
    return tier4_autoware_utils::calcDistance2d(current_pose, status_.pull_out_path.end_pose) <
           parameters_->th_arrived_distance;
  }

  // check that ego has passed pull out end point
  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  const auto arclength_current = lanelet::utils::getArcCoordinates(current_lanes, current_pose);
  const auto arclength_pull_out_end =
    lanelet::utils::getArcCoordinates(current_lanes, status_.pull_out_path.end_pose);

  // offset to not finish the module before engage
  constexpr double offset = 0.1;
  const bool has_finished = arclength_current.length - arclength_pull_out_end.length > offset;

  return has_finished;
}

bool StartPlannerModule::isStuck()
{
  if (!isStopped()) {
    return false;
  }

  if (status_.planner_type == PlannerType::STOP) {
    return true;
  }

  // not found safe path
  if (!status_.found_pull_out_path) {
    return true;
  }

  return false;
}

bool StartPlannerModule::hasFinishedCurrentPath()
{
  const auto current_path = getCurrentPath();
  const auto current_path_end = current_path.points.back();
  const auto self_pose = planner_data_->self_odometry->pose.pose;
  const bool is_near_target = tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
                              parameters_->th_arrived_distance;

  return is_near_target && isStopped();
}

TurnSignalInfo StartPlannerModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal{};  // output

  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const Pose & start_pose = status_.pull_out_path.start_pose;
  const Pose & end_pose = status_.pull_out_path.end_pose;

  // turn on hazard light when backward driving
  if (!status_.driving_forward) {
    turn_signal.hazard_signal.command = HazardLightsCommand::ENABLE;
    const auto back_start_pose = planner_data_->route_handler->getOriginalStartPose();
    turn_signal.desired_start_point = back_start_pose;
    turn_signal.required_start_point = back_start_pose;
    // pull_out start_pose is same to backward driving end_pose
    turn_signal.required_end_point = start_pose;
    turn_signal.desired_end_point = start_pose;
    return turn_signal;
  }

  // turn on right signal until passing pull_out end point
  const auto path = getFullPath();
  // pull out path does not overlap
  const double distance_from_end =
    motion_utils::calcSignedArcLength(path.points, end_pose.position, current_pose.position);

  if (path.points.empty()) {
    return {};
  }

  // calculate lateral offset from pull out target lane center line
  lanelet::ConstLanelet closest_road_lane;
  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;
  const auto road_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  lanelet::utils::query::getClosestLanelet(road_lanes, start_pose, &closest_road_lane);
  const double lateral_offset =
    lanelet::utils::getLateralDistanceToCenterline(closest_road_lane, start_pose);

  if (distance_from_end < 0.0 && lateral_offset > parameters_->th_turn_signal_on_lateral_offset) {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  } else if (
    distance_from_end < 0.0 && lateral_offset < -parameters_->th_turn_signal_on_lateral_offset) {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  } else {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::DISABLE;
  }

  turn_signal.desired_start_point = start_pose;
  turn_signal.required_start_point = start_pose;
  turn_signal.desired_end_point = end_pose;

  // check if intersection exists within search length
  const bool is_near_intersection = std::invoke([&]() {
    const double check_length = parameters_->intersection_search_length;
    double accumulated_length = 0.0;
    const size_t current_idx = motion_utils::findNearestIndex(path.points, current_pose.position);
    for (size_t i = current_idx; i < path.points.size() - 1; ++i) {
      const auto & p = path.points.at(i);
      for (const auto & lane : planner_data_->route_handler->getLaneletsFromIds(p.lane_ids)) {
        const std::string turn_direction = lane.attributeOr("turn_direction", "else");
        if (turn_direction == "right" || turn_direction == "left" || turn_direction == "straight") {
          return true;
        }
      }
      accumulated_length += tier4_autoware_utils::calcDistance2d(p, path.points.at(i + 1));
      if (accumulated_length > check_length) {
        return false;
      }
    }
    return false;
  });

  if (is_near_intersection) {
    // offset required end pose with ration to activate turn signal for intersection
    turn_signal.required_end_point = std::invoke([&]() {
      const double length_start_to_end =
        motion_utils::calcSignedArcLength(path.points, start_pose.position, end_pose.position);
      const auto ratio = std::clamp(
        parameters_->length_ratio_for_turn_signal_deactivation_near_intersection, 0.0, 1.0);

      const double required_end_length = length_start_to_end * ratio;
      double accumulated_length = 0.0;
      const size_t start_idx = motion_utils::findNearestIndex(path.points, start_pose.position);
      for (size_t i = start_idx; i < path.points.size() - 1; ++i) {
        accumulated_length +=
          tier4_autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
        if (accumulated_length > required_end_length) {
          return path.points.at(i).point.pose;
        }
      }
      // not found required end point
      return end_pose;
    });
  } else {
    turn_signal.required_end_point = end_pose;
  }

  return turn_signal;
}

bool StartPlannerModule::isSafePath() const
{
  // TODO(Sugahara): should safety check for backward path

  const auto pull_out_path = getCurrentPath();
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const double current_velocity = std::hypot(
    planner_data_->self_odometry->twist.twist.linear.x,
    planner_data_->self_odometry->twist.twist.linear.y);
  const auto & dynamic_object = planner_data_->dynamic_object;
  const auto & route_handler = planner_data_->route_handler;
  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;

  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  // for ego predicted path
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(pull_out_path.points);
  const std::pair<double, double> terminal_velocity_and_accel =
    utils::parking_departure::getPairsTerminalVelocityAndAccel(
      status_.pull_out_path.pairs_terminal_velocity_and_accel, status_.current_path_idx);
  RCLCPP_DEBUG(
    getLogger(), "pairs_terminal_velocity_and_accel for start_planner: %f, %f",
    terminal_velocity_and_accel.first, terminal_velocity_and_accel.second);
  RCLCPP_DEBUG(getLogger(), "current_path_idx %ld", status_.current_path_idx);
  utils::parking_departure::updatePathProperty(
    ego_predicted_path_params_, terminal_velocity_and_accel);
  // TODO(Sugahara): shoule judge is_object_front properly
  const bool is_object_front = true;
  const bool limit_to_max_velocity = true;
  const auto ego_predicted_path =
    behavior_path_planner::utils::path_safety_checker::createPredictedPath(
      ego_predicted_path_params_, pull_out_path.points, current_pose, current_velocity, ego_seg_idx,
      is_object_front, limit_to_max_velocity);

  // filtering objects with velocity, position and class
  const auto filtered_objects = utils::path_safety_checker::filterObjects(
    dynamic_object, route_handler, current_lanes, current_pose.position, objects_filtering_params_);

  // filtering objects based on the current position's lane
  const auto target_objects_on_lane = utils::path_safety_checker::createTargetObjectsOnLane(
    current_lanes, route_handler, filtered_objects, objects_filtering_params_);

  const double hysteresis_factor =
    status_.is_safe_dynamic_objects ? 1.0 : safety_check_params_->hysteresis_factor_expand_rate;

  utils::parking_departure::updateSafetyCheckTargetObjectsData(
    start_planner_data_, filtered_objects, target_objects_on_lane, ego_predicted_path);

  return behavior_path_planner::utils::path_safety_checker::checkSafetyWithRSS(
    pull_out_path, ego_predicted_path, target_objects_on_lane.on_current_lane,
    start_planner_data_.collision_check, planner_data_->parameters,
    safety_check_params_->rss_params, objects_filtering_params_->use_all_predicted_path,
    hysteresis_factor);
}

bool StartPlannerModule::isGoalBehindOfEgoInSameRouteSegment() const
{
  const auto & rh = planner_data_->route_handler;

  // Check if the goal and ego are in the same route segment. If not, this is out of scope of this
  // function. Return false.
  lanelet::ConstLanelet ego_lanelet;
  rh->getClosestLaneletWithinRoute(getEgoPose(), &ego_lanelet);
  const auto is_ego_in_goal_route_section = rh->isInGoalRouteSection(ego_lanelet);

  if (!is_ego_in_goal_route_section) {
    return false;
  }

  // If the goal and ego are in the same route segment, check the goal and ego pose relation.
  // Return true when the goal is located behind of ego.
  const auto ego_lane_path = rh->getCenterLinePath(
    lanelet::ConstLanelets{ego_lanelet}, 0.0, std::numeric_limits<double>::max());
  const auto dist_ego_to_goal = motion_utils::calcSignedArcLength(
    ego_lane_path.points, getEgoPosition(), rh->getGoalPose().position);

  const bool is_goal_behind_of_ego = (dist_ego_to_goal < 0.0);
  return is_goal_behind_of_ego;
}

// NOTE: this must be called after updatePullOutStatus(). This must be fixed.
BehaviorModuleOutput StartPlannerModule::generateStopOutput()
{
  BehaviorModuleOutput output;
  const PathWithLaneId stop_path = generateStopPath();
  output.path = stop_path;

  setDrivableAreaInfo(output);

  output.reference_path = getPreviousModuleOutput().reference_path;

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    status_.driving_forward = true;
    status_.planner_type = PlannerType::STOP;
    status_.pull_out_path.partial_paths.clear();
    status_.pull_out_path.partial_paths.push_back(stop_path);
    const Pose & current_pose = planner_data_->self_odometry->pose.pose;
    status_.pull_out_start_pose = current_pose;
    status_.pull_out_path.start_pose = current_pose;
    status_.pull_out_path.end_pose = current_pose;
  }

  path_candidate_ = std::make_shared<PathWithLaneId>(stop_path);
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  return output;
}

bool StartPlannerModule::planFreespacePath()
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & route_handler = planner_data_->route_handler;

  const double end_pose_search_start_distance = parameters_->end_pose_search_start_distance;
  const double end_pose_search_end_distance = parameters_->end_pose_search_end_distance;
  const double end_pose_search_interval = parameters_->end_pose_search_interval;

  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  const auto current_arc_coords = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  const double s_start = std::max(0.0, current_arc_coords.length + end_pose_search_start_distance);
  const double s_end = current_arc_coords.length + end_pose_search_end_distance;

  auto center_line_path = utils::resamplePathWithSpline(
    route_handler->getCenterLinePath(current_lanes, s_start, s_end), end_pose_search_interval);

  for (const auto & p : center_line_path.points) {
    const Pose end_pose = p.point.pose;
    freespace_planner_->setPlannerData(planner_data_);
    auto freespace_path = freespace_planner_->plan(current_pose, end_pose);

    if (!freespace_path) {
      continue;
    }

    const std::lock_guard<std::mutex> lock(mutex_);
    status_.pull_out_path = *freespace_path;
    status_.pull_out_start_pose = current_pose;
    status_.planner_type = freespace_planner_->getPlannerType();
    status_.found_pull_out_path = true;
    status_.driving_forward = true;
    return true;
  }

  return false;
}

void StartPlannerModule::setDrivableAreaInfo(BehaviorModuleOutput & output) const
{
  if (status_.planner_type == PlannerType::FREESPACE) {
    const double drivable_area_margin = planner_data_->parameters.vehicle_width;
    output.drivable_area_info.drivable_margin =
      planner_data_->parameters.vehicle_width / 2.0 + drivable_area_margin;
  } else {
    const auto target_drivable_lanes = utils::getNonOverlappingExpandedLanes(
      output.path, generateDrivableLanes(output.path),
      planner_data_->drivable_area_expansion_parameters);

    DrivableAreaInfo current_drivable_area_info;
    current_drivable_area_info.drivable_lanes = target_drivable_lanes;
    output.drivable_area_info =
      status_.driving_forward
        ? utils::combineDrivableAreaInfo(
            current_drivable_area_info, getPreviousModuleOutput().drivable_area_info)
        : current_drivable_area_info;
  }
}

void StartPlannerModule::setDebugData() const
{
  using marker_utils::addFootprintMarker;
  using marker_utils::createFootprintMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createPredictedPathMarkerArray;
  using marker_utils::showPolygon;
  using marker_utils::showPredictedPath;
  using marker_utils::showSafetyCheckInfo;
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerColor;
  using tier4_autoware_utils::createMarkerScale;
  using visualization_msgs::msg::Marker;

  const auto life_time = rclcpp::Duration::from_seconds(1.5);
  auto add = [&](MarkerArray added) {
    for (auto & marker : added.markers) {
      marker.lifetime = life_time;
    }
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  debug_marker_.markers.clear();
  add(createPoseMarkerArray(status_.pull_out_start_pose, "back_end_pose", 0, 0.9, 0.3, 0.3));
  add(createPoseMarkerArray(status_.pull_out_path.start_pose, "start_pose", 0, 0.3, 0.9, 0.3));
  add(createPoseMarkerArray(status_.pull_out_path.end_pose, "end_pose", 0, 0.9, 0.9, 0.3));
  add(createFootprintMarkerArray(
    start_planner_data_.refined_start_pose, vehicle_info_, "refined_start_pose", 0, 0.9, 0.9, 0.3));
  add(createPathMarkerArray(getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
  add(createPathMarkerArray(status_.backward_path, "backward_driving_path", 0, 0.0, 0.9, 0.0));

  // visualize collision_check_end_pose and footprint
  {
    const auto local_footprint = createVehicleFootprint(vehicle_info_);
    const auto collision_check_end_pose = motion_utils::calcLongitudinalOffsetPose(
      getFullPath().points, status_.pull_out_path.end_pose.position,
      parameters_->collision_check_distance_from_end);
    if (collision_check_end_pose) {
      add(createPoseMarkerArray(
        *collision_check_end_pose, "static_collision_check_end_pose", 0, 1.0, 0.0, 0.0));
      auto marker = tier4_autoware_utils::createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "static_collision_check_end_polygon", 0,
        Marker::LINE_LIST, tier4_autoware_utils::createMarkerScale(0.1, 0.1, 0.1),
        tier4_autoware_utils::createMarkerColor(1.0, 0.0, 0.0, 0.999));
      const auto footprint = transformVector(
        local_footprint, tier4_autoware_utils::pose2transform(*collision_check_end_pose));
      const double ego_z = planner_data_->self_odometry->pose.pose.position.z;
      for (size_t i = 0; i < footprint.size(); i++) {
        const auto & current_point = footprint.at(i);
        const auto & next_point = footprint.at((i + 1) % footprint.size());
        marker.points.push_back(
          tier4_autoware_utils::createPoint(current_point.x(), current_point.y(), ego_z));
        marker.points.push_back(
          tier4_autoware_utils::createPoint(next_point.x(), next_point.y(), ego_z));
      }
      marker.lifetime = life_time;
      debug_marker_.markers.push_back(marker);
    }
  }
  // start pose candidates
  {
    MarkerArray start_pose_footprint_marker_array{};
    MarkerArray start_pose_text_marker_array{};
    const auto purple = createMarkerColor(1.0, 0.0, 1.0, 0.99);
    Marker footprint_marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "start_pose_candidates", 0, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.2, 0.2), purple);
    Marker text_marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "start_pose_candidates_idx", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.3, 0.3, 0.3), purple);
    footprint_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    text_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    for (size_t i = 0; i < start_planner_data_.start_pose_candidates.size(); ++i) {
      footprint_marker.id = i;
      text_marker.id = i;
      footprint_marker.points.clear();
      text_marker.text = "idx[" + std::to_string(i) + "]";
      text_marker.pose = start_planner_data_.start_pose_candidates.at(i);
      addFootprintMarker(
        footprint_marker, start_planner_data_.start_pose_candidates.at(i), vehicle_info_);
      start_pose_footprint_marker_array.markers.push_back(footprint_marker);
      start_pose_text_marker_array.markers.push_back(text_marker);
    }

    add(start_pose_footprint_marker_array);
    add(start_pose_text_marker_array);
  }

  // safety check
  if (parameters_->safety_check_params.enable_safety_check) {
    if (start_planner_data_.ego_predicted_path.size() > 0) {
      const auto & ego_predicted_path = utils::path_safety_checker::convertToPredictedPath(
        start_planner_data_.ego_predicted_path, ego_predicted_path_params_->time_resolution);
      add(createPredictedPathMarkerArray(
        ego_predicted_path, vehicle_info_, "ego_predicted_path_start_planner", 0, 0.0, 0.5, 0.9));
    }

    if (start_planner_data_.filtered_objects.objects.size() > 0) {
      add(createObjectsMarkerArray(
        start_planner_data_.filtered_objects, "filtered_objects", 0, 0.0, 0.5, 0.9));
    }

    add(showSafetyCheckInfo(start_planner_data_.collision_check, "object_debug_info"));
    add(showPredictedPath(start_planner_data_.collision_check, "ego_predicted_path"));
    add(showPolygon(start_planner_data_.collision_check, "ego_and_target_polygon_relation"));
    initializeCollisionCheckDebugMap(start_planner_data_.collision_check);
  }

  // Visualize planner type text
  const auto header = planner_data_->route_handler->getRouteHeader();
  {
    visualization_msgs::msg::MarkerArray planner_type_marker_array{};
    const auto color = status_.found_pull_out_path ? createMarkerColor(1.0, 1.0, 1.0, 0.99)
                                                   : createMarkerColor(1.0, 0.0, 0.0, 0.99);
    auto marker = createDefaultMarker(
      header.frame_id, header.stamp, "planner_type", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0), color);
    marker.pose = status_.pull_out_start_pose;
    if (!status_.driving_forward) {
      marker.text = "BACK -> ";
    }
    marker.text += magic_enum::enum_name(status_.planner_type);
    marker.text += " " + std::to_string(status_.current_path_idx) + "/" +
                   std::to_string(status_.pull_out_path.partial_paths.size() - 1);
    marker.lifetime = life_time;
    planner_type_marker_array.markers.push_back(marker);
    add(planner_type_marker_array);
  }
}

void StartPlannerModule::logPullOutStatus(rclcpp::Logger::Level log_level) const
{
  const auto logger = getLogger();
  auto logFunc = [&logger, log_level](const char * format, ...) {
    char buffer[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    switch (log_level) {
      case rclcpp::Logger::Level::Debug:
        RCLCPP_DEBUG(logger, "%s", buffer);
        break;
      case rclcpp::Logger::Level::Info:
        RCLCPP_INFO(logger, "%s", buffer);
        break;
      case rclcpp::Logger::Level::Warn:
        RCLCPP_WARN(logger, "%s", buffer);
        break;
      case rclcpp::Logger::Level::Error:
        RCLCPP_ERROR(logger, "%s", buffer);
        break;
      case rclcpp::Logger::Level::Fatal:
        RCLCPP_FATAL(logger, "%s", buffer);
        break;
      default:
        RCLCPP_INFO(logger, "%s", buffer);
        break;
    }
  };

  logFunc("======== PullOutStatus Report ========");

  logFunc("[Path Info]");
  logFunc("  Current Path Index: %zu", status_.current_path_idx);

  logFunc("[Planner Info]");
  logFunc("  Planner Type: %s", magic_enum::enum_name(status_.planner_type).data());

  logFunc("[Safety and Direction Info]");
  logFunc("  Found Pull Out Path: %s", status_.found_pull_out_path ? "true" : "false");
  logFunc(
    "  Is Safe Against Dynamic Objects: %s", status_.is_safe_dynamic_objects ? "true" : "false");
  logFunc(
    "  Previous Is Safe Dynamic Objects: %s",
    status_.prev_is_safe_dynamic_objects ? "true" : "false");
  logFunc("  Driving Forward: %s", status_.driving_forward ? "true" : "false");
  logFunc("  Backward Driving Complete: %s", status_.backward_driving_complete ? "true" : "false");
  logFunc("  Has Stop Pose: %s", status_.stop_pose ? "true" : "false");

  logFunc("[Module State]");
  logFunc("  isActivated: %s", isActivated() ? "true" : "false");
  logFunc("  isWaitingForApproval: %s", isWaitingApproval() ? "true" : "false");
  const std::string current_status = magic_enum::enum_name(getCurrentStatus()).data();
  logFunc("  ModuleStatus: %s", current_status.c_str());

  logFunc("=======================================");
}
}  // namespace behavior_path_planner
