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

#include "behavior_path_planner/scene_module/start_planner/start_planner_module.hpp"

#include "behavior_path_planner/utils/create_vehicle_footprint.hpp"
#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/start_goal_planner_common/utils.hpp"
#include "behavior_path_planner/utils/start_planner/util.hpp"
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

using motion_utils::calcLongitudinalOffsetPose;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::inverseTransformPoint;

namespace behavior_path_planner
{
StartPlannerModule::StartPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<StartPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
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
  if (!isActivated()) {
    return planWaitingApproval();
  }

  return plan();
}

void StartPlannerModule::processOnEntry()
{
  // Initialize safety checker
  if (parameters_->safety_check_params.enable_safety_check) {
    initializeSafetyCheckParameters();
    utils::start_goal_planner_common::initializeCollisionCheckDebugMap(
      start_planner_data_.collision_check);
  }
}

void StartPlannerModule::processOnExit()
{
  resetPathCandidate();
  resetPathReference();
  debug_marker_.markers.clear();
}

bool StartPlannerModule::isExecutionRequested() const
{
  // TODO(Sugahara): if required lateral shift distance is small, don't engage this module.
  // Execute when current pose is near route start pose
  const Pose start_pose = planner_data_->route_handler->getOriginalStartPose();
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  if (
    tier4_autoware_utils::calcDistance2d(start_pose.position, current_pose.position) >
    parameters_->th_arrived_distance) {
    return false;
  }

  // Check if ego arrives at goal
  const Pose goal_pose = planner_data_->route_handler->getGoalPose();
  if (
    tier4_autoware_utils::calcDistance2d(goal_pose.position, current_pose.position) <
    parameters_->th_arrived_distance) {
    return false;
  }

  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  const bool is_stopped = utils::l2Norm(planner_data_->self_odometry->twist.twist.linear) <
                          parameters_->th_stopped_velocity;
  if (!is_stopped) {
    return false;
  }

  return true;
}

bool StartPlannerModule::isExecutionReady() const
{
  if (!status_.is_safe_static_objects) {
    return false;
  }

  if (status_.pull_out_path.partial_paths.empty()) {
    return true;
  }

  if (status_.is_safe_static_objects && parameters_->safety_check_params.enable_safety_check) {
    if (!isSafePath()) {
      RCLCPP_ERROR_THROTTLE(getLogger(), *clock_, 5000, "Path is not safe against dynamic objects");
      return false;
    }
  }
  return true;
}

ModuleStatus StartPlannerModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "START_PLANNER updateState");

  if (isActivated() && !isWaitingApproval()) {
    current_state_ = ModuleStatus::RUNNING;
  } else {
    current_state_ = ModuleStatus::IDLE;
  }

  if (hasFinishedPullOut()) {
    return ModuleStatus::SUCCESS;
  }

  checkBackFinished();

  return current_state_;
}

BehaviorModuleOutput StartPlannerModule::plan()
{
  if (IsGoalBehindOfEgoInSameRouteSegment()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Start plan for a backward goal is not supported now");
    const auto output = generateStopOutput();
    setDebugData();  // use status updated in generateStopOutput()
    updateRTCStatus(0, 0);
    return output;
  }

  if (isWaitingApproval()) {
    clearWaitingApproval();
    resetPathCandidate();
    resetPathReference();
    // save current_pose when approved for start_point of turn_signal for backward driving
    last_approved_pose_ = std::make_unique<Pose>(planner_data_->self_odometry->pose.pose);
  }

  BehaviorModuleOutput output;
  if (!status_.is_safe_static_objects) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull out path, publish stop path");
    const auto output = generateStopOutput();
    setDebugData();  // use status updated in generateStopOutput()
    updateRTCStatus(0, 0);
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

  output.path = std::make_shared<PathWithLaneId>(path);
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = getPreviousModuleOutput().reference_path;

  setDrivableAreaInfo(output);

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
      {start_distance, finish_distance}, SteeringFactor::START_PLANNER, steering_factor_direction,
      SteeringFactor::TURNING, "");
  } else {
    const double distance = motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    updateRTCStatus(0.0, distance);
    // TODO(tkhmy) add handle status TRYING
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
      SteeringFactor::START_PLANNER, steering_factor_direction, SteeringFactor::TURNING, "");
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
  utils::start_goal_planner_common::updateEgoPredictedPathParams(
    ego_predicted_path_params_, parameters_);
  utils::start_goal_planner_common::updateSafetyCheckParams(safety_check_params_, parameters_);
  utils::start_goal_planner_common::updateObjectsFilteringParams(
    objects_filtering_params_, parameters_);
}

PathWithLaneId StartPlannerModule::getFullPath() const
{
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

BehaviorModuleOutput StartPlannerModule::planWaitingApproval()
{
  updatePullOutStatus();

  if (IsGoalBehindOfEgoInSameRouteSegment()) {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Start plan for a backward goal is not supported now");
    clearWaitingApproval();
    const auto output = generateStopOutput();
    setDebugData();  // use status updated in generateStopOutput()
    updateRTCStatus(0, 0);
    return output;
  }

  BehaviorModuleOutput output;
  if (!status_.is_safe_static_objects) {
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

  auto stop_path = status_.back_finished ? getCurrentPath() : status_.backward_path;
  const auto drivable_lanes = generateDrivableLanes(stop_path);
  const auto & dp = planner_data_->drivable_area_expansion_parameters;
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);
  for (auto & p : stop_path.points) {
    p.point.longitudinal_velocity_mps = 0.0;
  }

  output.path = std::make_shared<PathWithLaneId>(stop_path);
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = getPreviousModuleOutput().reference_path;

  setDrivableAreaInfo(output);

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
      {start_distance, finish_distance}, SteeringFactor::START_PLANNER, steering_factor_direction,
      SteeringFactor::APPROACHING, "");
  } else {
    const double distance = motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    updateRTCStatus(0.0, distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
      SteeringFactor::START_PLANNER, steering_factor_direction, SteeringFactor::APPROACHING, "");
  }

  setDebugData();

  return output;
}

void StartPlannerModule::resetStatus()
{
  PullOutStatus initial_status;
  status_ = initial_status;
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
  // check if start pose candidates are valid
  if (start_pose_candidates.empty()) {
    return;
  }

  const auto is_safe_with_pose_planner = [&](const size_t i, const auto & planner) {
    // Get the pull_out_start_pose for the current index
    const auto & pull_out_start_pose = start_pose_candidates.at(i);

    // Set back_finished to true if the current start pose is same to refined_start_pose
    status_.back_finished =
      tier4_autoware_utils::calcDistance2d(pull_out_start_pose, refined_start_pose) < 0.01;

    planner->setPlannerData(planner_data_);
    const auto pull_out_path = planner->plan(pull_out_start_pose, goal_pose);
    // not found safe path
    if (!pull_out_path) {
      return false;
    }
    // use current path if back is not needed
    if (status_.back_finished) {
      const std::lock_guard<std::mutex> lock(mutex_);
      status_.is_safe_static_objects = true;
      status_.pull_out_path = *pull_out_path;
      status_.pull_out_start_pose = pull_out_start_pose;
      status_.planner_type = planner->getPlannerType();
      return true;
    }

    // If this is the last start pose candidate, return false
    if (i == start_pose_candidates.size() - 1) return false;

    // check next path if back is needed
    const auto & pull_out_start_pose_next = start_pose_candidates.at(i + 1);
    const auto pull_out_path_next = planner->plan(pull_out_start_pose_next, goal_pose);
    // not found safe path
    if (!pull_out_path_next) {
      return false;
    }

    // Update status variables with the next path information
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      status_.is_safe_static_objects = true;
      status_.pull_out_path = *pull_out_path_next;
      status_.pull_out_start_pose = pull_out_start_pose_next;
      status_.planner_type = planner->getPlannerType();
    }
    return true;
  };

  using PriorityOrder = std::vector<std::pair<size_t, std::shared_ptr<PullOutPlannerBase>>>;
  const auto make_loop_order_planner_first = [&]() {
    PriorityOrder order_priority;
    for (const auto & planner : start_planners_) {
      for (size_t i = 0; i < start_pose_candidates.size(); i++) {
        order_priority.emplace_back(i, planner);
      }
    }
    return order_priority;
  };

  const auto make_loop_order_pose_first = [&]() {
    PriorityOrder order_priority;
    for (size_t i = 0; i < start_pose_candidates.size(); i++) {
      for (const auto & planner : start_planners_) {
        order_priority.emplace_back(i, planner);
      }
    }
    return order_priority;
  };

  // Choose loop order based on priority_on_efficient_path
  PriorityOrder order_priority;
  if (search_priority == "efficient_path") {
    order_priority = make_loop_order_planner_first();
  } else if (search_priority == "short_back_distance") {
    order_priority = make_loop_order_pose_first();
  } else {
    RCLCPP_ERROR(
      getLogger(),
      "search_priority should be efficient_path or short_back_distance, but %s is given.",
      search_priority.c_str());
    throw std::domain_error("[start_planner] invalid search_priority");
  }

  for (const auto & p : order_priority) {
    if (is_safe_with_pose_planner(p.first, p.second)) {
      return;
    }
  }

  // not found safe path
  if (status_.planner_type != PlannerType::FREESPACE) {
    const std::lock_guard<std::mutex> lock(mutex_);
    status_.is_safe_static_objects = false;
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
    lanelet::Lanelet closest_lanelet;
    lanelet::utils::query::getClosestLanelet(status_.pull_out_lanes, pose, &closest_lanelet);
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
    if (id != lanelet::InvalId) {
      path_lanes.push_back(lanelet_layer.get(id));
    }
  }

  return path_lanes;
}

std::vector<DrivableLanes> StartPlannerModule::generateDrivableLanes(
  const PathWithLaneId & path) const
{
  const auto path_road_lanes = getPathRoadLanes(path);
  if (!path_road_lanes.empty()) {
    lanelet::ConstLanelets shoulder_lanes;
    const auto & rh = planner_data_->route_handler;
    std::copy_if(
      status_.pull_out_lanes.begin(), status_.pull_out_lanes.end(),
      std::back_inserter(shoulder_lanes),
      [&rh](const auto & pull_out_lane) { return rh->isShoulderLanelet(pull_out_lane); });

    return utils::generateDrivableLanesWithShoulderLanes(path_road_lanes, shoulder_lanes);
  }

  // if path_road_lanes is empty, use only pull_out_lanes as drivable lanes
  std::vector<DrivableLanes> drivable_lanes;
  for (const auto & lane : status_.pull_out_lanes) {
    DrivableLanes drivable_lane;
    drivable_lane.right_lane = lane;
    drivable_lane.left_lane = lane;
    drivable_lanes.push_back(drivable_lane);
  }
  return drivable_lanes;
}

void StartPlannerModule::updatePullOutStatus()
{
  const bool has_received_new_route =
    !planner_data_->prev_route_id ||
    *planner_data_->prev_route_id != planner_data_->route_handler->getRouteUuid();

  if (has_received_new_route) {
    status_ = PullOutStatus();
  }

  // save pull out lanes which is generated using current pose before starting pull out
  // (before approval)
  status_.pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);

  // skip updating if enough time has not passed for preventing chattering between back and
  // start_planner
  if (!has_received_new_route) {
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
  const PathWithLaneId start_pose_candidates_path = calcStartPoseCandidatesBackwardPath();
  const auto refined_start_pose = calcLongitudinalOffsetPose(
    start_pose_candidates_path.points, planner_data_->self_odometry->pose.pose.position, 0.0);
  if (!refined_start_pose) return;

  // search pull out start candidates backward
  const std::vector<Pose> start_pose_candidates = std::invoke([&]() -> std::vector<Pose> {
    if (parameters_->enable_back) {
      return searchPullOutStartPoses(start_pose_candidates_path);
    }
    return {*refined_start_pose};
  });

  planWithPriority(
    start_pose_candidates, *refined_start_pose, goal_pose, parameters_->search_priority);

  checkBackFinished();
  if (!status_.back_finished) {
    status_.backward_path = start_planner_utils::getBackwardPath(
      *route_handler, status_.pull_out_lanes, current_pose, status_.pull_out_start_pose,
      parameters_->backward_velocity);
  }
}

PathWithLaneId StartPlannerModule::calcStartPoseCandidatesBackwardPath() const
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  // get backward shoulder path
  const auto arc_position_pose =
    lanelet::utils::getArcCoordinates(status_.pull_out_lanes, current_pose);
  const double check_distance = parameters_->max_back_distance + 30.0;  // buffer
  auto path = planner_data_->route_handler->getCenterLinePath(
    status_.pull_out_lanes, arc_position_pose.length - check_distance,
    arc_position_pose.length + check_distance);

  // lateral shift to current_pose
  const double distance_from_center_line = arc_position_pose.distance;
  for (auto & p : path.points) {
    p.point.pose = calcOffsetPose(p.point.pose, 0, distance_from_center_line, 0);
  }

  return path;
}

std::vector<Pose> StartPlannerModule::searchPullOutStartPoses(
  const PathWithLaneId & start_pose_candidates) const
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  std::vector<Pose> pull_out_start_pose{};

  // filter pull out lanes stop objects
  const auto [pull_out_lane_objects, others] =
    utils::path_safety_checker::separateObjectsByLanelets(
      *planner_data_->dynamic_object, status_.pull_out_lanes);
  const auto pull_out_lane_stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
    pull_out_lane_objects, parameters_->th_moving_object_velocity);

  // Set the maximum backward distance less than the distance from the vehicle's base_link to the
  // lane's rearmost point to prevent lane departure.
  const double s_current =
    lanelet::utils::getArcCoordinates(status_.pull_out_lanes, current_pose).length;
  const double max_back_distance = std::clamp(
    s_current - planner_data_->parameters.base_link2rear, 0.0, parameters_->max_back_distance);

  // check collision between footprint and object at the backed pose
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info_);
  for (double back_distance = 0.0; back_distance <= max_back_distance;
       back_distance += parameters_->backward_search_resolution) {
    const auto backed_pose = calcLongitudinalOffsetPose(
      start_pose_candidates.points, current_pose.position, -back_distance);
    if (!backed_pose) {
      continue;
    }

    // check the back pose is near the lane end
    const double length_to_backed_pose =
      lanelet::utils::getArcCoordinates(status_.pull_out_lanes, *backed_pose).length;

    const double length_to_lane_end = std::accumulate(
      std::begin(status_.pull_out_lanes), std::end(status_.pull_out_lanes), 0.0,
      [](double acc, const auto & lane) { return acc + lanelet::utils::getLaneletLength2d(lane); });
    const double distance_from_lane_end = length_to_lane_end - length_to_backed_pose;
    if (distance_from_lane_end < parameters_->ignore_distance_from_lane_end) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000,
        "the ego is too close to the lane end, so needs backward driving");
      continue;
    }

    if (utils::checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, *backed_pose, pull_out_lane_stop_objects,
          parameters_->collision_check_margin)) {
      break;  // poses behind this has a collision, so break.
    }

    pull_out_start_pose.push_back(*backed_pose);
  }
  return pull_out_start_pose;
}

bool StartPlannerModule::isOverlappedWithLane(
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

bool StartPlannerModule::hasFinishedPullOut() const
{
  if (!status_.back_finished || !status_.is_safe_static_objects) {
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

void StartPlannerModule::checkBackFinished()
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

    // request start_planner approval
    waitApproval();
    removeRTCStatus();
    for (auto itr = uuid_map_.begin(); itr != uuid_map_.end(); ++itr) {
      itr->second = generateUUID();
    }
    current_state_ = ModuleStatus::SUCCESS;  // for breaking loop
  }
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

bool StartPlannerModule::isStuck()
{
  if (!isStopped()) {
    return false;
  }

  if (status_.planner_type == PlannerType::STOP) {
    return true;
  }

  // not found safe path
  if (!status_.is_safe_static_objects) {
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
  if (!status_.back_finished) {
    turn_signal.hazard_signal.command = HazardLightsCommand::ENABLE;
    const auto back_start_pose = isWaitingApproval() ? current_pose : *last_approved_pose_;
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
    utils::start_goal_planner_common::getPairsTerminalVelocityAndAccel(
      status_.pull_out_path.pairs_terminal_velocity_and_accel, status_.current_path_idx);
  RCLCPP_DEBUG(
    getLogger(), "pairs_terminal_velocity_and_accel for start_planner: %f, %f",
    terminal_velocity_and_accel.first, terminal_velocity_and_accel.second);
  RCLCPP_DEBUG(getLogger(), "current_path_idx %ld", status_.current_path_idx);
  utils::start_goal_planner_common::updatePathProperty(
    ego_predicted_path_params_, terminal_velocity_and_accel);
  const auto ego_predicted_path =
    behavior_path_planner::utils::path_safety_checker::createPredictedPath(
      ego_predicted_path_params_, pull_out_path.points, current_pose, current_velocity,
      ego_seg_idx);

  // filtering objects with velocity, position and class
  const auto & filtered_objects = utils::path_safety_checker::filterObjects(
    dynamic_object, route_handler, current_lanes, current_pose.position, objects_filtering_params_);

  // filtering objects based on the current position's lane
  const auto & target_objects_on_lane = utils::path_safety_checker::createTargetObjectsOnLane(
    current_lanes, route_handler, filtered_objects, objects_filtering_params_);

  const double hysteresis_factor =
    status_.is_safe_dynamic_objects ? 1.0 : safety_check_params_->hysteresis_factor_expand_rate;

  utils::start_goal_planner_common::updateSafetyCheckTargetObjectsData(
    start_planner_data_, filtered_objects, target_objects_on_lane, ego_predicted_path);

  bool is_safe_dynamic_objects = true;
  // Check for collisions with each predicted path of the object
  for (const auto & object : target_objects_on_lane.on_current_lane) {
    auto current_debug_data = marker_utils::createObjectDebug(object);

    bool is_safe_dynamic_object = true;

    const auto obj_predicted_paths = utils::path_safety_checker::getPredictedPathFromObj(
      object, objects_filtering_params_->check_all_predicted_path);

    // If a collision is detected, mark the object as unsafe and break the loop
    for (const auto & obj_path : obj_predicted_paths) {
      if (!utils::path_safety_checker::checkCollision(
            pull_out_path, ego_predicted_path, object, obj_path, planner_data_->parameters,
            safety_check_params_->rss_params, hysteresis_factor, current_debug_data.second)) {
        marker_utils::updateCollisionCheckDebugMap(
          start_planner_data_.collision_check, current_debug_data, false);
        is_safe_dynamic_objects = false;
        is_safe_dynamic_object = false;
        break;
      }
    }
    if (is_safe_dynamic_object) {
      marker_utils::updateCollisionCheckDebugMap(
        start_planner_data_.collision_check, current_debug_data, is_safe_dynamic_object);
    }
  }

  return is_safe_dynamic_objects;
}

bool StartPlannerModule::IsGoalBehindOfEgoInSameRouteSegment() const
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
  output.path = std::make_shared<PathWithLaneId>(stop_path);

  setDrivableAreaInfo(output);

  output.reference_path = getPreviousModuleOutput().reference_path;

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    status_.back_finished = true;
    status_.planner_type = PlannerType::STOP;
    status_.pull_out_path.partial_paths.clear();
    status_.pull_out_path.partial_paths.push_back(stop_path);
    const Pose & current_pose = planner_data_->self_odometry->pose.pose;
    status_.pull_out_start_pose = current_pose;
    status_.pull_out_path.start_pose = current_pose;
    status_.pull_out_path.end_pose = current_pose;
  }

  path_candidate_ = std::make_shared<PathWithLaneId>(stop_path);
  path_reference_ = getPreviousModuleOutput().reference_path;

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
    status_.is_safe_static_objects = true;
    status_.back_finished = true;
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
      *output.path, generateDrivableLanes(*output.path),
      planner_data_->drivable_area_expansion_parameters);

    DrivableAreaInfo current_drivable_area_info;
    current_drivable_area_info.drivable_lanes = target_drivable_lanes;
    output.drivable_area_info = utils::combineDrivableAreaInfo(
      current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  }
}

void StartPlannerModule::setDebugData() const
{
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
  add(createPathMarkerArray(getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
  if (start_planner_data_.ego_predicted_path.size() > 0) {
    const auto & ego_predicted_path = utils::path_safety_checker::convertToPredictedPath(
      start_planner_data_.ego_predicted_path, ego_predicted_path_params_->time_resolution);
    add(createPredictedPathMarkerArray(
      ego_predicted_path, vehicle_info_, "ego_predicted_path", 0, 0.0, 0.5, 0.9));
  }

  if (start_planner_data_.filtered_objects.objects.size() > 0) {
    add(createObjectsMarkerArray(
      start_planner_data_.filtered_objects, "filtered_objects", 0, 0.0, 0.5, 0.9));
  }

  // safety check
  {
    add(showSafetyCheckInfo(start_planner_data_.collision_check, "object_debug_info"));
    add(showPredictedPath(start_planner_data_.collision_check, "ego_predicted_path"));
    add(showPolygon(start_planner_data_.collision_check, "ego_and_target_polygon_relation"));
  }

  if (start_planner_data_.ego_predicted_path.size() > 0) {
    const auto & ego_predicted_path = utils::path_safety_checker::convertToPredictedPath(
      start_planner_data_.ego_predicted_path, ego_predicted_path_params_->time_resolution);
    add(createPredictedPathMarkerArray(
      ego_predicted_path, vehicle_info_, "ego_predicted_path", 0, 0.0, 0.5, 0.9));
  }

  if (start_planner_data_.filtered_objects.objects.size() > 0) {
    add(createObjectsMarkerArray(
      start_planner_data_.filtered_objects, "filtered_objects", 0, 0.0, 0.5, 0.9));
  }

  // Visualize planner type text
  const auto header = planner_data_->route_handler->getRouteHeader();
  {
    visualization_msgs::msg::MarkerArray planner_type_marker_array{};
    const auto color = status_.is_safe_static_objects ? createMarkerColor(1.0, 1.0, 1.0, 0.99)
                                                      : createMarkerColor(1.0, 0.0, 0.0, 0.99);
    auto marker = createDefaultMarker(
      header.frame_id, header.stamp, "planner_type", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0), color);
    marker.pose = status_.pull_out_start_pose;
    if (!status_.back_finished) {
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
}  // namespace behavior_path_planner
