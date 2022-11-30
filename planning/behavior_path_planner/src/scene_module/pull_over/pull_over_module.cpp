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

#include "behavior_path_planner/scene_module/pull_over/pull_over_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/pull_over/util.hpp"
#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"
#include "behavior_path_planner/util/create_vehicle_footprint.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using motion_utils::calcSignedArcLength;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using nav_msgs::msg::OccupancyGrid;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::inverseTransformPose;

namespace behavior_path_planner
{
PullOverModule::PullOverModule(
  const std::string & name, rclcpp::Node & node, const PullOverParameters & parameters)
: SceneModuleInterface{name, node},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  rtc_interface_ptr_ = std::make_shared<RTCInterface>(&node, "pull_over");
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, "pull_over");
  goal_pose_pub_ =
    node.create_publisher<PoseStamped>("/planning/scenario_planning/modified_goal", 1);

  LaneDepartureChecker lane_departure_checker{};
  lane_departure_checker.setVehicleInfo(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo());

  occupancy_grid_map_ = std::make_shared<OccupancyGridBasedCollisionDetector>();

  // set enabled planner
  if (parameters_.enable_shift_parking) {
    pull_over_planners_.push_back(std::make_shared<ShiftPullOver>(
      node, parameters, lane_departure_checker, occupancy_grid_map_));
  }
  if (parameters_.enable_arc_forward_parking) {
    constexpr bool is_forward = true;
    pull_over_planners_.push_back(std::make_shared<GeometricPullOver>(
      node, parameters, getGeometricPullOverParameters(), lane_departure_checker,
      occupancy_grid_map_, is_forward));
  }
  if (parameters_.enable_arc_backward_parking) {
    constexpr bool is_forward = false;
    pull_over_planners_.push_back(std::make_shared<GeometricPullOver>(
      node, parameters, getGeometricPullOverParameters(), lane_departure_checker,
      occupancy_grid_map_, is_forward));
  }

  // set selected goal searcher
  // currently there is only one goal_searcher_type
  const auto & vehicle_footprint =
    createVehicleFootprint(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo());
  goal_searcher_ =
    std::make_shared<GoalSearcher>(parameters, vehicle_footprint, occupancy_grid_map_);

  // for collision check with objects
  vehicle_footprint_ = createVehicleFootprint(vehicle_info_);

  resetStatus();
}

void PullOverModule::resetStatus()
{
  PUllOverStatus initial_status{};
  status_ = initial_status;
}

// This function is needed for waiting for planner_data_
void PullOverModule::updateOccupancyGrid()
{
  if (!planner_data_->occupancy_grid) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "occupancy_grid is not ready");
    return;
  }
  occupancy_grid_map_->setMap(*(planner_data_->occupancy_grid));
}

BehaviorModuleOutput PullOverModule::run()
{
  current_state_ = BT::NodeStatus::RUNNING;
  updateOccupancyGrid();
  return plan();
}

ParallelParkingParameters PullOverModule::getGeometricPullOverParameters() const
{
  ParallelParkingParameters params{};

  params.th_arrived_distance = parameters_.th_arrived_distance;
  params.th_stopped_velocity = parameters_.th_stopped_velocity;
  params.after_forward_parking_straight_distance =
    parameters_.after_forward_parking_straight_distance;
  params.after_backward_parking_straight_distance =
    parameters_.after_backward_parking_straight_distance;
  params.forward_parking_velocity = parameters_.forward_parking_velocity;
  params.backward_parking_velocity = parameters_.backward_parking_velocity;
  params.forward_parking_lane_departure_margin = parameters_.forward_parking_lane_departure_margin;
  params.backward_parking_lane_departure_margin =
    parameters_.backward_parking_lane_departure_margin;
  params.arc_path_interval = parameters_.arc_path_interval;
  params.maximum_deceleration = parameters_.maximum_deceleration;
  params.max_steer_angle = parameters_.pull_over_max_steer_angle;

  return params;
}

void PullOverModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OVER onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;

  // Initialize occupancy grid map
  if (parameters_.use_occupancy_grid) {
    OccupancyGridMapParam occupancy_grid_map_param{};
    const double margin = parameters_.occupancy_grid_collision_check_margin;
    occupancy_grid_map_param.vehicle_shape.length =
      planner_data_->parameters.vehicle_length + 2 * margin;
    occupancy_grid_map_param.vehicle_shape.width =
      planner_data_->parameters.vehicle_width + 2 * margin;
    occupancy_grid_map_param.vehicle_shape.base2back =
      planner_data_->parameters.base_link2rear + margin;
    occupancy_grid_map_param.theta_size = parameters_.theta_size;
    occupancy_grid_map_param.obstacle_threshold = parameters_.obstacle_threshold;
    occupancy_grid_map_->setParam(occupancy_grid_map_param);
  }

  // initialize when receiving new route
  if (
    !last_received_time_ ||
    *last_received_time_ != planner_data_->route_handler->getRouteHeader().stamp) {
    // Initialize parallel parking planner status
    parallel_parking_parameters_ = getGeometricPullOverParameters();
    resetStatus();
  }
  last_received_time_ =
    std::make_unique<rclcpp::Time>(planner_data_->route_handler->getRouteHeader().stamp);

  // Use refined goal as modified goal when disabling goal research
  refined_goal_pose_ = calcRefinedGoal();
  if (!parameters_.enable_goal_research) {
    goal_candidates_.clear();
    GoalCandidate goal_candidate{};
    goal_candidate.goal_pose = refined_goal_pose_;
    goal_candidate.distance_from_original_goal = 0.0;
    goal_candidates_.push_back(goal_candidate);
  }
}

void PullOverModule::onExit()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OVER onExit");
  clearWaitingApproval();
  removeRTCStatus();
  steering_factor_interface_ptr_->clearSteeringFactors();

  // A child node must never return IDLE
  current_state_ = BT::NodeStatus::SUCCESS;
}

bool PullOverModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }
  const auto & current_lanes = util::getCurrentLanes(planner_data_);
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & goal_pose = planner_data_->route_handler->getGoalPose();

  // check if goal_pose is far
  const bool is_in_goal_route_section =
    planner_data_->route_handler->isInGoalRouteSection(current_lanes.back());
  // current_lanes does not have the goal
  if (!is_in_goal_route_section) {
    return false;
  }
  const double self_to_goal_arc_length =
    util::getSignedDistance(current_pose, goal_pose, current_lanes);
  if (self_to_goal_arc_length > parameters_.request_length) {
    return false;
  }

  // check if goal_pose is in shoulder lane
  bool goal_is_in_shoulder_lane = false;
  lanelet::Lanelet closest_shoulder_lanelet;
  if (lanelet::utils::query::getClosestLanelet(
        planner_data_->route_handler->getShoulderLanelets(), goal_pose,
        &closest_shoulder_lanelet)) {
    // check if goal pose is in shoulder lane
    if (lanelet::utils::isInLanelet(goal_pose, closest_shoulder_lanelet, 0.1)) {
      const auto lane_yaw =
        lanelet::utils::getLaneletAngle(closest_shoulder_lanelet, goal_pose.position);
      const auto goal_yaw = tf2::getYaw(goal_pose.orientation);
      const auto angle_diff = tier4_autoware_utils::normalizeRadian(lane_yaw - goal_yaw);
      constexpr double th_angle = M_PI / 4;
      if (std::abs(angle_diff) < th_angle) {
        goal_is_in_shoulder_lane = true;
      }
    }
  }
  if (!goal_is_in_shoulder_lane) return false;

  // check if self pose is NOT in shoulder lane
  bool self_is_in_shoulder_lane = false;
  const auto self_pose = planner_data_->self_pose->pose;
  if (lanelet::utils::query::getClosestLanelet(
        planner_data_->route_handler->getShoulderLanelets(), self_pose,
        &closest_shoulder_lanelet)) {
    self_is_in_shoulder_lane =
      lanelet::utils::isInLanelet(self_pose, closest_shoulder_lanelet, 0.1);
  }
  if (self_is_in_shoulder_lane) return false;

  return true;
}

bool PullOverModule::isExecutionReady() const { return true; }

Pose PullOverModule::calcRefinedGoal() const
{
  lanelet::ConstLanelet goal_lane;
  Pose goal_pose = planner_data_->route_handler->getGoalPose();

  lanelet::Lanelet closest_shoulder_lanelet;
  lanelet::utils::query::getClosestLanelet(
    planner_data_->route_handler->getShoulderLanelets(), goal_pose, &closest_shoulder_lanelet);

  const Pose center_pose =
    lanelet::utils::getClosestCenterPose(closest_shoulder_lanelet, goal_pose.position);

  const double distance_to_left_bound = util::getSignedDistanceFromShoulderLeftBoundary(
    planner_data_->route_handler->getShoulderLanelets(), center_pose);
  const double offset_from_center_line = distance_to_left_bound +
                                         planner_data_->parameters.vehicle_width / 2 +
                                         parameters_.margin_from_boundary;
  const Pose refined_goal_pose = calcOffsetPose(center_pose, 0, -offset_from_center_line, 0);

  return refined_goal_pose;
}

BT::NodeStatus PullOverModule::updateState()
{
  // pull_out module will be run when setting new goal, so not need finishing pull_over module.
  // Finishing it causes wrong lane_following path generation.
  // if (hasFinishedPullOver()) {
  //   current_state_ = BT::NodeStatus::SUCCESS;
  //   return current_state_;
  // }

  return current_state_;
}

bool PullOverModule::isLongEnoughToParkingStart(
  const PathWithLaneId & path, const Pose & parking_start_pose) const
{
  const auto dist_to_parking_start_pose = calcSignedArcLength(
    path.points, planner_data_->self_pose->pose, parking_start_pose.position,
    std::numeric_limits<double>::max(), M_PI_2);
  if (!dist_to_parking_start_pose) return false;

  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;
  const double current_to_stop_distance =
    std::pow(current_vel, 2) / parameters_.maximum_deceleration / 2;

  // once stopped, it cannot start again if start_pose is close.
  // so need enough distance to restart
  const double eps_vel = 0.01;
  // dist to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  const double dist_to_restart = parameters_.decide_path_distance / 2;
  if (std::abs(current_vel) < eps_vel && *dist_to_parking_start_pose < dist_to_restart) {
    return false;
  }

  return *dist_to_parking_start_pose > current_to_stop_distance;
}

bool PullOverModule::planWithEfficientPath()
{
  for (const auto & planner : pull_over_planners_) {
    for (const auto & goal_candidate : goal_candidates_) {
      planner->setPlannerData(planner_data_);
      const auto pull_over_path = planner->plan(goal_candidate.goal_pose);
      if (!pull_over_path) {
        continue;
      }
      modified_goal_pose_ = goal_candidate.goal_pose;
      status_.pull_over_path = *pull_over_path;
      status_.planner = planner;
      return true;  // found safe path
    }
  }

  return false;  // not found safe path
}

bool PullOverModule::planWithCloseGoal()
{
  for (const auto & goal_candidate : goal_candidates_) {
    for (const auto & planner : pull_over_planners_) {
      planner->setPlannerData(planner_data_);
      const auto pull_over_path = planner->plan(goal_candidate.goal_pose);
      if (!pull_over_path) {
        continue;
      }
      modified_goal_pose_ = goal_candidate.goal_pose;
      status_.pull_over_path = *pull_over_path;
      status_.planner = planner;
      return true;  // found safe path
    }
  }

  return false;  // not found safe path
}

BehaviorModuleOutput PullOverModule::plan()
{
  const auto & current_pose = planner_data_->self_pose->pose;

  status_.current_lanes = util::getExtendedCurrentLanes(planner_data_);
  status_.pull_over_lanes = pull_over_utils::getPullOverLanes(*(planner_data_->route_handler));
  status_.lanes =
    util::generateDrivableLanesWithShoulderLanes(status_.current_lanes, status_.pull_over_lanes);

  // Check if it needs to decide path
  if (status_.is_safe) {
    const auto dist_to_parking_start_pose = calcSignedArcLength(
      getCurrentPath().points, current_pose, status_.pull_over_path.start_pose.position,
      std::numeric_limits<double>::max(), M_PI_2);

    if (*dist_to_parking_start_pose < parameters_.decide_path_distance) {
      status_.has_decided_path = true;
    }
  }

  // Use decided path
  if (status_.has_decided_path) {
    if (!status_.has_requested_approval) {
      // request approval again one the final path is decided
      waitApproval();
      removeRTCStatus();
      steering_factor_interface_ptr_->clearSteeringFactors();
      uuid_ = generateUUID();
      current_state_ = BT::NodeStatus::SUCCESS;  // for breaking loop
      status_.has_requested_approval = true;
    } else if (isActivated() && isWaitingApproval()) {
      // When it is approved again after path is decided
      clearWaitingApproval();
      last_approved_time_ = std::make_unique<rclcpp::Time>(clock_->now());
      last_approved_pose_ = std::make_unique<Pose>(current_pose);

      // decide velocity to guarantee turn signal lighting time
      if (!status_.has_decided_velocity) {
        auto & first_path = status_.pull_over_path.partial_paths.front();
        const auto vel = static_cast<float>(std::max(
          planner_data_->self_odometry->twist.twist.linear.x,
          parameters_.pull_over_minimum_velocity));
        for (auto & p : first_path.points) {
          p.point.longitudinal_velocity_mps = std::min(p.point.longitudinal_velocity_mps, vel);
        }
      }
      status_.has_decided_velocity = true;
    }

    if (isActivated() && last_approved_time_ != nullptr) {
      // if using arc_path and finishing current_path, get next path
      // enough time for turn signal
      const bool has_passed_enough_time = (clock_->now() - *last_approved_time_).seconds() >
                                          planner_data_->parameters.turn_signal_search_time;

      if (hasFinishedCurrentPath() && has_passed_enough_time) {
        incrementPathIndex();
      }
    }

  } else {  // Replan shift -> arc forward -> arc backward path with each goal candidate.
    // Research goal when enabling research and final path has not been decided
    if (parameters_.enable_goal_research) {
      goal_searcher_->setPlannerData(planner_data_);
      goal_candidates_ = goal_searcher_->search(refined_goal_pose_);
    }

    // plan paths with several goals and planner
    if (parameters_.search_priority == "efficient_path") {
      status_.is_safe = planWithEfficientPath();
    } else if (parameters_.search_priority == "close_goal") {
      status_.is_safe = planWithCloseGoal();
    } else {
      RCLCPP_ERROR(
        getLogger(), "search_priority should be efficient_path or close_goal, but %s is given.",
        parameters_.search_priority.c_str());
      throw std::domain_error("[pull_over] invalid search_priority");
    }

    // Decelerate before the minimum shift distance from the goal search area.
    if (status_.is_safe) {
      auto & first_path = status_.pull_over_path.partial_paths.front();
      const auto arc_coordinates =
        lanelet::utils::getArcCoordinates(status_.current_lanes, refined_goal_pose_);
      const Pose search_start_pose = calcOffsetPose(
        refined_goal_pose_, -parameters_.backward_goal_search_length, -arc_coordinates.distance, 0);
      first_path = util::setDecelerationVelocity(
        first_path, parameters_.pull_over_velocity, search_start_pose,
        -calcMinimumShiftPathDistance(), parameters_.deceleration_interval);
    }
  }

  // generate drivable area for each partial path
  for (size_t i = status_.current_path_idx; i < status_.pull_over_path.partial_paths.size(); ++i) {
    auto & path = status_.pull_over_path.partial_paths.at(i);
    const auto p = planner_data_->parameters;
    const auto lane = util::expandLanelets(
      status_.lanes, parameters_.drivable_area_left_bound_offset,
      parameters_.drivable_area_right_bound_offset);
    path.drivable_area = util::generateDrivableArea(
      path, lane, p.drivable_area_resolution, p.vehicle_length, planner_data_);
  }

  BehaviorModuleOutput output;
  // safe: use pull over path
  if (status_.is_safe) {
    output.path = std::make_shared<PathWithLaneId>(getCurrentPath());
    output.path_candidate = std::make_shared<PathWithLaneId>(getFullPath());
  } else {
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull_over path. Stop in road lane.");
    // safe -> not_safe or no prev_stop_path: generate new stop_path
    if (status_.prev_is_safe || status_.prev_stop_path == nullptr) {
      output.path = std::make_shared<PathWithLaneId>(generateStopPath());
      status_.prev_stop_path = output.path;
      // set stop path as pull over path
      PullOverPath pull_over_path{};
      status_.pull_over_path = pull_over_path;
      status_.pull_over_path.path = *output.path;
      status_.pull_over_path.partial_paths.push_back(*output.path);
    } else {  // not_safe -> not_safe: use previous stop path
      output.path = status_.prev_stop_path;
    }
  }
  status_.prev_is_safe = status_.is_safe;

  // set hazard and turn signal
  if (status_.has_decided_path) {
    output.turn_signal_info = calcTurnSignalInfo();
  }

  const auto distance_to_path_change = calcDistanceToPathChange();
  updateRTCStatus(distance_to_path_change.first, distance_to_path_change.second);

  setDebugData();

  // Publish the modified goal only when its path is safe.
  if (status_.is_safe) {
    PoseStamped goal_pose_stamped;
    goal_pose_stamped.header = planner_data_->route_handler->getRouteHeader();
    goal_pose_stamped.pose = modified_goal_pose_;
    goal_pose_pub_->publish(goal_pose_stamped);
  }

  const uint16_t steering_factor_direction = std::invoke([this]() {
    const auto turn_signal = calcTurnSignalInfo();
    if (turn_signal.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return SteeringFactor::LEFT;
    } else if (turn_signal.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::STRAIGHT;
  });

  // TODO(tkhmy) add handle status TRYING
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.pull_over_path.start_pose, modified_goal_pose_},
    {distance_to_path_change.first, distance_to_path_change.second}, SteeringFactor::PULL_OVER,
    steering_factor_direction, SteeringFactor::TURNING, "");

  // For evaluations
  if (parameters_.print_debug_info) {
    printParkingPositionError();
  }

  return output;
}

// This const function can not change the menber variables like the goal.
// so implement generating candidate path in planWaitingApproval().
// No specific path for the candidate. It's same to the one generated by plan().
CandidateOutput PullOverModule::planCandidate() const { return CandidateOutput{}; }

BehaviorModuleOutput PullOverModule::planWaitingApproval()
{
  updateOccupancyGrid();
  BehaviorModuleOutput out;
  plan();  // update status_
  out.path = std::make_shared<PathWithLaneId>(getReferencePath());
  out.path_candidate = status_.is_safe ? std::make_shared<PathWithLaneId>(getFullPath()) : out.path;

  const auto distance_to_path_change = calcDistanceToPathChange();
  updateRTCStatus(distance_to_path_change.first, distance_to_path_change.second);

  const uint16_t steering_factor_direction = std::invoke([this]() {
    const auto turn_signal = calcTurnSignalInfo();
    if (turn_signal.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
      return SteeringFactor::LEFT;
    } else if (turn_signal.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
      return SteeringFactor::RIGHT;
    }
    return SteeringFactor::STRAIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.pull_over_path.start_pose, modified_goal_pose_},
    {distance_to_path_change.first, distance_to_path_change.second}, SteeringFactor::PULL_OVER,
    steering_factor_direction, SteeringFactor::APPROACHING, "");
  waitApproval();

  return out;
}

std::pair<double, double> PullOverModule::calcDistanceToPathChange() const
{
  const auto & full_path = getFullPath();
  const auto dist_to_parking_start_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_pose->pose, status_.pull_over_path.start_pose.position,
    std::numeric_limits<double>::max(), M_PI_2);
  const double dist_to_parking_finish_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_pose->pose.position, modified_goal_pose_.position);
  const double start_distance_to_path_change =
    dist_to_parking_start_pose ? *dist_to_parking_start_pose : std::numeric_limits<double>::max();
  return {start_distance_to_path_change, dist_to_parking_finish_pose};
}

void PullOverModule::setParameters(const PullOverParameters & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId PullOverModule::getReferencePath() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & common_parameters = planner_data_->parameters;

  if (status_.current_lanes.empty()) {
    return PathWithLaneId{};
  }

  // generate reference path
  const auto s_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto reference_path =
    route_handler->getCenterLinePath(status_.current_lanes, s_start, s_end, true);

  // if not approved, stop parking start position or goal search start position.
  const auto refined_goal_arc_coordinates =
    lanelet::utils::getArcCoordinates(status_.current_lanes, refined_goal_pose_);
  const Pose search_start_pose = calcOffsetPose(
    refined_goal_pose_, -parameters_.backward_goal_search_length,
    -refined_goal_arc_coordinates.distance, 0);
  const Pose stop_pose = status_.is_safe ? status_.pull_over_path.start_pose : search_start_pose;

  // if stop pose is behind current pose, stop as soon as possible
  const size_t ego_idx = findEgoIndex(reference_path.points);
  const size_t stop_idx = findFirstNearestSegmentIndexWithSoftConstraints(
    reference_path.points, stop_pose, common_parameters.ego_nearest_dist_threshold,
    common_parameters.ego_nearest_yaw_threshold);
  const double ego_to_stop_distance = calcSignedArcLength(
    reference_path.points, current_pose.position, ego_idx, stop_pose.position, stop_idx);
  if (ego_to_stop_distance < 0.0) {
    return generateStopPath();
  }

  // slow down for turn signal, insert stop point to stop_pose
  reference_path = util::setDecelerationVelocityForTurnSignal(
    reference_path, stop_pose, planner_data_->parameters.turn_signal_search_time);

  // slow down before the search area.
  reference_path = util::setDecelerationVelocity(
    reference_path, parameters_.pull_over_velocity, search_start_pose,
    -calcMinimumShiftPathDistance(), parameters_.deceleration_interval);

  const auto drivable_lanes = util::generateDrivableLanes(status_.current_lanes);
  const auto lanes = util::expandLanelets(
    drivable_lanes, parameters_.drivable_area_left_bound_offset,
    parameters_.drivable_area_right_bound_offset);

  reference_path.drivable_area = util::generateDrivableArea(
    reference_path, lanes, common_parameters.drivable_area_resolution,
    common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

PathWithLaneId PullOverModule::generateStopPath() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & common_parameters = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  const auto s_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto stop_path = route_handler->getCenterLinePath(status_.current_lanes, s_start, s_end, true);

  // set deceleration velocity
  const size_t ego_idx = findEgoIndex(stop_path.points);
  const double current_to_stop_distance =
    std::pow(current_vel, 2) / parameters_.maximum_deceleration / 2;
  motion_utils::insertStopPoint(current_pose, current_to_stop_distance, stop_path.points);

  for (auto & point : stop_path.points) {
    auto & p = point.point;
    const size_t target_idx = findFirstNearestSegmentIndexWithSoftConstraints(
      stop_path.points, p.pose, common_parameters.ego_nearest_dist_threshold,
      common_parameters.ego_nearest_yaw_threshold);
    const double distance_to_target = calcSignedArcLength(
      stop_path.points, current_pose.position, ego_idx, p.pose.position, target_idx);
    if (0.0 < distance_to_target) {
      p.longitudinal_velocity_mps = std::clamp(
        static_cast<float>(
          current_vel * (current_to_stop_distance - distance_to_target) / current_to_stop_distance),
        0.0f, p.longitudinal_velocity_mps);
    } else {
      p.longitudinal_velocity_mps =
        std::min(p.longitudinal_velocity_mps, static_cast<float>(current_vel));
    }
  }

  const auto drivable_lanes = util::generateDrivableLanes(status_.current_lanes);
  const auto lanes = util::expandLanelets(
    drivable_lanes, parameters_.drivable_area_left_bound_offset,
    parameters_.drivable_area_right_bound_offset);

  stop_path.drivable_area = util::generateDrivableArea(
    stop_path, lanes, common_parameters.drivable_area_resolution, common_parameters.vehicle_length,
    planner_data_);

  return stop_path;
}

void PullOverModule::incrementPathIndex()
{
  status_.current_path_idx =
    std::min(status_.current_path_idx + 1, status_.pull_over_path.partial_paths.size() - 1);
}

PathWithLaneId PullOverModule::getCurrentPath() const
{
  return status_.pull_over_path.partial_paths.at(status_.current_path_idx);
}

PathWithLaneId PullOverModule::getFullPath() const
{
  PathWithLaneId path{};
  const auto & paths = status_.pull_over_path.partial_paths;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (i == 0) {
      path.points.insert(path.points.end(), paths.at(i).points.begin(), paths.at(i).points.end());
    } else {
      // skip overlapping point
      path.points.insert(
        path.points.end(), next(paths.at(i).points.begin()), paths.at(i).points.end());
    }
  }
  path.points = motion_utils::removeOverlapPoints(path.points);

  return path;
}

double PullOverModule::calcMinimumShiftPathDistance() const
{
  const double maximum_jerk = parameters_.maximum_lateral_jerk;
  const double pull_over_velocity = parameters_.pull_over_velocity;
  const auto current_pose = planner_data_->self_pose->pose;
  const double distance_after_pull_over = parameters_.after_pull_over_straight_distance;
  const double distance_before_pull_over = parameters_.before_pull_over_straight_distance;
  const auto & route_handler = planner_data_->route_handler;

  double distance_to_left_bound = util::getSignedDistanceFromShoulderLeftBoundary(
    route_handler->getShoulderLanelets(), current_pose);
  double offset_from_center_line = distance_to_left_bound +
                                   planner_data_->parameters.vehicle_width / 2 +
                                   parameters_.margin_from_boundary;

  // calculate minimum pull over distance at pull over velocity, maximum jerk and side offset
  const double pull_over_distance_min = PathShifter::calcLongitudinalDistFromJerk(
    abs(offset_from_center_line), maximum_jerk, pull_over_velocity);
  const double pull_over_total_distance_min =
    distance_after_pull_over + pull_over_distance_min + distance_before_pull_over;

  return pull_over_total_distance_min;
}

bool PullOverModule::isLongEnough(
  const lanelet::ConstLanelets & lanelets, const Pose & goal_pose, const double buffer) const
{
  const auto current_pose = planner_data_->self_pose->pose;
  const double distance_to_goal =
    std::abs(util::getSignedDistance(current_pose, goal_pose, lanelets));

  return distance_to_goal > calcMinimumShiftPathDistance() + buffer;
}

bool PullOverModule::isStopped()
{
  odometry_buffer_.push_back(planner_data_->self_odometry);
  // Delete old data in buffer
  while (rclcpp::ok()) {
    const auto time_diff = rclcpp::Time(odometry_buffer_.back()->header.stamp) -
                           rclcpp::Time(odometry_buffer_.front()->header.stamp);
    if (time_diff.seconds() < parameters_.th_stopped_time) {
      break;
    }
    odometry_buffer_.pop_front();
  }
  bool is_stopped = true;
  for (const auto & odometry : odometry_buffer_) {
    const double ego_vel = util::l2Norm(odometry->twist.twist.linear);
    if (ego_vel > parameters_.th_stopped_velocity) {
      is_stopped = false;
      break;
    }
  }
  return is_stopped;
}

bool PullOverModule::hasFinishedCurrentPath()
{
  const auto & current_path_end = getCurrentPath().points.back();
  const auto & self_pose = planner_data_->self_pose->pose;
  const bool is_near_target = tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
                              parameters_.th_arrived_distance;

  return is_near_target && isStopped();
}

bool PullOverModule::hasFinishedPullOver()
{
  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_pose->pose;
  const bool car_is_on_goal =
    calcDistance2d(current_pose, modified_goal_pose_) < parameters_.th_arrived_distance;

  return car_is_on_goal && isStopped();
}

TurnSignalInfo PullOverModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal{};  // output

  const auto & current_pose = planner_data_->self_pose->pose;
  const auto & start_pose = status_.pull_over_path.start_pose;
  const auto & end_pose = status_.pull_over_path.end_pose;
  const auto & full_path = getFullPath();

  // calc TurnIndicatorsCommand
  {
    const double distance_to_end =
      calcSignedArcLength(full_path.points, current_pose.position, end_pose.position);
    const bool is_before_end_pose = distance_to_end >= 0.0;
    turn_signal.turn_signal.command =
      is_before_end_pose ? TurnIndicatorsCommand::ENABLE_LEFT : TurnIndicatorsCommand::NO_COMMAND;
  }

  // calc desired/required start/end point
  {
    // ego decelerates so that current pose is the point `turn_light_on_threshold_time` seconds
    // before starting pull_over
    turn_signal.desired_start_point =
      last_approved_pose_ && status_.has_decided_path ? *last_approved_pose_ : current_pose;
    turn_signal.desired_end_point = end_pose;
    turn_signal.required_start_point = start_pose;
    turn_signal.required_end_point = end_pose;
  }

  return turn_signal;
}

void PullOverModule::setDebugData()
{
  debug_marker_.markers.clear();

  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using tier4_autoware_utils::createMarkerColor;

  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  if (parameters_.enable_goal_research) {
    // Visualize pull over areas
    const Pose start_pose =
      calcOffsetPose(refined_goal_pose_, -parameters_.backward_goal_search_length, 0, 0);
    const Pose end_pose =
      calcOffsetPose(refined_goal_pose_, parameters_.forward_goal_search_length, 0, 0);
    const auto header = planner_data_->route_handler->getRouteHeader();
    const auto color = status_.has_decided_path ? createMarkerColor(1.0, 1.0, 0.0, 0.999)  // yellow
                                                : createMarkerColor(0.0, 1.0, 0.0, 0.999);  // green
    const auto p = planner_data_->parameters;
    debug_marker_.markers.push_back(pull_over_utils::createPullOverAreaMarker(
      start_pose, end_pose, 0, header, p.base_link2front, p.base_link2rear, p.vehicle_width,
      color));

    // Visualize goal candidates
    add(pull_over_utils::createGoalCandidatesMarkerArray(goal_candidates_, color));
  }

  // Visualize path and related pose
  if (status_.is_safe) {
    add(createPoseMarkerArray(
      status_.pull_over_path.start_pose, "pull_over_start_pose", 0, 0.3, 0.3, 0.9));
    add(createPoseMarkerArray(
      status_.pull_over_path.end_pose, "pull_over_end_pose", 0, 0.9, 0.9, 0.3));
    add(createPathMarkerArray(getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
  }
}

void PullOverModule::printParkingPositionError() const
{
  const auto current_pose = planner_data_->self_pose->pose;
  const double real_shoulder_to_map_shoulder = 0.0;

  const Pose goal_to_ego = inverseTransformPose(current_pose, modified_goal_pose_);
  const double dx = goal_to_ego.position.x;
  const double dy = goal_to_ego.position.y;
  const double distance_from_real_shoulder =
    real_shoulder_to_map_shoulder + parameters_.margin_from_boundary - dy;
  RCLCPP_INFO(
    getLogger(), "current pose to goal, dx:%f dy:%f dyaw:%f from_real_shoulder:%f", dx, dy,
    tier4_autoware_utils::rad2deg(
      tf2::getYaw(current_pose.orientation) - tf2::getYaw(modified_goal_pose_.orientation)),
    distance_from_real_shoulder);
}
}  // namespace behavior_path_planner
