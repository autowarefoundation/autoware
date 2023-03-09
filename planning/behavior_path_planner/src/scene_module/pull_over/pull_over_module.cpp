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
#include "behavior_path_planner/util/create_vehicle_footprint.hpp"
#include "behavior_path_planner/util/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/util/pull_over/util.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>
#include <motion_utils/motion_utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using nav_msgs::msg::OccupancyGrid;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::inverseTransformPose;

namespace behavior_path_planner
{
#ifdef USE_OLD_ARCHITECTURE
PullOverModule::PullOverModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<PullOverParameters> & parameters)
: SceneModuleInterface{name, node},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  rtc_interface_ptr_ = std::make_shared<RTCInterface>(&node, "pull_over");
#else
PullOverModule::PullOverModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<PullOverParameters> & parameters,
  const std::shared_ptr<RTCInterface> & rtc_interface)
: SceneModuleInterface{name, node},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()}
{
  rtc_interface_ptr_ = rtc_interface;
#endif
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, "pull_over");

  LaneDepartureChecker lane_departure_checker{};
  lane_departure_checker.setVehicleInfo(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo());

  occupancy_grid_map_ = std::make_shared<OccupancyGridBasedCollisionDetector>();

  // set enabled planner
  if (parameters_->enable_shift_parking) {
    pull_over_planners_.push_back(std::make_shared<ShiftPullOver>(
      node, *parameters, lane_departure_checker, occupancy_grid_map_));
  }
  if (parameters_->enable_arc_forward_parking) {
    constexpr bool is_forward = true;
    pull_over_planners_.push_back(std::make_shared<GeometricPullOver>(
      node, *parameters, getGeometricPullOverParameters(), lane_departure_checker,
      occupancy_grid_map_, is_forward));
  }
  if (parameters_->enable_arc_backward_parking) {
    constexpr bool is_forward = false;
    pull_over_planners_.push_back(std::make_shared<GeometricPullOver>(
      node, *parameters, getGeometricPullOverParameters(), lane_departure_checker,
      occupancy_grid_map_, is_forward));
  }
  if (pull_over_planners_.empty()) {
    RCLCPP_ERROR(getLogger(), "Not found enabled planner");
  }

  // set selected goal searcher
  // currently there is only one goal_searcher_type
  const auto & vehicle_footprint =
    createVehicleFootprint(vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo());
  goal_searcher_ =
    std::make_shared<GoalSearcher>(*parameters, vehicle_footprint, occupancy_grid_map_);

  // for collision check with objects
  vehicle_footprint_ = createVehicleFootprint(vehicle_info_);

  // timer callback for generating lane parking candidate paths
  const auto lane_parking_period_ns = rclcpp::Rate(1.0).period();
  lane_parking_timer_cb_group_ =
    node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lane_parking_timer_ = rclcpp::create_timer(
    &node, clock_, lane_parking_period_ns, std::bind(&PullOverModule::onTimer, this),
    lane_parking_timer_cb_group_);

  // freespace parking
  if (parameters_->enable_freespace_parking) {
    freespace_planner_ = std::make_unique<FreespacePullOver>(node, *parameters, vehicle_info_);
    const auto freespace_parking_period_ns = rclcpp::Rate(1.0).period();
    freespace_parking_timer_cb_group_ =
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    freespace_parking_timer_ = rclcpp::create_timer(
      &node, clock_, freespace_parking_period_ns,
      std::bind(&PullOverModule::onFreespaceParkingTimer, this), freespace_parking_timer_cb_group_);
  }

  resetStatus();
}

void PullOverModule::resetStatus()
{
  PUllOverStatus initial_status{};
  status_ = initial_status;
  pull_over_path_candidates_.clear();
  closest_start_pose_.reset();
  goal_candidates_.clear();
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

// generate pull over candidate paths
void PullOverModule::onTimer()
{
  // already generated pull over candidate paths
  if (!pull_over_path_candidates_.empty()) {
    return;
  }

  // goals are not yet available.
  if (goal_candidates_.empty()) {
    return;
  }
  mutex_.lock();
  const auto goal_candidates = goal_candidates_;
  mutex_.unlock();

  // generate valid pull over path candidates and calculate closest start pose
  const auto current_lanes = util::getExtendedCurrentLanes(planner_data_);
  std::vector<PullOverPath> path_candidates{};
  std::optional<Pose> closest_start_pose{};
  double min_start_arc_length = std::numeric_limits<double>::max();
  const auto planCandidatePaths = [&](
                                    const std::shared_ptr<PullOverPlannerBase> & planner,
                                    const GoalCandidate & goal_candidate) {
    planner->setPlannerData(planner_data_);
    auto pull_over_path = planner->plan(goal_candidate.goal_pose);
    pull_over_path->goal_id = goal_candidate.id;
    if (pull_over_path) {
      path_candidates.push_back(*pull_over_path);
      // calculate closest pull over start pose for stop path
      const double start_arc_length =
        lanelet::utils::getArcCoordinates(current_lanes, pull_over_path->start_pose).length;
      if (start_arc_length < min_start_arc_length) {
        min_start_arc_length = start_arc_length;
        // closest start pose is stop point when not finding safe path
        closest_start_pose = pull_over_path->start_pose;
      }
    }
  };

  // plan candidate paths and set them to the member variable
  if (parameters_->search_priority == "efficient_path") {
    for (const auto & planner : pull_over_planners_) {
      for (const auto & goal_candidate : goal_candidates) {
        planCandidatePaths(planner, goal_candidate);
      }
    }
  } else if (parameters_->search_priority == "close_goal") {
    for (const auto & goal_candidate : goal_candidates) {
      for (const auto & planner : pull_over_planners_) {
        planCandidatePaths(planner, goal_candidate);
      }
    }
  } else {
    RCLCPP_ERROR(
      getLogger(), "search_priority should be efficient_path or close_goal, but %s is given.",
      parameters_->search_priority.c_str());
    throw std::domain_error("[pull_over] invalid search_priority");
  }

  // set member variables
  mutex_.lock();
  pull_over_path_candidates_ = path_candidates;
  closest_start_pose_ = closest_start_pose;
  mutex_.unlock();
}

void PullOverModule::onFreespaceParkingTimer()
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

BehaviorModuleOutput PullOverModule::run()
{
  current_state_ = ModuleStatus::RUNNING;
  updateOccupancyGrid();

#ifndef USE_OLD_ARCHITECTURE
  if (!isActivated()) {
    return planWaitingApproval();
  }
#endif

  return plan();
}

ParallelParkingParameters PullOverModule::getGeometricPullOverParameters() const
{
  ParallelParkingParameters params{};

  params.th_arrived_distance = parameters_->th_arrived_distance;
  params.th_stopped_velocity = parameters_->th_stopped_velocity;
  params.after_forward_parking_straight_distance =
    parameters_->after_forward_parking_straight_distance;
  params.after_backward_parking_straight_distance =
    parameters_->after_backward_parking_straight_distance;
  params.forward_parking_velocity = parameters_->forward_parking_velocity;
  params.backward_parking_velocity = parameters_->backward_parking_velocity;
  params.forward_parking_lane_departure_margin = parameters_->forward_parking_lane_departure_margin;
  params.backward_parking_lane_departure_margin =
    parameters_->backward_parking_lane_departure_margin;
  params.arc_path_interval = parameters_->arc_path_interval;
  params.maximum_deceleration = parameters_->maximum_deceleration;
  params.max_steer_angle = parameters_->pull_over_max_steer_angle;

  return params;
}

void PullOverModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OVER onEntry");
#ifdef USE_OLD_ARCHITECTURE
  current_state_ = ModuleStatus::SUCCESS;
#else
  current_state_ = ModuleStatus::IDLE;
#endif

  // Initialize occupancy grid map
  if (parameters_->use_occupancy_grid) {
    OccupancyGridMapParam occupancy_grid_map_param{};
    const double margin = parameters_->occupancy_grid_collision_check_margin;
    occupancy_grid_map_param.vehicle_shape.length =
      planner_data_->parameters.vehicle_length + 2 * margin;
    occupancy_grid_map_param.vehicle_shape.width =
      planner_data_->parameters.vehicle_width + 2 * margin;
    occupancy_grid_map_param.vehicle_shape.base2back =
      planner_data_->parameters.base_link2rear + margin;
    occupancy_grid_map_param.theta_size = parameters_->theta_size;
    occupancy_grid_map_param.obstacle_threshold = parameters_->obstacle_threshold;
    occupancy_grid_map_->setParam(occupancy_grid_map_param);
  }

  // initialize when receiving new route
  if (
    !last_received_time_ ||
    *last_received_time_ != planner_data_->route_handler->getRouteHeader().stamp) {
    // Initialize parallel parking planner status
    parallel_parking_parameters_ = getGeometricPullOverParameters();
    resetStatus();

    refined_goal_pose_ = calcRefinedGoal(planner_data_->route_handler->getGoalPose());
    if (parameters_->enable_goal_research) {
      goal_searcher_->setPlannerData(planner_data_);
      goal_candidates_ = goal_searcher_->search(refined_goal_pose_);
    } else {
      GoalCandidate goal_candidate{};
      goal_candidate.goal_pose = refined_goal_pose_;
      goal_candidate.distance_from_original_goal = 0.0;
      goal_candidates_.push_back(goal_candidate);
    }
  }
  last_received_time_ =
    std::make_unique<rclcpp::Time>(planner_data_->route_handler->getRouteHeader().stamp);
}

void PullOverModule::onExit()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OVER onExit");
  clearWaitingApproval();
  removeRTCStatus();
  resetPathCandidate();
  resetPathReference();
  steering_factor_interface_ptr_->clearSteeringFactors();
  debug_marker_.markers.clear();
  current_state_ = ModuleStatus::SUCCESS;
}

bool PullOverModule::isExecutionRequested() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }
  const auto & current_lanes = util::getCurrentLanes(planner_data_);
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
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
  if (self_to_goal_arc_length > parameters_->request_length) {
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
  const auto self_pose = planner_data_->self_odometry->pose.pose;
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

Pose PullOverModule::calcRefinedGoal(const Pose & goal_pose) const
{
  lanelet::Lanelet closest_shoulder_lanelet;
  lanelet::utils::query::getClosestLanelet(
    planner_data_->route_handler->getShoulderLanelets(), goal_pose, &closest_shoulder_lanelet);

  // calc closest center line pose
  Pose center_pose;
  {
    // find position
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal_pose.position);
    const auto segment = lanelet::utils::getClosestSegment(
      lanelet::utils::to2D(lanelet_point), closest_shoulder_lanelet.centerline());
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    center_pose.position.x = refined_point.x();
    center_pose.position.y = refined_point.y();
    center_pose.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    center_pose.orientation = tf2::toMsg(tf_quat);
  }

  const auto distance_from_left_bound = util::getSignedDistanceFromShoulderLeftBoundary(
    planner_data_->route_handler->getShoulderLanelets(), vehicle_footprint_, center_pose);
  if (!distance_from_left_bound) {
    RCLCPP_ERROR(getLogger(), "fail to calculate refined goal");
    return goal_pose;
  }

  const double offset_from_center_line =
    distance_from_left_bound.value() + parameters_->margin_from_boundary;
  const auto refined_goal_pose = calcOffsetPose(center_pose, 0, -offset_from_center_line, 0);

  return refined_goal_pose;
}

ModuleStatus PullOverModule::updateState()
{
  // pull_out module will be run when setting new goal, so not need finishing pull_over module.
  // Finishing it causes wrong lane_following path generation.
  return current_state_;
}

bool PullOverModule::planFreespacePath()
{
  mutex_.lock();
  goal_searcher_->update(goal_candidates_);
  const auto goal_candidates = goal_candidates_;
  mutex_.unlock();

  for (const auto & goal_candidate : goal_candidates) {
    if (!goal_candidate.is_safe) {
      continue;
    }
    freespace_planner_->setPlannerData(planner_data_);
    auto freespace_path = freespace_planner_->plan(goal_candidate.goal_pose);
    freespace_path->goal_id = goal_candidate.id;
    if (!freespace_path) {
      continue;
    }
    mutex_.lock();
    status_.pull_over_path = std::make_shared<PullOverPath>(*freespace_path);
    status_.current_path_idx = 0;
    status_.is_safe = true;
    modified_goal_pose_ = goal_candidate;
    mutex_.unlock();
    return true;
  }
  return false;
}

bool PullOverModule::returnToLaneParking()
{
  if (!status_.lane_parking_pull_over_path) {
    return false;
  }

  const PathWithLaneId path = status_.lane_parking_pull_over_path->getFullPath();
  if (checkCollision(path)) {
    return false;
  }

  const Point & current_point = planner_data_->self_odometry->pose.pose.position;
  constexpr double th_distance = 0.5;
  const bool is_close_to_path =
    std::abs(motion_utils::calcLateralOffset(path.points, current_point)) < th_distance;
  if (!is_close_to_path) {
    return false;
  }

  mutex_.lock();
  status_.has_decided_path = false;
  status_.pull_over_path = status_.lane_parking_pull_over_path;
  status_.current_path_idx = 0;
  mutex_.unlock();

  return true;
}

BehaviorModuleOutput PullOverModule::plan()
{
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  resetPathCandidate();
  resetPathReference();

  status_.current_lanes = util::getExtendedCurrentLanes(planner_data_);
  status_.pull_over_lanes = pull_over_utils::getPullOverLanes(*(planner_data_->route_handler));
  status_.lanes =
    util::generateDrivableLanesWithShoulderLanes(status_.current_lanes, status_.pull_over_lanes);

  // Check if it needs to decide path
  if (status_.is_safe) {
    const auto ego_segment_idx = motion_utils::findNearestSegmentIndex(
      getCurrentPath().points, current_pose, std::numeric_limits<double>::max(), M_PI_2);
    if (ego_segment_idx) {
      const size_t start_pose_segment_idx = motion_utils::findNearestSegmentIndex(
        getCurrentPath().points, status_.pull_over_path->start_pose.position);
      const auto dist_to_parking_start_pose = calcSignedArcLength(
        getCurrentPath().points, current_pose.position, *ego_segment_idx,
        status_.pull_over_path->start_pose.position, start_pose_segment_idx);
      if (dist_to_parking_start_pose < parameters_->decide_path_distance) {
        status_.has_decided_path = true;
      }
    }
  }

  if (status_.has_decided_path) {
    // Use decided path
    if (!status_.has_requested_approval) {
      // request approval again one the final path is decided
      waitApproval();
      removeRTCStatus();
      steering_factor_interface_ptr_->clearSteeringFactors();
      uuid_ = generateUUID();
      current_state_ = ModuleStatus::SUCCESS;  // for breaking loop
      status_.has_requested_approval = true;
    } else if (isActivated() && isWaitingApproval()) {
      // When it is approved again after path is decided
      clearWaitingApproval();
      last_approved_time_ = std::make_unique<rclcpp::Time>(clock_->now());
      last_approved_pose_ = std::make_unique<Pose>(current_pose);

      // decide velocity to guarantee turn signal lighting time
      if (!status_.has_decided_velocity) {
        auto & first_path = status_.pull_over_path->partial_paths.front();
        const auto vel =
          static_cast<float>(std::max(current_vel, parameters_->pull_over_minimum_velocity));
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

      if (hasFinishedCurrentPath() && has_passed_enough_time && status_.require_increment_) {
        if (incrementPathIndex()) {
          last_increment_time_ = std::make_unique<rclcpp::Time>(clock_->now());
        }
      }
    }
  } else if (!pull_over_path_candidates_.empty()) {
    // select safe path from pull over path candidates
    goal_searcher_->setPlannerData(planner_data_);
    mutex_.lock();
    goal_searcher_->update(goal_candidates_);
    const auto pull_over_path_candidates = pull_over_path_candidates_;
    const auto goal_candidates = goal_candidates_;
    mutex_.unlock();
    status_.is_safe = false;
    for (const auto & pull_over_path : pull_over_path_candidates) {
      // check if goal is safe
      const auto goal_candidate_it = std::find_if(
        goal_candidates.begin(), goal_candidates.end(),
        [pull_over_path](const auto & goal_candidate) {
          return goal_candidate.id == pull_over_path.goal_id;
        });
      if (goal_candidate_it != goal_candidates.end() && !goal_candidate_it->is_safe) {
        continue;
      }

      // check if path is valid and safe
      if (!hasEnoughDistance(pull_over_path) || checkCollision(pull_over_path.getParkingPath())) {
        continue;
      }

      status_.is_safe = true;
      mutex_.lock();
      status_.pull_over_path = std::make_shared<PullOverPath>(pull_over_path);
      status_.current_path_idx = 0;
      status_.lane_parking_pull_over_path = status_.pull_over_path;
      modified_goal_pose_ = *goal_candidate_it;
      mutex_.unlock();
      break;
    }

    // decelerate before the search area start
    if (status_.is_safe) {
      const auto search_start_pose = calcLongitudinalOffsetPose(
        status_.pull_over_path->getFullPath().points, refined_goal_pose_.position,
        -parameters_->backward_goal_search_length - planner_data_->parameters.base_link2front);
      auto & first_path = status_.pull_over_path->partial_paths.front();
      if (search_start_pose) {
        constexpr double deceleration_buffer = 15.0;
        first_path = util::setDecelerationVelocity(
          first_path, parameters_->pull_over_velocity, *search_start_pose, -deceleration_buffer,
          parameters_->deceleration_interval);
      } else {
        // if already passed the search start pose, set pull_over_velocity to first_path.
        for (auto & p : first_path.points) {
          p.point.longitudinal_velocity_mps = std::min(
            p.point.longitudinal_velocity_mps, static_cast<float>(parameters_->pull_over_velocity));
        }
      }
    }

    // generate drivable area for each partial path
    for (auto & path : status_.pull_over_path->partial_paths) {
      const size_t ego_idx = planner_data_->findEgoIndex(path.points);
      util::clipPathLength(path, ego_idx, planner_data_->parameters);
      const auto shorten_lanes = util::cutOverlappedLanes(path, status_.lanes);
      const auto expanded_lanes = util::expandLanelets(
        shorten_lanes, parameters_->drivable_area_left_bound_offset,
        parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);
      util::generateDrivableArea(
        path, expanded_lanes, planner_data_->parameters.vehicle_length, planner_data_);
    }
  }

  BehaviorModuleOutput output;
  if (status_.is_safe) {
    // safe: use pull over path
    status_.stop_pose.reset();

    // keep stop if not enough time passed,
    // because it takes time for the trajectory to be reflected
    auto current_path = getCurrentPath();
    keepStoppedWithCurrentPath(current_path);

    output.path = std::make_shared<PathWithLaneId>(current_path);
    output.reference_path = getPreviousModuleOutput().reference_path;
    path_candidate_ = std::make_shared<PathWithLaneId>(status_.pull_over_path->getFullPath());
    path_reference_ = getPreviousModuleOutput().reference_path;
  } else {
    // not safe: use stop_path
    if (status_.prev_is_safe || status_.prev_stop_path == nullptr) {
      // safe -> not_safe or no prev_stop_path: generate new stop_path
      output.path = std::make_shared<PathWithLaneId>(generateStopPath());
      output.reference_path = getPreviousModuleOutput().reference_path;
      status_.prev_stop_path = output.path;
      // set stop path as pull over path
      PullOverPath pull_over_path{};
      mutex_.lock();
      status_.pull_over_path = std::make_shared<PullOverPath>(pull_over_path);
      status_.current_path_idx = 0;
      status_.pull_over_path->partial_paths.push_back(*output.path);
      mutex_.unlock();
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000, "Not found safe pull_over path, generate stop path");
    } else {
      // not_safe -> not_safe: use previous stop path
      output.path = status_.prev_stop_path;
      output.reference_path = getPreviousModuleOutput().reference_path;
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000, "Not found safe pull_over path, use previous stop path");
    }
  }
  status_.prev_is_safe = status_.is_safe;

  // return to lane parking if it is possible
  if (status_.pull_over_path->type == PullOverPlannerType::FREESPACE) {
    // return only before starting free space parking
    if (!isStopped() || status_.during_freespace_parking) {
      status_.during_freespace_parking = true;
    } else if (returnToLaneParking()) {
      RCLCPP_INFO(getLogger(), "return to lane parking");
    }
  }

  // set hazard and turn signal
  if (status_.has_decided_path) {
    output.turn_signal_info = calcTurnSignalInfo();
  }

  const auto distance_to_path_change = calcDistanceToPathChange();
  updateRTCStatus(distance_to_path_change.first, distance_to_path_change.second);

  setDebugData();

  // Publish the modified goal only when it is updated
  if (
    status_.is_safe && modified_goal_pose_ &&
    (!prev_goal_id_ || *prev_goal_id_ != modified_goal_pose_->id)) {
    PoseWithUuidStamped modified_goal{};
    modified_goal.uuid = planner_data_->route_handler->getRouteUuid();
    modified_goal.pose = modified_goal_pose_->goal_pose;
    modified_goal.header = planner_data_->route_handler->getRouteHeader();
    output.modified_goal = modified_goal;
    prev_goal_id_ = modified_goal_pose_->id;
  } else {
    output.modified_goal = {};
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
    {status_.pull_over_path->start_pose, modified_goal_pose_->goal_pose},
    {distance_to_path_change.first, distance_to_path_change.second}, SteeringFactor::PULL_OVER,
    steering_factor_direction, SteeringFactor::TURNING, "");

  // For evaluations
  if (parameters_->print_debug_info) {
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
  out.modified_goal = plan().modified_goal;  // update status_
  out.path = std::make_shared<PathWithLaneId>(generateStopPath());
  out.reference_path = getPreviousModuleOutput().reference_path;
  path_candidate_ = status_.is_safe
                      ? std::make_shared<PathWithLaneId>(status_.pull_over_path->getFullPath())
                      : out.path;
  path_reference_ = getPreviousModuleOutput().reference_path;
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
    {status_.pull_over_path->start_pose, modified_goal_pose_->goal_pose},
    {distance_to_path_change.first, distance_to_path_change.second}, SteeringFactor::PULL_OVER,
    steering_factor_direction, SteeringFactor::APPROACHING, "");
  waitApproval();

  return out;
}

std::pair<double, double> PullOverModule::calcDistanceToPathChange() const
{
  const auto & full_path = status_.pull_over_path->getFullPath();

  const auto ego_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, planner_data_->self_odometry->pose.pose, std::numeric_limits<double>::max(),
    M_PI_2);
  if (!ego_segment_idx) {
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  }

  const size_t start_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, status_.pull_over_path->start_pose.position);
  const double dist_to_parking_start_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_odometry->pose.pose.position, *ego_segment_idx,
    status_.pull_over_path->start_pose.position, start_pose_segment_idx);
  const size_t goal_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, modified_goal_pose_->goal_pose.position);
  const double dist_to_parking_finish_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_odometry->pose.pose.position, *ego_segment_idx,
    modified_goal_pose_->goal_pose.position, goal_pose_segment_idx);

  return {dist_to_parking_start_pose, dist_to_parking_finish_pose};
}

void PullOverModule::setParameters(const std::shared_ptr<PullOverParameters> & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId PullOverModule::generateStopPath()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & common_parameters = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

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

  // if not approved stop road lane.
  // stop point priority is
  // 1. actual start pose
  // 2. closest candidate start pose
  // 3. search start pose
  //     (In the case of the curve lane, the position is not aligned due to the
  //     difference between the outer and inner sides)
  // 4. emergency stop
  const auto search_start_pose = calcLongitudinalOffsetPose(
    reference_path.points, refined_goal_pose_.position,
    -parameters_->backward_goal_search_length - planner_data_->parameters.base_link2front);
  if (!status_.is_safe && !closest_start_pose_ && !search_start_pose) {
    return generateEmergencyStopPath();
  }
  const Pose stop_pose =
    status_.is_safe ? status_.pull_over_path->start_pose
                    : (closest_start_pose_ ? closest_start_pose_.value() : *search_start_pose);

  // if stop pose is closer than min_stop_distance, stop as soon as possible
  const size_t ego_idx = planner_data_->findEgoIndex(reference_path.points);
  const size_t stop_idx = findFirstNearestSegmentIndexWithSoftConstraints(
    reference_path.points, stop_pose, common_parameters.ego_nearest_dist_threshold,
    common_parameters.ego_nearest_yaw_threshold);
  const double ego_to_stop_distance = calcSignedArcLength(
    reference_path.points, current_pose.position, ego_idx, stop_pose.position, stop_idx);
  const double min_stop_distance = std::pow(current_vel, 2) / parameters_->maximum_deceleration / 2;
  if (ego_to_stop_distance < min_stop_distance) {
    return generateEmergencyStopPath();
  }

  // slow down for turn signal, insert stop point to stop_pose
  reference_path = util::setDecelerationVelocityForTurnSignal(
    reference_path, stop_pose, planner_data_->parameters.turn_signal_search_time);
  status_.stop_pose = stop_pose;

  // slow down before the search area.
  if (search_start_pose) {
    constexpr double deceleration_buffer = 15.0;
    reference_path = util::setDecelerationVelocity(
      reference_path, parameters_->pull_over_velocity, *search_start_pose, -deceleration_buffer,
      parameters_->deceleration_interval);
  } else {
    // if already passed the search start pose, set pull_over_velocity to reference_path.
    for (auto & p : reference_path.points) {
      p.point.longitudinal_velocity_mps = std::min(
        p.point.longitudinal_velocity_mps, static_cast<float>(parameters_->pull_over_velocity));
    }
  }

  // generate drivable area
  const auto drivable_lanes = util::generateDrivableLanes(status_.current_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);
  util::generateDrivableArea(
    reference_path, expanded_lanes, common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

PathWithLaneId PullOverModule::generateEmergencyStopPath()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & common_parameters = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;
  constexpr double eps_vel = 0.01;

  // generate stop reference path
  const auto s_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto stop_path = route_handler->getCenterLinePath(status_.current_lanes, s_start, s_end, true);

  // calc minimum stop distance under maximum deceleration
  const double min_stop_distance = std::pow(current_vel, 2) / parameters_->maximum_deceleration / 2;

  // set stop point
  const auto stop_idx =
    motion_utils::insertStopPoint(current_pose, min_stop_distance, stop_path.points);
  if (stop_idx) {
    status_.stop_pose = stop_path.points.at(*stop_idx).point.pose;
  }

  // set deceleration velocity
  const size_t ego_idx = planner_data_->findEgoIndex(stop_path.points);
  for (auto & point : stop_path.points) {
    auto & p = point.point;
    const size_t target_idx = findFirstNearestSegmentIndexWithSoftConstraints(
      stop_path.points, p.pose, common_parameters.ego_nearest_dist_threshold,
      common_parameters.ego_nearest_yaw_threshold);
    const double distance_to_target = calcSignedArcLength(
      stop_path.points, current_pose.position, ego_idx, p.pose.position, target_idx);
    if (0.0 < distance_to_target && eps_vel < current_vel) {
      p.longitudinal_velocity_mps = std::clamp(
        static_cast<float>(
          current_vel * (min_stop_distance - distance_to_target) / min_stop_distance),
        0.0f, p.longitudinal_velocity_mps);
    } else {
      p.longitudinal_velocity_mps =
        std::min(p.longitudinal_velocity_mps, static_cast<float>(current_vel));
    }
  }

  // generate drivable area
  const auto drivable_lanes = util::generateDrivableLanes(status_.current_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(stop_path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset, parameters_->drivable_area_types_to_skip);
  util::generateDrivableArea(
    stop_path, expanded_lanes, common_parameters.vehicle_length, planner_data_);

  return stop_path;
}

bool PullOverModule::incrementPathIndex()
{
  if (status_.current_path_idx == status_.pull_over_path->partial_paths.size() - 1) {
    return false;
  }
  status_.current_path_idx = status_.current_path_idx + 1;
  return true;
}

PathWithLaneId PullOverModule::getCurrentPath() const
{
  return status_.pull_over_path->partial_paths.at(status_.current_path_idx);
}

bool PullOverModule::isStopped(
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> & odometry_buffer, const double time)
{
  odometry_buffer.push_back(planner_data_->self_odometry);
  // Delete old data in buffer
  while (rclcpp::ok()) {
    const auto time_diff = rclcpp::Time(odometry_buffer.back()->header.stamp) -
                           rclcpp::Time(odometry_buffer.front()->header.stamp);
    if (time_diff.seconds() < time) {
      break;
    }
    odometry_buffer.pop_front();
  }
  bool is_stopped = true;
  for (const auto & odometry : odometry_buffer) {
    const double ego_vel = util::l2Norm(odometry->twist.twist.linear);
    if (ego_vel > parameters_->th_stopped_velocity) {
      is_stopped = false;
      break;
    }
  }
  return is_stopped;
}

bool PullOverModule::isStopped()
{
  return isStopped(odometry_buffer_stopped_, parameters_->th_stopped_time);
}

bool PullOverModule::isStuck()
{
  if (!status_.pull_over_path) {
    return false;
  }
  constexpr double stuck_time = 5.0;
  return isStopped(odometry_buffer_stuck_, stuck_time) && checkCollision(getCurrentPath());
}

bool PullOverModule::hasFinishedCurrentPath()
{
  const auto & current_path_end = getCurrentPath().points.back();
  const auto & self_pose = planner_data_->self_odometry->pose.pose;
  const bool is_near_target = tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
                              parameters_->th_arrived_distance;

  return is_near_target && isStopped();
}

bool PullOverModule::hasFinishedPullOver()
{
  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const bool car_is_on_goal =
    calcDistance2d(current_pose, modified_goal_pose_->goal_pose) < parameters_->th_arrived_distance;

  return car_is_on_goal && isStopped();
}

TurnSignalInfo PullOverModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal{};  // output

  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & start_pose = status_.pull_over_path->start_pose;
  const auto & end_pose = status_.pull_over_path->end_pose;
  const auto full_path = status_.pull_over_path->getFullPath();

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
  using motion_utils::createStopVirtualWallMarker;
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerColor;
  using tier4_autoware_utils::createMarkerScale;

  const auto header = planner_data_->route_handler->getRouteHeader();

  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  if (parameters_->enable_goal_research) {
    // Visualize pull over areas
    const auto color = status_.has_decided_path ? createMarkerColor(1.0, 1.0, 0.0, 0.999)  // yellow
                                                : createMarkerColor(0.0, 1.0, 0.0, 0.999);  // green
    const double z = refined_goal_pose_.position.z;
    add(pull_over_utils::createPullOverAreaMarkerArray(
      goal_searcher_->getAreaPolygons(), header, color, z));

    // Visualize goal candidates
    add(pull_over_utils::createGoalCandidatesMarkerArray(goal_candidates_, color));
  }

  // Visualize path and related pose
  if (status_.is_safe) {
    add(createPoseMarkerArray(
      status_.pull_over_path->start_pose, "pull_over_start_pose", 0, 0.3, 0.3, 0.9));
    add(createPoseMarkerArray(
      status_.pull_over_path->end_pose, "pull_over_end_pose", 0, 0.3, 0.3, 0.9));
    add(
      createPathMarkerArray(status_.pull_over_path->getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
    add(createPathMarkerArray(getCurrentPath(), "current_path", 0, 0.9, 0.5, 0.0));

    // visualize each partial path
    for (size_t i = 0; i < status_.pull_over_path->partial_paths.size(); ++i) {
      const auto & partial_path = status_.pull_over_path->partial_paths.at(i);
      add(
        createPathMarkerArray(partial_path, "partial_path_" + std::to_string(i), 0, 0.9, 0.5, 0.9));
    }
  }

  // Visualize planner type text
  {
    visualization_msgs::msg::MarkerArray planner_type_marker_array{};
    const auto color = status_.is_safe ? createMarkerColor(1.0, 1.0, 1.0, 0.99)
                                       : createMarkerColor(1.0, 0.0, 0.0, 0.99);
    auto marker = createDefaultMarker(
      header.frame_id, header.stamp, "planner_type", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0), color);
    marker.pose = modified_goal_pose_->goal_pose;
    marker.text = magic_enum::enum_name(status_.pull_over_path->type);
    marker.text += " " + std::to_string(status_.current_path_idx) + "/" +
                   std::to_string(status_.pull_over_path->partial_paths.size() - 1);
    if (isStuck()) {
      marker.text += " stuck";
    } else if (isStopped()) {
      marker.text += " stopped";
    }

    planner_type_marker_array.markers.push_back(marker);
    add(planner_type_marker_array);
  }

  // Visualize debug poses
  const auto & debug_poses = status_.pull_over_path->debug_poses;
  for (size_t i = 0; i < debug_poses.size(); ++i) {
    add(createPoseMarkerArray(
      debug_poses.at(i), "debug_pose_" + std::to_string(i), 0, 0.3, 0.3, 0.3));
  }

  // Visualize stop pose
  if (status_.stop_pose) {
    add(createStopVirtualWallMarker(
      *status_.stop_pose, "pull_over", clock_->now(), 0,
      planner_data_->parameters.base_link2front));
  }
}

bool PullOverModule::checkCollision(const PathWithLaneId & path) const
{
  if (parameters_->use_occupancy_grid || !occupancy_grid_map_) {
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->hasObstacleOnPath(path, check_out_of_range)) {
      return true;
    }
  }

  if (parameters_->use_object_recognition) {
    if (util::checkCollisionBetweenPathFootprintsAndObjects(
          vehicle_footprint_, path, *(planner_data_->dynamic_object),
          parameters_->object_recognition_collision_check_margin)) {
      return true;
    }
  }

  return false;
}

bool PullOverModule::hasEnoughDistance(const PullOverPath & pull_over_path) const
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  // once stopped, the vehicle cannot start again if start_pose is close.
  // so need enough distance to restart.
  // distance to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  constexpr double eps_vel = 0.01;
  const double distance_to_start = calcSignedArcLength(
    pull_over_path.getFullPath().points, current_pose.position, pull_over_path.start_pose.position);
  const double distance_to_restart = parameters_->decide_path_distance / 2;
  if (std::abs(current_vel) < eps_vel && distance_to_start < distance_to_restart) {
    return false;
  }

  // prevent emergency stop
  const double current_to_stop_distance =
    std::pow(current_vel, 2) / parameters_->maximum_deceleration / 2;
  if (distance_to_start < current_to_stop_distance) {
    return false;
  }

  return true;
}

void PullOverModule::keepStoppedWithCurrentPath(PathWithLaneId & path)
{
  constexpr double keep_stop_time = 2.0;
  constexpr double keep_current_idx_buffer_time = 2.0;
  if (last_increment_time_) {
    const auto time_diff = (clock_->now() - *last_increment_time_).seconds();
    if (time_diff < keep_stop_time) {
      status_.require_increment_ = false;
      for (auto & p : path.points) {
        p.point.longitudinal_velocity_mps = 0.0;
      }
    } else if (time_diff > keep_stop_time + keep_current_idx_buffer_time) {
      // require increment only when the time passed is enough
      // to prevent increment before driving
      // when the end of the current path is close to the current pose
      status_.require_increment_ = true;
    }
  }
}

void PullOverModule::printParkingPositionError() const
{
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const double real_shoulder_to_map_shoulder = 0.0;

  const Pose goal_to_ego = inverseTransformPose(current_pose, modified_goal_pose_->goal_pose);
  const double dx = goal_to_ego.position.x;
  const double dy = goal_to_ego.position.y;
  const double distance_from_real_shoulder =
    real_shoulder_to_map_shoulder + parameters_->margin_from_boundary - dy;
  RCLCPP_INFO(
    getLogger(), "current pose to goal, dx:%f dy:%f dyaw:%f from_real_shoulder:%f", dx, dy,
    tier4_autoware_utils::rad2deg(
      tf2::getYaw(current_pose.orientation) -
      tf2::getYaw(modified_goal_pose_->goal_pose.orientation)),
    distance_from_real_shoulder);
}
}  // namespace behavior_path_planner
