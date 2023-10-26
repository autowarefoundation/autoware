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

#include "behavior_path_planner/scene_module/goal_planner/goal_planner_module.hpp"

#include "behavior_path_planner/utils/create_vehicle_footprint.hpp"
#include "behavior_path_planner/utils/goal_planner/util.hpp"
#include "behavior_path_planner/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner/utils/path_safety_checker/safety_check.hpp"
#include "behavior_path_planner/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"
#include "behavior_path_planner/utils/start_goal_planner_common/utils.hpp"
#include "behavior_path_planner/utils/utils.hpp"
#include "tier4_autoware_utils/geometry/boost_polygon_utils.hpp"
#include "tier4_autoware_utils/math/unit_conversion.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using behavior_path_planner::utils::start_goal_planner_common::calcFeasibleDecelDistance;
using motion_utils::calcLongitudinalOffsetPose;
using motion_utils::calcSignedArcLength;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using motion_utils::insertDecelPoint;
using nav_msgs::msg::OccupancyGrid;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
using tier4_autoware_utils::createMarkerColor;
using tier4_autoware_utils::inverseTransformPose;

namespace behavior_path_planner
{
GoalPlannerModule::GoalPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<GoalPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface> > & rtc_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map},
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()},
  status_{mutex_}
{
  LaneDepartureChecker lane_departure_checker{};
  lane_departure_checker.setVehicleInfo(vehicle_info_);

  occupancy_grid_map_ = std::make_shared<OccupancyGridBasedCollisionDetector>();

  left_side_parking_ = parameters_->parking_policy == ParkingPolicy::LEFT_SIDE;

  // planner when goal modification is not allowed
  fixed_goal_planner_ = std::make_unique<DefaultFixedGoalPlanner>();

  for (const std::string & planner_type : parameters_->efficient_path_order) {
    if (planner_type == "SHIFT" && parameters_->enable_shift_parking) {
      pull_over_planners_.push_back(std::make_shared<ShiftPullOver>(
        node, *parameters, lane_departure_checker, occupancy_grid_map_));
    } else if (planner_type == "ARC_FORWARD" && parameters_->enable_arc_forward_parking) {
      pull_over_planners_.push_back(std::make_shared<GeometricPullOver>(
        node, *parameters, lane_departure_checker, occupancy_grid_map_, /*is_forward*/ true));
    } else if (planner_type == "ARC_BACKWARD" && parameters_->enable_arc_backward_parking) {
      pull_over_planners_.push_back(std::make_shared<GeometricPullOver>(
        node, *parameters, lane_departure_checker, occupancy_grid_map_, /*is_forward*/ false));
    }
  }

  if (pull_over_planners_.empty()) {
    RCLCPP_ERROR(getLogger(), "Not found enabled planner");
  }

  // set selected goal searcher
  // currently there is only one goal_searcher_type
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo();
  vehicle_footprint_ = createVehicleFootprint(vehicle_info);
  goal_searcher_ =
    std::make_shared<GoalSearcher>(*parameters, vehicle_footprint_, occupancy_grid_map_);

  // timer callback for generating lane parking candidate paths
  const auto lane_parking_period_ns = rclcpp::Rate(1.0).period();
  lane_parking_timer_cb_group_ =
    node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  lane_parking_timer_ = rclcpp::create_timer(
    &node, clock_, lane_parking_period_ns, std::bind(&GoalPlannerModule::onTimer, this),
    lane_parking_timer_cb_group_);

  // freespace parking
  if (parameters_->enable_freespace_parking) {
    freespace_planner_ = std::make_unique<FreespacePullOver>(node, *parameters, vehicle_info);
    const auto freespace_parking_period_ns = rclcpp::Rate(1.0).period();
    freespace_parking_timer_cb_group_ =
      node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    freespace_parking_timer_ = rclcpp::create_timer(
      &node, clock_, freespace_parking_period_ns,
      std::bind(&GoalPlannerModule::onFreespaceParkingTimer, this),
      freespace_parking_timer_cb_group_);
  }

  // Initialize safety checker
  if (parameters_->safety_check_params.enable_safety_check) {
    initializeSafetyCheckParameters();
    utils::start_goal_planner_common::initializeCollisionCheckDebugMap(
      goal_planner_data_.collision_check);
  }

  status_.reset();
}

// This function is needed for waiting for planner_data_
void GoalPlannerModule::updateOccupancyGrid()
{
  if (!planner_data_->occupancy_grid) {
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "occupancy_grid is not ready");
    return;
  }
  occupancy_grid_map_->setMap(*(planner_data_->occupancy_grid));
}

// generate pull over candidate paths
void GoalPlannerModule::onTimer()
{
  // already generated pull over candidate paths
  if (!status_.get_pull_over_path_candidates().empty()) {
    return;
  }

  // goals are not yet available.
  if (status_.get_goal_candidates().empty()) {
    return;
  }
  const auto goal_candidates = status_.get_goal_candidates();

  // generate valid pull over path candidates and calculate closest start pose
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length,
    /*forward_only_in_route*/ false);
  std::vector<PullOverPath> path_candidates{};
  std::optional<Pose> closest_start_pose{};
  double min_start_arc_length = std::numeric_limits<double>::max();
  const auto planCandidatePaths = [&](
                                    const std::shared_ptr<PullOverPlannerBase> & planner,
                                    const GoalCandidate & goal_candidate) {
    planner->setPlannerData(planner_data_);
    auto pull_over_path = planner->plan(goal_candidate.goal_pose);
    if (pull_over_path && isCrossingPossible(*pull_over_path)) {
      pull_over_path->goal_id = goal_candidate.id;
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
  if (parameters_->path_priority == "efficient_path") {
    for (const auto & planner : pull_over_planners_) {
      for (const auto & goal_candidate : goal_candidates) {
        planCandidatePaths(planner, goal_candidate);
      }
    }
  } else if (parameters_->path_priority == "close_goal") {
    for (const auto & goal_candidate : goal_candidates) {
      for (const auto & planner : pull_over_planners_) {
        planCandidatePaths(planner, goal_candidate);
      }
    }
  } else {
    RCLCPP_ERROR(
      getLogger(), "path_priority should be efficient_path or close_goal, but %s is given.",
      parameters_->path_priority.c_str());
    throw std::domain_error("[pull_over] invalid path_priority");
  }

  // set member variables
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    status_.set_pull_over_path_candidates(path_candidates);
    status_.set_closest_start_pose(closest_start_pose);
  }
}

void GoalPlannerModule::onFreespaceParkingTimer()
{
  if (!planner_data_) {
    return;
  }
  if (!planner_data_->costmap) {
    return;
  }
  // fixed goal planner do not use freespace planner
  if (!goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return;
  }

  if (isOnModifiedGoal()) {
    return;
  }

  const bool is_new_costmap =
    (clock_->now() - planner_data_->costmap->header.stamp).seconds() < 1.0;
  constexpr double path_update_duration = 1.0;
  if (isStuck() && is_new_costmap && needPathUpdate(path_update_duration)) {
    planFreespacePath();
  }
}

BehaviorModuleOutput GoalPlannerModule::run()
{
  current_state_ = ModuleStatus::RUNNING;
  updateOccupancyGrid();

  if (!isActivated()) {
    return planWaitingApproval();
  }

  return plan();
}

void GoalPlannerModule::updateData()
{
  // Initialize Occupancy Grid Map
  // This operation requires waiting for `planner_data_`, hence it is executed here instead of in
  // the constructor. Ideally, this operation should only need to be performed once.
  if (
    parameters_->use_occupancy_grid_for_goal_search ||
    parameters_->use_occupancy_grid_for_path_collision_check) {
    initializeOccupancyGridMap();
  }

  updateOccupancyGrid();

  // set current road lanes, pull over lanes, and drivable lane
  setLanes();

  generateGoalCandidates();
}

void GoalPlannerModule::initializeOccupancyGridMap()
{
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

void GoalPlannerModule::initializeSafetyCheckParameters()
{
  utils::start_goal_planner_common::updateEgoPredictedPathParams(
    ego_predicted_path_params_, parameters_);
  utils::start_goal_planner_common::updateSafetyCheckParams(safety_check_params_, parameters_);
  utils::start_goal_planner_common::updateObjectsFilteringParams(
    objects_filtering_params_, parameters_);
}

void GoalPlannerModule::processOnExit()
{
  resetPathCandidate();
  resetPathReference();
  debug_marker_.markers.clear();
}

bool GoalPlannerModule::isExecutionRequested() const
{
  if (current_state_ == ModuleStatus::RUNNING) {
    return true;
  }

  const auto & route_handler = planner_data_->route_handler;
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const Pose goal_pose = route_handler->getOriginalGoalPose();

  // check if goal_pose is in current_lanes.
  lanelet::ConstLanelet current_lane{};
  const lanelet::ConstLanelets current_lanes = utils::getCurrentLanes(planner_data_);
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const bool goal_is_in_current_lanes = std::any_of(
    current_lanes.begin(), current_lanes.end(), [&](const lanelet::ConstLanelet & current_lane) {
      return lanelet::utils::isInLanelet(goal_pose, current_lane);
    });

  // check that goal is in current neighbor shoulder lane
  const bool goal_is_in_current_shoulder_lanes = std::invoke([&]() {
    lanelet::ConstLanelet neighbor_shoulder_lane{};
    for (const auto & lane : current_lanes) {
      const bool has_shoulder_lane =
        left_side_parking_ ? route_handler->getLeftShoulderLanelet(lane, &neighbor_shoulder_lane)
                           : route_handler->getRightShoulderLanelet(lane, &neighbor_shoulder_lane);
      if (has_shoulder_lane && lanelet::utils::isInLanelet(goal_pose, neighbor_shoulder_lane)) {
        return true;
      }
    }
    return false;
  });

  // if goal is not in current_lanes and current_shoulder_lanes, do not execute goal_planner,
  // because goal arc coordinates cannot be calculated.
  if (!goal_is_in_current_lanes && !goal_is_in_current_shoulder_lanes) {
    return false;
  }

  // if goal modification is not allowed
  // 1) goal_pose is in current_lanes, plan path to the original fixed goal
  // 2) goal_pose is NOT in current_lanes, do not execute goal_planner
  if (!goal_planner_utils::isAllowedGoalModification(route_handler)) {
    return goal_is_in_current_lanes;
  }

  // if goal arc coordinates can be calculated, check if goal is in request_length
  const double self_to_goal_arc_length =
    utils::getSignedDistance(current_pose, goal_pose, current_lanes);
  const double request_length = goal_planner_utils::isAllowedGoalModification(route_handler)
                                  ? calcModuleRequestLength()
                                  : parameters_->pull_over_minimum_request_length;
  if (self_to_goal_arc_length < 0.0 || self_to_goal_arc_length > request_length) {
    // if current position is far from goal or behind goal, do not execute goal_planner
    return false;
  }

  // if goal modification is not allowed
  // 1) goal_pose is in current_lanes, plan path to the original fixed goal
  // 2) goal_pose is NOT in current_lanes, do not execute goal_planner
  if (!goal_planner_utils::isAllowedGoalModification(route_handler)) {
    return goal_is_in_current_lanes;
  }

  // if (A) or (B) is met execute pull over
  // (A) target lane is `road` and same to the current lanes
  // (B) target lane is `road_shoulder` and neighboring to the current lanes
  const lanelet::ConstLanelets pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *(route_handler), left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);
  lanelet::ConstLanelet target_lane{};
  lanelet::utils::query::getClosestLanelet(pull_over_lanes, goal_pose, &target_lane);
  if (!isCrossingPossible(current_lane, target_lane)) {
    return false;
  }

  return true;
}

bool GoalPlannerModule::isExecutionReady() const
{
  if (parameters_->safety_check_params.enable_safety_check && isWaitingApproval()) {
    if (!isSafePath()) {
      RCLCPP_ERROR_THROTTLE(getLogger(), *clock_, 5000, "Path is not safe against dynamic objects");
      return false;
    }
  }
  return true;
}

double GoalPlannerModule::calcModuleRequestLength() const
{
  const auto min_stop_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, 0.0);
  if (!min_stop_distance) {
    return parameters_->pull_over_minimum_request_length;
  }

  const double minimum_request_length =
    *min_stop_distance + parameters_->backward_goal_search_length + approximate_pull_over_distance_;

  return std::max(minimum_request_length, parameters_->pull_over_minimum_request_length);
}

Pose GoalPlannerModule::calcRefinedGoal(const Pose & goal_pose) const
{
  const double vehicle_width = planner_data_->parameters.vehicle_width;
  const double base_link2front = planner_data_->parameters.base_link2front;
  const double base_link2rear = planner_data_->parameters.base_link2rear;

  const lanelet::ConstLanelets pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *(planner_data_->route_handler), left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);

  lanelet::Lanelet closest_pull_over_lanelet{};
  lanelet::utils::query::getClosestLanelet(pull_over_lanes, goal_pose, &closest_pull_over_lanelet);

  // calc closest center line pose
  Pose center_pose{};
  {
    // find position
    const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal_pose.position);
    const auto segment = lanelet::utils::getClosestSegment(
      lanelet::utils::to2D(lanelet_point), closest_pull_over_lanelet.centerline());
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

  const auto distance_from_bound = utils::getSignedDistanceFromBoundary(
    pull_over_lanes, vehicle_width, base_link2front, base_link2rear, center_pose,
    left_side_parking_);
  if (!distance_from_bound) {
    RCLCPP_ERROR(getLogger(), "fail to calculate refined goal");
    return goal_pose;
  }

  const double sign = left_side_parking_ ? -1.0 : 1.0;
  const double offset_from_center_line =
    sign * (distance_from_bound.value() + parameters_->margin_from_boundary);

  const auto refined_goal_pose = calcOffsetPose(center_pose, 0, -offset_from_center_line, 0);

  return refined_goal_pose;
}

ModuleStatus GoalPlannerModule::updateState()
{
  // start_planner module will be run when setting new goal, so not need finishing pull_over module.
  // Finishing it causes wrong lane_following path generation.
  return current_state_;
}

bool GoalPlannerModule::planFreespacePath()
{
  goal_searcher_->setPlannerData(planner_data_);
  auto goal_candidates = status_.get_goal_candidates();
  goal_searcher_->update(goal_candidates);
  status_.set_goal_candidates(goal_candidates);

  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    debug_data_.freespace_planner.num_goal_candidates = goal_candidates.size();
    debug_data_.freespace_planner.is_planning = true;
  }

  for (size_t i = 0; i < goal_candidates.size(); i++) {
    const auto goal_candidate = goal_candidates.at(i);
    {
      const std::lock_guard<std::recursive_mutex> lock(mutex_);
      debug_data_.freespace_planner.current_goal_idx = i;
    }

    if (!goal_candidate.is_safe) {
      continue;
    }
    freespace_planner_->setPlannerData(planner_data_);
    auto freespace_path = freespace_planner_->plan(goal_candidate.goal_pose);
    freespace_path->goal_id = goal_candidate.id;
    if (!freespace_path) {
      continue;
    }

    {
      const std::lock_guard<std::recursive_mutex> lock(mutex_);
      status_.set_pull_over_path(std::make_shared<PullOverPath>(*freespace_path));
      status_.set_current_path_idx(0);
      status_.set_is_safe_static_objects(true);
      status_.set_modified_goal_pose(goal_candidate);
      status_.set_last_path_update_time(std::make_shared<rclcpp::Time>(clock_->now()));
      debug_data_.freespace_planner.is_planning = false;
    }

    return true;
  }

  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  debug_data_.freespace_planner.is_planning = false;
  return false;
}

void GoalPlannerModule::returnToLaneParking()
{
  // return only before starting free space parking
  if (!isStopped()) {
    return;
  }

  if (!status_.get_lane_parking_pull_over_path()) {
    return;
  }

  const PathWithLaneId path = status_.get_lane_parking_pull_over_path()->getFullPath();
  if (checkCollision(path)) {
    return;
  }

  const Point & current_point = planner_data_->self_odometry->pose.pose.position;
  constexpr double th_distance = 0.5;
  const bool is_close_to_path =
    std::abs(motion_utils::calcLateralOffset(path.points, current_point)) < th_distance;
  if (!is_close_to_path) {
    return;
  }

  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    status_.set_is_safe_static_objects(true);
    status_.set_has_decided_path(false);
    status_.set_pull_over_path(status_.get_lane_parking_pull_over_path());
    status_.set_current_path_idx(0);
    status_.set_last_path_update_time(std::make_shared<rclcpp::Time>(clock_->now()));
  }

  RCLCPP_INFO(getLogger(), "return to lane parking");
}

void GoalPlannerModule::generateGoalCandidates()
{
  const auto & route_handler = planner_data_->route_handler;

  // with old architecture, module instance is not cleared when new route is received
  // so need to reset status here.
  // todo: move this check out of this function after old architecture is removed
  if (!status_.get_goal_candidates().empty()) {
    return;
  }

  // calculate goal candidates
  const Pose goal_pose = route_handler->getOriginalGoalPose();
  status_.set_refined_goal_pose(calcRefinedGoal(goal_pose));
  if (goal_planner_utils::isAllowedGoalModification(route_handler)) {
    goal_searcher_->setPlannerData(planner_data_);
    goal_searcher_->setReferenceGoal(status_.get_refined_goal_pose());
    status_.set_goal_candidates(goal_searcher_->search());
  } else {
    GoalCandidate goal_candidate{};
    goal_candidate.goal_pose = goal_pose;
    goal_candidate.distance_from_original_goal = 0.0;
    GoalCandidates goal_candidates{};
    goal_candidates.push_back(goal_candidate);
    status_.set_goal_candidates(goal_candidates);
  }
}

BehaviorModuleOutput GoalPlannerModule::plan()
{
  resetPathCandidate();
  resetPathReference();

  path_reference_ = getPreviousModuleOutput().reference_path;

  if (goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return planWithGoalModification();
  } else {
    fixed_goal_planner_->setPreviousModuleOutput(getPreviousModuleOutput());
    return fixed_goal_planner_->plan(planner_data_);
  }
}

std::vector<PullOverPath> GoalPlannerModule::sortPullOverPathCandidatesByGoalPriority(
  const std::vector<PullOverPath> & pull_over_path_candidates,
  const GoalCandidates & goal_candidates) const
{
  auto sorted_pull_over_path_candidates = pull_over_path_candidates;

  // Create a map of goal_id to its index in goal_candidates
  std::map<size_t, size_t> goal_id_to_index;
  for (size_t i = 0; i < goal_candidates.size(); ++i) {
    goal_id_to_index[goal_candidates[i].id] = i;
  }

  // Sort pull_over_path_candidates based on the order in goal_candidates
  std::stable_sort(
    sorted_pull_over_path_candidates.begin(), sorted_pull_over_path_candidates.end(),
    [&goal_id_to_index](const auto & a, const auto & b) {
      return goal_id_to_index[a.goal_id] < goal_id_to_index[b.goal_id];
    });

  // Sort pull_over_path_candidates based on the order in efficient_path_order
  if (parameters_->path_priority == "efficient_path") {
    std::stable_sort(
      sorted_pull_over_path_candidates.begin(), sorted_pull_over_path_candidates.end(),
      [this](const auto & a, const auto & b) {
        const auto & order = parameters_->efficient_path_order;
        const auto a_pos = std::find(order.begin(), order.end(), magic_enum::enum_name(a.type));
        const auto b_pos = std::find(order.begin(), order.end(), magic_enum::enum_name(b.type));
        return a_pos < b_pos;
      });
  }

  return sorted_pull_over_path_candidates;
}

void GoalPlannerModule::selectSafePullOverPath()
{
  // select safe lane pull over path from candidates
  std::vector<PullOverPath> pull_over_path_candidates{};
  GoalCandidates goal_candidates{};
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    goal_searcher_->setPlannerData(planner_data_);
    goal_candidates = status_.get_goal_candidates();
    goal_searcher_->update(goal_candidates);
    status_.set_goal_candidates(goal_candidates);
    status_.set_pull_over_path_candidates(sortPullOverPathCandidatesByGoalPriority(
      status_.get_pull_over_path_candidates(), status_.get_goal_candidates()));
    pull_over_path_candidates = status_.get_pull_over_path_candidates();
    status_.set_is_safe_static_objects(false);
  }

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
    if (
      !hasEnoughDistance(pull_over_path) ||
      checkCollision(utils::resamplePathWithSpline(pull_over_path.getParkingPath(), 0.5))) {
      continue;
    }

    // found safe pull over path
    {
      const std::lock_guard<std::recursive_mutex> lock(mutex_);
      status_.set_is_safe_static_objects(true);
      status_.set_pull_over_path(std::make_shared<PullOverPath>(pull_over_path));
      status_.set_current_path_idx(0);
      status_.set_lane_parking_pull_over_path(status_.get_pull_over_path());
      status_.set_modified_goal_pose(*goal_candidate_it);
      status_.set_last_path_update_time(std::make_shared<rclcpp::Time>(clock_->now()));
    }
    break;
  }

  if (!status_.get_is_safe_static_objects()) {
    return;
  }

  // decelerate before the search area start
  const auto search_start_offset_pose = calcLongitudinalOffsetPose(
    status_.get_pull_over_path()->getFullPath().points, status_.get_refined_goal_pose().position,
    -parameters_->backward_goal_search_length - planner_data_->parameters.base_link2front -
      approximate_pull_over_distance_);
  auto & first_path = status_.get_pull_over_path()->partial_paths.front();
  if (search_start_offset_pose) {
    decelerateBeforeSearchStart(*search_start_offset_pose, first_path);
  } else {
    // if already passed the search start offset pose, set pull_over_velocity to first_path.
    const auto min_decel_distance = calcFeasibleDecelDistance(
      planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk,
      parameters_->pull_over_velocity);
    for (auto & p : first_path.points) {
      const double distance_from_ego = calcSignedArcLengthFromEgo(first_path, p.point.pose);
      if (min_decel_distance && distance_from_ego < *min_decel_distance) {
        continue;
      }
      p.point.longitudinal_velocity_mps = std::min(
        p.point.longitudinal_velocity_mps, static_cast<float>(parameters_->pull_over_velocity));
    }
  }
}

void GoalPlannerModule::setLanes()
{
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  status_.set_current_lanes(utils::getExtendedCurrentLanes(
    planner_data_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length,
    /*forward_only_in_route*/ false));
  status_.set_pull_over_lanes(goal_planner_utils::getPullOverLanes(
    *(planner_data_->route_handler), left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length));
  status_.set_lanes(utils::generateDrivableLanesWithShoulderLanes(
    status_.get_current_lanes(), status_.get_pull_over_lanes()));
}

void GoalPlannerModule::setOutput(BehaviorModuleOutput & output)
{
  if (!status_.get_is_safe_static_objects()) {
    // situation : not safe against static objects use stop_path
    setStopPath(output);
  } else if (
    parameters_->safety_check_params.enable_safety_check && !isSafePath() &&
    status_.get_has_decided_path() && isActivated()) {
    // situation : not safe against dynamic objects after approval
    // insert stop point in current path if ego is able to stop with acceleration and jerk
    // constraints
    setStopPathFromCurrentPath(output);
  } else {
    // situation : (safe against static and dynamic objects) or (safe against static objects and
    // before approval) don't stop
    if (isActivated()) {
      resetWallPoses();
    }
    // keep stop if not enough time passed,
    // because it takes time for the trajectory to be reflected
    auto current_path = getCurrentPath();
    keepStoppedWithCurrentPath(current_path);

    output.path = std::make_shared<PathWithLaneId>(current_path);
    output.reference_path = getPreviousModuleOutput().reference_path;
  }

  setDrivableAreaInfo(output);

  setModifiedGoal(output);

  // set hazard and turn signal
  if (status_.get_has_decided_path()) {
    setTurnSignalInfo(output);
  }

  // for the next loop setOutput().
  // this is used to determine whether to generate a new stop path or keep the current stop path.
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  status_.set_prev_is_safe(status_.get_is_safe_static_objects());
  status_.set_prev_is_safe_dynamic_objects(
    parameters_->safety_check_params.enable_safety_check ? isSafePath() : true);
}

void GoalPlannerModule::setStopPath(BehaviorModuleOutput & output)
{
  if (status_.get_prev_is_safe() || !status_.get_prev_stop_path()) {
    // safe -> not_safe or no prev_stop_path: generate new stop_path
    output.path = std::make_shared<PathWithLaneId>(generateStopPath());
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    status_.set_prev_stop_path(output.path);
    // set stop path as pull over path
    auto stop_pull_over_path = std::make_shared<PullOverPath>();
    stop_pull_over_path->partial_paths.push_back(*output.path);
    status_.set_pull_over_path(stop_pull_over_path);
    status_.set_current_path_idx(0);
    status_.set_last_path_update_time(std::make_shared<rclcpp::Time>(clock_->now()));

    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull_over path, generate stop path");
  } else {
    // not_safe -> not_safe: use previous stop path
    output.path = status_.get_prev_stop_path();
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull_over path, use previous stop path");
  }
  output.reference_path = getPreviousModuleOutput().reference_path;
}

void GoalPlannerModule::setStopPathFromCurrentPath(BehaviorModuleOutput & output)
{
  // safe or not safe(no feasible stop_path found) -> not_safe: try to find new feasible stop_path
  if (status_.get_prev_is_safe_dynamic_objects() || !status_.get_prev_stop_path_after_approval()) {
    auto current_path = getCurrentPath();
    const auto stop_path =
      behavior_path_planner::utils::start_goal_planner_common::generateFeasibleStopPath(
        current_path, planner_data_, *stop_pose_, parameters_->maximum_deceleration_for_stop,
        parameters_->maximum_jerk_for_stop);
    if (stop_path) {
      output.path = std::make_shared<PathWithLaneId>(*stop_path);
      status_.set_prev_stop_path_after_approval(output.path);
      RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "Collision detected, generate stop path");
    } else {
      output.path = std::make_shared<PathWithLaneId>(getCurrentPath());
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000,
        "Collision detected, no feasible stop path found, cannot stop.");
    }
    status_.set_last_path_update_time(std::make_shared<rclcpp::Time>(clock_->now()));
  } else {
    // not_safe safe(no feasible stop path found) -> not_safe: use previous stop path
    output.path = status_.get_prev_stop_path_after_approval();
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "Collision detected, use previous stop path");
  }
  output.reference_path = getPreviousModuleOutput().reference_path;
}

void GoalPlannerModule::setDrivableAreaInfo(BehaviorModuleOutput & output) const
{
  if (status_.get_pull_over_path()->type == PullOverPlannerType::FREESPACE) {
    const double drivable_area_margin = planner_data_->parameters.vehicle_width;
    output.drivable_area_info.drivable_margin =
      planner_data_->parameters.vehicle_width / 2.0 + drivable_area_margin;
  } else {
    const auto target_drivable_lanes = utils::getNonOverlappingExpandedLanes(
      *output.path, status_.get_lanes(), planner_data_->drivable_area_expansion_parameters);

    DrivableAreaInfo current_drivable_area_info;
    current_drivable_area_info.drivable_lanes = target_drivable_lanes;
    output.drivable_area_info = utils::combineDrivableAreaInfo(
      current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  }
}

void GoalPlannerModule::setModifiedGoal(BehaviorModuleOutput & output) const
{
  const auto & route_handler = planner_data_->route_handler;
  if (status_.get_is_safe_static_objects()) {
    PoseWithUuidStamped modified_goal{};
    modified_goal.uuid = route_handler->getRouteUuid();
    modified_goal.pose = status_.get_modified_goal_pose()->goal_pose;
    modified_goal.header = route_handler->getRouteHeader();
    output.modified_goal = modified_goal;
  } else {
    output.modified_goal = {};
  }
}

void GoalPlannerModule::setTurnSignalInfo(BehaviorModuleOutput & output) const
{
  const auto original_signal = getPreviousModuleOutput().turn_signal_info;
  const auto new_signal = calcTurnSignalInfo();
  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path->points);
  output.turn_signal_info = planner_data_->turn_signal_decider.use_prior_turn_signal(
    *output.path, getEgoPose(), current_seg_idx, original_signal, new_signal,
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);
}

void GoalPlannerModule::updateSteeringFactor(
  const std::array<Pose, 2> & pose, const std::array<double, 2> distance, const uint16_t type)
{
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
    pose, distance, SteeringFactor::GOAL_PLANNER, steering_factor_direction, type, "");
}

bool GoalPlannerModule::hasDecidedPath() const
{
  // once decided, keep the decision
  if (status_.get_has_decided_path()) {
    return true;
  }

  // if path is not safe, not decided
  if (!status_.get_is_safe_static_objects()) {
    return false;
  }

  // if ego is sufficiently close to the start of the nearest candidate path, the path is decided
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto ego_segment_idx = motion_utils::findNearestSegmentIndex(
    getCurrentPath().points, current_pose, std::numeric_limits<double>::max(), M_PI_2);
  if (!ego_segment_idx) {
    return false;
  }
  const size_t start_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    getCurrentPath().points, status_.get_pull_over_path()->start_pose.position);
  const double dist_to_parking_start_pose = calcSignedArcLength(
    getCurrentPath().points, current_pose.position, *ego_segment_idx,
    status_.get_pull_over_path()->start_pose.position, start_pose_segment_idx);
  return dist_to_parking_start_pose < parameters_->decide_path_distance;
}

void GoalPlannerModule::decideVelocity()
{
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  // decide velocity to guarantee turn signal lighting time
  if (!status_.get_has_decided_velocity()) {
    auto & first_path = status_.get_pull_over_path()->partial_paths.front();
    const auto vel =
      static_cast<float>(std::max(current_vel, parameters_->pull_over_minimum_velocity));
    for (auto & p : first_path.points) {
      p.point.longitudinal_velocity_mps = std::min(p.point.longitudinal_velocity_mps, vel);
    }
  }
  status_.set_has_decided_velocity(true);
}

BehaviorModuleOutput GoalPlannerModule::planWithGoalModification()
{
  constexpr double path_update_duration = 1.0;

  resetPathCandidate();
  resetPathReference();

  // set current road lanes, pull over lanes, and drivable lane
  setLanes();

  // Check if it needs to decide path
  status_.set_has_decided_path(hasDecidedPath());

  // Use decided path
  if (status_.get_has_decided_path()) {
    if (isActivated() && isWaitingApproval()) {
      {
        const std::lock_guard<std::recursive_mutex> lock(mutex_);
        status_.set_last_approved_time(std::make_shared<rclcpp::Time>(clock_->now()));
        status_.set_last_approved_pose(
          std::make_shared<Pose>(planner_data_->self_odometry->pose.pose));
      }
      clearWaitingApproval();
      decideVelocity();
    }
    transitionToNextPathIfFinishingCurrentPath();
  } else if (
    !status_.get_pull_over_path_candidates().empty() && needPathUpdate(path_update_duration)) {
    // if the final path is not decided and enough time has passed since last path update,
    // select safe path from lane parking pull over path candidates
    // and set it to status_.get_pull_over_path()
    selectSafePullOverPath();
  }
  // else: stop path is generated and set by setOutput()

  // set output and status
  BehaviorModuleOutput output{};
  setOutput(output);
  path_candidate_ = std::make_shared<PathWithLaneId>(status_.get_pull_over_path()->getFullPath());
  path_reference_ = getPreviousModuleOutput().reference_path;

  // return to lane parking if it is possible
  if (status_.get_pull_over_path()->type == PullOverPlannerType::FREESPACE) {
    returnToLaneParking();
  }

  const auto distance_to_path_change = calcDistanceToPathChange();
  if (status_.get_has_decided_path()) {
    updateRTCStatus(distance_to_path_change.first, distance_to_path_change.second);
  }
  // TODO(tkhmy) add handle status TRYING
  updateSteeringFactor(
    {status_.get_pull_over_path()->start_pose, status_.get_modified_goal_pose()->goal_pose},
    {distance_to_path_change.first, distance_to_path_change.second}, SteeringFactor::TURNING);

  // For debug
  setDebugData();
  if (parameters_->print_debug_info) {
    // For evaluations
    printParkingPositionError();
  }

  setStopReason(StopReason::GOAL_PLANNER, status_.get_pull_over_path()->getFullPath());

  return output;
}

BehaviorModuleOutput GoalPlannerModule::planWaitingApproval()
{
  resetPathCandidate();
  resetPathReference();

  path_reference_ = getPreviousModuleOutput().reference_path;

  if (goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return planWaitingApprovalWithGoalModification();
  } else {
    fixed_goal_planner_->setPreviousModuleOutput(getPreviousModuleOutput());
    return fixed_goal_planner_->plan(planner_data_);
  }
}

BehaviorModuleOutput GoalPlannerModule::planWaitingApprovalWithGoalModification()
{
  waitApproval();

  BehaviorModuleOutput out;
  out.modified_goal = plan().modified_goal;  // update status_
  out.path = std::make_shared<PathWithLaneId>(generateStopPath());
  out.reference_path = getPreviousModuleOutput().reference_path;
  path_candidate_ =
    status_.get_is_safe_static_objects()
      ? std::make_shared<PathWithLaneId>(status_.get_pull_over_path()->getFullPath())
      : out.path;
  path_reference_ = getPreviousModuleOutput().reference_path;
  const auto distance_to_path_change = calcDistanceToPathChange();

  // generate drivable area info for new architecture
  if (status_.get_pull_over_path()->type == PullOverPlannerType::FREESPACE) {
    const double drivable_area_margin = planner_data_->parameters.vehicle_width;
    out.drivable_area_info.drivable_margin =
      planner_data_->parameters.vehicle_width / 2.0 + drivable_area_margin;
  } else {
    const auto target_drivable_lanes = utils::getNonOverlappingExpandedLanes(
      *out.path, status_.get_lanes(), planner_data_->drivable_area_expansion_parameters);

    DrivableAreaInfo current_drivable_area_info;
    current_drivable_area_info.drivable_lanes = target_drivable_lanes;
    out.drivable_area_info = utils::combineDrivableAreaInfo(
      current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  }

  if (status_.get_has_decided_path()) {
    updateRTCStatus(distance_to_path_change.first, distance_to_path_change.second);
  }
  updateSteeringFactor(
    {status_.get_pull_over_path()->start_pose, status_.get_modified_goal_pose()->goal_pose},
    {distance_to_path_change.first, distance_to_path_change.second}, SteeringFactor::APPROACHING);

  setStopReason(StopReason::GOAL_PLANNER, status_.get_pull_over_path()->getFullPath());

  return out;
}

std::pair<double, double> GoalPlannerModule::calcDistanceToPathChange() const
{
  const auto & full_path = status_.get_pull_over_path()->getFullPath();

  const auto ego_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, planner_data_->self_odometry->pose.pose, std::numeric_limits<double>::max(),
    M_PI_2);
  if (!ego_segment_idx) {
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  }

  const size_t start_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, status_.get_pull_over_path()->start_pose.position);
  const double dist_to_parking_start_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_odometry->pose.pose.position, *ego_segment_idx,
    status_.get_pull_over_path()->start_pose.position, start_pose_segment_idx);
  const size_t goal_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, status_.get_modified_goal_pose()->goal_pose.position);
  const double dist_to_parking_finish_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_odometry->pose.pose.position, *ego_segment_idx,
    status_.get_modified_goal_pose()->goal_pose.position, goal_pose_segment_idx);

  return {dist_to_parking_start_pose, dist_to_parking_finish_pose};
}

void GoalPlannerModule::setParameters(const std::shared_ptr<GoalPlannerParameters> & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId GoalPlannerModule::generateStopPath()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & common_parameters = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;
  const double pull_over_velocity = parameters_->pull_over_velocity;

  if (status_.get_current_lanes().empty()) {
    return PathWithLaneId{};
  }

  // generate reference path
  const auto s_current =
    lanelet::utils::getArcCoordinates(status_.get_current_lanes(), current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto reference_path =
    route_handler->getCenterLinePath(status_.get_current_lanes(), s_start, s_end, true);

  // if not approved stop road lane.
  // stop point priority is
  // 1. actual start pose
  // 2. closest candidate start pose
  // 3. pose offset by approximate_pull_over_distance_ from search start pose.
  //     (In the case of the curve lane, the position is not aligned due to the
  //     difference between the outer and inner sides)
  // 4. feasible stop
  const auto search_start_offset_pose = calcLongitudinalOffsetPose(
    reference_path.points, status_.get_refined_goal_pose().position,
    -parameters_->backward_goal_search_length - common_parameters.base_link2front -
      approximate_pull_over_distance_);
  if (
    !status_.get_is_safe_static_objects() && !status_.get_closest_start_pose() &&
    !search_start_offset_pose) {
    return generateFeasibleStopPath();
  }

  const Pose stop_pose =
    status_.get_is_safe_static_objects()
      ? status_.get_pull_over_path()->start_pose
      : (status_.get_closest_start_pose() ? status_.get_closest_start_pose().value()
                                          : *search_start_offset_pose);

  // if stop pose is closer than min_stop_distance, stop as soon as possible
  const double ego_to_stop_distance = calcSignedArcLengthFromEgo(reference_path, stop_pose);
  const auto min_stop_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, 0.0);
  const double eps_vel = 0.01;
  const bool is_stopped = std::abs(current_vel) < eps_vel;
  const double buffer = is_stopped ? stop_distance_buffer_ : 0.0;
  if (min_stop_distance && ego_to_stop_distance + buffer < *min_stop_distance) {
    return generateFeasibleStopPath();
  }

  // slow down for turn signal, insert stop point to stop_pose
  decelerateForTurnSignal(stop_pose, reference_path);
  stop_pose_ = stop_pose;

  // slow down before the search area.
  if (search_start_offset_pose) {
    decelerateBeforeSearchStart(*search_start_offset_pose, reference_path);
  } else {
    // if already passed the search start offset pose, set pull_over_velocity to reference_path.
    const auto min_decel_distance = calcFeasibleDecelDistance(
      planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk,
      pull_over_velocity);
    for (auto & p : reference_path.points) {
      const double distance_from_ego = calcSignedArcLengthFromEgo(reference_path, p.point.pose);
      if (min_decel_distance && distance_from_ego < *min_decel_distance) {
        continue;
      }
      p.point.longitudinal_velocity_mps =
        std::min(p.point.longitudinal_velocity_mps, static_cast<float>(pull_over_velocity));
    }
  }

  return reference_path;
}

PathWithLaneId GoalPlannerModule::generateFeasibleStopPath()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & common_parameters = planner_data_->parameters;

  // generate stop reference path
  const auto s_current =
    lanelet::utils::getArcCoordinates(status_.get_current_lanes(), current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto stop_path =
    route_handler->getCenterLinePath(status_.get_current_lanes(), s_start, s_end, true);

  // calc minimum stop distance under maximum deceleration
  const auto min_stop_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, 0.0);
  if (!min_stop_distance) {
    return stop_path;
  }

  // set stop point
  const auto stop_idx =
    motion_utils::insertStopPoint(current_pose, *min_stop_distance, stop_path.points);
  if (stop_idx) {
    stop_pose_ = stop_path.points.at(*stop_idx).point.pose;
  }

  return stop_path;
}

void GoalPlannerModule::transitionToNextPathIfFinishingCurrentPath()
{
  if (isActivated() && status_.get_last_approved_time()) {
    // if using arc_path and finishing current_path, get next path
    // enough time for turn signal
    const bool has_passed_enough_time =
      (clock_->now() - *status_.get_last_approved_time()).seconds() >
      planner_data_->parameters.turn_signal_search_time;

    if (hasFinishedCurrentPath() && has_passed_enough_time && status_.get_require_increment()) {
      if (incrementPathIndex()) {
        status_.set_last_increment_time(std::make_shared<rclcpp::Time>(clock_->now()));
      }
    }
  }
}

bool GoalPlannerModule::incrementPathIndex()
{
  if (status_.get_current_path_idx() == status_.get_pull_over_path()->partial_paths.size() - 1) {
    return false;
  }
  status_.set_current_path_idx(status_.get_current_path_idx() + 1);
  return true;
}

PathWithLaneId GoalPlannerModule::getCurrentPath() const
{
  if (status_.get_pull_over_path() == nullptr) {
    return PathWithLaneId{};
  }

  if (status_.get_pull_over_path()->partial_paths.size() <= status_.get_current_path_idx()) {
    return PathWithLaneId{};
  }
  return status_.get_pull_over_path()->partial_paths.at(status_.get_current_path_idx());
}

bool GoalPlannerModule::isStopped(
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
    const double ego_vel = utils::l2Norm(odometry->twist.twist.linear);
    if (ego_vel > parameters_->th_stopped_velocity) {
      is_stopped = false;
      break;
    }
  }
  return is_stopped;
}

bool GoalPlannerModule::isStopped()
{
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  return isStopped(odometry_buffer_stopped_, parameters_->th_stopped_time);
}

bool GoalPlannerModule::isStuck()
{
  if (isOnModifiedGoal()) {
    return false;
  }

  constexpr double stuck_time = 5.0;
  if (!isStopped(odometry_buffer_stuck_, stuck_time)) {
    return false;
  }

  // not found safe path
  if (!status_.get_is_safe_static_objects()) {
    return true;
  }

  // any path has never been found
  if (!status_.get_pull_over_path()) {
    return false;
  }

  return checkCollision(getCurrentPath());
}

bool GoalPlannerModule::hasFinishedCurrentPath()
{
  const auto current_path_end = getCurrentPath().points.back();
  const auto & self_pose = planner_data_->self_odometry->pose.pose;
  const bool is_near_target = tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
                              parameters_->th_arrived_distance;

  return is_near_target && isStopped();
}

bool GoalPlannerModule::isOnModifiedGoal() const
{
  if (!status_.get_modified_goal_pose()) {
    return false;
  }

  const Pose current_pose = planner_data_->self_odometry->pose.pose;
  return calcDistance2d(current_pose, status_.get_modified_goal_pose()->goal_pose) <
         parameters_->th_arrived_distance;
}

TurnSignalInfo GoalPlannerModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal{};  // output

  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & start_pose = status_.get_pull_over_path()->start_pose;
  const auto & end_pose = status_.get_pull_over_path()->end_pose;
  const auto full_path = status_.get_pull_over_path()->getFullPath();

  // calc TurnIndicatorsCommand
  {
    const double distance_to_end =
      calcSignedArcLength(full_path.points, current_pose.position, end_pose.position);
    const bool is_before_end_pose = distance_to_end >= 0.0;
    if (is_before_end_pose) {
      if (left_side_parking_) {
        turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
      } else {
        turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
      }
    } else {
      turn_signal.turn_signal.command = TurnIndicatorsCommand::NO_COMMAND;
    }
  }

  // calc desired/required start/end point
  {
    // ego decelerates so that current pose is the point `turn_light_on_threshold_time` seconds
    // before starting pull_over
    turn_signal.desired_start_point =
      status_.get_last_approved_pose() && status_.get_has_decided_path()
        ? *status_.get_last_approved_pose()
        : current_pose;
    turn_signal.desired_end_point = end_pose;
    turn_signal.required_start_point = start_pose;
    turn_signal.required_end_point = end_pose;
  }

  return turn_signal;
}

bool GoalPlannerModule::checkCollision(const PathWithLaneId & path) const
{
  if (parameters_->use_occupancy_grid_for_path_collision_check && occupancy_grid_map_) {
    const bool check_out_of_range = false;
    if (occupancy_grid_map_->hasObstacleOnPath(path, check_out_of_range)) {
      return true;
    }
  }
  if (!parameters_->use_object_recognition) {
    return false;
  }

  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *(planner_data_->route_handler), left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);
  const auto [pull_over_lane_objects, others] =
    utils::path_safety_checker::separateObjectsByLanelets(
      *(planner_data_->dynamic_object), pull_over_lanes,
      utils::path_safety_checker::isPolygonOverlapLanelet);
  const auto pull_over_lane_stop_objects = utils::path_safety_checker::filterObjectsByVelocity(
    pull_over_lane_objects, parameters_->th_moving_object_velocity);
  std::vector<Polygon2d> obj_polygons;
  for (const auto & object : pull_over_lane_stop_objects.objects) {
    obj_polygons.push_back(tier4_autoware_utils::toPolygon2d(object));
  }

  std::vector<Polygon2d> ego_polygons_expanded;
  const auto curvatures = motion_utils::calcCurvature(path.points);
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto p = path.points.at(i);

    const double extra_stopping_margin = std::min(
      std::pow(p.point.longitudinal_velocity_mps, 2) * 0.5 / parameters_->maximum_deceleration,
      parameters_->object_recognition_collision_check_max_extra_stopping_margin);

    double extra_lateral_margin = (-1) * curvatures.at(i) * p.point.longitudinal_velocity_mps *
                                  std::abs(p.point.longitudinal_velocity_mps);
    extra_lateral_margin =
      std::clamp(extra_lateral_margin, -extra_stopping_margin, extra_stopping_margin);

    const auto lateral_offset_pose =
      tier4_autoware_utils::calcOffsetPose(p.point.pose, 0.0, extra_lateral_margin / 2.0, 0.0);
    const auto ego_polygon = tier4_autoware_utils::toFootprint(
      lateral_offset_pose,
      planner_data_->parameters.base_link2front +
        parameters_->object_recognition_collision_check_margin + extra_stopping_margin,
      planner_data_->parameters.base_link2rear +
        parameters_->object_recognition_collision_check_margin,
      planner_data_->parameters.vehicle_width +
        parameters_->object_recognition_collision_check_margin * 2.0 +
        std::abs(extra_lateral_margin));
    ego_polygons_expanded.push_back(ego_polygon);
  }
  debug_data_.ego_polygons_expanded = ego_polygons_expanded;

  return utils::path_safety_checker::checkPolygonsIntersects(ego_polygons_expanded, obj_polygons);
}

bool GoalPlannerModule::hasEnoughDistance(const PullOverPath & pull_over_path) const
{
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  // when the path is separated and start_pose is close,
  // once stopped, the vehicle cannot start again.
  // so need enough distance to restart.
  // distance to restart should be less than decide_path_distance.
  // otherwise, the goal would change immediately after departure.
  const bool is_separated_path = pull_over_path.partial_paths.size() > 1;
  const double distance_to_start = calcSignedArcLength(
    pull_over_path.getFullPath().points, current_pose.position, pull_over_path.start_pose.position);
  const double distance_to_restart = parameters_->decide_path_distance / 2;
  const double eps_vel = 0.01;
  const bool is_stopped = std::abs(current_vel) < eps_vel;
  if (is_separated_path && is_stopped && distance_to_start < distance_to_restart) {
    return false;
  }

  const auto current_to_stop_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, 0.0);
  if (!current_to_stop_distance) {
    return false;
  }

  // If the stop line is subtly exceeded, it is assumed that there is not enough distance to the
  // starting point of parking, so to prevent this, once the vehicle has stopped, it also has a
  // stop_distance_buffer to allow for the amount exceeded.
  const double buffer = is_stopped ? stop_distance_buffer_ : 0.0;
  if (distance_to_start + buffer < *current_to_stop_distance) {
    return false;
  }

  return true;
}

void GoalPlannerModule::keepStoppedWithCurrentPath(PathWithLaneId & path)
{
  constexpr double keep_stop_time = 2.0;
  constexpr double keep_current_idx_buffer_time = 2.0;
  if (status_.get_last_increment_time()) {
    const auto time_diff = (clock_->now() - *status_.get_last_increment_time()).seconds();
    if (time_diff < keep_stop_time) {
      status_.set_require_increment(false);
      for (auto & p : path.points) {
        p.point.longitudinal_velocity_mps = 0.0;
      }
    } else if (time_diff > keep_stop_time + keep_current_idx_buffer_time) {
      // require increment only when the time passed is enough
      // to prevent increment before driving
      // when the end of the current path is close to the current pose
      status_.set_require_increment(true);
    }
  }
}

double GoalPlannerModule::calcSignedArcLengthFromEgo(
  const PathWithLaneId & path, const Pose & pose) const
{
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & p = planner_data_->parameters;

  const size_t ego_idx = planner_data_->findEgoIndex(path.points);
  const size_t target_idx = findFirstNearestSegmentIndexWithSoftConstraints(
    path.points, pose, p.ego_nearest_dist_threshold, p.ego_nearest_yaw_threshold);
  return calcSignedArcLength(
    path.points, current_pose.position, ego_idx, pose.position, target_idx);
}

void GoalPlannerModule::decelerateForTurnSignal(const Pose & stop_pose, PathWithLaneId & path) const
{
  const double time = planner_data_->parameters.turn_signal_search_time;
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  for (auto & point : path.points) {
    const double distance_to_stop = std::max(
      0.0, calcSignedArcLength(path.points, point.point.pose.position, stop_pose.position));
    const float decel_vel =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(distance_to_stop / time));
    const double distance_from_ego = calcSignedArcLengthFromEgo(path, point.point.pose);
    const auto min_decel_distance = calcFeasibleDecelDistance(
      planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, decel_vel);

    // when current velocity already lower than decel_vel, min_decel_distance will be 0.0,
    // and do not need to decelerate.
    // skip next process to avoid inserting decel point at the same current position.
    constexpr double eps_distance = 0.1;
    if (!min_decel_distance || *min_decel_distance < eps_distance) {
      continue;
    }

    if (*min_decel_distance < distance_from_ego) {
      point.point.longitudinal_velocity_mps = decel_vel;
    } else {
      insertDecelPoint(current_pose.position, *min_decel_distance, decel_vel, path.points);
    }
  }

  const double stop_point_length = calcSignedArcLength(path.points, 0, stop_pose.position);
  const auto min_stop_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, 0.0);

  if (min_stop_distance && *min_stop_distance < stop_point_length) {
    const auto stop_point = utils::insertStopPoint(stop_point_length, path);
  }
}

void GoalPlannerModule::decelerateBeforeSearchStart(
  const Pose & search_start_offset_pose, PathWithLaneId & path) const
{
  const double pull_over_velocity = parameters_->pull_over_velocity;
  const Pose & current_pose = planner_data_->self_odometry->pose.pose;

  // slow down before the search area.
  const auto min_decel_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk,
    pull_over_velocity);
  if (min_decel_distance) {
    const double distance_to_search_start =
      calcSignedArcLengthFromEgo(path, search_start_offset_pose);
    const double distance_to_decel =
      std::max(*min_decel_distance, distance_to_search_start - approximate_pull_over_distance_);
    insertDecelPoint(current_pose.position, distance_to_decel, pull_over_velocity, path.points);
  }
}

bool GoalPlannerModule::isCrossingPossible(
  const lanelet::ConstLanelet & start_lane, const lanelet::ConstLanelet & end_lane) const
{
  if (start_lane.centerline().empty() || end_lane.centerline().empty()) {
    return false;
  }

  if (start_lane == end_lane) {
    return true;
  }

  const auto & route_handler = planner_data_->route_handler;

  lanelet::ConstLanelets start_lane_sequence = route_handler->getLaneletSequence(start_lane);

  // get end lane sequence based on whether it is shoulder lanelet or not
  lanelet::ConstLanelets end_lane_sequence{};
  const bool is_shoulder_lane = route_handler->isShoulderLanelet(end_lane);
  if (is_shoulder_lane) {
    Pose end_lane_pose{};
    end_lane_pose.orientation.w = 1.0;
    end_lane_pose.position = lanelet::utils::conversion::toGeomMsgPt(end_lane.centerline().front());
    end_lane_sequence = route_handler->getShoulderLaneletSequence(end_lane, end_lane_pose);
  } else {
    const double dist = std::numeric_limits<double>::max();
    end_lane_sequence = route_handler->getLaneletSequence(end_lane, dist, dist, false);
  }

  // Lambda function to get the neighboring lanelet based on left_side_parking_
  auto getNeighboringLane =
    [&](const lanelet::ConstLanelet & lane) -> boost::optional<lanelet::ConstLanelet> {
    lanelet::ConstLanelet neighboring_lane{};
    if (left_side_parking_) {
      if (route_handler->getLeftShoulderLanelet(lane, &neighboring_lane)) {
        return neighboring_lane;
      } else {
        return route_handler->getLeftLanelet(lane);
      }
    } else {
      if (route_handler->getRightShoulderLanelet(lane, &neighboring_lane)) {
        return neighboring_lane;
      } else {
        return route_handler->getRightLanelet(lane);
      }
    }
  };

  // Iterate through start_lane_sequence to find a path to end_lane_sequence
  for (auto it = start_lane_sequence.rbegin(); it != start_lane_sequence.rend(); ++it) {
    lanelet::ConstLanelet current_lane = *it;

    // Check if the current lane is in the end_lane_sequence
    auto end_it = std::find(end_lane_sequence.rbegin(), end_lane_sequence.rend(), current_lane);
    if (end_it != end_lane_sequence.rend()) {
      return true;
    }

    // Traversing is not allowed between road lanes
    if (!is_shoulder_lane) {
      continue;
    }

    // Traverse the lanes horizontally until the end_lane_sequence is reached
    boost::optional<lanelet::ConstLanelet> neighboring_lane = getNeighboringLane(current_lane);
    if (neighboring_lane) {
      // Check if the neighboring lane is in the end_lane_sequence
      end_it =
        std::find(end_lane_sequence.rbegin(), end_lane_sequence.rend(), neighboring_lane.get());
      if (end_it != end_lane_sequence.rend()) {
        return true;
      }
    }
  }

  return false;
}

bool GoalPlannerModule::isCrossingPossible(
  const Pose & start_pose, const Pose & end_pose, const lanelet::ConstLanelets lanes) const
{
  lanelet::ConstLanelet start_lane{};
  lanelet::utils::query::getClosestLanelet(lanes, start_pose, &start_lane);

  lanelet::ConstLanelet end_lane{};
  lanelet::utils::query::getClosestLanelet(lanes, end_pose, &end_lane);

  return isCrossingPossible(start_lane, end_lane);
}

bool GoalPlannerModule::isCrossingPossible(const PullOverPath & pull_over_path) const
{
  const lanelet::ConstLanelets lanes = utils::transformToLanelets(status_.get_lanes());
  const Pose & start_pose = pull_over_path.start_pose;
  const Pose & end_pose = pull_over_path.end_pose;

  return isCrossingPossible(start_pose, end_pose, lanes);
}

void GoalPlannerModule::updateSafetyCheckTargetObjectsData(
  const PredictedObjects & filtered_objects, const TargetObjectsOnLane & target_objects_on_lane,
  const std::vector<PoseWithVelocityStamped> & ego_predicted_path) const
{
  goal_planner_data_.filtered_objects = filtered_objects;
  goal_planner_data_.target_objects_on_lane = target_objects_on_lane;
  goal_planner_data_.ego_predicted_path = ego_predicted_path;
}

bool GoalPlannerModule::isSafePath() const
{
  const auto pull_over_path = getCurrentPath();
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const double current_velocity = std::hypot(
    planner_data_->self_odometry->twist.twist.linear.x,
    planner_data_->self_odometry->twist.twist.linear.y);
  const auto & dynamic_object = planner_data_->dynamic_object;
  const auto & route_handler = planner_data_->route_handler;
  const double backward_path_length = planner_data_->parameters.backward_path_length;
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);
  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(pull_over_path.points);
  const std::pair<double, double> terminal_velocity_and_accel =
    utils::start_goal_planner_common::getPairsTerminalVelocityAndAccel(
      status_.get_pull_over_path()->pairs_terminal_velocity_and_accel,
      status_.get_current_path_idx());
  RCLCPP_DEBUG(
    getLogger(), "pairs_terminal_velocity_and_accel for goal_planner: %f, %f",
    terminal_velocity_and_accel.first, terminal_velocity_and_accel.second);
  RCLCPP_DEBUG(getLogger(), "current_path_idx %ld", status_.get_current_path_idx());
  utils::start_goal_planner_common::updatePathProperty(
    ego_predicted_path_params_, terminal_velocity_and_accel);
  // TODO(Sugahara): shoule judge is_object_front properly
  const bool is_object_front = true;
  const bool limit_to_max_velocity = true;
  const auto ego_predicted_path =
    behavior_path_planner::utils::path_safety_checker::createPredictedPath(
      ego_predicted_path_params_, pull_over_path.points, current_pose, current_velocity,
      ego_seg_idx, is_object_front, limit_to_max_velocity);

  // filtering objects with velocity, position and class
  const auto & filtered_objects = utils::path_safety_checker::filterObjects(
    dynamic_object, route_handler, pull_over_lanes, current_pose.position,
    objects_filtering_params_);

  // filtering objects based on the current position's lane
  const auto & target_objects_on_lane = utils::path_safety_checker::createTargetObjectsOnLane(
    pull_over_lanes, route_handler, filtered_objects, objects_filtering_params_);

  const double hysteresis_factor =
    status_.get_prev_is_safe_dynamic_objects() ? 1.0 : parameters_->hysteresis_factor_expand_rate;

  utils::start_goal_planner_common::updateSafetyCheckTargetObjectsData(
    goal_planner_data_, filtered_objects, target_objects_on_lane, ego_predicted_path);

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
            pull_over_path, ego_predicted_path, object, obj_path, planner_data_->parameters,
            safety_check_params_->rss_params, hysteresis_factor, current_debug_data.second)) {
        marker_utils::updateCollisionCheckDebugMap(
          goal_planner_data_.collision_check, current_debug_data, false);
        is_safe_dynamic_objects = false;
        is_safe_dynamic_object = false;
        break;
      }
    }
    if (is_safe_dynamic_object) {
      marker_utils::updateCollisionCheckDebugMap(
        goal_planner_data_.collision_check, current_debug_data, is_safe_dynamic_object);
    }
  }

  return is_safe_dynamic_objects;
}

void GoalPlannerModule::setDebugData()
{
  debug_marker_.markers.clear();

  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createPredictedPathMarkerArray;
  using marker_utils::showPolygon;
  using marker_utils::showPredictedPath;
  using marker_utils::showSafetyCheckInfo;
  using motion_utils::createStopVirtualWallMarker;
  using tier4_autoware_utils::createDefaultMarker;
  using tier4_autoware_utils::createMarkerColor;
  using tier4_autoware_utils::createMarkerScale;

  const auto header = planner_data_->route_handler->getRouteHeader();

  const auto add = [this](MarkerArray added) {
    for (auto & marker : added.markers) {
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    }
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };
  if (goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    // Visualize pull over areas
    const auto color = status_.get_has_decided_path()
                         ? createMarkerColor(1.0, 1.0, 0.0, 0.999)   // yellow
                         : createMarkerColor(0.0, 1.0, 0.0, 0.999);  // green
    const double z = status_.get_refined_goal_pose().position.z;
    add(goal_planner_utils::createPullOverAreaMarkerArray(
      goal_searcher_->getAreaPolygons(), header, color, z));

    // Visualize goal candidates
    const auto goal_candidates = status_.get_goal_candidates();
    add(goal_planner_utils::createGoalCandidatesMarkerArray(goal_candidates, color));
  }

  // Visualize path and related pose
  if (status_.get_is_safe_static_objects()) {
    add(createPoseMarkerArray(
      status_.get_pull_over_path()->start_pose, "pull_over_start_pose", 0, 0.3, 0.3, 0.9));
    add(createPoseMarkerArray(
      status_.get_pull_over_path()->end_pose, "pull_over_end_pose", 0, 0.3, 0.3, 0.9));
    add(createPathMarkerArray(
      status_.get_pull_over_path()->getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
    add(createPathMarkerArray(getCurrentPath(), "current_path", 0, 0.9, 0.5, 0.0));

    // visualize each partial path
    for (size_t i = 0; i < status_.get_pull_over_path()->partial_paths.size(); ++i) {
      const auto & partial_path = status_.get_pull_over_path()->partial_paths.at(i);
      add(
        createPathMarkerArray(partial_path, "partial_path_" + std::to_string(i), 0, 0.9, 0.5, 0.9));
    }

    auto marker = tier4_autoware_utils::createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "detection_polygons", 0, Marker::LINE_LIST,
      tier4_autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      tier4_autoware_utils::createMarkerColor(0.0, 0.0, 1.0, 0.999));

    for (const auto & ego_polygon : debug_data_.ego_polygons_expanded) {
      for (size_t ep_idx = 0; ep_idx < ego_polygon.outer().size(); ++ep_idx) {
        const auto & current_point = ego_polygon.outer().at(ep_idx);
        const auto & next_point = ego_polygon.outer().at((ep_idx + 1) % ego_polygon.outer().size());

        marker.points.push_back(
          tier4_autoware_utils::createPoint(current_point.x(), current_point.y(), 0.0));
        marker.points.push_back(
          tier4_autoware_utils::createPoint(next_point.x(), next_point.y(), 0.0));
      }
    }
    debug_marker_.markers.push_back(marker);
  }
  // safety check
  if (parameters_->safety_check_params.enable_safety_check) {
    if (goal_planner_data_.ego_predicted_path.size() > 0) {
      const auto & ego_predicted_path = utils::path_safety_checker::convertToPredictedPath(
        goal_planner_data_.ego_predicted_path, ego_predicted_path_params_->time_resolution);
      add(createPredictedPathMarkerArray(
        ego_predicted_path, vehicle_info_, "ego_predicted_path_goal_planner", 0, 0.0, 0.5, 0.9));
    }
    if (goal_planner_data_.filtered_objects.objects.size() > 0) {
      add(createObjectsMarkerArray(
        goal_planner_data_.filtered_objects, "filtered_objects", 0, 0.0, 0.5, 0.9));
    }

    add(showSafetyCheckInfo(goal_planner_data_.collision_check, "object_debug_info"));
    add(showPredictedPath(goal_planner_data_.collision_check, "ego_predicted_path"));
    add(showPolygon(goal_planner_data_.collision_check, "ego_and_target_polygon_relation"));
    utils::start_goal_planner_common::initializeCollisionCheckDebugMap(
      goal_planner_data_.collision_check);
  }

  // Visualize planner type text
  {
    visualization_msgs::msg::MarkerArray planner_type_marker_array{};
    const auto color = status_.get_is_safe_static_objects()
                         ? createMarkerColor(1.0, 1.0, 1.0, 0.99)
                         : createMarkerColor(1.0, 0.0, 0.0, 0.99);
    auto marker = createDefaultMarker(
      header.frame_id, header.stamp, "planner_type", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0), color);
    marker.pose = status_.get_modified_goal_pose() ? status_.get_modified_goal_pose()->goal_pose
                                                   : planner_data_->self_odometry->pose.pose;
    marker.text = magic_enum::enum_name(status_.get_pull_over_path()->type);
    marker.text += " " + std::to_string(status_.get_current_path_idx()) + "/" +
                   std::to_string(status_.get_pull_over_path()->partial_paths.size() - 1);
    if (isStuck()) {
      marker.text += " stuck";
    } else if (isStopped()) {
      marker.text += " stopped";
    }

    if (debug_data_.freespace_planner.is_planning) {
      marker.text +=
        " freespace: " + std::to_string(debug_data_.freespace_planner.current_goal_idx) + "/" +
        std::to_string(debug_data_.freespace_planner.num_goal_candidates);
    }

    planner_type_marker_array.markers.push_back(marker);
    add(planner_type_marker_array);
  }

  // Visualize debug poses
  const auto & debug_poses = status_.get_pull_over_path()->debug_poses;
  for (size_t i = 0; i < debug_poses.size(); ++i) {
    add(createPoseMarkerArray(
      debug_poses.at(i), "debug_pose_" + std::to_string(i), 0, 0.3, 0.3, 0.3));
  }
}

void GoalPlannerModule::printParkingPositionError() const
{
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const double real_shoulder_to_map_shoulder = 0.0;

  const Pose goal_to_ego =
    inverseTransformPose(current_pose, status_.get_modified_goal_pose()->goal_pose);
  const double dx = goal_to_ego.position.x;
  const double dy = goal_to_ego.position.y;
  const double distance_from_real_shoulder =
    real_shoulder_to_map_shoulder + parameters_->margin_from_boundary - dy;
  RCLCPP_INFO(
    getLogger(), "current pose to goal, dx:%f dy:%f dyaw:%f from_real_shoulder:%f", dx, dy,
    tier4_autoware_utils::rad2deg(
      tf2::getYaw(current_pose.orientation) -
      tf2::getYaw(status_.get_modified_goal_pose()->goal_pose.orientation)),
    distance_from_real_shoulder);
}

bool GoalPlannerModule::checkOriginalGoalIsInShoulder() const
{
  const auto & route_handler = planner_data_->route_handler;
  const Pose & goal_pose = route_handler->getOriginalGoalPose();

  const lanelet::ConstLanelets pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *(route_handler), left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);
  lanelet::ConstLanelet target_lane{};
  lanelet::utils::query::getClosestLanelet(pull_over_lanes, goal_pose, &target_lane);

  return route_handler->isShoulderLanelet(target_lane) &&
         lanelet::utils::isInLanelet(goal_pose, target_lane, 0.1);
}

bool GoalPlannerModule::needPathUpdate(const double path_update_duration) const
{
  return !isOnModifiedGoal() && hasEnoughTimePassedSincePathUpdate(path_update_duration);
}

bool GoalPlannerModule::hasEnoughTimePassedSincePathUpdate(const double duration) const
{
  if (!status_.get_last_path_update_time()) {
    return true;
  }

  return (clock_->now() - *status_.get_last_path_update_time()).seconds() > duration;
}
}  // namespace behavior_path_planner
