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

#include "behavior_path_goal_planner_module/goal_planner_module.hpp"

#include "behavior_path_goal_planner_module/util.hpp"
#include "behavior_path_planner_common/utils/create_vehicle_footprint.hpp"
#include "behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_planner_common/utils/path_utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"
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
#include <optional>
#include <string>
#include <utility>
#include <vector>

using behavior_path_planner::utils::parking_departure::calcFeasibleDecelDistance;
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
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map},  // NOLINT
  parameters_{parameters},
  vehicle_info_{vehicle_info_util::VehicleInfoUtil(node).getVehicleInfo()},
  thread_safe_data_{mutex_, clock_}
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
    utils::parking_departure::initializeCollisionCheckDebugMap(goal_planner_data_.collision_check);
  }

  prev_data_.reset();
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
  if (!thread_safe_data_.get_pull_over_path_candidates().empty()) {
    return;
  }

  // goals are not yet available.
  if (thread_safe_data_.get_goal_candidates().empty()) {
    return;
  }

  if (
    !planner_data_ ||
    !goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return;
  }

  if (getCurrentStatus() == ModuleStatus::IDLE) {
    return;
  }

  const auto goal_candidates = thread_safe_data_.get_goal_candidates();

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
      pull_over_path->id = path_candidates.size();
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
    thread_safe_data_.set_pull_over_path_candidates(path_candidates);
    thread_safe_data_.set_closest_start_pose(closest_start_pose);
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

void GoalPlannerModule::updateEgoPredictedPathParams(
  std::shared_ptr<EgoPredictedPathParams> & ego_predicted_path_params,
  const std::shared_ptr<GoalPlannerParameters> & goal_planner_params)
{
  ego_predicted_path_params =
    std::make_shared<EgoPredictedPathParams>(goal_planner_params->ego_predicted_path_params);
}

void GoalPlannerModule::updateSafetyCheckParams(
  std::shared_ptr<SafetyCheckParams> & safety_check_params,
  const std::shared_ptr<GoalPlannerParameters> & goal_planner_params)
{
  safety_check_params =
    std::make_shared<SafetyCheckParams>(goal_planner_params->safety_check_params);
}

void GoalPlannerModule::updateObjectsFilteringParams(
  std::shared_ptr<ObjectsFilteringParams> & objects_filtering_params,
  const std::shared_ptr<GoalPlannerParameters> & goal_planner_params)
{
  objects_filtering_params =
    std::make_shared<ObjectsFilteringParams>(goal_planner_params->objects_filtering_params);
}

void GoalPlannerModule::updateData()
{
  if (getCurrentStatus() == ModuleStatus::IDLE && !isExecutionRequested()) {
    return;
  }

  // Initialize Occupancy Grid Map
  // This operation requires waiting for `planner_data_`, hence it is executed here instead of in
  // the constructor. Ideally, this operation should only need to be performed once.
  if (
    parameters_->use_occupancy_grid_for_goal_search ||
    parameters_->use_occupancy_grid_for_path_collision_check) {
    initializeOccupancyGridMap();
  }

  resetPathCandidate();
  resetPathReference();
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  updateOccupancyGrid();

  // update goal searcher and generate goal candidates
  if (thread_safe_data_.get_goal_candidates().empty()) {
    goal_searcher_->setPlannerData(planner_data_);
    goal_searcher_->setReferenceGoal(
      calcRefinedGoal(planner_data_->route_handler->getOriginalGoalPose()));
    thread_safe_data_.set_goal_candidates(generateGoalCandidates());
  }

  if (!isActivated()) {
    return;
  }

  if (hasFinishedCurrentPath()) {
    thread_safe_data_.incrementPathIndex();
  }

  if (!last_approval_data_) {
    last_approval_data_ =
      std::make_unique<LastApprovalData>(clock_->now(), planner_data_->self_odometry->pose.pose);
    decideVelocity();
  }
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
  updateEgoPredictedPathParams(ego_predicted_path_params_, parameters_);
  updateSafetyCheckParams(safety_check_params_, parameters_);
  updateObjectsFilteringParams(objects_filtering_params_, parameters_);
}

void GoalPlannerModule::processOnExit()
{
  resetPathCandidate();
  resetPathReference();
  debug_marker_.markers.clear();
  prev_data_.reset();
  last_approval_data_.reset();
}

bool GoalPlannerModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  // TODO(someone): if goal is behind of ego, do not execute goal_planner

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
    if (!isSafePath().first) {
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

bool GoalPlannerModule::planFreespacePath()
{
  GoalCandidates goal_candidates{};
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    goal_searcher_->setPlannerData(planner_data_);
    goal_candidates = thread_safe_data_.get_goal_candidates();
    goal_searcher_->update(goal_candidates);
    thread_safe_data_.set_goal_candidates(goal_candidates);
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
      thread_safe_data_.set_pull_over_path(*freespace_path);
      thread_safe_data_.set_modified_goal_pose(goal_candidate);
      debug_data_.freespace_planner.is_planning = false;
    }

    return true;
  }

  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  debug_data_.freespace_planner.is_planning = false;
  return false;
}

bool GoalPlannerModule::canReturnToLaneParking()
{
  // return only before starting free space parking
  if (!isStopped()) {
    return false;
  }

  if (!thread_safe_data_.get_lane_parking_pull_over_path()) {
    return false;
  }

  const PathWithLaneId path = thread_safe_data_.get_lane_parking_pull_over_path()->getFullPath();

  if (
    parameters_->use_object_recognition &&
    checkObjectsCollision(path, parameters_->object_recognition_collision_check_margin)) {
    return false;
  }

  if (
    parameters_->use_occupancy_grid_for_path_collision_check && checkOccupancyGridCollision(path)) {
    return false;
  }

  const Point & current_point = planner_data_->self_odometry->pose.pose.position;
  constexpr double th_distance = 0.5;
  const bool is_close_to_path =
    std::abs(motion_utils::calcLateralOffset(path.points, current_point)) < th_distance;
  if (!is_close_to_path) {
    return false;
  }

  return true;
}

GoalCandidates GoalPlannerModule::generateGoalCandidates() const
{
  // calculate goal candidates
  const auto & route_handler = planner_data_->route_handler;
  if (goal_planner_utils::isAllowedGoalModification(route_handler)) {
    return goal_searcher_->search();
  }

  // NOTE:
  // currently since pull over is performed only when isAllowedGoalModification is true,
  // never be in the following process.
  GoalCandidate goal_candidate{};
  goal_candidate.goal_pose = route_handler->getOriginalGoalPose();
  goal_candidate.distance_from_original_goal = 0.0;
  GoalCandidates goal_candidates{};
  goal_candidates.push_back(goal_candidate);

  return goal_candidates;
}

BehaviorModuleOutput GoalPlannerModule::plan()
{
  if (goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return planPullOver();
  }

  fixed_goal_planner_->setPreviousModuleOutput(getPreviousModuleOutput());
  return fixed_goal_planner_->plan(planner_data_);
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

  if (parameters_->use_object_recognition) {
    // Create a map of PullOverPath pointer to largest collision check margin
    auto calcLargestMargin = [&](const PullOverPath & pull_over_path) {
      // check has safe goal
      const auto goal_candidate_it = std::find_if(
        goal_candidates.begin(), goal_candidates.end(),
        [pull_over_path](const auto & goal_candidate) {
          return goal_candidate.id == pull_over_path.goal_id;
        });
      if (goal_candidate_it != goal_candidates.end() && !goal_candidate_it->is_safe) {
        return 0.0;
      }
      // calc largest margin
      std::vector<double> scale_factors{3.0, 2.0, 1.0};
      const double margin = parameters_->object_recognition_collision_check_margin;
      const auto resampled_path =
        utils::resamplePathWithSpline(pull_over_path.getParkingPath(), 0.5);
      for (const auto & scale_factor : scale_factors) {
        if (!checkObjectsCollision(resampled_path, margin * scale_factor)) {
          return margin * scale_factor;
        }
      }
      return 0.0;
    };

    // Create a map of PullOverPath pointer to largest collision check margin
    std::map<size_t, double> path_id_to_margin_map;
    for (const auto & path : sorted_pull_over_path_candidates) {
      path_id_to_margin_map[path.id] = calcLargestMargin(path);
    }

    // sorts in descending order so the item with larger margin comes first
    std::stable_sort(
      sorted_pull_over_path_candidates.begin(), sorted_pull_over_path_candidates.end(),
      [&path_id_to_margin_map](const PullOverPath & a, const PullOverPath & b) {
        if (std::abs(path_id_to_margin_map[a.id] - path_id_to_margin_map[b.id]) < 0.01) {
          return false;
        }
        return path_id_to_margin_map[a.id] > path_id_to_margin_map[b.id];
      });
  }

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

std::optional<std::pair<PullOverPath, GoalCandidate>> GoalPlannerModule::selectPullOverPath(
  const std::vector<PullOverPath> & pull_over_path_candidates,
  const GoalCandidates & goal_candidates, const double collision_check_margin) const
{
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

    if (!hasEnoughDistance(pull_over_path)) {
      continue;
    }

    const auto resampled_path = utils::resamplePathWithSpline(pull_over_path.getParkingPath(), 0.5);
    if (
      parameters_->use_object_recognition &&
      checkObjectsCollision(resampled_path, collision_check_margin, true /*update_debug_data*/)) {
      continue;
    }

    if (
      parameters_->use_occupancy_grid_for_path_collision_check &&
      checkOccupancyGridCollision(resampled_path)) {
      continue;
    }

    return std::make_pair(pull_over_path, *goal_candidate_it);
  }

  return {};
}

std::vector<DrivableLanes> GoalPlannerModule::generateDrivableLanes() const
{
  const lanelet::ConstLanelets current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length,
    /*forward_only_in_route*/ false);
  const lanelet::ConstLanelets pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *(planner_data_->route_handler), left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);
  return utils::generateDrivableLanesWithShoulderLanes(current_lanes, pull_over_lanes);
}

void GoalPlannerModule::setOutput(BehaviorModuleOutput & output) const
{
  output.reference_path = getPreviousModuleOutput().reference_path;

  if (!thread_safe_data_.foundPullOverPath()) {
    // situation : not safe against static objects use stop_path
    setStopPath(output);
    setDrivableAreaInfo(output);
    return;
  }

  if (
    parameters_->safety_check_params.enable_safety_check && !isSafePath().first && isActivated()) {
    // situation : not safe against dynamic objects after approval
    // insert stop point in current path if ego is able to stop with acceleration and jerk
    // constraints
    setStopPathFromCurrentPath(output);
  } else {
    // situation : (safe against static and dynamic objects) or (safe against static objects and
    // before approval) don't stop
    // keep stop if not enough time passed,
    // because it takes time for the trajectory to be reflected
    auto current_path = thread_safe_data_.get_pull_over_path()->getCurrentPath();
    keepStoppedWithCurrentPath(current_path);
    output.path = current_path;
  }

  setModifiedGoal(output);
  setDrivableAreaInfo(output);

  // set hazard and turn signal
  if (hasDecidedPath() && isActivated()) {
    setTurnSignalInfo(output);
  }
}

void GoalPlannerModule::setStopPath(BehaviorModuleOutput & output) const
{
  if (prev_data_.found_path || !prev_data_.stop_path) {
    // safe -> not_safe or no prev_stop_path: generate new stop_path
    output.path = generateStopPath();
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull_over path, generate stop path");
  } else {
    // not_safe -> not_safe: use previous stop path
    output.path = *prev_data_.stop_path;
    // stop_pose_ is removed in manager every loop, so need to set every loop.
    stop_pose_ = utils::getFirstStopPoseFromPath(output.path);
    RCLCPP_WARN_THROTTLE(
      getLogger(), *clock_, 5000, "Not found safe pull_over path, use previous stop path");
  }
}

void GoalPlannerModule::setStopPathFromCurrentPath(BehaviorModuleOutput & output) const
{
  // safe or not safe(no feasible stop_path found) -> not_safe: try to find new feasible stop_path
  if (prev_data_.safety_status.is_safe || !prev_data_.stop_path_after_approval) {
    auto current_path = thread_safe_data_.get_pull_over_path()->getCurrentPath();
    const auto stop_path =
      behavior_path_planner::utils::parking_departure::generateFeasibleStopPath(
        current_path, planner_data_, stop_pose_, parameters_->maximum_deceleration_for_stop,
        parameters_->maximum_jerk_for_stop);
    if (stop_path) {
      output.path = *stop_path;
      RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "Collision detected, generate stop path");
    } else {
      output.path = thread_safe_data_.get_pull_over_path()->getCurrentPath();
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000,
        "Collision detected, no feasible stop path found, cannot stop.");
    }
  } else {
    // not_safe safe(no feasible stop path found) -> not_safe: use previous stop path
    output.path = *prev_data_.stop_path_after_approval;
    // stop_pose_ is removed in manager every loop, so need to set every loop.
    stop_pose_ = utils::getFirstStopPoseFromPath(output.path);
    RCLCPP_WARN_THROTTLE(getLogger(), *clock_, 5000, "Collision detected, use previous stop path");
  }
}

void GoalPlannerModule::setDrivableAreaInfo(BehaviorModuleOutput & output) const
{
  if (thread_safe_data_.getPullOverPlannerType() == PullOverPlannerType::FREESPACE) {
    const double drivable_area_margin = planner_data_->parameters.vehicle_width;
    output.drivable_area_info.drivable_margin =
      planner_data_->parameters.vehicle_width / 2.0 + drivable_area_margin;
  } else {
    const auto target_drivable_lanes = utils::getNonOverlappingExpandedLanes(
      output.path, generateDrivableLanes(), planner_data_->drivable_area_expansion_parameters);

    DrivableAreaInfo current_drivable_area_info;
    current_drivable_area_info.drivable_lanes = target_drivable_lanes;
    output.drivable_area_info = utils::combineDrivableAreaInfo(
      current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);
  }
}

void GoalPlannerModule::setModifiedGoal(BehaviorModuleOutput & output) const
{
  const auto & route_handler = planner_data_->route_handler;
  if (thread_safe_data_.foundPullOverPath()) {
    PoseWithUuidStamped modified_goal{};
    modified_goal.uuid = route_handler->getRouteUuid();
    modified_goal.pose = thread_safe_data_.get_modified_goal_pose()->goal_pose;
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
  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path.points);
  output.turn_signal_info = planner_data_->turn_signal_decider.use_prior_turn_signal(
    output.path, getEgoPose(), current_seg_idx, original_signal, new_signal,
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

  steering_factor_interface_ptr_->updateSteeringFactor(
    pose, distance, PlanningBehavior::GOAL_PLANNER, steering_factor_direction, type, "");
}

bool GoalPlannerModule::hasDecidedPath() const
{
  // Once this function returns true, it will continue to return true thereafter
  if (prev_data_.has_decided_path) {
    return true;
  }

  // if path is not safe, not decided
  if (!thread_safe_data_.foundPullOverPath()) {
    return false;
  }

  // If it is dangerous before approval, do not determine the path.
  // This eliminates a unsafe path to be approved
  if (
    parameters_->safety_check_params.enable_safety_check && !isSafePath().first && !isActivated()) {
    return false;
  }

  // if ego is sufficiently close to the start of the nearest candidate path, the path is decided
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const size_t ego_segment_idx = motion_utils::findNearestSegmentIndex(
    thread_safe_data_.get_pull_over_path()->getCurrentPath().points, current_pose.position);

  const size_t start_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    thread_safe_data_.get_pull_over_path()->getCurrentPath().points,
    thread_safe_data_.get_pull_over_path()->start_pose.position);
  const double dist_to_parking_start_pose = calcSignedArcLength(
    thread_safe_data_.get_pull_over_path()->getCurrentPath().points, current_pose.position,
    ego_segment_idx, thread_safe_data_.get_pull_over_path()->start_pose.position,
    start_pose_segment_idx);
  return dist_to_parking_start_pose < parameters_->decide_path_distance;
}

void GoalPlannerModule::decideVelocity()
{
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;

  auto & first_path = thread_safe_data_.get_pull_over_path()->partial_paths.front();
  const auto vel =
    static_cast<float>(std::max(current_vel, parameters_->pull_over_minimum_velocity));
  for (auto & p : first_path.points) {
    p.point.longitudinal_velocity_mps = std::min(p.point.longitudinal_velocity_mps, vel);
  }
}

BehaviorModuleOutput GoalPlannerModule::planPullOver()
{
  if (!hasDecidedPath()) {
    return planPullOverAsCandidate();
  }

  return planPullOverAsOutput();
}

BehaviorModuleOutput GoalPlannerModule::planPullOverAsCandidate()
{
  // if pull over path candidates generation is not finished, use previous module output
  if (thread_safe_data_.get_pull_over_path_candidates().empty()) {
    return getPreviousModuleOutput();
  }

  BehaviorModuleOutput output{};
  const BehaviorModuleOutput pull_over_output = planPullOverAsOutput();
  output.modified_goal = pull_over_output.modified_goal;
  output.path = generateStopPath();
  output.reference_path = getPreviousModuleOutput().reference_path;

  const auto target_drivable_lanes = utils::getNonOverlappingExpandedLanes(
    output.path, generateDrivableLanes(), planner_data_->drivable_area_expansion_parameters);

  DrivableAreaInfo current_drivable_area_info{};
  current_drivable_area_info.drivable_lanes = target_drivable_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, getPreviousModuleOutput().drivable_area_info);

  if (!thread_safe_data_.foundPullOverPath()) {
    return output;
  }

  return output;
}

BehaviorModuleOutput GoalPlannerModule::planPullOverAsOutput()
{
  // if pull over path candidates generation is not finished, use previous module output
  if (thread_safe_data_.get_pull_over_path_candidates().empty()) {
    return getPreviousModuleOutput();
  }

  if (!hasDecidedPath() && needPathUpdate(1.0 /*path_update_duration*/)) {
    // if the final path is not decided and enough time has passed since last path update,
    // select safe path from lane parking pull over path candidates
    // and set it to thread_safe_data_

    thread_safe_data_.clearPullOverPath();

    // update goal candidates
    goal_searcher_->setPlannerData(planner_data_);
    auto goal_candidates = thread_safe_data_.get_goal_candidates();
    goal_searcher_->update(goal_candidates);

    // update pull over path candidates
    const auto pull_over_path_candidates = sortPullOverPathCandidatesByGoalPriority(
      thread_safe_data_.get_pull_over_path_candidates(), goal_candidates);

    // select pull over path which is safe against static objects and get it's goal
    const auto path_and_goal_opt = selectPullOverPath(
      pull_over_path_candidates, goal_candidates,
      parameters_->object_recognition_collision_check_margin);

    // update thread_safe_data_
    if (path_and_goal_opt) {
      auto [pull_over_path, modified_goal] = *path_and_goal_opt;
      deceleratePath(pull_over_path);
      thread_safe_data_.set(
        goal_candidates, pull_over_path_candidates, pull_over_path, modified_goal);
    } else {
      thread_safe_data_.set(goal_candidates, pull_over_path_candidates);
    }
  }

  // set output and status
  BehaviorModuleOutput output{};
  setOutput(output);

  // return to lane parking if it is possible
  if (
    thread_safe_data_.getPullOverPlannerType() == PullOverPlannerType::FREESPACE &&
    canReturnToLaneParking()) {
    thread_safe_data_.set_pull_over_path(thread_safe_data_.get_lane_parking_pull_over_path());
  }

  // For debug
  setDebugData();
  if (parameters_->print_debug_info) {
    // For evaluations
    printParkingPositionError();
  }

  if (!thread_safe_data_.foundPullOverPath()) {
    return output;
  }

  path_candidate_ =
    std::make_shared<PathWithLaneId>(thread_safe_data_.get_pull_over_path()->getFullPath());

  updatePreviousData(output);

  return output;
}

void GoalPlannerModule::postProcess()
{
  if (!thread_safe_data_.foundPullOverPath()) {
    return;
  }

  const bool has_decided_path = hasDecidedPath();
  const auto distance_to_path_change = calcDistanceToPathChange();

  if (has_decided_path) {
    updateRTCStatus(distance_to_path_change.first, distance_to_path_change.second);
  }

  updateSteeringFactor(
    {thread_safe_data_.get_pull_over_path()->start_pose,
     thread_safe_data_.get_modified_goal_pose()->goal_pose},
    {distance_to_path_change.first, distance_to_path_change.second},
    has_decided_path ? SteeringFactor::TURNING : SteeringFactor::APPROACHING);

  setStopReason(StopReason::GOAL_PLANNER, thread_safe_data_.get_pull_over_path()->getFullPath());
}

void GoalPlannerModule::updatePreviousData(const BehaviorModuleOutput & output)
{
  if (prev_data_.found_path || !prev_data_.stop_path) {
    prev_data_.stop_path = std::make_shared<PathWithLaneId>(output.path);
  }

  // for the next loop setOutput().
  // this is used to determine whether to generate a new stop path or keep the current stop path.
  prev_data_.found_path = thread_safe_data_.foundPullOverPath();

  prev_data_.has_decided_path = hasDecidedPath();

  // This is related to safety check, so if it is disabled, return here.
  if (!parameters_->safety_check_params.enable_safety_check) {
    prev_data_.safety_status.is_safe = true;
    return;
  }

  // Even if the current path is safe, it will not be safe unless it stands for a certain period of
  // time. Record the time when the current path has become safe
  const auto [is_safe, current_is_safe] = isSafePath();
  if (current_is_safe) {
    if (!prev_data_.safety_status.safe_start_time) {
      prev_data_.safety_status.safe_start_time = clock_->now();
    }
  } else {
    prev_data_.safety_status.safe_start_time = std::nullopt;
  }
  prev_data_.safety_status.is_safe = is_safe;

  // If safety check is enabled, save current path as stop_path_after_approval
  // This path is set only once after approval.
  if (!isActivated() || (!is_safe && prev_data_.stop_path_after_approval)) {
    return;
  }
  auto stop_path = std::make_shared<PathWithLaneId>(output.path);
  for (auto & point : stop_path->points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }
  prev_data_.stop_path_after_approval = stop_path;
}

BehaviorModuleOutput GoalPlannerModule::planWaitingApproval()
{
  if (goal_planner_utils::isAllowedGoalModification(planner_data_->route_handler)) {
    return planPullOverAsCandidate();
  }

  fixed_goal_planner_->setPreviousModuleOutput(getPreviousModuleOutput());
  return fixed_goal_planner_->plan(planner_data_);
}

std::pair<double, double> GoalPlannerModule::calcDistanceToPathChange() const
{
  if (!thread_safe_data_.foundPullOverPath()) {
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  }

  const auto full_path = thread_safe_data_.get_pull_over_path()->getFullPath();

  const auto ego_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, planner_data_->self_odometry->pose.pose, std::numeric_limits<double>::max(),
    M_PI_2);
  if (!ego_segment_idx) {
    return {std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
  }

  const size_t start_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, thread_safe_data_.get_pull_over_path()->start_pose.position);
  const double dist_to_parking_start_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_odometry->pose.pose.position, *ego_segment_idx,
    thread_safe_data_.get_pull_over_path()->start_pose.position, start_pose_segment_idx);
  const size_t goal_pose_segment_idx = motion_utils::findNearestSegmentIndex(
    full_path.points, thread_safe_data_.get_modified_goal_pose()->goal_pose.position);
  const double dist_to_parking_finish_pose = calcSignedArcLength(
    full_path.points, planner_data_->self_odometry->pose.pose.position, *ego_segment_idx,
    thread_safe_data_.get_modified_goal_pose()->goal_pose.position, goal_pose_segment_idx);

  return {dist_to_parking_start_pose, dist_to_parking_finish_pose};
}

void GoalPlannerModule::setParameters(const std::shared_ptr<GoalPlannerParameters> & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId GoalPlannerModule::generateStopPath() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & common_parameters = planner_data_->parameters;
  const double current_vel = planner_data_->self_odometry->twist.twist.linear.x;
  const double pull_over_velocity = parameters_->pull_over_velocity;

  const lanelet::ConstLanelets current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length,
    /*forward_only_in_route*/ false);

  if (current_lanes.empty()) {
    return PathWithLaneId{};
  }

  // generate reference path
  const auto s_current = lanelet::utils::getArcCoordinates(current_lanes, current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto reference_path = route_handler->getCenterLinePath(current_lanes, s_start, s_end, true);

  // calculate search start offset pose from the closest goal candidate pose with
  // approximate_pull_over_distance_ ego vehicle decelerates to this position. or if no feasible
  // stop point is found, stop at this position.
  const auto closest_goal_candidate =
    goal_searcher_->getClosetGoalCandidateAlongLanes(thread_safe_data_.get_goal_candidates());
  const auto decel_pose = calcLongitudinalOffsetPose(
    reference_path.points, closest_goal_candidate.goal_pose.position,
    -approximate_pull_over_distance_);

  // if not approved stop road lane.
  // stop point priority is
  // 1. actual start pose
  // 2. closest candidate start pose
  // 3. pose offset by approximate_pull_over_distance_ from search start pose.
  //     (In the case of the curve lane, the position is not aligned due to the
  //     difference between the outer and inner sides)
  // 4. feasible stop
  const auto stop_pose = std::invoke([&]() -> std::optional<Pose> {
    if (thread_safe_data_.foundPullOverPath()) {
      return thread_safe_data_.get_pull_over_path()->start_pose;
    }
    if (thread_safe_data_.get_closest_start_pose()) {
      return thread_safe_data_.get_closest_start_pose().value();
    }
    if (!decel_pose) {
      return std::nullopt;
    }
    return decel_pose.value();
  });
  if (!stop_pose) {
    return generateFeasibleStopPath();
  }

  // if stop pose is closer than min_stop_distance, stop as soon as possible
  const double ego_to_stop_distance = calcSignedArcLengthFromEgo(reference_path, *stop_pose);
  const auto min_stop_distance = calcFeasibleDecelDistance(
    planner_data_, parameters_->maximum_deceleration, parameters_->maximum_jerk, 0.0);
  const double eps_vel = 0.01;
  const bool is_stopped = std::abs(current_vel) < eps_vel;
  const double buffer = is_stopped ? stop_distance_buffer_ : 0.0;
  if (min_stop_distance && ego_to_stop_distance + buffer < *min_stop_distance) {
    return generateFeasibleStopPath();
  }

  // slow down for turn signal, insert stop point to stop_pose
  decelerateForTurnSignal(*stop_pose, reference_path);
  stop_pose_ = *stop_pose;  // for debug wall marker

  // slow down before the search area.
  if (decel_pose) {
    decelerateBeforeSearchStart(*decel_pose, reference_path);
    return reference_path;
  }

  // if already passed the decel pose, set pull_over_velocity to reference_path.
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
  return reference_path;
}

PathWithLaneId GoalPlannerModule::generateFeasibleStopPath() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & common_parameters = planner_data_->parameters;

  // generate stop reference path
  const lanelet::ConstLanelets current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length,
    /*forward_only_in_route*/ false);

  const auto s_current = lanelet::utils::getArcCoordinates(current_lanes, current_pose).length;
  const double s_start = std::max(0.0, s_current - common_parameters.backward_path_length);
  const double s_end = s_current + common_parameters.forward_path_length;
  auto stop_path = route_handler->getCenterLinePath(current_lanes, s_start, s_end, true);

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

bool GoalPlannerModule::isStopped(
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> & odometry_buffer, const double time)
{
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
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
  const std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (isOnModifiedGoal()) {
    return false;
  }

  constexpr double stuck_time = 5.0;
  if (!isStopped(odometry_buffer_stuck_, stuck_time)) {
    return false;
  }

  // not found safe path
  if (!thread_safe_data_.foundPullOverPath()) {
    return true;
  }

  // any path has never been found
  if (!thread_safe_data_.get_pull_over_path()) {
    return false;
  }

  if (
    parameters_->use_object_recognition &&
    checkObjectsCollision(
      thread_safe_data_.get_pull_over_path()->getCurrentPath(),
      parameters_->object_recognition_collision_check_margin)) {
    return true;
  }

  if (
    parameters_->use_occupancy_grid_for_path_collision_check &&
    checkOccupancyGridCollision(thread_safe_data_.get_pull_over_path()->getCurrentPath())) {
    return true;
  }

  return false;
}

bool GoalPlannerModule::hasFinishedCurrentPath()
{
  if (!last_approval_data_) {
    return false;
  }

  if (!isStopped()) {
    return false;
  }

  // check if enough time has passed since last approval
  // this is necessary to give turn signal for enough time
  const bool has_passed_enough_time_from_approval =
    (clock_->now() - last_approval_data_->time).seconds() >
    planner_data_->parameters.turn_signal_search_time;
  if (!has_passed_enough_time_from_approval) {
    return false;
  }

  // require increment only when the time passed is enough
  // to prevent increment before driving
  // when the end of the current path is close to the current pose
  // this value should be `keep_stop_time` in keepStoppedWithCurrentPath
  constexpr double keep_current_idx_time = 4.0;
  const bool has_passed_enough_time_from_increment =
    (clock_->now() - *thread_safe_data_.get_last_path_update_time()).seconds() >
    keep_current_idx_time;
  if (!has_passed_enough_time_from_increment) {
    return false;
  }

  // check if self pose is near the end of current path
  const auto current_path_end =
    thread_safe_data_.get_pull_over_path()->getCurrentPath().points.back();
  const auto & self_pose = planner_data_->self_odometry->pose.pose;
  return tier4_autoware_utils::calcDistance2d(current_path_end, self_pose) <
         parameters_->th_arrived_distance;
}

bool GoalPlannerModule::isOnModifiedGoal() const
{
  if (!thread_safe_data_.get_modified_goal_pose()) {
    return false;
  }

  const Pose current_pose = planner_data_->self_odometry->pose.pose;
  return calcDistance2d(current_pose, thread_safe_data_.get_modified_goal_pose()->goal_pose) <
         parameters_->th_arrived_distance;
}

TurnSignalInfo GoalPlannerModule::calcTurnSignalInfo() const
{
  TurnSignalInfo turn_signal{};  // output

  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & start_pose = thread_safe_data_.get_pull_over_path()->start_pose;
  const auto & end_pose = thread_safe_data_.get_pull_over_path()->end_pose;
  const auto full_path = thread_safe_data_.get_pull_over_path()->getFullPath();

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
      last_approval_data_ && hasDecidedPath() ? last_approval_data_->pose : current_pose;
    turn_signal.desired_end_point = end_pose;
    turn_signal.required_start_point = start_pose;
    turn_signal.required_end_point = end_pose;
  }

  return turn_signal;
}

bool GoalPlannerModule::checkOccupancyGridCollision(const PathWithLaneId & path) const
{
  if (!occupancy_grid_map_) {
    return false;
  }
  const bool check_out_of_range = false;
  return occupancy_grid_map_->hasObstacleOnPath(path, check_out_of_range);
}

bool GoalPlannerModule::checkObjectsCollision(
  const PathWithLaneId & path, const double collision_check_margin,
  const bool update_debug_data) const
{
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

  /* Expand ego collision check polygon
   *   - `collision_check_margin` in all directions
   *   - `extra_stopping_margin` in the moving direction
   *   - `extra_lateral_margin` in external direction of path curve
   *
   *
   *                                        ^ moving direction
   *                                        x
   *                                        x
   *                                        x
   *          +----------------------+------x--+
   *          |                      |      x  |
   *          |   +---------------+  |    xx   |
   *          |   |               |  |  xx     |
   *          |   | ego footprint |xxxxx       |
   *          |   |               |  |         |
   *          |   +---------------+  | extra_stopping_margin
   *          |       margin         |         |
   *          +----------------------+         |
   *          |       extra_lateral_margin     |
   *          +--------------------------------+
   *
   */
  std::vector<Polygon2d> ego_polygons_expanded{};
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
      planner_data_->parameters.base_link2front + collision_check_margin + extra_stopping_margin,
      planner_data_->parameters.base_link2rear + collision_check_margin,
      planner_data_->parameters.vehicle_width + collision_check_margin * 2.0 +
        std::abs(extra_lateral_margin));
    ego_polygons_expanded.push_back(ego_polygon);
  }

  if (update_debug_data) {
    debug_data_.ego_polygons_expanded = ego_polygons_expanded;
  }

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

void GoalPlannerModule::keepStoppedWithCurrentPath(PathWithLaneId & path) const
{
  constexpr double keep_stop_time = 2.0;
  if (!thread_safe_data_.get_last_path_idx_increment_time()) {
    return;
  }

  const auto time_diff =
    (clock_->now() - *thread_safe_data_.get_last_path_idx_increment_time()).seconds();
  if (time_diff > keep_stop_time) {
    return;
  }

  for (auto & p : path.points) {
    p.point.longitudinal_velocity_mps = 0.0;
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

void GoalPlannerModule::deceleratePath(PullOverPath & pull_over_path) const
{
  // decelerate before the search area start
  const auto closest_goal_candidate =
    goal_searcher_->getClosetGoalCandidateAlongLanes(thread_safe_data_.get_goal_candidates());
  const auto decel_pose = calcLongitudinalOffsetPose(
    pull_over_path.getFullPath().points, closest_goal_candidate.goal_pose.position,
    -approximate_pull_over_distance_);
  auto & first_path = pull_over_path.partial_paths.front();
  if (decel_pose) {
    decelerateBeforeSearchStart(*decel_pose, first_path);
    return;
  }

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
    utils::insertStopPoint(stop_point_length, path);
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
    [&](const lanelet::ConstLanelet & lane) -> std::optional<lanelet::ConstLanelet> {
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
    std::optional<lanelet::ConstLanelet> neighboring_lane = getNeighboringLane(current_lane);
    if (neighboring_lane) {
      // Check if the neighboring lane is in the end_lane_sequence
      end_it =
        std::find(end_lane_sequence.rbegin(), end_lane_sequence.rend(), neighboring_lane.value());
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
  const lanelet::ConstLanelets lanes = utils::transformToLanelets(generateDrivableLanes());
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

std::pair<bool, bool> GoalPlannerModule::isSafePath() const
{
  const auto pull_over_path = thread_safe_data_.get_pull_over_path()->getCurrentPath();
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const double current_velocity = std::hypot(
    planner_data_->self_odometry->twist.twist.linear.x,
    planner_data_->self_odometry->twist.twist.linear.y);
  const auto & dynamic_object = planner_data_->dynamic_object;
  const auto & route_handler = planner_data_->route_handler;
  const lanelet::ConstLanelets current_lanes = utils::getExtendedCurrentLanes(
    planner_data_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length,
    /*forward_only_in_route*/ false);
  const auto pull_over_lanes = goal_planner_utils::getPullOverLanes(
    *route_handler, left_side_parking_, parameters_->backward_goal_search_length,
    parameters_->forward_goal_search_length);
  const size_t ego_seg_idx = planner_data_->findEgoSegmentIndex(pull_over_path.points);
  const std::pair<double, double> terminal_velocity_and_accel =
    utils::parking_departure::getPairsTerminalVelocityAndAccel(
      thread_safe_data_.get_pull_over_path()->pairs_terminal_velocity_and_accel,
      thread_safe_data_.get_pull_over_path()->path_idx);
  RCLCPP_DEBUG(
    getLogger(), "pairs_terminal_velocity_and_accel for goal_planner: %f, %f",
    terminal_velocity_and_accel.first, terminal_velocity_and_accel.second);
  utils::parking_departure::updatePathProperty(
    ego_predicted_path_params_, terminal_velocity_and_accel);
  // TODO(Sugahara): shoule judge is_object_front properly
  const bool is_object_front = true;
  const bool limit_to_max_velocity = true;
  const auto ego_predicted_path =
    behavior_path_planner::utils::path_safety_checker::createPredictedPath(
      ego_predicted_path_params_, pull_over_path.points, current_pose, current_velocity,
      ego_seg_idx, is_object_front, limit_to_max_velocity);

  // filtering objects with velocity, position and class
  const auto filtered_objects = utils::path_safety_checker::filterObjects(
    dynamic_object, route_handler, pull_over_lanes, current_pose.position,
    objects_filtering_params_);

  // filtering objects based on the current position's lane
  const auto target_objects_on_lane = utils::path_safety_checker::createTargetObjectsOnLane(
    pull_over_lanes, route_handler, filtered_objects, objects_filtering_params_);

  utils::parking_departure::updateSafetyCheckTargetObjectsData(
    goal_planner_data_, filtered_objects, target_objects_on_lane, ego_predicted_path);

  const double hysteresis_factor =
    prev_data_.safety_status.is_safe ? 1.0 : parameters_->hysteresis_factor_expand_rate;

  const bool current_is_safe = std::invoke([&]() {
    if (parameters_->safety_check_params.method == "RSS") {
      return behavior_path_planner::utils::path_safety_checker::checkSafetyWithRSS(
        pull_over_path, ego_predicted_path, target_objects_on_lane.on_current_lane,
        goal_planner_data_.collision_check, planner_data_->parameters,
        safety_check_params_->rss_params, objects_filtering_params_->use_all_predicted_path,
        hysteresis_factor);
    } else if (parameters_->safety_check_params.method == "integral_predicted_polygon") {
      return utils::path_safety_checker::checkSafetyWithIntegralPredictedPolygon(
        ego_predicted_path, vehicle_info_, target_objects_on_lane.on_current_lane,
        objects_filtering_params_->check_all_predicted_path,
        parameters_->safety_check_params.integral_predicted_polygon_params,
        goal_planner_data_.collision_check);
    }
    RCLCPP_ERROR(
      getLogger(), " [pull_over] invalid safety check method: %s",
      parameters_->safety_check_params.method.c_str());
    throw std::domain_error("[pull_over] invalid safety check method");
  });

  /*
   *　　                    ==== is_safe
   *　　                    ---- current_is_safe
   *　　  is_safe
   *　　   |
   *　　   |                   time
   *　　 1 +--+    +---+       +---=========   +--+
   *　　   |  |    |   |       |           |   |  |
   *　　   |  |    |   |       |           |   |  |
   *　　   |  |    |   |       |           |   |  |
   *　　   |  |    |   |       |           |   |  |
   *　　 0 =========================-------==========-- t
   */
  if (current_is_safe) {
    if (
      prev_data_.safety_status.safe_start_time &&
      (clock_->now() - prev_data_.safety_status.safe_start_time.value()).seconds() >
        parameters_->safety_check_params.keep_unsafe_time) {
      return {true /*is_safe*/, true /*current_is_safe*/};
    }
  }

  return {false /*is_safe*/, current_is_safe};
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
    const auto color = hasDecidedPath() ? createMarkerColor(1.0, 1.0, 0.0, 0.999)   // yellow
                                        : createMarkerColor(0.0, 1.0, 0.0, 0.999);  // green
    const double z = planner_data_->route_handler->getGoalPose().position.z;
    add(goal_planner_utils::createPullOverAreaMarkerArray(
      goal_searcher_->getAreaPolygons(), header, color, z));

    // Visualize goal candidates
    const auto goal_candidates = thread_safe_data_.get_goal_candidates();
    add(goal_planner_utils::createGoalCandidatesMarkerArray(goal_candidates, color));
  }

  // Visualize path and related pose
  if (thread_safe_data_.foundPullOverPath()) {
    add(createPoseMarkerArray(
      thread_safe_data_.get_pull_over_path()->start_pose, "pull_over_start_pose", 0, 0.3, 0.3,
      0.9));
    add(createPoseMarkerArray(
      thread_safe_data_.get_pull_over_path()->end_pose, "pull_over_end_pose", 0, 0.3, 0.3, 0.9));
    add(createPathMarkerArray(
      thread_safe_data_.get_pull_over_path()->getFullPath(), "full_path", 0, 0.0, 0.5, 0.9));
    add(createPathMarkerArray(
      thread_safe_data_.get_pull_over_path()->getCurrentPath(), "current_path", 0, 0.9, 0.5, 0.0));

    // visualize each partial path
    for (size_t i = 0; i < thread_safe_data_.get_pull_over_path()->partial_paths.size(); ++i) {
      const auto & partial_path = thread_safe_data_.get_pull_over_path()->partial_paths.at(i);
      add(
        createPathMarkerArray(partial_path, "partial_path_" + std::to_string(i), 0, 0.9, 0.5, 0.9));
    }

    auto marker = tier4_autoware_utils::createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "detection_polygons", 0, Marker::LINE_LIST,
      tier4_autoware_utils::createMarkerScale(0.01, 0.0, 0.0),
      tier4_autoware_utils::createMarkerColor(0.0, 0.0, 1.0, 0.999));
    const double ego_z = planner_data_->self_odometry->pose.pose.position.z;
    for (const auto & ego_polygon : debug_data_.ego_polygons_expanded) {
      for (size_t ep_idx = 0; ep_idx < ego_polygon.outer().size(); ++ep_idx) {
        const auto & current_point = ego_polygon.outer().at(ep_idx);
        const auto & next_point = ego_polygon.outer().at((ep_idx + 1) % ego_polygon.outer().size());

        marker.points.push_back(
          tier4_autoware_utils::createPoint(current_point.x(), current_point.y(), ego_z));
        marker.points.push_back(
          tier4_autoware_utils::createPoint(next_point.x(), next_point.y(), ego_z));
      }
    }
    debug_marker_.markers.push_back(marker);

    // Visualize debug poses
    const auto & debug_poses = thread_safe_data_.get_pull_over_path()->debug_poses;
    for (size_t i = 0; i < debug_poses.size(); ++i) {
      add(createPoseMarkerArray(
        debug_poses.at(i), "debug_pose_" + std::to_string(i), 0, 0.3, 0.3, 0.3));
    }
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

    if (parameters_->safety_check_params.method == "RSS") {
      add(showSafetyCheckInfo(goal_planner_data_.collision_check, "object_debug_info"));
    }
    add(showPredictedPath(goal_planner_data_.collision_check, "ego_predicted_path"));
    add(showPolygon(goal_planner_data_.collision_check, "ego_and_target_polygon_relation"));
    utils::parking_departure::initializeCollisionCheckDebugMap(goal_planner_data_.collision_check);

    // visualize safety status maker
    {
      visualization_msgs::msg::MarkerArray marker_array{};
      const auto color = prev_data_.safety_status.is_safe ? createMarkerColor(1.0, 1.0, 1.0, 0.99)
                                                          : createMarkerColor(1.0, 0.0, 0.0, 0.99);
      auto marker = createDefaultMarker(
        header.frame_id, header.stamp, "safety_status", 0,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0), color);

      marker.pose = planner_data_->self_odometry->pose.pose;
      marker.text += "is_safe: " + std::to_string(prev_data_.safety_status.is_safe) + "\n";
      if (prev_data_.safety_status.safe_start_time) {
        const double elapsed_time_from_safe_start =
          (clock_->now() - prev_data_.safety_status.safe_start_time.value()).seconds();
        marker.text +=
          "elapsed_time_from_safe_start: " + std::to_string(elapsed_time_from_safe_start) + "\n";
      }
      marker_array.markers.push_back(marker);
      add(marker_array);
    }
  }

  // Visualize planner type text
  {
    visualization_msgs::msg::MarkerArray planner_type_marker_array{};
    const auto color = thread_safe_data_.foundPullOverPath()
                         ? createMarkerColor(1.0, 1.0, 1.0, 0.99)
                         : createMarkerColor(1.0, 0.0, 0.0, 0.99);
    auto marker = createDefaultMarker(
      header.frame_id, header.stamp, "planner_type", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.0, 0.0, 1.0), color);
    marker.pose = thread_safe_data_.get_modified_goal_pose()
                    ? thread_safe_data_.get_modified_goal_pose()->goal_pose
                    : planner_data_->self_odometry->pose.pose;
    marker.text = magic_enum::enum_name(thread_safe_data_.getPullOverPlannerType());
    if (thread_safe_data_.foundPullOverPath()) {
      marker.text +=
        " " + std::to_string(thread_safe_data_.get_pull_over_path()->path_idx) + "/" +
        std::to_string(thread_safe_data_.get_pull_over_path()->partial_paths.size() - 1);
    }

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
}

void GoalPlannerModule::printParkingPositionError() const
{
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const double real_shoulder_to_map_shoulder = 0.0;

  const Pose goal_to_ego =
    inverseTransformPose(current_pose, thread_safe_data_.get_modified_goal_pose()->goal_pose);
  const double dx = goal_to_ego.position.x;
  const double dy = goal_to_ego.position.y;
  const double distance_from_real_shoulder =
    real_shoulder_to_map_shoulder + parameters_->margin_from_boundary - dy;
  RCLCPP_INFO(
    getLogger(), "current pose to goal, dx:%f dy:%f dyaw:%f from_real_shoulder:%f", dx, dy,
    tier4_autoware_utils::rad2deg(
      tf2::getYaw(current_pose.orientation) -
      tf2::getYaw(thread_safe_data_.get_modified_goal_pose()->goal_pose.orientation)),
    distance_from_real_shoulder);
}

bool GoalPlannerModule::needPathUpdate(const double path_update_duration) const
{
  return !isOnModifiedGoal() && hasEnoughTimePassedSincePathUpdate(path_update_duration);
}

bool GoalPlannerModule::hasEnoughTimePassedSincePathUpdate(const double duration) const
{
  if (!thread_safe_data_.get_last_path_update_time()) {
    return true;
  }

  return (clock_->now() - *thread_safe_data_.get_last_path_update_time()).seconds() > duration;
}
}  // namespace behavior_path_planner
