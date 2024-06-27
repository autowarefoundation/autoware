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

#include "autoware/behavior_path_start_planner_module/start_planner_module.hpp"

#include "autoware/behavior_path_planner_common/utils/parking_departure/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_start_planner_module/debug.hpp"
#include "autoware/behavior_path_start_planner_module/util.hpp"
#include "autoware/motion_utils/trajectory/trajectory.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <lanelet2_extension/visualization/visualization.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

using autoware::behavior_path_planner::utils::parking_departure::initializeCollisionCheckDebugMap;
using autoware::behavior_path_planner::utils::path_safety_checker::ExtendedPredictedObject;
using autoware::motion_utils::calcLateralOffset;
using autoware::motion_utils::calcLongitudinalOffsetPose;
using autoware::universe_utils::calcOffsetPose;

// set as macro so that calling function name will be printed.
// debug print is heavy. turn on only when debugging.
#define DEBUG_PRINT(...) \
  RCLCPP_DEBUG_EXPRESSION(getLogger(), parameters_->print_debug_info, __VA_ARGS__)

namespace autoware::behavior_path_planner
{
StartPlannerModule::StartPlannerModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<StartPlannerParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map)
: SceneModuleInterface{name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map},  // NOLINT
  parameters_{parameters},
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo()},
  is_freespace_planner_cb_running_{false}
{
  lane_departure_checker_ = std::make_shared<LaneDepartureChecker>();
  lane_departure_checker_->setVehicleInfo(vehicle_info_);
  autoware::lane_departure_checker::Param lane_departure_checker_params{};
  lane_departure_checker_params.footprint_extra_margin =
    parameters->lane_departure_check_expansion_margin;

  lane_departure_checker_->setParam(lane_departure_checker_params);

  // set enabled planner
  if (parameters_->enable_shift_pull_out) {
    start_planners_.push_back(
      std::make_shared<ShiftPullOut>(node, *parameters, lane_departure_checker_));
  }
  if (parameters_->enable_geometric_pull_out) {
    start_planners_.push_back(
      std::make_shared<GeometricPullOut>(node, *parameters, lane_departure_checker_));
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
  const ScopedFlag flag(is_freespace_planner_cb_running_);

  std::shared_ptr<const PlannerData> local_planner_data{nullptr};
  std::optional<ModuleStatus> current_status_opt{std::nullopt};
  std::optional<StartPlannerParameters> parameters_opt{std::nullopt};
  std::optional<PullOutStatus> pull_out_status_opt{std::nullopt};
  bool is_stopped{false};

  // making a local copy of thread sensitive data
  {
    std::lock_guard<std::mutex> guard(start_planner_data_mutex_);
    if (start_planner_data_) {
      const auto & start_planner_data = start_planner_data_.value();
      local_planner_data = std::make_shared<PlannerData>(start_planner_data.planner_data);
      current_status_opt = start_planner_data.current_status;
      parameters_opt = start_planner_data.parameters;
      pull_out_status_opt = start_planner_data.main_thread_pull_out_status;
      is_stopped = start_planner_data.is_stopped;
    }
  }
  // finish copying thread sensitive data
  if (!local_planner_data || !current_status_opt || !parameters_opt || !pull_out_status_opt) {
    return;
  }

  const auto & current_status = current_status_opt.value();
  const auto & parameters = parameters_opt.value();
  const auto & pull_out_status = pull_out_status_opt.value();

  if (current_status == ModuleStatus::IDLE) {
    return;
  }

  if (!local_planner_data->costmap) {
    return;
  }

  const bool is_new_costmap =
    (clock_->now() - local_planner_data->costmap->header.stamp).seconds() < 1.0;
  if (!is_new_costmap) {
    return;
  }

  const bool is_stuck = is_stopped && pull_out_status.planner_type == PlannerType::STOP &&
                        !pull_out_status.found_pull_out_path;
  if (is_stuck) {
    const auto free_space_status =
      planFreespacePath(parameters, local_planner_data, pull_out_status);
    if (free_space_status) {
      std::lock_guard<std::mutex> guard(start_planner_data_mutex_);
      freespace_thread_status_ = free_space_status;
    }
  }
}

BehaviorModuleOutput StartPlannerModule::run()
{
  updateData();
  if (!isActivated() || needToPrepareBlinkerBeforeStartDrivingForward()) {
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
  info_marker_.markers.clear();
  initializeSafetyCheckParameters();
  initializeCollisionCheckDebugMap(debug_data_.collision_check);
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
  // The method PlannerManager::run() calls SceneModuleInterface::setData and
  // SceneModuleInterface::setPreviousModuleOutput() before this module's run() method is called
  // with module_ptr->run(). Then module_ptr->run() invokes StartPlannerModule::updateData and,
  // finally, the planWaitingApproval()/plan() methods are called by run(). So we can copy the
  // latest current_status to start_planner_data_ here for later usage.

  // NOTE: onFreespacePlannerTimer copies start_planner_data to its thread local variable, so we
  // need to lock start_planner_data_ here to avoid data race. But the following clone process is
  // lightweight because most of the member variables of PlannerData/RouteHandler is
  // shared_ptrs/bool
  // making a local copy of thread sensitive data
  {
    std::lock_guard<std::mutex> guard(start_planner_data_mutex_);
    if (!start_planner_data_) {
      start_planner_data_ = StartPlannerData();
    }
    start_planner_data_.value().update(
      *parameters_, *planner_data_, getCurrentStatus(), status_, isStopped());
    if (freespace_thread_status_) {
      // if freespace solution is available, copy it to status_ on main thread
      const auto & freespace_status = freespace_thread_status_.value();
      status_.pull_out_path = freespace_status.pull_out_path;
      status_.pull_out_start_pose = freespace_status.pull_out_start_pose;
      status_.planner_type = freespace_status.planner_type;
      status_.found_pull_out_path = freespace_status.found_pull_out_path;
      status_.driving_forward = freespace_status.driving_forward;
      // and then reset it
      freespace_thread_status_ = std::nullopt;
    }
  }
  // finish copying thread sensitive data

  if (receivedNewRoute()) {
    resetStatus();
    DEBUG_PRINT("StartPlannerModule::updateData() received new route, reset status");
  }

  if (
    planner_data_->operation_mode->mode == OperationModeState::AUTONOMOUS &&
    status_.driving_forward && !status_.first_engaged_and_driving_forward_time) {
    status_.first_engaged_and_driving_forward_time = clock_->now();
  }

  constexpr double moving_velocity_threshold = 0.1;
  const double & ego_velocity = planner_data_->self_odometry->twist.twist.linear.x;
  if (status_.first_engaged_and_driving_forward_time && ego_velocity > moving_velocity_threshold) {
    // Ego is engaged, and has moved
    status_.has_departed = true;
  }

  status_.backward_driving_complete = hasFinishedBackwardDriving();
  if (status_.backward_driving_complete) {
    updateStatusAfterBackwardDriving();
    DEBUG_PRINT("StartPlannerModule::updateData() completed backward driving");
  }

  status_.is_safe_dynamic_objects =
    (!requiresDynamicObjectsCollisionDetection()) ? true : !hasCollisionWithDynamicObjects();
}

bool StartPlannerModule::hasFinishedBackwardDriving() const
{
  // check ego car is close enough to pull out start pose and stopped
  const auto current_pose = planner_data_->self_odometry->pose.pose;
  const auto distance =
    autoware::universe_utils::calcDistance2d(current_pose, status_.pull_out_start_pose);

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
  return parameters_->safety_check_params.enable_safety_check && status_.driving_forward &&
         !isPreventingRearVehicleFromPassingThrough();
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

bool StartPlannerModule::isPreventingRearVehicleFromPassingThrough() const
{
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  const auto & dynamic_object = planner_data_->dynamic_object;
  const auto & route_handler = planner_data_->route_handler;
  const Pose start_pose = planner_data_->route_handler->getOriginalStartPose();

  const auto target_lanes = utils::getCurrentLanes(planner_data_);
  if (target_lanes.empty()) return false;

  // Define functions to get distance between a point and a lane's boundaries.
  auto calc_absolute_lateral_offset = [&](
                                        const lanelet::ConstLineString2d & boundary_line,
                                        const geometry_msgs::msg::Pose & search_pose) {
    std::vector<geometry_msgs::msg::Point> boundary_path;
    std::for_each(
      boundary_line.begin(), boundary_line.end(), [&boundary_path](const auto & boundary_point) {
        const double x = boundary_point.x();
        const double y = boundary_point.y();
        boundary_path.push_back(autoware::universe_utils::createPoint(x, y, 0.0));
      });

    return std::fabs(calcLateralOffset(boundary_path, search_pose.position));
  };

  // Check from what side of the road the ego is merging
  const auto centerline_path =
    route_handler->getCenterLinePath(target_lanes, 0.0, std::numeric_limits<double>::max());
  const auto start_pose_nearest_segment_index =
    autoware::motion_utils::findNearestSegmentIndex(centerline_path.points, start_pose);
  if (!start_pose_nearest_segment_index) return false;

  const auto start_pose_point_msg = autoware::universe_utils::createPoint(
    start_pose.position.x, start_pose.position.y, start_pose.position.z);
  const auto starting_pose_lateral_offset = autoware::motion_utils::calcLateralOffset(
    centerline_path.points, start_pose_point_msg, start_pose_nearest_segment_index.value());
  if (std::isnan(starting_pose_lateral_offset)) return false;

  RCLCPP_DEBUG(getLogger(), "starting pose lateral offset: %f", starting_pose_lateral_offset);
  const bool ego_is_merging_from_the_left = (starting_pose_lateral_offset > 0.0);

  // Get the ego's overhang point closest to the centerline path and the gap between said point and
  // the lane's border.
  auto get_gap_between_ego_and_lane_border =
    [&](
      geometry_msgs::msg::Pose & ego_overhang_point_as_pose,
      const bool ego_is_merging_from_the_left) -> std::optional<std::pair<double, double>> {
    const auto local_vehicle_footprint = vehicle_info_.createFootprint();
    const auto vehicle_footprint = transformVector(
      local_vehicle_footprint, autoware::universe_utils::pose2transform(current_pose));
    double smallest_lateral_gap_between_ego_and_border = std::numeric_limits<double>::max();
    double corresponding_lateral_gap_with_other_lane_bound = std::numeric_limits<double>::max();

    for (const auto & point : vehicle_footprint) {
      geometry_msgs::msg::Pose point_pose;
      point_pose.position.x = point.x();
      point_pose.position.y = point.y();
      point_pose.position.z = 0.0;

      lanelet::Lanelet closest_lanelet;
      lanelet::utils::query::getClosestLanelet(target_lanes, point_pose, &closest_lanelet);
      lanelet::ConstLanelet closest_lanelet_const(closest_lanelet.constData());

      const auto [current_lane_bound, other_side_lane_bound] =
        (ego_is_merging_from_the_left)
          ? std::make_pair(
              closest_lanelet_const.rightBound2d(), closest_lanelet_const.leftBound2d())
          : std::make_pair(
              closest_lanelet_const.leftBound2d(), closest_lanelet_const.rightBound2d());
      const double current_point_lateral_gap =
        calc_absolute_lateral_offset(current_lane_bound, point_pose);
      if (current_point_lateral_gap < smallest_lateral_gap_between_ego_and_border) {
        smallest_lateral_gap_between_ego_and_border = current_point_lateral_gap;
        ego_overhang_point_as_pose.position.x = point.x();
        ego_overhang_point_as_pose.position.y = point.y();
        ego_overhang_point_as_pose.position.z = 0.0;
        corresponding_lateral_gap_with_other_lane_bound =
          calc_absolute_lateral_offset(other_side_lane_bound, point_pose);
      }
    }

    if (smallest_lateral_gap_between_ego_and_border == std::numeric_limits<double>::max()) {
      return std::nullopt;
    }
    return std::make_pair(
      (smallest_lateral_gap_between_ego_and_border),
      (corresponding_lateral_gap_with_other_lane_bound));
  };

  geometry_msgs::msg::Pose ego_overhang_point_as_pose;
  const auto gaps_with_lane_borders_pair =
    get_gap_between_ego_and_lane_border(ego_overhang_point_as_pose, ego_is_merging_from_the_left);

  if (!gaps_with_lane_borders_pair.has_value()) {
    return false;
  }

  const auto & gap_between_ego_and_lane_border = gaps_with_lane_borders_pair.value().first;
  const auto & corresponding_lateral_gap_with_other_lane_bound =
    gaps_with_lane_borders_pair.value().second;

  // middle of the lane is crossed, no need to check for collisions anymore
  if (gap_between_ego_and_lane_border < corresponding_lateral_gap_with_other_lane_bound) {
    return true;
  }
  // Get the lanelets that will be queried for target objects
  const auto relevant_lanelets = std::invoke([&]() -> std::optional<lanelet::ConstLanelets> {
    lanelet::Lanelet closest_lanelet;
    const bool is_closest_lanelet = lanelet::utils::query::getClosestLanelet(
      target_lanes, ego_overhang_point_as_pose, &closest_lanelet);
    if (!is_closest_lanelet) return std::nullopt;
    lanelet::ConstLanelet closest_lanelet_const(closest_lanelet.constData());
    // Check backwards just in case the Vehicle behind ego is in a different lanelet
    constexpr double backwards_length = 200.0;
    const auto prev_lanes = autoware::behavior_path_planner::utils::getBackwardLanelets(
      *route_handler, target_lanes, current_pose, backwards_length);
    // return all the relevant lanelets
    lanelet::ConstLanelets relevant_lanelets{closest_lanelet_const};
    relevant_lanelets.insert(relevant_lanelets.end(), prev_lanes.begin(), prev_lanes.end());
    return relevant_lanelets;
  });
  if (!relevant_lanelets) return false;

  // filtering objects with velocity, position and class
  const auto filtered_objects = utils::path_safety_checker::filterObjects(
    dynamic_object, route_handler, relevant_lanelets.value(), current_pose.position,
    objects_filtering_params_);
  if (filtered_objects.objects.empty()) return false;

  // filtering objects based on the current position's lane
  const auto target_objects_on_lane = utils::path_safety_checker::createTargetObjectsOnLane(
    relevant_lanelets.value(), route_handler, filtered_objects, objects_filtering_params_);
  if (target_objects_on_lane.on_current_lane.empty()) return false;

  // Get the closest target obj width in the relevant lanes
  const auto closest_object_width = std::invoke([&]() -> std::optional<double> {
    double arc_length_to_closet_object = std::numeric_limits<double>::max();
    std::optional<double> closest_object_width = std::nullopt;
    std::for_each(
      target_objects_on_lane.on_current_lane.begin(), target_objects_on_lane.on_current_lane.end(),
      [&](const auto & o) {
        const auto arc_length = autoware::motion_utils::calcSignedArcLength(
          centerline_path.points, current_pose.position, o.initial_pose.pose.position);
        if (arc_length > 0.0) return;
        if (std::abs(arc_length) >= std::abs(arc_length_to_closet_object)) return;
        arc_length_to_closet_object = arc_length;
        closest_object_width = o.shape.dimensions.y;
      });
    return closest_object_width;
  });
  if (!closest_object_width) return false;
  // Decide if the closest object does not fit in the gap left by the ego vehicle.
  return closest_object_width.value() + parameters_->extra_width_margin_for_rear_obstacle >
         gap_between_ego_and_lane_border;
}

bool StartPlannerModule::isCloseToOriginalStartPose() const
{
  const Pose start_pose = planner_data_->route_handler->getOriginalStartPose();
  return autoware::universe_utils::calcDistance2d(
           start_pose.position, planner_data_->self_odometry->pose.pose.position) >
         parameters_->th_arrived_distance;
}

bool StartPlannerModule::hasArrivedAtGoal() const
{
  const Pose goal_pose = planner_data_->route_handler->getGoalPose();
  return autoware::universe_utils::calcDistance2d(
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
  // Evaluate safety. The situation is not safe if any of the following conditions are met:
  // 1. pull out path has not been found
  // 2. there is a moving objects around ego
  // 3. waiting for approval and there is a collision with dynamic objects

  const bool is_safe = [&]() -> bool {
    if (!status_.found_pull_out_path) return false;
    if (!isWaitingApproval()) return true;
    if (!noMovingObjectsAround()) return false;
    return !(requiresDynamicObjectsCollisionDetection() && hasCollisionWithDynamicObjects());
  }();

  if (!is_safe) {
    stop_pose_ = planner_data_->self_odometry->pose.pose;
  }

  return is_safe;
}

bool StartPlannerModule::canTransitSuccessState()
{
  // Freespace Planner:
  // - Can transit to success if the goal position is reached.
  // - Cannot transit to success if the goal position is not reached.
  if (status_.planner_type == PlannerType::FREESPACE) {
    if (hasReachedFreespaceEnd()) {
      RCLCPP_DEBUG(
        getLogger(), "Transit to success: Freespace planner reached the end point of the path.");
      return true;
    }
    return false;
  }

  // Other Planners:
  // - Cannot transit to success if the vehicle is driving in reverse.
  // - Cannot transit to success if a safe path cannot be found due to:
  //   - Insufficient margin against static objects.
  //   - No path found that stays within the lane.
  //   In such cases, a stop point needs to be embedded and keep running start_planner.
  // - Can transit to success if the end point of the pullout path is reached.
  if (!status_.driving_forward || !status_.found_pull_out_path) {
    return false;
  }

  if (hasReachedPullOutEnd()) {
    RCLCPP_DEBUG(getLogger(), "Transit to success: Reached the end point of the pullout path.");
    return true;
  }

  return false;
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

  const auto path = std::invoke([&]() {
    if (!status_.driving_forward && !status_.backward_driving_complete) {
      return status_.backward_path;
    }

    // Increment path index if the current path is finished
    if (hasFinishedCurrentPath()) {
      RCLCPP_INFO(getLogger(), "Increment path index");
      incrementPathIndex();
    }

    if (isWaitingApproval()) return getCurrentPath();

    if (status_.stop_pose) {
      // Delete stop point if conditions are met
      if (status_.is_safe_dynamic_objects && isStopped()) {
        status_.stop_pose = std::nullopt;
      }
      stop_pose_ = status_.stop_pose;
      return *status_.prev_stop_path_after_approval;
    }

    if (!status_.is_safe_dynamic_objects) {
      auto current_path = getCurrentPath();
      const auto stop_path =
        autoware::behavior_path_planner::utils::parking_departure::generateFeasibleStopPath(
          current_path, planner_data_, stop_pose_, parameters_->maximum_deceleration_for_stop,
          parameters_->maximum_jerk_for_stop);

      if (!stop_path.has_value()) return current_path;
      // Insert stop point in the path if needed
      RCLCPP_ERROR_THROTTLE(
        getLogger(), *clock_, 5000, "Insert stop point in the path because of dynamic objects");
      status_.prev_stop_path_after_approval = std::make_shared<PathWithLaneId>(stop_path.value());
      status_.stop_pose = stop_pose_;
      return stop_path.value();
    }
    return getCurrentPath();
  });

  output.path = path;
  output.reference_path = getPreviousModuleOutput().reference_path;
  output.turn_signal_info = calcTurnSignalInfo();
  path_candidate_ = std::make_shared<PathWithLaneId>(getFullPath());
  path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

  setDrivableAreaInfo(output);

  const auto steering_factor_direction = getSteeringFactorDirection(output);

  if (status_.driving_forward) {
    const double start_distance = autoware::motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    const double finish_distance = autoware::motion_utils::calcSignedArcLength(
      path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.end_pose.position);
    updateRTCStatus(start_distance, finish_distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose},
      {start_distance, finish_distance}, PlanningBehavior::START_PLANNER, steering_factor_direction,
      SteeringFactor::TURNING, "");
    setDebugData();
    return output;
  }
  const double distance = autoware::motion_utils::calcSignedArcLength(
    path.points, planner_data_->self_odometry->pose.pose.position,
    status_.pull_out_path.start_pose.position);
  updateRTCStatus(0.0, distance);
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
    PlanningBehavior::START_PLANNER, steering_factor_direction, SteeringFactor::TURNING, "");

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

  const auto steering_factor_direction = getSteeringFactorDirection(output);

  if (status_.driving_forward) {
    const double start_distance = autoware::motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.start_pose.position);
    const double finish_distance = autoware::motion_utils::calcSignedArcLength(
      stop_path.points, planner_data_->self_odometry->pose.pose.position,
      status_.pull_out_path.end_pose.position);
    updateRTCStatus(start_distance, finish_distance);
    steering_factor_interface_ptr_->updateSteeringFactor(
      {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose},
      {start_distance, finish_distance}, PlanningBehavior::START_PLANNER, steering_factor_direction,
      SteeringFactor::APPROACHING, "");
    setDebugData();

    return output;
  }
  const double distance = autoware::motion_utils::calcSignedArcLength(
    stop_path.points, planner_data_->self_odometry->pose.pose.position,
    status_.pull_out_path.start_pose.position);
  updateRTCStatus(0.0, distance);
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.pull_out_path.start_pose, status_.pull_out_path.end_pose}, {0.0, distance},
    PlanningBehavior::START_PLANNER, steering_factor_direction, SteeringFactor::APPROACHING, "");

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
  const Pose & goal_pose, const std::string & search_priority)
{
  if (start_pose_candidates.empty()) return;

  auto get_accumulated_debug_stream = [](const std::vector<PlannerDebugData> & debug_data_vector) {
    std::stringstream ss;
    if (debug_data_vector.empty()) return ss;
    ss << debug_data_vector.front().header_str();
    for (const auto & debug_data : debug_data_vector) {
      ss << debug_data.str();
    }
    return ss;
  };

  const PriorityOrder order_priority =
    determinePriorityOrder(search_priority, start_pose_candidates.size());

  std::vector<PlannerDebugData> debug_data_vector;
  for (const auto & collision_check_margin : parameters_->collision_check_margins) {
    for (const auto & [index, planner] : order_priority) {
      if (findPullOutPath(
            start_pose_candidates[index], planner, refined_start_pose, goal_pose,
            collision_check_margin, debug_data_vector)) {
        debug_data_.selected_start_pose_candidate_index = index;
        debug_data_.margin_for_start_pose_candidate = collision_check_margin;
        if (parameters_->print_debug_info) {
          const auto ss = get_accumulated_debug_stream(debug_data_vector);
          DEBUG_PRINT("\nPull out path search results:\n%s", ss.str().c_str());
        }
        return;
      }
    }
  }

  if (parameters_->print_debug_info) {
    const auto ss = get_accumulated_debug_stream(debug_data_vector);
    DEBUG_PRINT("\nPull out path search results:\n%s", ss.str().c_str());
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
  const Pose & refined_start_pose, const Pose & goal_pose, const double collision_check_margin,
  std::vector<PlannerDebugData> & debug_data_vector)
{
  // if start_pose_candidate is far from refined_start_pose, backward driving is necessary
  constexpr double epsilon = 0.01;
  const double backwards_distance =
    autoware::universe_utils::calcDistance2d(start_pose_candidate, refined_start_pose);
  const bool backward_is_unnecessary = backwards_distance < epsilon;

  planner->setCollisionCheckMargin(collision_check_margin);
  planner->setPlannerData(planner_data_);
  PlannerDebugData debug_data{
    planner->getPlannerType(), {}, collision_check_margin, backwards_distance};

  const auto pull_out_path = planner->plan(start_pose_candidate, goal_pose, debug_data);
  debug_data_vector.push_back(debug_data);
  // If no path is found, return false
  if (!pull_out_path) {
    return false;
  }

  if (backward_is_unnecessary) {
    updateStatusWithCurrentPath(*pull_out_path, start_pose_candidate, planner->getPlannerType());
    return true;
  }

  updateStatusWithNextPath(*pull_out_path, start_pose_candidate, planner->getPlannerType());

  return true;
}

void StartPlannerModule::updateStatusWithCurrentPath(
  const autoware::behavior_path_planner::PullOutPath & path, const Pose & start_pose,
  const autoware::behavior_path_planner::PlannerType & planner_type)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  status_.driving_forward = true;
  status_.found_pull_out_path = true;
  status_.pull_out_path = path;
  status_.pull_out_start_pose = start_pose;
  status_.planner_type = planner_type;
}

void StartPlannerModule::updateStatusWithNextPath(
  const autoware::behavior_path_planner::PullOutPath & path, const Pose & start_pose,
  const autoware::behavior_path_planner::PlannerType & planner_type)
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
  lanelet::ConstLanelets path_lanes;

  for (const auto & p : path.points) {
    for (const auto & id : p.lane_ids) {
      if (id == lanelet::InvalId) {
        continue;
      }
      const auto lanelet = lanelet_layer.get(id);
      if (route_handler->isShoulderLanelet(lanelet)) {
        continue;
      }
      if (std::find(lane_ids.begin(), lane_ids.end(), id) == lane_ids.end()) {
        lane_ids.push_back(id);
        path_lanes.push_back(lanelet);
      }
    }
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

  if (!status_.backward_driving_complete) {
    planWithPriority(
      start_pose_candidates, *refined_start_pose, goal_pose, parameters_->search_priority);
  }

  debug_data_.refined_start_pose = *refined_start_pose;
  debug_data_.start_pose_candidates = start_pose_candidates;
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);

  if (hasFinishedBackwardDriving()) {
    updateStatusAfterBackwardDriving();
    return;
  }
  status_.backward_path = start_planner_utils::getBackwardPath(
    *route_handler, pull_out_lanes, current_pose, status_.pull_out_start_pose,
    parameters_->backward_velocity);
  return;
}

void StartPlannerModule::updateStatusAfterBackwardDriving()
{
  status_.driving_forward = true;
  status_.backward_driving_complete = true;
  // request start_planner approval
  waitApproval();
  // To enable approval of the forward path, the RTC status is removed.
  removeRTCStatus();
  for (auto & itr : uuid_map_) {
    itr.second = generateUUID();
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
  const auto local_vehicle_footprint = vehicle_info_.createFootprint();
  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);
  const double backward_path_length =
    planner_data_->parameters.backward_path_length + parameters_->max_back_distance;

  const auto stop_objects_in_pull_out_lanes = filterStopObjectsInPullOutLanes(
    pull_out_lanes, start_pose.position, parameters_->th_moving_object_velocity,
    backward_path_length, std::numeric_limits<double>::max());

  const auto front_stop_objects_in_pull_out_lanes = filterStopObjectsInPullOutLanes(
    pull_out_lanes, start_pose.position, parameters_->th_moving_object_velocity, 0,
    std::numeric_limits<double>::max());

  // Set the maximum backward distance less than the distance from the vehicle's base_link to
  // the lane's rearmost point to prevent lane departure.
  const double current_arc_length =
    lanelet::utils::getArcCoordinates(pull_out_lanes, start_pose).length;
  const double allowed_backward_distance = std::clamp(
    current_arc_length - planner_data_->parameters.base_link2rear, 0.0,
    parameters_->max_back_distance);

  for (double back_distance = 0.0; back_distance <= allowed_backward_distance;
       back_distance += parameters_->backward_search_resolution) {
    const auto backed_pose = calcLongitudinalOffsetPose(
      back_path_from_start_pose.points, start_pose.position, -back_distance);
    if (!backed_pose) continue;

    if (utils::checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, *backed_pose, front_stop_objects_in_pull_out_lanes,
          parameters_->collision_check_margin_from_front_object))
      continue;

    const double backed_pose_arc_length =
      lanelet::utils::getArcCoordinates(pull_out_lanes, *backed_pose).length;
    const double length_to_lane_end = std::accumulate(
      std::begin(pull_out_lanes), std::end(pull_out_lanes), 0.0,
      [](double acc, const auto & lane) { return acc + lanelet::utils::getLaneletLength2d(lane); });
    const double distance_from_lane_end = length_to_lane_end - backed_pose_arc_length;
    if (distance_from_lane_end < parameters_->ignore_distance_from_lane_end) {
      RCLCPP_WARN_THROTTLE(
        getLogger(), *clock_, 5000,
        "the ego vehicle is too close to the lane end, so backwards driving is necessary");
      continue;
    }

    if (utils::checkCollisionBetweenFootprintAndObjects(
          local_vehicle_footprint, *backed_pose, stop_objects_in_pull_out_lanes,
          parameters_->collision_check_margins.back())) {
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
      stop_objects, pull_out_lanes, [](const auto & obj, const auto & lane) {
        return utils::path_safety_checker::isPolygonOverlapLanelet(obj, lane);
      });

  const auto path = planner_data_->route_handler->getCenterLinePath(
    pull_out_lanes, object_check_backward_distance, object_check_forward_distance);

  utils::path_safety_checker::filterObjectsByPosition(
    stop_objects_in_pull_out_lanes, path.points, current_point, object_check_forward_distance,
    object_check_backward_distance);

  utils::path_safety_checker::filterObjectsByClass(
    stop_objects_in_pull_out_lanes, parameters_->object_types_to_check_for_path_generation);

  return stop_objects_in_pull_out_lanes;
}

bool StartPlannerModule::hasReachedFreespaceEnd() const
{
  const auto & current_pose = planner_data_->self_odometry->pose.pose;
  return autoware::universe_utils::calcDistance2d(current_pose, status_.pull_out_path.end_pose) <
         parameters_->th_arrived_distance;
}

bool StartPlannerModule::hasReachedPullOutEnd() const
{
  const auto current_pose = planner_data_->self_odometry->pose.pose;

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

  return arclength_current.length - arclength_pull_out_end.length > offset;
}

bool StartPlannerModule::needToPrepareBlinkerBeforeStartDrivingForward() const
{
  if (!status_.first_engaged_and_driving_forward_time) {
    return false;
  }
  const auto first_engaged_and_driving_forward_time =
    status_.first_engaged_and_driving_forward_time.value();
  const double elapsed =
    rclcpp::Duration(clock_->now() - first_engaged_and_driving_forward_time).seconds();
  return elapsed < parameters_->prepare_time_before_start;
}

bool StartPlannerModule::hasFinishedCurrentPath()
{
  const auto current_path = getCurrentPath();
  const auto current_path_end = current_path.points.back();
  const auto self_pose = planner_data_->self_odometry->pose.pose;
  const bool is_near_target = autoware::universe_utils::calcDistance2d(
                                current_path_end, self_pose) < parameters_->th_arrived_distance;

  return is_near_target && isStopped();
}

TurnSignalInfo StartPlannerModule::calcTurnSignalInfo()
{
  const auto path = getFullPath();
  if (path.points.empty()) return getPreviousModuleOutput().turn_signal_info;

  const Pose & current_pose = planner_data_->self_odometry->pose.pose;
  const auto shift_start_idx = autoware::motion_utils::findNearestIndex(
    path.points, status_.pull_out_path.start_pose.position);
  const auto shift_end_idx =
    autoware::motion_utils::findNearestIndex(path.points, status_.pull_out_path.end_pose.position);
  const lanelet::ConstLanelets current_lanes = utils::getCurrentLanes(planner_data_);

  const auto is_ignore_signal = [this](const lanelet::Id & id) {
    if (!ignore_signal_.has_value()) {
      return false;
    }
    return ignore_signal_.value() == id;
  };

  const auto update_ignore_signal = [](const lanelet::Id & id, const bool is_ignore) {
    return is_ignore ? std::make_optional(id) : std::nullopt;
  };

  lanelet::Lanelet closest_lanelet;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &closest_lanelet);

  if (is_ignore_signal(closest_lanelet.id())) {
    return getPreviousModuleOutput().turn_signal_info;
  }

  const double current_shift_length =
    lanelet::utils::getArcCoordinates(current_lanes, current_pose).distance;

  constexpr bool egos_lane_is_shifted = true;
  constexpr bool is_pull_out = true;

  // In Geometric pull out, the ego stops once and then steers the wheels to the opposite direction.
  // This sometimes causes the getBehaviorTurnSignalInfo method to detect the ego as stopped and
  // close to complete its shift, so it wrongly turns off the blinkers, this override helps avoid
  // this issue. Also, if the ego is not engaged (so it is stopped), the blinkers should still be
  // activated.

  const bool geometric_planner_has_not_finished_first_path = std::invoke([&]() {
    if (status_.planner_type != PlannerType::GEOMETRIC) {
      return false;
    }
    constexpr double distance_threshold = 1.0;
    const auto stop_point = status_.pull_out_path.partial_paths.front().points.back();
    const double distance_from_ego_to_stop_point =
      std::abs(autoware::motion_utils::calcSignedArcLength(
        path.points, stop_point.point.pose.position, current_pose.position));
    return distance_from_ego_to_stop_point < distance_threshold;
  });

  const bool override_ego_stopped_check =
    !status_.has_departed || geometric_planner_has_not_finished_first_path;

  const auto [new_signal, is_ignore] = planner_data_->getBehaviorTurnSignalInfo(
    path, shift_start_idx, shift_end_idx, current_lanes, current_shift_length,
    status_.driving_forward, egos_lane_is_shifted, override_ego_stopped_check, is_pull_out);
  ignore_signal_ = update_ignore_signal(closest_lanelet.id(), is_ignore);

  const auto original_signal = getPreviousModuleOutput().turn_signal_info;
  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(path.points);
  const auto output_turn_signal_info = planner_data_->turn_signal_decider.use_prior_turn_signal(
    path, current_pose, current_seg_idx, original_signal, new_signal,
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  return output_turn_signal_info;
}

bool StartPlannerModule::isSafePath() const
{
  // TODO(Sugahara): should safety check for backward path

  const auto pull_out_path = getCurrentPath();
  if (pull_out_path.points.empty()) {
    return false;
  }
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
    autoware::behavior_path_planner::utils::path_safety_checker::createPredictedPath(
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

  // debug
  {
    debug_data_.filtered_objects = filtered_objects;
    debug_data_.target_objects_on_lane = target_objects_on_lane;
    debug_data_.ego_predicted_path = ego_predicted_path;
  }
  std::vector<ExtendedPredictedObject> merged_target_object;
  merged_target_object.reserve(
    target_objects_on_lane.on_current_lane.size() + target_objects_on_lane.on_shoulder_lane.size());
  merged_target_object.insert(
    merged_target_object.end(), target_objects_on_lane.on_current_lane.begin(),
    target_objects_on_lane.on_current_lane.end());
  merged_target_object.insert(
    merged_target_object.end(), target_objects_on_lane.on_shoulder_lane.begin(),
    target_objects_on_lane.on_shoulder_lane.end());

  return autoware::behavior_path_planner::utils::path_safety_checker::checkSafetyWithRSS(
    pull_out_path, ego_predicted_path, merged_target_object, debug_data_.collision_check,
    planner_data_->parameters, safety_check_params_->rss_params,
    objects_filtering_params_->use_all_predicted_path, hysteresis_factor,
    safety_check_params_->collision_check_yaw_diff_threshold);
}

bool StartPlannerModule::isGoalBehindOfEgoInSameRouteSegment() const
{
  const auto & rh = planner_data_->route_handler;

  // Check if the goal and ego are in the same route segment. If not, this is out of scope of
  // this function. Return false.
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
  const auto dist_ego_to_goal = autoware::motion_utils::calcSignedArcLength(
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

std::optional<PullOutStatus> StartPlannerModule::planFreespacePath(
  const StartPlannerParameters & parameters,
  const std::shared_ptr<const PlannerData> & planner_data, const PullOutStatus & pull_out_status)
{
  const Pose & current_pose = planner_data->self_odometry->pose.pose;
  const auto & route_handler = planner_data->route_handler;

  const double end_pose_search_start_distance = parameters.end_pose_search_start_distance;
  const double end_pose_search_end_distance = parameters.end_pose_search_end_distance;
  const double end_pose_search_interval = parameters.end_pose_search_interval;

  const double backward_path_length =
    planner_data->parameters.backward_path_length + parameters.max_back_distance;
  const auto current_lanes = utils::getExtendedCurrentLanes(
    planner_data, backward_path_length, std::numeric_limits<double>::max(),
    /*forward_only_in_route*/ true);

  const auto current_arc_coords = lanelet::utils::getArcCoordinates(current_lanes, current_pose);

  const double s_start = std::max(0.0, current_arc_coords.length + end_pose_search_start_distance);
  const double s_end = current_arc_coords.length + end_pose_search_end_distance;

  auto center_line_path = utils::resamplePathWithSpline(
    route_handler->getCenterLinePath(current_lanes, s_start, s_end), end_pose_search_interval);

  for (const auto & p : center_line_path.points) {
    const Pose end_pose = p.point.pose;
    freespace_planner_->setPlannerData(planner_data);
    PlannerDebugData debug_data{freespace_planner_->getPlannerType(), {}, 0.0, 0.0};
    auto freespace_path = freespace_planner_->plan(current_pose, end_pose, debug_data);
    DEBUG_PRINT(
      "\nFreespace Pull out path search results\n%s%s", debug_data.header_str().c_str(),
      debug_data.str().c_str());
    if (!freespace_path) {
      continue;
    }

    auto status = pull_out_status;
    status.pull_out_path = *freespace_path;
    status.pull_out_start_pose = current_pose;
    status.planner_type = freespace_planner_->getPlannerType();
    status.found_pull_out_path = true;
    status.driving_forward = true;
    return std::make_optional<PullOutStatus>(status);
  }

  return std::nullopt;
}

void StartPlannerModule::setDrivableAreaInfo(BehaviorModuleOutput & output) const
{
  switch (status_.planner_type) {
    case PlannerType::FREESPACE: {
      const double drivable_area_margin = planner_data_->parameters.vehicle_width;
      output.drivable_area_info.drivable_margin =
        planner_data_->parameters.vehicle_width / 2.0 + drivable_area_margin;
      return;
    }
    default: {
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
      return;
    }
  }
}

void StartPlannerModule::setDebugData()
{
  using autoware::universe_utils::createDefaultMarker;
  using autoware::universe_utils::createMarkerColor;
  using autoware::universe_utils::createMarkerScale;
  using lanelet::visualization::laneletsAsTriangleMarkerArray;
  using marker_utils::addFootprintMarker;
  using marker_utils::createFootprintMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createPathMarkerArray;
  using marker_utils::createPoseMarkerArray;
  using marker_utils::createPredictedPathMarkerArray;
  using marker_utils::showPolygon;
  using marker_utils::showPredictedPath;
  using marker_utils::showSafetyCheckInfo;
  using visualization_msgs::msg::Marker;

  const auto red_color = createMarkerColor(1.0, 0.0, 0.0, 0.999);
  const auto cyan_color = createMarkerColor(0.0, 1.0, 1.0, 0.2);
  const auto pink_color = createMarkerColor(1.0, 0.5, 0.5, 0.35);
  const auto purple_color = createMarkerColor(1.0, 0.0, 1.0, 0.99);
  const auto white_color = createMarkerColor(1.0, 1.0, 1.0, 0.99);

  const auto life_time = rclcpp::Duration::from_seconds(1.5);
  auto add = [&](MarkerArray added, MarkerArray & target_marker_array) {
    for (auto & marker : added.markers) {
      marker.lifetime = life_time;
    }
    autoware::universe_utils::appendMarkerArray(added, &target_marker_array);
  };

  debug_marker_.markers.clear();
  info_marker_.markers.clear();
  add(
    createPoseMarkerArray(status_.pull_out_start_pose, "back_end_pose", 0, 0.9, 0.3, 0.3),
    info_marker_);
  add(
    createPoseMarkerArray(status_.pull_out_path.start_pose, "start_pose", 0, 0.3, 0.9, 0.3),
    info_marker_);
  add(
    createPoseMarkerArray(status_.pull_out_path.end_pose, "end_pose", 0, 0.9, 0.9, 0.3),
    info_marker_);
  add(
    createFootprintMarkerArray(
      debug_data_.refined_start_pose, vehicle_info_, "refined_start_pose", 0, 0.9, 0.9, 0.3),
    debug_marker_);
  add(createPathMarkerArray(getFullPath(), "full_path", 0, 0.0, 0.5, 0.9), debug_marker_);
  add(
    createPathMarkerArray(status_.backward_path, "backward_driving_path", 0, 0.0, 0.9, 0.0),
    debug_marker_);

  // visualize collision_check_end_pose and footprint
  {
    std::map<PlannerType, double> collision_check_distances = {
      {PlannerType::SHIFT, parameters_->shift_collision_check_distance_from_end},
      {PlannerType::GEOMETRIC, parameters_->geometric_collision_check_distance_from_end}};

    double collision_check_distance_from_end = collision_check_distances[status_.planner_type];
    const auto collision_check_end_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
      getFullPath().points, status_.pull_out_path.end_pose.position,
      collision_check_distance_from_end);
    if (collision_check_end_pose) {
      add(
        createPoseMarkerArray(
          *collision_check_end_pose, "static_collision_check_end_pose", 0, 1.0, 0.0, 0.0),
        info_marker_);
      auto marker = createDefaultMarker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "static_collision_check_end_polygon", 0,
        Marker::LINE_STRIP, createMarkerScale(0.1, 0.1, 0.1), red_color);
      addFootprintMarker(marker, *collision_check_end_pose, vehicle_info_);
      marker.lifetime = life_time;
      info_marker_.markers.push_back(marker);
    }
  }
  // start pose candidates
  {
    MarkerArray start_pose_footprint_marker_array{};
    MarkerArray start_pose_text_marker_array{};
    Marker footprint_marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "start_pose_candidates", 0, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.2, 0.2), purple_color);
    Marker text_marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "start_pose_candidates_idx", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING, createMarkerScale(0.3, 0.3, 0.3),
      purple_color);
    footprint_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    text_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    for (size_t i = 0; i < debug_data_.start_pose_candidates.size(); ++i) {
      footprint_marker.id = i;
      text_marker.id = i;
      footprint_marker.points.clear();
      text_marker.text = "idx[" + std::to_string(i) + "]";
      text_marker.pose = debug_data_.start_pose_candidates.at(i);
      addFootprintMarker(footprint_marker, debug_data_.start_pose_candidates.at(i), vehicle_info_);
      start_pose_footprint_marker_array.markers.push_back(footprint_marker);
      start_pose_text_marker_array.markers.push_back(text_marker);
    }

    add(start_pose_footprint_marker_array, debug_marker_);
    add(start_pose_text_marker_array, debug_marker_);
  }

  // visualize the footprint from pull_out_start pose to pull_out_end pose along the path
  {
    MarkerArray pull_out_path_footprint_marker_array{};
    Marker pull_out_path_footprint_marker = createDefaultMarker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), "shift_path_footprint", 0, Marker::LINE_STRIP,
      createMarkerScale(0.2, 0.2, 0.2), pink_color);
    pull_out_path_footprint_marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    PathWithLaneId path_shift_start_to_end{};
    const auto shift_path = status_.pull_out_path.partial_paths.front();
    {
      const size_t pull_out_start_idx = autoware::motion_utils::findNearestIndex(
        shift_path.points, status_.pull_out_path.start_pose.position);
      const size_t pull_out_end_idx = autoware::motion_utils::findNearestIndex(
        shift_path.points, status_.pull_out_path.end_pose.position);

      path_shift_start_to_end.points.insert(
        path_shift_start_to_end.points.begin(), shift_path.points.begin() + pull_out_start_idx,
        shift_path.points.begin() + pull_out_end_idx + 1);
    }

    for (size_t i = 0; i < path_shift_start_to_end.points.size(); ++i) {
      pull_out_path_footprint_marker.id = i;
      pull_out_path_footprint_marker.points.clear();
      addFootprintMarker(
        pull_out_path_footprint_marker, path_shift_start_to_end.points.at(i).point.pose,
        vehicle_info_);
      pull_out_path_footprint_marker_array.markers.push_back(pull_out_path_footprint_marker);
    }

    add(pull_out_path_footprint_marker_array, debug_marker_);
  }

  // safety check
  if (parameters_->safety_check_params.enable_safety_check) {
    if (!debug_data_.ego_predicted_path.empty()) {
      const auto & ego_predicted_path = utils::path_safety_checker::convertToPredictedPath(
        debug_data_.ego_predicted_path, ego_predicted_path_params_->time_resolution);
      add(
        createPredictedPathMarkerArray(
          ego_predicted_path, vehicle_info_, "ego_predicted_path_start_planner", 0, 0.0, 0.5, 0.9),
        debug_marker_);
    }

    if (!debug_data_.filtered_objects.objects.empty()) {
      add(
        createObjectsMarkerArray(
          debug_data_.filtered_objects, "filtered_objects", 0, 0.0, 0.5, 0.9),
        info_marker_);
    }

    add(showSafetyCheckInfo(debug_data_.collision_check, "object_debug_info"), debug_marker_);
    add(
      showPredictedPath(debug_data_.collision_check, "predicted_path_for_safety_check"),
      info_marker_);
    add(showPolygon(debug_data_.collision_check, "ego_and_target_polygon_relation"), info_marker_);

    // set objects of interest
    for (const auto & [uuid, data] : debug_data_.collision_check) {
      const auto color = data.is_safe ? ColorName::GREEN : ColorName::RED;
      setObjectsOfInterestData(data.current_obj_pose, data.obj_shape, color);
    }

    initializeCollisionCheckDebugMap(debug_data_.collision_check);
  }

  // Visualize planner type text
  const auto header = planner_data_->route_handler->getRouteHeader();
  {
    visualization_msgs::msg::MarkerArray planner_type_marker_array{};
    const auto color = status_.found_pull_out_path ? white_color : red_color;
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
    add(planner_type_marker_array, info_marker_);
  }

  add(
    laneletsAsTriangleMarkerArray(
      "departure_check_lanes_for_shift_pull_out_path", debug_data_.departure_check_lanes,
      cyan_color),
    debug_marker_);

  const auto pull_out_lanes = start_planner_utils::getPullOutLanes(
    planner_data_, planner_data_->parameters.backward_path_length + parameters_->max_back_distance);
  add(
    laneletsAsTriangleMarkerArray(
      "pull_out_lanes_for_static_objects_collision_check", pull_out_lanes, pink_color),
    debug_marker_);
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

StartPlannerModule::StartPlannerData StartPlannerModule::StartPlannerData::clone() const
{
  StartPlannerData start_planner_data;
  start_planner_data.update(
    parameters, planner_data, current_status, main_thread_pull_out_status, is_stopped);
  return start_planner_data;
}

void StartPlannerModule::StartPlannerData::update(
  const StartPlannerParameters & parameters_, const PlannerData & planner_data_,
  const ModuleStatus & current_status_, const PullOutStatus & pull_out_status_,
  const bool is_stopped_)
{
  parameters = parameters_;
  planner_data = planner_data_;
  // TODO(Mamoru Sobue): in the future we will add planner_data->is_route_handler_updated flag to
  // avoid the copy overhead of lanelet objects inside the RouteHandler. behavior_path_planner can
  // tell us the flag if map/route changed, so we can skip route_handler update if it
  // is false in the following way
  /*
    auto route_handler_self = planner_data.route_handler;
    planner_data = planner_data_; // sync planer_data to planner_data_, planner_data.route_handler
    is once re-pointed

    if (!planner_data_->is_route_handler_updated && route_handler_self != nullptr) {
      // we do not need to sync planner_data.route_handler with that of planner_data_
      // re-point to the original again
      planner_data.route_handler = route_handler_self;
    } else {
      // this is actually heavy if the lanelet_map is HUGE
      planner_data.route_handler = std::make_shared<RouteHandler>(*(planner_data_.route_handler));
    }
   */
  planner_data.route_handler = std::make_shared<RouteHandler>(*(planner_data_.route_handler));
  current_status = current_status_;
  main_thread_pull_out_status = pull_out_status_;
  is_stopped = is_stopped_;
}
}  // namespace autoware::behavior_path_planner
