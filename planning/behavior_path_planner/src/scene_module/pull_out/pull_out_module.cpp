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

#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"

#include "behavior_path_planner/behavior_path_planner_node.hpp"
#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/debug.hpp"
#include "behavior_path_planner/scene_module/pull_out/util.hpp"
#include "behavior_path_planner/util/create_vehicle_footprint.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <autoware_utils/autoware_utils.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
PullOutModule::PullOutModule(
  const std::string & name, rclcpp::Node & node, const PullOutParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  approval_handler_.waitApproval();
}

BehaviorModuleOutput PullOutModule::run()
{
  approval_handler_.clearWaitApproval();
  current_state_ = BT::NodeStatus::RUNNING;
  return plan();
}

void PullOutModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OUT onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;
  updatePullOutStatus();

  // Get arclength to start lane change
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.pull_out_lanes, current_pose);
  status_.back_finished = false;
  status_.start_distance = arclength_start.length;
  approval_handler_.waitApproval();
}

void PullOutModule::onExit()
{
  approval_handler_.clearWaitApproval();
  current_state_ = BT::NodeStatus::IDLE;
  RCLCPP_DEBUG(getLogger(), "PULL_OUT onExit");
}

bool PullOutModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  const bool car_is_stopping =
    (util::l2Norm(planner_data_->self_odometry->twist.twist.linear) <= 1.5) ? true : false;

  lanelet::Lanelet closest_shoulder_lanelet;

  if (
    lanelet::utils::query::getClosestLanelet(
      planner_data_->route_handler->getShoulderLanelets(), planner_data_->self_pose->pose,
      &closest_shoulder_lanelet) &&
    car_is_stopping) {
    // Create vehicle footprint
    const auto vehicle_info = getVehicleInfo(planner_data_->parameters);
    const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info);
    const auto vehicle_footprint = transformVector(
      local_vehicle_footprint, autoware_utils::pose2transform(planner_data_->self_pose->pose));
    const auto road_lanes = getCurrentLanes();

    // check if goal pose is in shoulder lane and distance is long enough for pull out
    if (isInLane(closest_shoulder_lanelet, vehicle_footprint) && isLongEnough(road_lanes)) {
      return true;
    }
  }

  return false;
}

bool PullOutModule::isExecutionReady() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // TODO(sugahara) move to utility function
  const auto road_lanes = getCurrentLanes();
  const auto shoulder_lanes = getPullOutLanes(road_lanes);

  // Find pull_out path
  bool found_valid_path, found_safe_path;
  PullOutPath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(shoulder_lanes, check_distance_, selected_path);

  if (found_valid_path && !found_safe_path) {
    double back_distance;
    if (getBackDistance(shoulder_lanes, check_distance_, selected_path, back_distance)) {
      return true;
    }
  }
  return found_safe_path;
}  // namespace behavior_path_planner

BT::NodeStatus PullOutModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OUT updateState");

  // finish after lane change
  if (status_.back_finished && hasFinishedPullOut()) {
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }
  if (status_.is_retreat_path_valid) {
    if (hasFinishedBack()) {
      status_.back_finished = true;
    }
  }
  current_state_ = BT::NodeStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput PullOutModule::plan()
{
  constexpr double RESAMPLE_INTERVAL = 1.0;

  PathWithLaneId path;
  if (status_.is_retreat_path_valid && !status_.is_safe) {
    path = util::resamplePathWithSpline(status_.straight_back_path.path, RESAMPLE_INTERVAL);
  } else {
    path = util::resamplePathWithSpline(status_.pull_out_path.path, RESAMPLE_INTERVAL);
    status_.back_finished = true;
  }

  if (status_.is_retreat_path_valid && status_.back_finished) {
    path = util::resamplePathWithSpline(status_.retreat_path.path, RESAMPLE_INTERVAL);
  }

  path.drivable_area = status_.pull_out_path.path.drivable_area;

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(path);
  output.turn_signal_info = calcTurnSignalInfo(status_.pull_out_path.shift_point);

  return output;
}

PathWithLaneId PullOutModule::planCandidate() const
{
  // Get lane change lanes
  const auto current_lanes = getCurrentLanes();
  const auto shoulder_lanes = getPullOutLanes(current_lanes);

  // Find pull out path
  bool found_valid_path, found_safe_path;
  PullOutPath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(shoulder_lanes, check_distance_, selected_path);

  if (found_valid_path && !found_safe_path) {
    double back_distance;
    if (getBackDistance(shoulder_lanes, check_distance_, selected_path, back_distance)) {
      bool found_valid_retreat_path, found_safe_retreat_path;
      RetreatPath selected_retreat_path;
      std::tie(found_valid_retreat_path, found_safe_retreat_path) =
        getSafeRetreatPath(shoulder_lanes, check_distance_, selected_retreat_path, back_distance);
      // ROS_ERROR("found safe retreat path in plan candidate :%d", found_safe_retreat_path);
      if (found_safe_retreat_path == true) {
        selected_retreat_path.pull_out_path.path.header =
          planner_data_->route_handler->getRouteHeader();
        return selected_retreat_path.pull_out_path.path;
      }
    }
  }
  // ROS_ERROR("found safe path in plan candidate :%d", found_safe_path);

  selected_path.path.header = planner_data_->route_handler->getRouteHeader();
  return selected_path.path;
}

BehaviorModuleOutput PullOutModule::planWaitingApproval()
{
  BehaviorModuleOutput out;
  const auto & route_handler = planner_data_->route_handler;
  const auto common_parameters = planner_data_->parameters;
  const auto current_lanes = getCurrentLanes();
  const auto shoulder_lanes = getPullOutLanes(current_lanes);

  PathWithLaneId candidatePath;
  // Generate drivable area
  {
    candidatePath = planCandidate();
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), current_lanes.begin(), current_lanes.end());
    lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());
    const double width = common_parameters.drivable_area_width;
    const double height = common_parameters.drivable_area_height;
    const double resolution = common_parameters.drivable_area_resolution;
    candidatePath.drivable_area = util::generateDrivableArea(
      lanes, *(planner_data_->self_pose), width, height, resolution,
      common_parameters.vehicle_length, *route_handler);
  }
  for (size_t i = 1; i < candidatePath.points.size(); i++) {
    candidatePath.points.at(i).point.longitudinal_velocity_mps = 0.0;
  }
  out.path = std::make_shared<PathWithLaneId>(candidatePath);

  out.path_candidate = std::make_shared<PathWithLaneId>(planCandidate());

  return out;
}

void PullOutModule::setParameters(const PullOutParameters & parameters)
{
  parameters_ = parameters;
}

void PullOutModule::updatePullOutStatus()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto common_parameters = planner_data_->parameters;

  const auto current_lanes = getCurrentLanes();
  status_.current_lanes = current_lanes;

  // Get pull_out lanes
  const auto pull_out_lanes = getPullOutLanes(current_lanes);
  status_.pull_out_lanes = pull_out_lanes;

  const auto current_pose = planner_data_->self_pose->pose;
  // const auto current_twist = planner_data_->self_odometry->twist.twist;
  // const auto common_parameters = planner_data_->parameters;

  // Find pull_out path
  bool found_valid_path, found_safe_path;
  PullOutPath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(pull_out_lanes, check_distance_, selected_path);

  if (found_valid_path && !found_safe_path) {
    double back_distance;
    if (getBackDistance(pull_out_lanes, check_distance_, selected_path, back_distance)) {
      bool found_valid_retreat_path, found_safe_retreat_path;
      RetreatPath selected_retreat_path;
      std::tie(found_valid_retreat_path, found_safe_retreat_path) =
        getSafeRetreatPath(pull_out_lanes, check_distance_, selected_retreat_path, back_distance);
      if (found_valid_retreat_path && found_safe_retreat_path) {
        status_.is_retreat_path_valid = true;
        status_.backed_pose = selected_retreat_path.backed_pose;
        status_.retreat_path = selected_retreat_path.pull_out_path;
        status_.retreat_path.path.header = planner_data_->route_handler->getRouteHeader();
        status_.straight_back_path = pull_out_utils::getBackPaths(
          *route_handler, pull_out_lanes, current_pose, common_parameters, parameters_,
          back_distance);
      }
    }
  }

  // Update status
  status_.is_safe = found_safe_path;
  status_.pull_out_path = selected_path;

  status_.lane_follow_lane_ids = util::getIds(current_lanes);
  status_.pull_out_lane_ids = util::getIds(pull_out_lanes);

  // Generate drivable area
  {
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), current_lanes.begin(), current_lanes.end());
    lanes.insert(lanes.end(), pull_out_lanes.begin(), pull_out_lanes.end());

    const double width = common_parameters.drivable_area_width;
    const double height = common_parameters.drivable_area_height;
    const double resolution = common_parameters.drivable_area_resolution;
    status_.pull_out_path.path.drivable_area = util::generateDrivableArea(
      lanes, *(planner_data_->self_pose), width, height, resolution,
      common_parameters.vehicle_length, *route_handler);
  }

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.pull_out_lanes, current_pose);
  status_.start_distance = arclength_start.length;

  status_.pull_out_path.path.header = planner_data_->route_handler->getRouteHeader();
}

PathWithLaneId PullOutModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto goal_pose = planner_data_->route_handler->getGoalPose();
  const auto common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  const auto current_lanes = getCurrentLanes();
  const auto pull_out_lanes = getPullOutLanes(current_lanes);

  if (current_lanes.empty()) {
    return reference_path;
  }

  reference_path = util::getCenterLinePath(
    *route_handler, pull_out_lanes, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length, common_parameters);

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, parameters_.after_pull_out_straight_distance,
    common_parameters.minimum_pull_out_length, parameters_.before_pull_out_straight_distance,
    parameters_.deceleration_interval, goal_pose);

  reference_path.drivable_area = util::generateDrivableArea(
    pull_out_lanes, *planner_data_->self_pose, common_parameters.drivable_area_width,
    common_parameters.drivable_area_height, common_parameters.drivable_area_resolution,
    common_parameters.vehicle_length, *planner_data_->route_handler);

  return reference_path;
}

lanelet::ConstLanelets PullOutModule::getCurrentLanes() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    RCLCPP_ERROR(getLogger(), "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);
}

// getShoulderLanesOnCurrentPose?
lanelet::ConstLanelets PullOutModule::getPullOutLanes(
  const lanelet::ConstLanelets & current_lanes) const
{
  lanelet::ConstLanelets shoulder_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  lanelet::ConstLanelet shoulder_lane;

  if (current_lanes.empty()) {
    return shoulder_lanes;
  }

  // Get pull out lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(
    current_lanes, planner_data_->self_pose->pose, &current_lane);

  if (route_handler->getPullOutStart(
        route_handler->getShoulderLanelets(), &shoulder_lane, current_pose,
        planner_data_->parameters.vehicle_width)) {
    shoulder_lanes = route_handler->getShoulderLaneletSequence(
      shoulder_lane, current_pose, pull_out_lane_length_, pull_out_lane_length_);

  } else {
    RCLCPP_ERROR(getLogger(), "getPullOverTarget didn't work");
    shoulder_lanes.clear();
  }

  return shoulder_lanes;
}

std::pair<bool, bool> PullOutModule::getSafePath(
  const lanelet::ConstLanelets & pull_out_lanes, const double check_distance,
  PullOutPath & safe_path) const
{
  std::vector<PullOutPath> valid_paths;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto common_parameters = planner_data_->parameters;
  const auto road_lanes = getCurrentLanes();

  const auto vehicle_info = getVehicleInfo(planner_data_->parameters);
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info);

  if (!pull_out_lanes.empty()) {
    // find candidate paths
    const auto pull_out_paths = pull_out_utils::getPullOutPaths(
      *route_handler, road_lanes, pull_out_lanes, current_pose, common_parameters, parameters_);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!pull_out_paths.empty()) {
      const auto & longest_path = pull_out_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.pull_out_length;
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, pull_out_lanes, check_distance_with_path);
    }

    // select valid path
    valid_paths = pull_out_utils::selectValidPaths(
      pull_out_paths, road_lanes, check_lanes, route_handler->getOverallGraph(), current_pose,
      route_handler->isInGoalRouteSection(road_lanes.back()), route_handler->getGoalPose());

    if (valid_paths.empty()) {
      RCLCPP_DEBUG(getLogger(), "valid path is empty");
      return std::make_pair(false, false);
    }
    // select safe path
    bool found_safe_path = pull_out_utils::selectSafePath(
      valid_paths, road_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters.vehicle_width, parameters_, local_vehicle_footprint,
      &safe_path);

    return std::make_pair(true, found_safe_path);
  }
  return std::make_pair(false, false);
}

std::pair<bool, bool> PullOutModule::getSafeRetreatPath(
  const lanelet::ConstLanelets & pull_out_lanes, const double check_distance,
  RetreatPath & safe_retreat_path, double & back_distance) const
{
  std::vector<PullOutPath> valid_paths;
  PullOutPath safe_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto common_parameters = planner_data_->parameters;

  const auto road_lanes = getCurrentLanes();

  const auto vehicle_info = getVehicleInfo(planner_data_->parameters);
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info);

  lanelet::ConstLanelet closest_shoulder_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(
        pull_out_lanes, current_pose, &closest_shoulder_lanelet)) {
    // RCLCPP_ERROR_THROTTLE(getLogger(), clock, 1000, "Failed to find closest lane!");
  }
  const auto arc_position_pose = lanelet::utils::getArcCoordinates(pull_out_lanes, current_pose);

  const auto shoulder_line_path = route_handler->getCenterLinePath(
    pull_out_lanes, arc_position_pose.length - pull_out_lane_length_,
    arc_position_pose.length + pull_out_lane_length_);
  const auto idx = autoware_utils::findNearestIndex(shoulder_line_path.points, current_pose);
  const auto yaw_shoulder_lane =
    tf2::getYaw(shoulder_line_path.points.at(*idx).point.pose.orientation);

  const auto backed_pose =
    pull_out_utils::getBackedPose(current_pose, yaw_shoulder_lane, back_distance);

  if (!pull_out_lanes.empty()) {
    // find candidate paths
    const auto pull_out_paths = pull_out_utils::getPullOutPaths(
      *route_handler, road_lanes, pull_out_lanes, backed_pose, common_parameters, parameters_,
      true);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!pull_out_paths.empty()) {
      const auto & longest_path = pull_out_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.pull_out_length;
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, pull_out_lanes, check_distance_with_path);
    }

    // select valid path
    valid_paths = pull_out_utils::selectValidPaths(
      pull_out_paths, road_lanes, check_lanes, route_handler->getOverallGraph(), current_pose,
      route_handler->isInGoalRouteSection(road_lanes.back()), route_handler->getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }
    // select safe path
    bool found_safe_path = pull_out_utils::selectSafePath(
      valid_paths, road_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters.vehicle_width, parameters_, local_vehicle_footprint,
      &safe_path);
    safe_retreat_path.pull_out_path = safe_path;
    safe_retreat_path.backed_pose = backed_pose;

    return std::make_pair(true, found_safe_path);
  }
  return std::make_pair(false, false);
}

bool PullOutModule::getBackDistance(
  const lanelet::ConstLanelets & pull_out_lanes, const double check_distance,
  PullOutPath & safe_path, double & back_distance) const
{
  std::vector<PullOutPath> valid_paths;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto common_parameters = planner_data_->parameters;

  const double back_distance_search_resolution = 1;
  const double maximum_back_distance = 15;

  const auto road_lanes = getCurrentLanes();

  const auto vehicle_info = getVehicleInfo(planner_data_->parameters);
  const auto local_vehicle_footprint = createVehicleFootprint(vehicle_info);

  double yaw_shoulder_lane;
  {
    lanelet::ConstLanelet closest_shoulder_lanelet;
    if (!lanelet::utils::query::getClosestLanelet(
          pull_out_lanes, current_pose, &closest_shoulder_lanelet)) {
      return false;
    }

    const auto arc_position_pose = lanelet::utils::getArcCoordinates(pull_out_lanes, current_pose);

    const auto shoulder_line_path = route_handler->getCenterLinePath(
      pull_out_lanes, arc_position_pose.length - pull_out_lane_length_,
      arc_position_pose.length + pull_out_lane_length_);
    const auto idx = autoware_utils::findNearestIndex(shoulder_line_path.points, current_pose);
    yaw_shoulder_lane = tf2::getYaw(shoulder_line_path.points.at(*idx).point.pose.orientation);
  }

  for (double current_back_distance = back_distance_search_resolution;
       current_back_distance <= maximum_back_distance;
       current_back_distance += back_distance_search_resolution) {
    if (!pull_out_lanes.empty()) {
      const auto backed_pose =
        pull_out_utils::getBackedPose(current_pose, yaw_shoulder_lane, current_back_distance);

      // find candidate paths
      const auto pull_out_paths = pull_out_utils::getPullOutPaths(
        *route_handler, road_lanes, pull_out_lanes, backed_pose, common_parameters, parameters_,
        true);

      // get lanes used for detection
      lanelet::ConstLanelets check_lanes;
      if (!pull_out_paths.empty()) {
        const auto & longest_path = pull_out_paths.front();
        // we want to see check_distance [m] behind vehicle so add lane changing length
        const double check_distance_with_path =
          check_distance + longest_path.preparation_length + longest_path.pull_out_length;
        check_lanes = route_handler->getCheckTargetLanesFromPath(
          longest_path.path, pull_out_lanes, check_distance_with_path);
      }

      // select valid path
      valid_paths = pull_out_utils::selectValidPaths(
        pull_out_paths, road_lanes, check_lanes, route_handler->getOverallGraph(), current_pose,
        route_handler->isInGoalRouteSection(road_lanes.back()), route_handler->getGoalPose());

      if (valid_paths.empty()) {
        continue;
      }
      // select safe path
      bool found_safe_path = pull_out_utils::selectSafePath(
        valid_paths, road_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
        current_twist, common_parameters.vehicle_width, parameters_, local_vehicle_footprint,
        &safe_path);
      if (found_safe_path) {
        back_distance = current_back_distance;
        return found_safe_path;
      }
    }
  }
  return false;
}

bool PullOutModule::isInLane(
  const lanelet::ConstLanelet & candidate_lanelet,
  const autoware_utils::LinearRing2d & vehicle_footprint) const
{
  for (const auto & point : vehicle_footprint) {
    if (boost::geometry::within(point, candidate_lanelet.polygon2d().basicPolygon())) {
      return true;
    }
  }

  return false;
}

bool PullOutModule::isLongEnough(const lanelet::ConstLanelets & lanelets) const
{
  PathShifter path_shifter;
  const double maximum_jerk = parameters_.maximum_lateral_jerk;
  const double pull_out_velocity = parameters_.minimum_pull_out_velocity;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto goal_pose = planner_data_->route_handler->getGoalPose();
  const double distance_before_pull_out = parameters_.before_pull_out_straight_distance;
  const double distance_after_pull_out = parameters_.after_pull_out_straight_distance;
  const double distance_to_road_center =
    lanelet::utils::getArcCoordinates(lanelets, planner_data_->self_pose->pose).distance;

  // calculate minimum pull_out distance at pull_out velocity,
  // maximum jerk and calculated side offset
  const double pull_out_distance_min = path_shifter.calcLongitudinalDistFromJerk(
    abs(distance_to_road_center), maximum_jerk, pull_out_velocity);
  const double pull_out_total_distance_min =
    distance_before_pull_out + pull_out_distance_min + distance_after_pull_out;
  const double distance_to_goal_on_road_lane =
    util::getSignedDistance(current_pose, goal_pose, lanelets);

  return distance_to_goal_on_road_lane > pull_out_total_distance_min;
}

bool PullOutModule::isSafe() const { return status_.is_safe; }

bool PullOutModule::isNearEndOfLane() const
{
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;
  const double threshold = 5 + common_parameters.minimum_pull_over_length;

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool PullOutModule::isCurrentSpeedLow() const
{
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const double threshold_kmph = 10;
  return util::l2Norm(current_twist.linear) < threshold_kmph * 1000 / 3600;
}

bool PullOutModule::hasFinishedPullOut() const
{
  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.current_lanes, current_pose);
  const auto arclength_shift_end =
    lanelet::utils::getArcCoordinates(status_.current_lanes, status_.pull_out_path.shift_point.end);
  const bool car_is_on_goal = (arclength_shift_end.length - arclength_current.length <
                               parameters_.pull_out_finish_judge_buffer)
                                ? true
                                : false;

  return car_is_on_goal;
}

bool PullOutModule::hasFinishedBack() const
{
  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_pose->pose;
  const auto backed_pose = status_.backed_pose;
  const auto distance = autoware_utils::calcDistance2d(current_pose, backed_pose);

  return distance < 1;
}

TurnSignalInfo PullOutModule::calcTurnSignalInfo(const ShiftPoint & shift_point) const
{
  TurnSignalInfo turn_signal;

  if (status_.is_retreat_path_valid && !status_.back_finished) {
    turn_signal.hazard_signal.command = HazardLightsCommand::ENABLE;
    turn_signal.signal_distance =
      autoware_utils::calcDistance2d(status_.backed_pose, planner_data_->self_pose->pose);
    return turn_signal;
  }

  const auto current_lanes = getCurrentLanes();
  const auto pull_out_lanes = getPullOutLanes(current_lanes);
  const double turn_signal_on_threshold = 30;
  const double turn_signal_off_threshold = -3;
  const double turn_hazard_on_threshold = 3;

  // calculate distance to pull_out start on current lanes
  double distance_to_pull_out_start;
  {
    const auto pull_out_start = shift_point.start;
    const auto arc_position_pull_out_start =
      lanelet::utils::getArcCoordinates(current_lanes, pull_out_start);
    const auto arc_position_current_pose =
      lanelet::utils::getArcCoordinates(current_lanes, planner_data_->self_pose->pose);
    distance_to_pull_out_start =
      arc_position_pull_out_start.length - arc_position_current_pose.length;
  }

  // calculate distance to pull_out end on target lanes
  double distance_to_pull_out_end;
  {
    const auto pull_out_end = shift_point.end;
    const auto arc_position_pull_out_end =
      lanelet::utils::getArcCoordinates(pull_out_lanes, pull_out_end);
    const auto arc_position_current_pose =
      lanelet::utils::getArcCoordinates(pull_out_lanes, planner_data_->self_pose->pose);
    distance_to_pull_out_end = arc_position_pull_out_end.length - arc_position_current_pose.length;
  }

  // calculate distance to pull_out start on target lanes
  double distance_to_target_pose;
  {
    const auto arc_position_target_pose = lanelet::utils::getArcCoordinates(
      pull_out_lanes, planner_data_->route_handler->getGoalPose());
    const auto arc_position_current_pose =
      lanelet::utils::getArcCoordinates(pull_out_lanes, planner_data_->self_pose->pose);
    distance_to_target_pose = arc_position_target_pose.length - arc_position_current_pose.length;
  }

  if (distance_to_pull_out_start < turn_signal_on_threshold) {
    turn_signal.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
    if (distance_to_pull_out_end < turn_signal_off_threshold) {
      turn_signal.turn_signal.command = TurnIndicatorsCommand::DISABLE;
      if (distance_to_target_pose < turn_hazard_on_threshold) {
        turn_signal.hazard_signal.command = HazardLightsCommand::ENABLE;
      }
    }
  }
  turn_signal.signal_distance = distance_to_pull_out_end;

  return turn_signal;
}

vehicle_info_util::VehicleInfo PullOutModule::getVehicleInfo(
  const BehaviorPathPlannerParameters & parameters) const
{
  vehicle_info_util::VehicleInfo vehicle_info;
  vehicle_info.front_overhang_m = parameters.front_overhang;
  vehicle_info.wheel_base_m = parameters.wheel_base;
  vehicle_info.rear_overhang_m = parameters.rear_overhang;
  vehicle_info.wheel_tread_m = parameters.wheel_tread;
  vehicle_info.left_overhang_m = parameters.left_over_hang;
  vehicle_info.right_overhang_m = parameters.right_over_hang;
  return vehicle_info;
}

}  // namespace behavior_path_planner
