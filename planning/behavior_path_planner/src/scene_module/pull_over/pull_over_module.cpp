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

#include "behavior_path_planner/behavior_path_planner_node.hpp"
#include "behavior_path_planner/path_shifter/path_shifter.hpp"
#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/debug.hpp"
#include "behavior_path_planner/scene_module/pull_over/util.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
PullOverModule::PullOverModule(
  const std::string & name, rclcpp::Node & node, const PullOverParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  approval_handler_.waitApproval();
}

BehaviorModuleOutput PullOverModule::run()
{
  approval_handler_.clearWaitApproval();
  current_state_ = BT::NodeStatus::RUNNING;
  return plan();
}

void PullOverModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OVER onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;
  updatePullOverStatus();
  // Get arclength to start lane change
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.pull_over_lanes, current_pose);
  status_.start_distance = arclength_start.length;
  approval_handler_.waitApproval();
}

void PullOverModule::onExit()
{
  approval_handler_.clearWaitApproval();
  current_state_ = BT::NodeStatus::IDLE;
  RCLCPP_DEBUG(getLogger(), "PULL_OVER onExit");
}

bool PullOverModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  PathShifter path_shifter;
  lanelet::Lanelet closest_shoulder_lanelet;
  bool goal_is_in_shoulder_lane = false;
  const auto goal_pose = planner_data_->route_handler->getGoalPose();
  const auto current_lanes = getCurrentLanes();

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
  return goal_is_in_shoulder_lane && isLongEnough(current_lanes);
}

bool PullOverModule::isExecutionReady() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  const auto current_lanes = getCurrentLanes();
  const auto pull_over_lanes = getPullOverLanes(current_lanes);

  // Find pull_over path
  bool found_valid_path, found_safe_path;
  PullOverPath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(pull_over_lanes, check_distance_, selected_path);
  return found_safe_path;
}

BT::NodeStatus PullOverModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "PULL_OVER updateState");

  if (hasFinishedPullOver()) {
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }
  current_state_ = BT::NodeStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput PullOverModule::plan()
{
  constexpr double RESAMPLE_INTERVAL = 1.0;
  auto path = util::resamplePathWithSpline(status_.pull_over_path.path, RESAMPLE_INTERVAL);

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(path);

  const auto hazard_info = getHazard(
    status_.pull_over_lanes, planner_data_->self_pose->pose,
    planner_data_->route_handler->getGoalPose(), planner_data_->self_odometry->twist.twist.linear.x,
    parameters_.hazard_on_threshold_dis, parameters_.hazard_on_threshold_vel,
    planner_data_->parameters.base_link2front);

  const auto turn_info = util::getPathTurnSignal(
    status_.current_lanes, status_.pull_over_path.shifted_path, status_.pull_over_path.shift_point,
    planner_data_->self_pose->pose, planner_data_->self_odometry->twist.twist.linear.x,
    planner_data_->parameters, parameters_.pull_over_search_distance);

  if (hazard_info.first.command == HazardLightsCommand::ENABLE) {
    output.turn_signal_info.hazard_signal.command = hazard_info.first.command;
    output.turn_signal_info.signal_distance = hazard_info.second;
  } else {
    output.turn_signal_info.turn_signal.command = turn_info.first.command;
    output.turn_signal_info.signal_distance = turn_info.second;
  }
  return output;
}

PathWithLaneId PullOverModule::planCandidate() const
{
  // Get lane change lanes
  const auto current_lanes = getCurrentLanes();
  const auto pull_over_lanes = getPullOverLanes(current_lanes);

  // Find lane change path
  bool found_valid_path, found_safe_path;
  PullOverPath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(pull_over_lanes, check_distance_, selected_path);
  selected_path.path.header = planner_data_->route_handler->getRouteHeader();

  return selected_path.path;
}

BehaviorModuleOutput PullOverModule::planWaitingApproval()
{
  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(getReferencePath());
  out.path_candidate = std::make_shared<PathWithLaneId>(planCandidate());
  return out;
}

void PullOverModule::setParameters(const PullOverParameters & parameters)
{
  parameters_ = parameters;
}

void PullOverModule::updatePullOverStatus()
{
  const auto & route_handler = planner_data_->route_handler;
  const auto common_parameters = planner_data_->parameters;

  const auto current_lanes = getCurrentLanes();
  status_.current_lanes = current_lanes;

  lanelet::ConstLanelet target_shoulder_lane;

  if (route_handler->getPullOverTarget(
        route_handler->getShoulderLanelets(), &target_shoulder_lane)) {
    route_handler->setPullOverGoalPose(
      target_shoulder_lane, common_parameters.vehicle_width, parameters_.margin_from_boundary);
  } else {
    RCLCPP_ERROR(getLogger(), "failed to get shoulder lane!!!");
  }

  // Get pull_over lanes
  const auto pull_over_lanes = getPullOverLanes(current_lanes);
  status_.pull_over_lanes = pull_over_lanes;

  // Find pull_over path
  bool found_valid_path, found_safe_path;
  PullOverPath selected_path;
  std::tie(found_valid_path, found_safe_path) =
    getSafePath(pull_over_lanes, check_distance_, selected_path);

  // Update status
  status_.is_safe = found_safe_path;
  status_.pull_over_path = selected_path;

  status_.lane_follow_lane_ids = util::getIds(current_lanes);
  status_.pull_over_lane_ids = util::getIds(pull_over_lanes);

  // Generate drivable area
  {
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), current_lanes.begin(), current_lanes.end());
    lanes.insert(lanes.end(), pull_over_lanes.begin(), pull_over_lanes.end());

    const double width = common_parameters.drivable_area_width;
    const double height = common_parameters.drivable_area_height;
    const double resolution = common_parameters.drivable_area_resolution;
    status_.pull_over_path.path.drivable_area = util::generateDrivableArea(
      lanes, *(planner_data_->self_pose), width, height, resolution,
      common_parameters.vehicle_length, *route_handler);
  }

  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.pull_over_lanes, current_pose);
  status_.start_distance = arclength_start.length;

  status_.pull_over_path.path.header = planner_data_->route_handler->getRouteHeader();
}

PathWithLaneId PullOverModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto goal_pose = planner_data_->route_handler->getGoalPose();
  const auto common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return reference_path;
  }

  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length, common_parameters);

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, parameters_.after_pull_over_straight_distance,
    common_parameters.minimum_pull_over_length, parameters_.before_pull_over_straight_distance,
    parameters_.deceleration_interval, goal_pose);

  reference_path.drivable_area = util::generateDrivableArea(
    current_lanes, *planner_data_->self_pose, common_parameters.drivable_area_width,
    common_parameters.drivable_area_height, common_parameters.drivable_area_resolution,
    common_parameters.vehicle_length, *planner_data_->route_handler);

  return reference_path;
}

lanelet::ConstLanelets PullOverModule::getCurrentLanes() const
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

lanelet::ConstLanelets PullOverModule::getPullOverLanes(
  const lanelet::ConstLanelets & current_lanes) const
{
  lanelet::ConstLanelets pull_over_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  lanelet::ConstLanelet target_shoulder_lane;

  if (current_lanes.empty()) {
    return pull_over_lanes;
  }

  // Get shoulder lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(
    current_lanes, planner_data_->self_pose->pose, &current_lane);

  if (route_handler->getPullOverTarget(
        route_handler->getShoulderLanelets(), &target_shoulder_lane)) {
    pull_over_lanes = route_handler->getShoulderLaneletSequence(
      target_shoulder_lane, current_pose, pull_over_lane_length_, pull_over_lane_length_);

  } else {
    pull_over_lanes.clear();
  }

  return pull_over_lanes;
}

std::pair<bool, bool> PullOverModule::getSafePath(
  const lanelet::ConstLanelets & pull_over_lanes, const double check_distance,
  PullOverPath & safe_path) const
{
  std::vector<PullOverPath> valid_paths;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto common_parameters = planner_data_->parameters;

  const auto current_lanes = getCurrentLanes();

  if (!pull_over_lanes.empty()) {
    // find candidate paths
    const auto pull_over_paths = pull_over_utils::getPullOverPaths(
      *route_handler, current_lanes, pull_over_lanes, current_pose, current_twist,
      common_parameters, parameters_);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!pull_over_paths.empty()) {
      const auto & longest_path = pull_over_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.pull_over_length;
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, pull_over_lanes, check_distance_with_path);
    }

    // select valid path
    valid_paths = pull_over_utils::selectValidPaths(
      pull_over_paths, current_lanes, check_lanes, route_handler->getOverallGraph(), current_pose,
      route_handler->isInGoalRouteSection(current_lanes.back()), route_handler->getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }
    // select safe path
    bool found_safe_path = pull_over_utils::selectSafePath(
      valid_paths, current_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters.vehicle_width, parameters_, &safe_path);
    return std::make_pair(true, found_safe_path);
  }
  return std::make_pair(false, false);
}

bool PullOverModule::isLongEnough(const lanelet::ConstLanelets & lanelets) const
{
  PathShifter path_shifter;
  const double maximum_jerk = parameters_.maximum_lateral_jerk;
  const double pull_over_velocity = parameters_.minimum_pull_over_velocity;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto goal_pose = planner_data_->route_handler->getGoalPose();
  const double distance_after_pull_over = parameters_.after_pull_over_straight_distance;
  const double distance_before_pull_over = parameters_.before_pull_over_straight_distance;
  const auto & route_handler = planner_data_->route_handler;

  double distance_to_left_bound =
    util::getDistanceToShoulderBoundary(route_handler->getShoulderLanelets(), current_pose);
  double offset_from_center_line = distance_to_left_bound +
                                   planner_data_->parameters.vehicle_width / 2 +
                                   parameters_.margin_from_boundary;

  // calculate minimum pull over distance at pull over velocity, maximum jerk and side offset
  const double pull_over_distance_min = path_shifter.calcLongitudinalDistFromJerk(
    abs(offset_from_center_line), maximum_jerk, pull_over_velocity);
  const double pull_over_total_distance_min =
    distance_after_pull_over + pull_over_distance_min + distance_before_pull_over;
  const double distance_to_goal = util::getSignedDistance(current_pose, goal_pose, lanelets);

  return distance_to_goal > pull_over_total_distance_min;
}

bool PullOverModule::isSafe() const { return status_.is_safe; }

bool PullOverModule::isNearEndOfLane() const
{
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;
  const double threshold = 5 + common_parameters.minimum_pull_over_length;

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool PullOverModule::isCurrentSpeedLow() const
{
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const double threshold_kmph = 10;
  return util::l2Norm(current_twist.linear) < threshold_kmph * 1000 / 3600;
}

bool PullOverModule::hasFinishedPullOver() const
{
  // check ego car is close enough to goal pose
  const auto current_pose = planner_data_->self_pose->pose;
  const auto goal_pose = planner_data_->route_handler->getGoalPose();
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.pull_over_lanes, current_pose);
  const auto arclength_goal = lanelet::utils::getArcCoordinates(status_.pull_over_lanes, goal_pose);
  const bool car_is_on_goal =
    (arclength_goal.length - arclength_current.length < parameters_.pull_over_finish_judge_buffer)
      ? true
      : false;

  // check ego car is stopping
  const double ego_vel = util::l2Norm(planner_data_->self_odometry->twist.twist.linear);
  const bool car_is_stopping = (ego_vel == 0.0) ? true : false;

  lanelet::Lanelet closest_shoulder_lanelet;

  if (
    lanelet::utils::query::getClosestLanelet(
      planner_data_->route_handler->getShoulderLanelets(), planner_data_->self_pose->pose,
      &closest_shoulder_lanelet) &&
    car_is_on_goal && car_is_stopping) {
    const auto road_lanes = getCurrentLanes();

    // check if goal pose is in shoulder lane and distance is long enough for pull out
    // if (isLongEnough(road_lanes)) {
    //   return true;
    // }
  }

  return false;
}

std::pair<HazardLightsCommand, double> PullOverModule::getHazard(
  const lanelet::ConstLanelets & target_lanes, const Pose & current_pose, const Pose & goal_pose,
  const double & velocity, const double & hazard_on_threshold_dis,
  const double & hazard_on_threshold_vel, const double & base_link2front) const
{
  HazardLightsCommand hazard_signal;
  const double max_distance = std::numeric_limits<double>::max();

  double distance_to_target_pose;   // distance from current pose to target pose on target lanes
  double distance_to_target_point;  // distance from vehicle front to target point on target lanes.
  {
    const auto arc_position_target_pose =
      lanelet::utils::getArcCoordinates(target_lanes, goal_pose);
    const auto arc_position_current_pose =
      lanelet::utils::getArcCoordinates(target_lanes, current_pose);
    distance_to_target_pose = arc_position_target_pose.length - arc_position_current_pose.length;
    distance_to_target_point = distance_to_target_pose - base_link2front;
  }

  if (
    distance_to_target_pose < hazard_on_threshold_dis && abs(velocity) < hazard_on_threshold_vel) {
    hazard_signal.command = HazardLightsCommand::ENABLE;
    return std::make_pair(hazard_signal, distance_to_target_point);
  }

  return std::make_pair(hazard_signal, max_distance);
}

}  // namespace behavior_path_planner
