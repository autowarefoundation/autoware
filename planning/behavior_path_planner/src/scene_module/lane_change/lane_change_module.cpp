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

#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/lane_change/util.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using route_handler::LaneChangeDirection;

LaneChangeModule::LaneChangeModule(
  const std::string & name, rclcpp::Node & node, const LaneChangeParameters & parameters)
: SceneModuleInterface{name, node},
  parameters_{parameters},
  rtc_interface_left_(&node, "lane_change_left"),
  rtc_interface_right_(&node, "lane_change_right"),
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
{
}

BehaviorModuleOutput LaneChangeModule::run()
{
  RCLCPP_DEBUG(getLogger(), "Was waiting approval, and now approved. Do plan().");
  current_state_ = BT::NodeStatus::RUNNING;
  is_activated_ = isActivated();
  const auto output = plan();
  const auto turn_signal_info = output.turn_signal_info;
  if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
    waitApprovalLeft(turn_signal_info.signal_distance);
  } else if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
    waitApprovalRight(turn_signal_info.signal_distance);
  }
  return output;
}

void LaneChangeModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;
  updateLaneChangeStatus();
  // Get arclength to start lane change
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  status_.start_distance = arclength_start.length;
}

void LaneChangeModule::onExit()
{
  clearWaitingApproval();
  removeRTCStatus();
  current_state_ = BT::NodeStatus::IDLE;
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onExit");
}

bool LaneChangeModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // Get lane change lanes
  const auto current_lanes = getCurrentLanes();
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_valid_path;
}

bool LaneChangeModule::isExecutionReady() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  // Get lane change lanes
  const auto current_lanes = getCurrentLanes();
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_safe_path;
}

BT::NodeStatus LaneChangeModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE updateState");
  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      current_state_ = BT::NodeStatus::RUNNING;
      return current_state_;
    }
    current_state_ = BT::NodeStatus::FAILURE;
    return current_state_;
  }

  if (hasFinishedLaneChange()) {
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }
  current_state_ = BT::NodeStatus::RUNNING;
  return current_state_;
}

BehaviorModuleOutput LaneChangeModule::plan()
{
  constexpr double RESAMPLE_INTERVAL = 1.0;
  auto path = util::resamplePathWithSpline(status_.lane_change_path.path, RESAMPLE_INTERVAL);
  // Generate drivable area
  {
    const auto common_parameters = planner_data_->parameters;
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), status_.current_lanes.begin(), status_.current_lanes.end());
    lanes.insert(lanes.end(), status_.lane_change_lanes.begin(), status_.lane_change_lanes.end());

    const double resolution = common_parameters.drivable_area_resolution;
    path.drivable_area = util::generateDrivableArea(
      path, lanes, resolution, common_parameters.vehicle_length, planner_data_);
  }

  if (isAbortConditionSatisfied()) {
    if (isNearEndOfLane() && isCurrentSpeedLow()) {
      const auto stop_point = util::insertStopPoint(0.1, &path);
    }
  }

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(path);
  const auto turn_signal_info = util::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_point, planner_data_->self_pose->pose,
    planner_data_->self_odometry->twist.twist.linear.x, planner_data_->parameters);
  output.turn_signal_info.turn_signal.command = turn_signal_info.first.command;
  output.turn_signal_info.signal_distance = turn_signal_info.second;
  return output;
}

CandidateOutput LaneChangeModule::planCandidate() const
{
  CandidateOutput output;

  // Get lane change lanes
  const auto current_lanes = getCurrentLanes();
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  [[maybe_unused]] const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);
  selected_path.path.header = planner_data_->route_handler->getRouteHeader();

  if (selected_path.path.points.empty()) {
    return output;
  }

  const auto start_idx = selected_path.shift_point.start_idx;
  const auto end_idx = selected_path.shift_point.end_idx;

  output.path_candidate = selected_path.path;
  output.lateral_shift = selected_path.shifted_path.shift_length.at(end_idx) -
                         selected_path.shifted_path.shift_length.at(start_idx);
  output.distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, planner_data_->self_pose->pose.position,
    selected_path.shift_point.start.position);

  return output;
}

BehaviorModuleOutput LaneChangeModule::planWaitingApproval()
{
  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(getReferencePath());
  const auto candidate = planCandidate();
  out.path_candidate = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  updateRTCStatus(candidate);
  waitApproval();
  return out;
}

void LaneChangeModule::setParameters(const LaneChangeParameters & parameters)
{
  parameters_ = parameters;
}

void LaneChangeModule::updateLaneChangeStatus()
{
  const auto current_lanes = getCurrentLanes();
  status_.current_lanes = current_lanes;

  // Get lane change lanes
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);
  status_.lane_change_lanes = lane_change_lanes;

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_change_path = selected_path;
  status_.lane_follow_lane_ids = util::getIds(current_lanes);
  status_.lane_change_lane_ids = util::getIds(lane_change_lanes);

  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  status_.start_distance = arclength_start.length;

  status_.lane_change_path.path.header = planner_data_->route_handler->getRouteHeader();
}

PathWithLaneId LaneChangeModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return reference_path;
  }

  if (reference_path.points.empty()) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters);
  }

  double optional_lengths{0.0};
  const auto isInIntersection = util::checkLaneIsInIntersection(
    *route_handler, reference_path, current_lanes, common_parameters, optional_lengths);
  if (isInIntersection) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters, optional_lengths);
  }
  const double buffer =
    common_parameters.backward_length_buffer_for_end_of_lane;  // buffer for min_lane_change_length
  const int num_lane_change =
    std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
  const double lane_change_buffer =
    num_lane_change * (common_parameters.minimum_lane_change_length + buffer);

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, parameters_.lane_change_prepare_duration,
    lane_change_buffer);

  reference_path.drivable_area = util::generateDrivableArea(
    reference_path, current_lanes, common_parameters.drivable_area_resolution,
    common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets LaneChangeModule::getCurrentLanes() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 5000, "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe) what should be returned?
  }

  // For current_lanes with desired length
  return route_handler->getLaneletSequence(
    current_lane, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length);
}

lanelet::ConstLanelets LaneChangeModule::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(
    current_lanes, planner_data_->self_pose->pose, &current_lane);
  const double lane_change_prepare_length = std::max(
    current_twist.linear.x * parameters_.lane_change_prepare_duration,
    planner_data_->parameters.minimum_lane_change_length);
  lanelet::ConstLanelets current_check_lanes =
    route_handler->getLaneletSequence(current_lane, current_pose, 0.0, lane_change_prepare_length);
  lanelet::ConstLanelet lane_change_lane;
  if (route_handler->getLaneChangeTarget(current_check_lanes, &lane_change_lane)) {
    lane_change_lanes = route_handler->getLaneletSequence(
      lane_change_lane, current_pose, lane_change_lane_length, lane_change_lane_length);
  } else {
    lane_change_lanes.clear();
  }

  return lane_change_lanes;
}

std::pair<bool, bool> LaneChangeModule::getSafePath(
  const lanelet::ConstLanelets & lane_change_lanes, const double check_distance,
  LaneChangePath & safe_path) const
{
  std::vector<LaneChangePath> valid_paths;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto common_parameters = planner_data_->parameters;

  const auto current_lanes = getCurrentLanes();

  if (!lane_change_lanes.empty()) {
    // find candidate paths
    const auto lane_change_paths = lane_change_utils::getLaneChangePaths(
      *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
      common_parameters, parameters_);

    // get lanes used for detection
    lanelet::ConstLanelets check_lanes;
    if (!lane_change_paths.empty()) {
      const auto & longest_path = lane_change_paths.front();
      // we want to see check_distance [m] behind vehicle so add lane changing length
      const double check_distance_with_path =
        check_distance + longest_path.preparation_length + longest_path.lane_change_length;
      check_lanes = route_handler->getCheckTargetLanesFromPath(
        longest_path.path, lane_change_lanes, check_distance_with_path);
    }

    // select valid path
    valid_paths = lane_change_utils::selectValidPaths(
      lane_change_paths, current_lanes, check_lanes, *route_handler->getOverallGraphPtr(),
      current_pose, route_handler->isInGoalRouteSection(current_lanes.back()),
      route_handler->getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }

    // select safe path
    bool found_safe_path = lane_change_utils::selectSafePath(
      valid_paths, current_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters, parameters_, &safe_path);
    return std::make_pair(true, found_safe_path);
  }

  return std::make_pair(false, false);
}

bool LaneChangeModule::isSafe() const { return status_.is_safe; }

bool LaneChangeModule::isNearEndOfLane() const
{
  const auto current_pose = planner_data_->self_pose->pose;
  const auto common_parameters = planner_data_->parameters;
  const double threshold = 5 + common_parameters.minimum_lane_change_length;

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool LaneChangeModule::isCurrentSpeedLow() const
{
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const double threshold_kmph = 10;
  return util::l2Norm(current_twist.linear) < threshold_kmph * 1000 / 3600;
}

bool LaneChangeModule::isAbortConditionSatisfied() const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto current_twist = planner_data_->self_odometry->twist.twist;
  const auto objects = planner_data_->dynamic_object;
  const auto common_parameters = planner_data_->parameters;

  const auto current_lanes = status_.current_lanes;

  // check abort enable flag
  if (!parameters_.enable_abort_lane_change) {
    return false;
  }

  if (!is_activated_) {
    return false;
  }

  // find closest lanelet in original lane
  lanelet::ConstLanelet closest_lanelet{};
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};
  if (!lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &closest_lanelet)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), clock, 1000,
      "Failed to find closest lane! Lane change aborting function is not working!");
    return false;
  }

  // check if lane change path is still safe
  bool is_path_safe{false};
  {
    constexpr double check_distance = 100.0;
    // get lanes used for detection
    const auto & path = status_.lane_change_path;
    const double check_distance_with_path =
      check_distance + path.preparation_length + path.lane_change_length;
    const auto check_lanes = route_handler->getCheckTargetLanesFromPath(
      path.path, status_.lane_change_lanes, check_distance_with_path);

    is_path_safe = lane_change_utils::isLaneChangePathSafe(
      path.path, current_lanes, check_lanes, objects, current_pose, current_twist,
      common_parameters, parameters_, false, status_.lane_change_path.acceleration);
  }

  // check vehicle velocity thresh
  const bool is_velocity_low =
    util::l2Norm(current_twist.linear) < parameters_.abort_lane_change_velocity_thresh;

  // check if vehicle is within lane
  bool is_within_original_lane = false;
  {
    const auto lane_length = lanelet::utils::getLaneletLength2d(current_lanes);
    const auto lane_poly = lanelet::utils::getPolygonFromArcLength(current_lanes, 0, lane_length);
    const auto vehicle_poly = util::getVehiclePolygon(
      current_pose, common_parameters.vehicle_width, common_parameters.base_link2front);
    is_within_original_lane = boost::geometry::within(
      lanelet::utils::to2D(vehicle_poly).basicPolygon(),
      lanelet::utils::to2D(lane_poly).basicPolygon());
  }

  // check distance from original lane's centerline
  bool is_distance_small = false;
  {
    const auto centerline2d = lanelet::utils::to2D(closest_lanelet.centerline()).basicLineString();
    lanelet::BasicPoint2d vehicle_pose2d(current_pose.position.x, current_pose.position.y);
    const double distance = lanelet::geometry::distance2d(centerline2d, vehicle_pose2d);
    is_distance_small = distance < parameters_.abort_lane_change_distance_thresh;
  }

  // check angle thresh from original lane
  bool is_angle_diff_small = false;
  {
    const double lane_angle =
      lanelet::utils::getLaneletAngle(closest_lanelet, current_pose.position);
    const double vehicle_yaw = tf2::getYaw(current_pose.orientation);
    const double yaw_diff = tier4_autoware_utils::normalizeRadian(lane_angle - vehicle_yaw);
    is_angle_diff_small = std::abs(yaw_diff) < parameters_.abort_lane_change_angle_thresh;
  }

  // abort only if velocity is low or vehicle pose is close enough
  if (!is_path_safe) {
    if (is_velocity_low && is_within_original_lane) {
      return true;
    }
    if (is_distance_small && is_angle_diff_small) {
      return true;
    }
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger(), clock, 1000,
      "DANGER!!! Path is not safe anymore, but it is too late to abort! Please be cautious");
  }

  return false;
}

bool LaneChangeModule::hasFinishedLaneChange() const
{
  const auto current_pose = planner_data_->self_pose->pose;
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  const double travel_distance = arclength_current.length - status_.start_distance;
  const double finish_distance = status_.lane_change_path.preparation_length +
                                 status_.lane_change_path.lane_change_length +
                                 parameters_.lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

}  // namespace behavior_path_planner
