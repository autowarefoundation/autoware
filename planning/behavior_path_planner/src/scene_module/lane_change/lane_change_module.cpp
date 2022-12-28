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
#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

std::string to_string(const unique_identifier_msgs::msg::UUID & uuid)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid.uuid[i];
  }
  return ss.str();
}

namespace behavior_path_planner
{
using autoware_auto_perception_msgs::msg::ObjectClassification;

LaneChangeModule::LaneChangeModule(
  const std::string & name, rclcpp::Node & node, std::shared_ptr<LaneChangeParameters> parameters)
: SceneModuleInterface{name, node},
  parameters_{std::move(parameters)},
  rtc_interface_left_(&node, "lane_change_left"),
  rtc_interface_right_(&node, "lane_change_right"),
  uuid_left_{generateUUID()},
  uuid_right_{generateUUID()}
{
  steering_factor_interface_ptr_ = std::make_unique<SteeringFactorInterface>(&node, "lane_change");
}

void LaneChangeModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onEntry");
  current_state_ = BT::NodeStatus::SUCCESS;
  current_lane_change_state_ = LaneChangeStates::Normal;
  updateLaneChangeStatus();
}

void LaneChangeModule::onExit()
{
  resetParameters();
  current_state_ = BT::NodeStatus::SUCCESS;
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE onExit");
}

bool LaneChangeModule::isExecutionRequested() const
{
  if (current_state_ == BT::NodeStatus::RUNNING) {
    return true;
  }

  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

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

  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);

  return found_safe_path;
}

BT::NodeStatus LaneChangeModule::updateState()
{
  RCLCPP_DEBUG(getLogger(), "LANE_CHANGE updateState");
  if (!isSafe()) {
    current_state_ = BT::NodeStatus::SUCCESS;
    return current_state_;
  }

  const auto is_within_current_lane = lane_change_utils::isEgoWithinOriginalLane(
    status_.current_lanes, getEgoPose(), planner_data_->parameters);
  if (isAbortState() && !is_within_current_lane) {
    current_state_ = BT::NodeStatus::RUNNING;
    return current_state_;
  }

  if (isAbortConditionSatisfied()) {
    if ((isNearEndOfLane() && isCurrentSpeedLow()) || !is_within_current_lane) {
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
  resetPathCandidate();
  is_activated_ = isActivated();

  PathWithLaneId path = status_.lane_change_path.path;
  if (!isValidPath(path)) {
    status_.is_safe = false;
    return BehaviorModuleOutput{};
  }

  if ((is_abort_condition_satisfied_ && isNearEndOfLane() && isCurrentSpeedLow())) {
    const auto stop_point = util::insertStopPoint(0.1, &path);
  }

  if (isAbortState()) {
    resetPathIfAbort();
    if (is_activated_) {
      path = abort_path_->path;
    }
  }

  generateExtendedDrivableArea(path);

  prev_approved_path_ = path;

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(path);
  updateOutputTurnSignal(output);

  updateSteeringFactorPtr(output);

  return output;
}

void LaneChangeModule::resetPathIfAbort()
{
  if (!is_abort_approval_requested_) {
    const auto lateral_shift = lane_change_utils::getLateralShift(*abort_path_);
    if (lateral_shift > 0.0) {
      removePreviousRTCStatusRight();
      uuid_right_ = generateUUID();
    } else if (lateral_shift < 0.0) {
      removePreviousRTCStatusLeft();
      uuid_left_ = generateUUID();
    }
    RCLCPP_DEBUG(getLogger(), "[abort] uuid is reset to request abort approval.");
    is_abort_approval_requested_ = true;
    is_abort_path_approved_ = false;
    return;
  }

  if (isActivated()) {
    RCLCPP_DEBUG(getLogger(), "[abort] isActivated() is true. set is_abort_path_approved to true.");
    is_abort_path_approved_ = true;
    clearWaitingApproval();
  } else {
    RCLCPP_DEBUG(getLogger(), "[abort] isActivated() is False.");
    is_abort_path_approved_ = false;
    waitApproval();
  }
}

CandidateOutput LaneChangeModule::planCandidate() const
{
  CandidateOutput output;

  LaneChangePath selected_path;
  // Get lane change lanes
  const auto current_lanes = util::getCurrentLanes(planner_data_);
  const auto lane_change_lanes = getLaneChangeLanes(current_lanes, lane_change_lane_length_);

  [[maybe_unused]] const auto [found_valid_path, found_safe_path] =
    getSafePath(lane_change_lanes, check_distance_, selected_path);
  selected_path.path.header = planner_data_->route_handler->getRouteHeader();

  if (isAbortState()) {
    selected_path = *abort_path_;
  }

  if (selected_path.path.points.empty()) {
    return output;
  }

  output.path_candidate = selected_path.path;
  output.lateral_shift = lane_change_utils::getLateralShift(selected_path);
  output.start_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, getEgoPose().position, selected_path.shift_line.start.position);
  output.finish_distance_to_path_change = motion_utils::calcSignedArcLength(
    selected_path.path.points, getEgoPose().position, selected_path.shift_line.end.position);

  updateSteeringFactorPtr(output, selected_path);
  return output;
}

BehaviorModuleOutput LaneChangeModule::planWaitingApproval()
{
  const auto is_within_current_lane = lane_change_utils::isEgoWithinOriginalLane(
    status_.current_lanes, getEgoPose(), planner_data_->parameters);

  if (is_within_current_lane) {
    prev_approved_path_ = getReferencePath();
  }

  BehaviorModuleOutput out;
  out.path = std::make_shared<PathWithLaneId>(prev_approved_path_);

  const auto candidate = planCandidate();
  path_candidate_ = std::make_shared<PathWithLaneId>(candidate.path_candidate);
  updateRTCStatus(candidate);
  waitApproval();
  is_abort_path_approved_ = false;
  return out;
}

void LaneChangeModule::updateLaneChangeStatus()
{
  status_.current_lanes = util::getCurrentLanes(planner_data_);
  status_.lane_change_lanes = getLaneChangeLanes(status_.current_lanes, lane_change_lane_length_);

  // Find lane change path
  LaneChangePath selected_path;
  const auto [found_valid_path, found_safe_path] =
    getSafePath(status_.lane_change_lanes, check_distance_, selected_path);

  // Update status
  status_.is_safe = found_safe_path;
  status_.lane_change_path = selected_path;
  status_.lane_follow_lane_ids = util::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = util::getIds(status_.lane_change_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

PathWithLaneId LaneChangeModule::getReferencePath() const
{
  PathWithLaneId reference_path;

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = getRouteHeader();

  const auto current_lanes = util::getCurrentLanes(planner_data_);

  if (current_lanes.empty()) {
    return reference_path;
  }

  if (reference_path.points.empty()) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters);
  }

  const int num_lane_change =
    std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
  double optional_lengths{0.0};
  const auto isInIntersection = util::checkLaneIsInIntersection(
    *route_handler, reference_path, current_lanes, common_parameters, num_lane_change,
    optional_lengths);
  if (isInIntersection) {
    reference_path = util::getCenterLinePath(
      *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
      common_parameters.forward_path_length, common_parameters, optional_lengths);
  }

  const double lane_change_buffer =
    util::calcLaneChangeBuffer(common_parameters, num_lane_change, optional_lengths);

  reference_path = util::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, parameters_->lane_change_prepare_duration,
    lane_change_buffer);

  const auto drivable_lanes = util::generateDrivableLanes(current_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);
  util::generateDrivableArea(
    reference_path, expanded_lanes, common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets LaneChangeModule::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, const double lane_change_lane_length) const
{
  lanelet::ConstLanelets lane_change_lanes;
  const auto & route_handler = planner_data_->route_handler;
  const auto minimum_lane_change_length = planner_data_->parameters.minimum_lane_change_length;
  const auto lane_change_prepare_duration = parameters_->lane_change_prepare_duration;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (current_lanes.empty()) {
    return lane_change_lanes;
  }

  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, current_pose, &current_lane);
  const double lane_change_prepare_length =
    std::max(current_twist.linear.x * lane_change_prepare_duration, minimum_lane_change_length);
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
  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & common_parameters = planner_data_->parameters;

  const auto current_lanes = util::getCurrentLanes(planner_data_);

  if (!lane_change_lanes.empty()) {
    // find candidate paths
    const auto lane_change_paths = lane_change_utils::getLaneChangePaths(
      *route_handler, current_lanes, lane_change_lanes, current_pose, current_twist,
      common_parameters, *parameters_);

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
    const LaneChangePaths valid_paths = lane_change_utils::selectValidPaths(
      lane_change_paths, current_lanes, check_lanes, *route_handler->getOverallGraphPtr(),
      current_pose, route_handler->isInGoalRouteSection(current_lanes.back()),
      route_handler->getGoalPose());

    if (valid_paths.empty()) {
      return std::make_pair(false, false);
    }
    debug_valid_path_ = valid_paths;

    // select safe path
    const bool found_safe_path = lane_change_utils::selectSafePath(
      valid_paths, current_lanes, check_lanes, planner_data_->dynamic_object, current_pose,
      current_twist, common_parameters, *parameters_, &safe_path, object_debug_);

    if (parameters_->publish_debug_marker) {
      setObjectDebugVisualization();
    } else {
      debug_marker_.markers.clear();
    }

    return std::make_pair(true, found_safe_path);
  }

  return std::make_pair(false, false);
}

bool LaneChangeModule::isSafe() const { return status_.is_safe; }

bool LaneChangeModule::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;

  // check lane departure
  const auto drivable_lanes = lane_change_utils::generateDrivableLanes(
    *route_handler, util::extendLanes(route_handler, status_.current_lanes),
    util::extendLanes(route_handler, status_.lane_change_lanes));
  const auto expanded_lanes = util::expandLanelets(
    drivable_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);
  const auto lanelets = util::transformToLanelets(expanded_lanes);

  // check path points are in any lanelets
  for (const auto & point : path.points) {
    bool is_in_lanelet = false;
    for (const auto & lanelet : lanelets) {
      if (lanelet::utils::isInLanelet(point.point.pose, lanelet)) {
        is_in_lanelet = true;
        break;
      }
    }
    if (!is_in_lanelet) {
      RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *clock_, 1000, "path is out of lanes");
      return false;
    }
  }

  // check relative angle
  if (!util::checkPathRelativeAngle(path, M_PI)) {
    RCLCPP_WARN_STREAM_THROTTLE(getLogger(), *clock_, 1000, "path relative angle is invalid");
    return false;
  }

  return true;
}

bool LaneChangeModule::isNearEndOfLane() const
{
  const auto & current_pose = getEgoPose();
  const double threshold = util::calcTotalLaneChangeDistance(planner_data_->parameters);

  return std::max(0.0, util::getDistanceToEndOfLane(current_pose, status_.current_lanes)) <
         threshold;
}

bool LaneChangeModule::isCurrentSpeedLow() const
{
  constexpr double threshold_ms = 10.0 * 1000 / 3600;
  return util::l2Norm(getEgoTwist().linear) < threshold_ms;
}

bool LaneChangeModule::isAbortConditionSatisfied()
{
  is_abort_condition_satisfied_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;

  if (!parameters_->enable_cancel_lane_change) {
    return false;
  }

  if (!is_activated_) {
    return false;
  }

  Pose ego_pose_before_collision;
  const auto is_path_safe = isApprovedPathSafe(ego_pose_before_collision);

  if (!is_path_safe) {
    const auto & common_parameters = planner_data_->parameters;
    const bool is_within_original_lane = lane_change_utils::isEgoWithinOriginalLane(
      status_.current_lanes, getEgoPose(), common_parameters);

    if (is_within_original_lane) {
      current_lane_change_state_ = LaneChangeStates::Cancel;
      return true;
    }

    // check abort enable flag
    RCLCPP_WARN_STREAM_THROTTLE(
      getLogger(), *clock_, 1000,
      "DANGER!!! Path is not safe anymore, but it is too late to CANCEL! Please be cautious");

    if (!parameters_->enable_abort_lane_change) {
      current_lane_change_state_ = LaneChangeStates::Stop;
      return false;
    }

    const auto found_abort_path = lane_change_utils::getAbortPaths(
      planner_data_, status_.lane_change_path, ego_pose_before_collision, common_parameters,
      *parameters_);

    if (!found_abort_path && !is_abort_path_approved_) {
      current_lane_change_state_ = LaneChangeStates::Stop;
      return true;
    }

    current_lane_change_state_ = LaneChangeStates::Abort;

    if (!is_abort_path_approved_) {
      abort_path_ = std::make_shared<LaneChangePath>(*found_abort_path);
    }

    return true;
  }

  return false;
}

bool LaneChangeModule::isAbortState() const
{
  if (!parameters_->enable_abort_lane_change) {
    return false;
  }

  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    return false;
  }

  if (!abort_path_) {
    return false;
  }

  RCLCPP_WARN_STREAM_THROTTLE(
    getLogger(), *clock_, 1000,
    "DANGER!!! Lane change transition to ABORT state, return path will be computed!");
  return true;
}

bool LaneChangeModule::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto arclength_current =
    lanelet::utils::getArcCoordinates(status_.lane_change_lanes, current_pose);
  const double travel_distance = arclength_current.length - status_.start_distance;
  const double finish_distance = status_.lane_change_path.preparation_length +
                                 status_.lane_change_path.lane_change_length +
                                 parameters_->lane_change_finish_judge_buffer;
  return travel_distance > finish_distance;
}

void LaneChangeModule::setObjectDebugVisualization() const
{
  using marker_utils::lane_change_markers::showAllValidLaneChangePath;
  using marker_utils::lane_change_markers::showLerpedPose;
  using marker_utils::lane_change_markers::showObjectInfo;
  using marker_utils::lane_change_markers::showPolygon;
  using marker_utils::lane_change_markers::showPolygonPose;

  debug_marker_.markers.clear();
  const auto add = [this](const MarkerArray & added) {
    tier4_autoware_utils::appendMarkerArray(added, &debug_marker_);
  };

  add(showObjectInfo(object_debug_, "object_debug_info"));
  add(showLerpedPose(object_debug_, "lerp_pose_before_true"));
  add(showPolygonPose(object_debug_, "expected_pose"));
  add(showPolygon(object_debug_, "lerped_polygon"));
  add(showAllValidLaneChangePath(debug_valid_path_, "lane_change_valid_paths"));
}

std::shared_ptr<LaneChangeDebugMsgArray> LaneChangeModule::get_debug_msg_array() const
{
  LaneChangeDebugMsgArray debug_msg_array;
  debug_msg_array.lane_change_info.reserve(object_debug_.size());
  for (const auto & [uuid, debug_data] : object_debug_) {
    LaneChangeDebugMsg debug_msg;
    debug_msg.object_id = uuid;
    debug_msg.allow_lane_change = debug_data.allow_lane_change;
    debug_msg.is_front = debug_data.is_front;
    debug_msg.relative_distance = debug_data.relative_to_ego;
    debug_msg.failed_reason = debug_data.failed_reason;
    debug_msg.velocity = util::l2Norm(debug_data.object_twist.linear);
    debug_msg_array.lane_change_info.push_back(debug_msg);
  }
  lane_change_debug_msg_array_ = debug_msg_array;

  lane_change_debug_msg_array_.header.stamp = clock_->now();
  return std::make_shared<LaneChangeDebugMsgArray>(lane_change_debug_msg_array_);
}

void LaneChangeModule::updateSteeringFactorPtr(const BehaviorModuleOutput & output)
{
  const auto turn_signal_info = output.turn_signal_info;
  const auto current_pose = getEgoPose();
  const double start_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status_.lane_change_path.shift_line.start.position);
  const double finish_distance = motion_utils::calcSignedArcLength(
    output.path->points, current_pose.position, status_.lane_change_path.shift_line.end.position);

  const uint16_t steering_factor_direction =
    std::invoke([this, &start_distance, &finish_distance, &turn_signal_info]() {
      if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_LEFT) {
        waitApprovalLeft(start_distance, finish_distance);
        return SteeringFactor::LEFT;
      }
      if (turn_signal_info.turn_signal.command == TurnIndicatorsCommand::ENABLE_RIGHT) {
        waitApprovalRight(start_distance, finish_distance);
        return SteeringFactor::RIGHT;
      }
      return SteeringFactor::UNKNOWN;
    });

  // TODO(tkhmy) add handle status TRYING
  steering_factor_interface_ptr_->updateSteeringFactor(
    {status_.lane_change_path.shift_line.start, status_.lane_change_path.shift_line.end},
    {start_distance, finish_distance}, SteeringFactor::LANE_CHANGE, steering_factor_direction,
    SteeringFactor::TURNING, "");
}

void LaneChangeModule::updateSteeringFactorPtr(
  const CandidateOutput & output, const LaneChangePath & selected_path) const
{
  const uint16_t steering_factor_direction = std::invoke([&output]() {
    if (output.lateral_shift > 0.0) {
      return SteeringFactor::LEFT;
    }
    return SteeringFactor::RIGHT;
  });

  steering_factor_interface_ptr_->updateSteeringFactor(
    {selected_path.shift_line.start, selected_path.shift_line.end},
    {output.start_distance_to_path_change, output.finish_distance_to_path_change},
    SteeringFactor::LANE_CHANGE, steering_factor_direction, SteeringFactor::APPROACHING, "");
}
Pose LaneChangeModule::getEgoPose() const { return planner_data_->self_pose->pose; }
Twist LaneChangeModule::getEgoTwist() const { return planner_data_->self_odometry->twist.twist; }
std_msgs::msg::Header LaneChangeModule::getRouteHeader() const
{
  return planner_data_->route_handler->getRouteHeader();
}
void LaneChangeModule::generateExtendedDrivableArea(PathWithLaneId & path)
{
  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;
  const auto drivable_lanes = lane_change_utils::generateDrivableLanes(
    *route_handler, status_.current_lanes, status_.lane_change_lanes);
  const auto shorten_lanes = util::cutOverlappedLanes(path, drivable_lanes);
  const auto expanded_lanes = util::expandLanelets(
    shorten_lanes, parameters_->drivable_area_left_bound_offset,
    parameters_->drivable_area_right_bound_offset);
  util::generateDrivableArea(path, expanded_lanes, common_parameters.vehicle_length, planner_data_);
}

bool LaneChangeModule::isApprovedPathSafe(Pose & ego_pose_before_collision) const
{
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();
  const auto & dynamic_objects = planner_data_->dynamic_object;
  const auto & current_lanes = status_.current_lanes;
  const auto & common_parameters = planner_data_->parameters;
  const auto & route_handler = planner_data_->route_handler;
  const auto path = status_.lane_change_path;

  constexpr double check_distance = 100.0;
  // get lanes used for detection
  const double check_distance_with_path =
    check_distance + path.preparation_length + path.lane_change_length;
  const auto check_lanes = route_handler->getCheckTargetLanesFromPath(
    path.path, status_.lane_change_lanes, check_distance_with_path);

  std::unordered_map<std::string, CollisionCheckDebug> debug_data;

  const size_t current_seg_idx = motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    path.path.points, current_pose, common_parameters.ego_nearest_dist_threshold,
    common_parameters.ego_nearest_yaw_threshold);
  return lane_change_utils::isLaneChangePathSafe(
    path.path, current_lanes, check_lanes, dynamic_objects, current_pose, current_seg_idx,
    current_twist, common_parameters, *parameters_,
    common_parameters.expected_front_deceleration_for_abort,
    common_parameters.expected_rear_deceleration_for_abort, ego_pose_before_collision, debug_data,
    false, status_.lane_change_path.acceleration);
}

void LaneChangeModule::updateOutputTurnSignal(BehaviorModuleOutput & output)
{
  const auto turn_signal_info = util::getPathTurnSignal(
    status_.current_lanes, status_.lane_change_path.shifted_path,
    status_.lane_change_path.shift_line, getEgoPose(), getEgoTwist().linear.x,
    planner_data_->parameters);
  output.turn_signal_info.turn_signal.command = turn_signal_info.first.command;

  lane_change_utils::get_turn_signal_info(status_.lane_change_path, &output.turn_signal_info);
}

void LaneChangeModule::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;

  clearWaitingApproval();
  removeRTCStatus();
  steering_factor_interface_ptr_->clearSteeringFactors();
  object_debug_.clear();
  debug_marker_.markers.clear();
  resetPathCandidate();
}

void LaneChangeModule::acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const
{
  if (visitor) {
    visitor->visitLaneChangeModule(this);
  }
}

void SceneModuleVisitor::visitLaneChangeModule(const LaneChangeModule * module) const
{
  lane_change_visitor_ = module->get_debug_msg_array();
}
}  // namespace behavior_path_planner
