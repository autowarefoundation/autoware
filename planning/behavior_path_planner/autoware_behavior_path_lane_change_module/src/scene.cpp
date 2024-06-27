// Copyright 2023 TIER IV, Inc.
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

#include "autoware/behavior_path_lane_change_module/scene.hpp"

#include "autoware/behavior_path_lane_change_module/utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/drivable_area_expansion/static_drivable_area.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/objects_filtering.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/safety_check.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/traffic_light_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/universe_utils/geometry/boost_polygon_utils.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::motion_utils::calcSignedArcLength;
using utils::lane_change::calcMinimumLaneChangeLength;
using utils::lane_change::createLanesPolygon;
using utils::path_safety_checker::isPolygonOverlapLanelet;
using utils::traffic_light::getDistanceToNextTrafficLight;

NormalLaneChange::NormalLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: LaneChangeBase(parameters, type, direction)
{
  stop_watch_.tic(getModuleTypeStr());
  stop_watch_.tic("stop_time");
}

void NormalLaneChange::updateLaneChangeStatus()
{
  updateStopTime();
  const auto [found_valid_path, found_safe_path] = getSafePath(status_.lane_change_path);

  // Update status
  status_.current_lanes = status_.lane_change_path.info.current_lanes;
  status_.target_lanes = status_.lane_change_path.info.target_lanes;
  status_.is_valid_path = found_valid_path;
  status_.is_safe = found_safe_path;
  status_.lane_follow_lane_ids = utils::getIds(status_.current_lanes);
  status_.lane_change_lane_ids = utils::getIds(status_.target_lanes);

  const auto arclength_start =
    lanelet::utils::getArcCoordinates(status_.target_lanes, getEgoPose());
  status_.start_distance = arclength_start.length;
  status_.lane_change_path.path.header = getRouteHeader();
}

std::pair<bool, bool> NormalLaneChange::getSafePath(LaneChangePath & safe_path) const
{
  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return {false, false};
  }

  const auto target_lanes = getLaneChangeLanes(current_lanes, direction_);

  if (target_lanes.empty()) {
    return {false, false};
  }

  // find candidate paths
  LaneChangePaths valid_paths{};
  const bool is_stuck = isVehicleStuck(current_lanes);
  bool found_safe_path = getLaneChangePaths(
    current_lanes, target_lanes, direction_, &valid_paths, lane_change_parameters_->rss_params,
    is_stuck);
  // if no safe path is found and ego is stuck, try to find a path with a small margin
  if (!found_safe_path && is_stuck) {
    found_safe_path = getLaneChangePaths(
      current_lanes, target_lanes, direction_, &valid_paths,
      lane_change_parameters_->rss_params_for_stuck, is_stuck);
  }

  lane_change_debug_.valid_paths = valid_paths;

  if (valid_paths.empty()) {
    safe_path.info.current_lanes = current_lanes;
    safe_path.info.target_lanes = target_lanes;
    return {false, false};
  }

  if (found_safe_path) {
    safe_path = valid_paths.back();
  } else {
    // force candidate
    safe_path = valid_paths.front();
  }

  return {true, found_safe_path};
}

bool NormalLaneChange::isLaneChangeRequired()
{
  status_.current_lanes = getCurrentLanes();

  if (status_.current_lanes.empty()) {
    return false;
  }

  status_.target_lanes = getLaneChangeLanes(status_.current_lanes, direction_);

  return !status_.target_lanes.empty();
}

bool NormalLaneChange::isStoppedAtRedTrafficLight() const
{
  return utils::traffic_light::isStoppedAtRedTrafficLightWithinDistance(
    status_.current_lanes, status_.lane_change_path.path, planner_data_,
    status_.lane_change_path.info.length.sum());
}

TurnSignalInfo NormalLaneChange::get_current_turn_signal_info()
{
  const auto original_turn_signal_info = prev_module_output_.turn_signal_info;

  const auto & current_lanes = getLaneChangeStatus().current_lanes;
  const auto is_valid = getLaneChangeStatus().is_valid_path;
  const auto & lane_change_path = getLaneChangeStatus().lane_change_path;
  const auto & lane_change_param = getLaneChangeParam();

  if (getModuleType() != LaneChangeModuleType::NORMAL || current_lanes.empty() || !is_valid) {
    return original_turn_signal_info;
  }

  // check direction
  TurnSignalInfo current_turn_signal_info;
  const auto & current_pose = getEgoPose();
  const auto direction = getDirection();
  if (direction == Direction::LEFT) {
    current_turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
  } else if (direction == Direction::RIGHT) {
    current_turn_signal_info.turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
  }

  const auto & path = prev_module_output_.path;
  if (path.points.empty()) {
    current_turn_signal_info.desired_start_point = current_pose;
    current_turn_signal_info.required_start_point = current_pose;
    current_turn_signal_info.desired_end_point = lane_change_path.info.lane_changing_end;
    current_turn_signal_info.required_end_point = lane_change_path.info.lane_changing_end;
    return current_turn_signal_info;
  }

  const auto min_length_for_turn_signal_activation =
    lane_change_param.min_length_for_turn_signal_activation;
  const auto & route_handler = getRouteHandler();
  const auto & common_parameter = getCommonParam();
  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const double next_lane_change_buffer =
    utils::lane_change::calcMinimumLaneChangeLength(lane_change_param, shift_intervals);
  const double nearest_dist_threshold = common_parameter.ego_nearest_dist_threshold;
  const double nearest_yaw_threshold = common_parameter.ego_nearest_yaw_threshold;
  const double base_to_front = common_parameter.base_link2front;

  const double buffer =
    next_lane_change_buffer + min_length_for_turn_signal_activation + base_to_front;
  const double path_length = autoware::motion_utils::calcArcLength(path.points);
  const size_t current_nearest_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      path.points, current_pose, nearest_dist_threshold, nearest_yaw_threshold);
  const double dist_to_terminal = utils::getDistanceToEndOfLane(current_pose, current_lanes);
  const auto start_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    path.points, 0, std::max(path_length - buffer, 0.0));
  if (dist_to_terminal - base_to_front < buffer && start_pose) {
    // modify turn signal
    current_turn_signal_info.desired_start_point = *start_pose;
    current_turn_signal_info.desired_end_point = lane_change_path.info.lane_changing_end;
    current_turn_signal_info.required_start_point = current_turn_signal_info.desired_start_point;
    current_turn_signal_info.required_end_point = current_turn_signal_info.desired_end_point;

    const auto & original_command = original_turn_signal_info.turn_signal.command;
    if (
      original_command == TurnIndicatorsCommand::DISABLE ||
      original_command == TurnIndicatorsCommand::NO_COMMAND) {
      return current_turn_signal_info;
    }

    // check the priority of turn signals
    return getTurnSignalDecider().overwrite_turn_signal(
      path, current_pose, current_nearest_seg_idx, original_turn_signal_info,
      current_turn_signal_info, nearest_dist_threshold, nearest_yaw_threshold);
  }

  // not in the vicinity of the end of the path. return original
  return original_turn_signal_info;
}

LaneChangePath NormalLaneChange::getLaneChangePath() const
{
  return status_.lane_change_path;
}

BehaviorModuleOutput NormalLaneChange::getTerminalLaneChangePath() const
{
  auto output = prev_module_output_;

  if (isAbortState() && abort_path_) {
    output.path = abort_path_->path;
    extendOutputDrivableArea(output);
    const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path.points);
    output.turn_signal_info = planner_data_->turn_signal_decider.overwrite_turn_signal(
      output.path, getEgoPose(), current_seg_idx, prev_module_output_.turn_signal_info,
      output.turn_signal_info, planner_data_->parameters.ego_nearest_dist_threshold,
      planner_data_->parameters.ego_nearest_yaw_threshold);
    return output;
  }

  const auto current_lanes = getCurrentLanes();
  if (current_lanes.empty()) {
    RCLCPP_DEBUG(logger_, "Current lanes not found. Returning previous module's path as output.");
    return prev_module_output_;
  }

  const auto terminal_path =
    calcTerminalLaneChangePath(current_lanes, getLaneChangeLanes(current_lanes, direction_));
  if (!terminal_path) {
    RCLCPP_DEBUG(logger_, "Terminal path not found. Returning previous module's path as output.");
    return prev_module_output_;
  }

  output.path = terminal_path->path;
  output.turn_signal_info = updateOutputTurnSignal();

  extendOutputDrivableArea(output);

  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path.points);
  output.turn_signal_info = planner_data_->turn_signal_decider.overwrite_turn_signal(
    output.path, getEgoPose(), current_seg_idx, prev_module_output_.turn_signal_info,
    output.turn_signal_info, planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  return output;
}

BehaviorModuleOutput NormalLaneChange::generateOutput()
{
  if (!status_.is_valid_path) {
    RCLCPP_DEBUG(logger_, "No valid path found. Returning previous module's path as output.");
    return prev_module_output_;
  }

  auto output = prev_module_output_;
  if (isAbortState() && abort_path_) {
    output.path = abort_path_->path;
    insertStopPoint(status_.current_lanes, output.path);
  } else {
    output.path = getLaneChangePath().path;

    const auto found_extended_path = extendPath();
    if (found_extended_path) {
      output.path = utils::combinePath(output.path, *found_extended_path);
    }
    output.reference_path = getReferencePath();
    output.turn_signal_info = updateOutputTurnSignal();

    if (isStopState()) {
      const auto current_velocity = getEgoVelocity();
      const auto current_dist = calcSignedArcLength(
        output.path.points, output.path.points.front().point.pose.position, getEgoPosition());
      const auto stop_dist =
        -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));
      const auto stop_point = utils::insertStopPoint(stop_dist + current_dist, output.path);
      setStopPose(stop_point.point.pose);
    } else {
      insertStopPoint(status_.target_lanes, output.path);
    }
  }

  extendOutputDrivableArea(output);

  const auto current_seg_idx = planner_data_->findEgoSegmentIndex(output.path.points);
  output.turn_signal_info = planner_data_->turn_signal_decider.overwrite_turn_signal(
    output.path, getEgoPose(), current_seg_idx, prev_module_output_.turn_signal_info,
    output.turn_signal_info, planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  return output;
}

void NormalLaneChange::extendOutputDrivableArea(BehaviorModuleOutput & output) const
{
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), status_.current_lanes, status_.target_lanes);
  const auto shorten_lanes = utils::cutOverlappedLanes(output.path, drivable_lanes);
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  // for new architecture
  DrivableAreaInfo current_drivable_area_info;
  current_drivable_area_info.drivable_lanes = expanded_lanes;
  output.drivable_area_info = utils::combineDrivableAreaInfo(
    current_drivable_area_info, prev_module_output_.drivable_area_info);
}

void NormalLaneChange::insertStopPoint(
  const lanelet::ConstLanelets & lanelets, PathWithLaneId & path)
{
  if (lanelets.empty()) {
    return;
  }

  const auto & route_handler = getRouteHandler();

  if (route_handler->getNumLaneToPreferredLane(lanelets.back()) == 0) {
    return;
  }

  const auto shift_intervals = route_handler->getLateralIntervalsToPreferredLane(lanelets.back());
  const auto lane_change_buffer =
    calcMinimumLaneChangeLength(*lane_change_parameters_, shift_intervals);

  const auto getDistanceAlongLanelet = [&](const geometry_msgs::msg::Pose & target) {
    return utils::getSignedDistance(path.points.front().point.pose, target, lanelets);
  };

  // If lanelets.back() is in goal route section, get distance to goal.
  // Otherwise, get distance to end of lane.
  double distance_to_terminal = 0.0;
  if (route_handler->isInGoalRouteSection(lanelets.back())) {
    const auto goal = route_handler->getGoalPose();
    distance_to_terminal = getDistanceAlongLanelet(goal);
  } else {
    distance_to_terminal = utils::getDistanceToEndOfLane(path.points.front().point.pose, lanelets);
  }

  const double stop_point_buffer = lane_change_parameters_->backward_length_buffer_for_end_of_lane;
  const auto target_objects = filterObjects(status_.current_lanes, status_.target_lanes);
  double stopping_distance = distance_to_terminal - lane_change_buffer - stop_point_buffer;

  const auto is_valid_start_point = std::invoke([&]() -> bool {
    auto lc_start_point = lanelet::utils::conversion::toLaneletPoint(
      status_.lane_change_path.info.lane_changing_start.position);
    const auto target_neighbor_preferred_lane_poly_2d =
      utils::lane_change::getTargetNeighborLanesPolygon(
        *route_handler, status_.current_lanes, type_);
    return boost::geometry::covered_by(
      lanelet::traits::to2D(lc_start_point), target_neighbor_preferred_lane_poly_2d);
  });

  if (!is_valid_start_point) {
    const auto stop_point = utils::insertStopPoint(stopping_distance, path);
    setStopPose(stop_point.point.pose);

    return;
  }

  // calculate minimum distance from path front to the stationary object on the ego lane.
  const auto distance_to_ego_lane_obj = [&]() -> double {
    double distance_to_obj = distance_to_terminal;
    const double distance_to_ego = getDistanceAlongLanelet(getEgoPose());

    for (const auto & object : target_objects.current_lane) {
      // check if stationary
      const auto obj_v = std::abs(object.initial_twist.twist.linear.x);
      if (obj_v > lane_change_parameters_->stop_velocity_threshold) {
        continue;
      }

      // calculate distance from path front to the stationary object polygon on the ego lane.
      const auto polygon =
        autoware::universe_utils::toPolygon2d(object.initial_pose.pose, object.shape).outer();
      for (const auto & polygon_p : polygon) {
        const auto p_fp = autoware::universe_utils::toMsg(polygon_p.to_3d());
        const auto lateral_fp = autoware::motion_utils::calcLateralOffset(path.points, p_fp);

        // ignore if the point is around the ego path
        if (std::abs(lateral_fp) > planner_data_->parameters.vehicle_width) {
          continue;
        }

        const double current_distance_to_obj = calcSignedArcLength(path.points, 0, p_fp);

        // ignore backward object
        if (current_distance_to_obj < distance_to_ego) {
          continue;
        }
        distance_to_obj = std::min(distance_to_obj, current_distance_to_obj);
      }
    }
    return distance_to_obj;
  }();

  // Need to stop before blocking obstacle
  if (distance_to_ego_lane_obj < distance_to_terminal) {
    // consider rss distance when the LC need to avoid obstacles
    const auto rss_dist = calcRssDistance(
      0.0, lane_change_parameters_->minimum_lane_changing_velocity,
      lane_change_parameters_->rss_params);
    const double lane_change_buffer_for_blocking_object =
      utils::lane_change::calcMinimumLaneChangeLength(*lane_change_parameters_, shift_intervals);

    const auto stopping_distance_for_obj =
      distance_to_ego_lane_obj - lane_change_buffer_for_blocking_object -
      lane_change_parameters_->backward_length_buffer_for_blocking_object - rss_dist -
      getCommonParam().base_link2front;

    //  If the target lane in the lane change section is blocked by a stationary obstacle, there
    //  is no reason for stopping with a lane change margin. Instead, stop right behind the
    //  obstacle.
    //  ----------------------------------------------------------
    //                            [obj]>
    //  ----------------------------------------------------------
    //    [ego]>          | <--- lane change margin --->  [obj]>
    //  ----------------------------------------------------------
    const bool has_blocking_target_lane_obj = std::any_of(
      target_objects.target_lane.begin(), target_objects.target_lane.end(), [&](const auto & o) {
        const auto v = std::abs(o.initial_twist.twist.linear.x);
        if (v > lane_change_parameters_->stop_velocity_threshold) {
          return false;
        }

        // target_objects includes objects out of target lanes, so filter them out
        if (!boost::geometry::intersects(
              autoware::universe_utils::toPolygon2d(o.initial_pose.pose, o.shape).outer(),
              lanelet::utils::combineLaneletsShape(status_.target_lanes)
                .polygon2d()
                .basicPolygon())) {
          return false;
        }

        const double distance_to_target_lane_obj = getDistanceAlongLanelet(o.initial_pose.pose);
        return stopping_distance_for_obj < distance_to_target_lane_obj &&
               distance_to_target_lane_obj < distance_to_ego_lane_obj;
      });

    if (!has_blocking_target_lane_obj) {
      stopping_distance = stopping_distance_for_obj;
    }
  }

  if (stopping_distance > 0.0) {
    const auto stop_point = utils::insertStopPoint(stopping_distance, path);
    setStopPose(stop_point.point.pose);
  }
}

PathWithLaneId NormalLaneChange::getReferencePath() const
{
  return utils::getCenterLinePathFromLanelet(
    status_.lane_change_path.info.target_lanes.front(), planner_data_);
}

std::optional<PathWithLaneId> NormalLaneChange::extendPath()
{
  const auto path = status_.lane_change_path.path;
  const auto lc_start_point = status_.lane_change_path.info.lane_changing_start.position;

  const auto dist = calcSignedArcLength(path.points, lc_start_point, getEgoPosition());

  if (dist < 0.0) {
    return std::nullopt;
  }

  auto & target_lanes = status_.target_lanes;
  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanes);
  const auto dist_in_target = lanelet::utils::getArcCoordinates(target_lanes, getEgoPose());

  const auto forward_path_length = getCommonParam().forward_path_length;

  if ((target_lane_length - dist_in_target.length) > forward_path_length) {
    return std::nullopt;
  }

  const auto is_goal_in_target = getRouteHandler()->isInGoalRouteSection(target_lanes.back());

  if (is_goal_in_target) {
    const auto goal_pose = getRouteHandler()->getGoalPose();

    const auto dist_to_goal = lanelet::utils::getArcCoordinates(target_lanes, goal_pose).length;
    const auto dist_to_end_of_path =
      lanelet::utils::getArcCoordinates(target_lanes, path.points.back().point.pose).length;

    return getRouteHandler()->getCenterLinePath(target_lanes, dist_to_end_of_path, dist_to_goal);
  }

  lanelet::ConstLanelet next_lane;
  if (!getRouteHandler()->getNextLaneletWithinRoute(target_lanes.back(), &next_lane)) {
    return std::nullopt;
  }

  target_lanes.push_back(next_lane);

  const auto target_pose = std::invoke([&]() {
    const auto is_goal_in_next_lane = getRouteHandler()->isInGoalRouteSection(next_lane);
    if (is_goal_in_next_lane) {
      return getRouteHandler()->getGoalPose();
    }

    Pose back_pose;
    const auto back_point =
      lanelet::utils::conversion::toGeomMsgPt(next_lane.centerline2d().back());
    const double front_yaw = lanelet::utils::getLaneletAngle(next_lane, back_point);
    back_pose.position = back_point;
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, front_yaw);
    back_pose.orientation = tf2::toMsg(tf_quat);
    return back_pose;
  });

  const auto dist_to_target_pose =
    lanelet::utils::getArcCoordinates(target_lanes, target_pose).length;
  const auto dist_to_end_of_path =
    lanelet::utils::getArcCoordinates(target_lanes, path.points.back().point.pose).length;

  return getRouteHandler()->getCenterLinePath(
    target_lanes, dist_to_end_of_path, dist_to_target_pose);
}
void NormalLaneChange::resetParameters()
{
  is_abort_path_approved_ = false;
  is_abort_approval_requested_ = false;
  current_lane_change_state_ = LaneChangeStates::Normal;
  abort_path_ = nullptr;
  status_ = {};
  unsafe_hysteresis_count_ = 0;
  lane_change_debug_.reset();

  RCLCPP_DEBUG(logger_, "reset all flags and debug information.");
}

TurnSignalInfo NormalLaneChange::updateOutputTurnSignal() const
{
  const auto & pose = getEgoPose();
  const auto & current_lanes = status_.current_lanes;
  const auto & shift_line = status_.lane_change_path.info.shift_line;
  const auto & shift_path = status_.lane_change_path.shifted_path;
  const auto current_shift_length = lanelet::utils::getArcCoordinates(current_lanes, pose).distance;
  constexpr bool is_driving_forward = true;
  // The getBehaviorTurnSignalInfo method expects the shifted line to be generated off of the ego's
  // current lane, lane change is different, so we set this flag to false.
  constexpr bool egos_lane_is_shifted = false;

  const auto [new_signal, is_ignore] = planner_data_->getBehaviorTurnSignalInfo(
    shift_path, shift_line, current_lanes, current_shift_length, is_driving_forward,
    egos_lane_is_shifted);
  return new_signal;
}

lanelet::ConstLanelets NormalLaneChange::getCurrentLanes() const
{
  return utils::getCurrentLanesFromPath(prev_module_output_.path, planner_data_);
}

lanelet::ConstLanelets NormalLaneChange::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes, Direction direction) const
{
  if (current_lanes.empty()) {
    return {};
  }
  // Get lane change lanes
  const auto & route_handler = getRouteHandler();

  const auto lane_change_lane = utils::lane_change::getLaneChangeTargetLane(
    *getRouteHandler(), current_lanes, type_, direction);

  if (!lane_change_lane) {
    return {};
  }

  const auto front_pose = std::invoke([&lane_change_lane]() {
    const auto & p = lane_change_lane->centerline().front();
    const auto front_point = lanelet::utils::conversion::toGeomMsgPt(p);
    const auto front_yaw = lanelet::utils::getLaneletAngle(*lane_change_lane, front_point);
    geometry_msgs::msg::Pose front_pose;
    front_pose.position = front_point;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, front_yaw);
    front_pose.orientation = tf2::toMsg(quat);
    return front_pose;
  });

  const auto forward_length = std::invoke([&]() {
    const auto signed_distance = utils::getSignedDistance(front_pose, getEgoPose(), current_lanes);
    const auto forward_path_length = planner_data_->parameters.forward_path_length;
    if (signed_distance <= 0.0) {
      return forward_path_length;
    }

    return signed_distance + forward_path_length;
  });
  const auto backward_length = lane_change_parameters_->backward_lane_length;

  return route_handler->getLaneletSequence(
    lane_change_lane.value(), getEgoPose(), backward_length, forward_length);
}

bool NormalLaneChange::isNearEndOfCurrentLanes(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  const double threshold) const
{
  if (current_lanes.empty()) {
    return false;
  }

  const auto & route_handler = getRouteHandler();
  const auto & current_pose = getEgoPose();
  const auto lane_change_buffer = calcMinimumLaneChangeLength(
    route_handler, current_lanes.back(), *lane_change_parameters_, Direction::NONE);

  const auto distance_to_lane_change_end = std::invoke([&]() {
    auto distance_to_end = utils::getDistanceToEndOfLane(current_pose, current_lanes);

    if (!target_lanes.empty() && route_handler->isInGoalRouteSection(target_lanes.back())) {
      distance_to_end = std::min(
        distance_to_end,
        utils::getSignedDistance(current_pose, route_handler->getGoalPose(), current_lanes));
    }

    return std::max(0.0, distance_to_end) - lane_change_buffer;
  });

  lane_change_debug_.distance_to_end_of_current_lane = distance_to_lane_change_end;
  return distance_to_lane_change_end < threshold;
}

bool NormalLaneChange::hasFinishedLaneChange() const
{
  const auto & current_pose = getEgoPose();
  const auto & lane_change_end = status_.lane_change_path.info.shift_line.end;
  const double dist_to_lane_change_end = utils::getSignedDistance(
    current_pose, lane_change_end, status_.lane_change_path.info.target_lanes);
  double finish_judge_buffer = lane_change_parameters_->lane_change_finish_judge_buffer;

  // If ego velocity is low, relax finish judge buffer
  const double ego_velocity = getEgoVelocity();
  if (std::abs(ego_velocity) < 1.0) {
    finish_judge_buffer = 0.0;
  }

  const auto reach_lane_change_end = dist_to_lane_change_end + finish_judge_buffer < 0.0;

  lane_change_debug_.distance_to_lane_change_finished =
    dist_to_lane_change_end + finish_judge_buffer;

  if (!reach_lane_change_end) {
    return false;
  }

  const auto arc_length = lanelet::utils::getArcCoordinates(status_.target_lanes, current_pose);
  const auto reach_target_lane =
    std::abs(arc_length.distance) < lane_change_parameters_->finish_judge_lateral_threshold;

  lane_change_debug_.distance_to_lane_change_finished = arc_length.distance;

  return reach_target_lane;
}

bool NormalLaneChange::isAbleToReturnCurrentLane() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    lane_change_debug_.is_able_to_return_to_current_lane = false;
    return false;
  }

  if (!utils::isEgoWithinOriginalLane(
        status_.current_lanes, getEgoPose(), planner_data_->parameters,
        lane_change_parameters_->cancel.overhang_tolerance)) {
    lane_change_debug_.is_able_to_return_to_current_lane = false;
    return false;
  }

  const auto nearest_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const double ego_velocity =
    std::max(getEgoVelocity(), lane_change_parameters_->minimum_lane_changing_velocity);
  const double estimated_travel_dist = ego_velocity * lane_change_parameters_->cancel.delta_time;

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > estimated_travel_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      auto is_ego_within_original_lane = utils::isEgoWithinOriginalLane(
        status_.current_lanes, estimated_pose, planner_data_->parameters,
        lane_change_parameters_->cancel.overhang_tolerance);
      lane_change_debug_.is_able_to_return_to_current_lane = is_ego_within_original_lane;
      return is_ego_within_original_lane;
    }
  }

  lane_change_debug_.is_able_to_return_to_current_lane = true;
  return true;
}

bool NormalLaneChange::isEgoOnPreparePhase() const
{
  const auto & start_position = status_.lane_change_path.info.shift_line.start.position;
  const auto & path_points = status_.lane_change_path.path.points;
  return calcSignedArcLength(path_points, start_position, getEgoPosition()) < 0.0;
}

bool NormalLaneChange::isAbleToStopSafely() const
{
  if (status_.lane_change_path.path.points.size() < 2) {
    return false;
  }

  const auto nearest_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    status_.lane_change_path.path.points, getEgoPose(),
    planner_data_->parameters.ego_nearest_dist_threshold,
    planner_data_->parameters.ego_nearest_yaw_threshold);

  const auto current_velocity = getEgoVelocity();
  const auto stop_dist =
    -(current_velocity * current_velocity / (2.0 * planner_data_->parameters.min_acc));

  double dist = 0.0;
  for (size_t idx = nearest_idx; idx < status_.lane_change_path.path.points.size() - 1; ++idx) {
    dist += calcSignedArcLength(status_.lane_change_path.path.points, idx, idx + 1);
    if (dist > stop_dist) {
      const auto & estimated_pose = status_.lane_change_path.path.points.at(idx + 1).point.pose;
      return utils::isEgoWithinOriginalLane(
        status_.current_lanes, estimated_pose, planner_data_->parameters);
    }
  }
  return true;
}

bool NormalLaneChange::hasFinishedAbort() const
{
  if (!abort_path_) {
    lane_change_debug_.is_abort = true;
    return true;
  }

  const auto distance_to_finish = calcSignedArcLength(
    abort_path_->path.points, getEgoPosition(), abort_path_->info.shift_line.end.position);
  lane_change_debug_.distance_to_abort_finished = distance_to_finish;

  const auto has_finished_abort = distance_to_finish < 0.0;
  lane_change_debug_.is_abort = has_finished_abort;

  return has_finished_abort;
}

bool NormalLaneChange::isAbortState() const
{
  if (!lane_change_parameters_->cancel.enable_on_lane_changing_phase) {
    return false;
  }

  if (current_lane_change_state_ != LaneChangeStates::Abort) {
    return false;
  }

  if (!abort_path_) {
    return false;
  }

  lane_change_debug_.is_abort = true;
  return true;
}
int NormalLaneChange::getNumToPreferredLane(const lanelet::ConstLanelet & lane) const
{
  const auto get_opposite_direction =
    (direction_ == Direction::RIGHT) ? Direction::LEFT : Direction::RIGHT;
  return std::abs(getRouteHandler()->getNumLaneToPreferredLane(lane, get_opposite_direction));
}

std::pair<double, double> NormalLaneChange::calcCurrentMinMaxAcceleration() const
{
  const auto & p = getCommonParam();

  const auto vehicle_min_acc = std::max(p.min_acc, lane_change_parameters_->min_longitudinal_acc);
  const auto vehicle_max_acc = std::min(p.max_acc, lane_change_parameters_->max_longitudinal_acc);

  const auto ego_seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
    prev_module_output_.path.points, getEgoPose(), p.ego_nearest_dist_threshold,
    p.ego_nearest_yaw_threshold);
  const auto max_path_velocity =
    prev_module_output_.path.points.at(ego_seg_idx).point.longitudinal_velocity_mps;

  // calculate minimum and maximum acceleration
  const auto min_acc = utils::lane_change::calcMinimumAcceleration(
    getEgoVelocity(), vehicle_min_acc, *lane_change_parameters_);
  const auto max_acc = utils::lane_change::calcMaximumAcceleration(
    getEgoVelocity(), max_path_velocity, vehicle_max_acc, *lane_change_parameters_);

  return {min_acc, max_acc};
}

double NormalLaneChange::calcMaximumLaneChangeLength(
  const lanelet::ConstLanelet & current_terminal_lanelet, const double max_acc) const
{
  const auto shift_intervals =
    getRouteHandler()->getLateralIntervalsToPreferredLane(current_terminal_lanelet);
  return utils::lane_change::calcMaximumLaneChangeLength(
    std::max(lane_change_parameters_->minimum_lane_changing_velocity, getEgoVelocity()),
    *lane_change_parameters_, shift_intervals, max_acc);
}

std::vector<double> NormalLaneChange::sampleLongitudinalAccValues(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  if (prev_module_output_.path.points.empty()) {
    return {};
  }

  const auto & route_handler = *getRouteHandler();
  const auto current_pose = getEgoPose();
  const auto longitudinal_acc_sampling_num = lane_change_parameters_->longitudinal_acc_sampling_num;

  const auto [min_acc, max_acc] = calcCurrentMinMaxAcceleration();

  // if max acc is not positive, then we do the normal sampling
  if (max_acc <= 0.0) {
    RCLCPP_DEBUG(
      logger_, "Available max acc <= 0. Normal sampling for acc: [%f ~ %f]", min_acc, max_acc);
    return utils::lane_change::getAccelerationValues(
      min_acc, max_acc, longitudinal_acc_sampling_num);
  }

  // calculate maximum lane change length
  const double max_lane_change_length = calcMaximumLaneChangeLength(current_lanes.back(), max_acc);

  if (max_lane_change_length > utils::getDistanceToEndOfLane(current_pose, current_lanes)) {
    RCLCPP_DEBUG(
      logger_, "No enough distance to the end of lane. Normal sampling for acc: [%f ~ %f]", min_acc,
      max_acc);
    return utils::lane_change::getAccelerationValues(
      min_acc, max_acc, longitudinal_acc_sampling_num);
  }

  // If the ego is in stuck, sampling all possible accelerations to find avoiding path.
  if (isVehicleStuck(current_lanes)) {
    auto clock = rclcpp::Clock(RCL_ROS_TIME);
    RCLCPP_INFO_THROTTLE(
      logger_, clock, 1000, "Vehicle is in stuck. Sample all possible acc: [%f ~ %f]", min_acc,
      max_acc);
    return utils::lane_change::getAccelerationValues(
      min_acc, max_acc, longitudinal_acc_sampling_num);
  }

  // if maximum lane change length is less than length to goal or the end of target lanes, only
  // sample max acc
  if (route_handler.isInGoalRouteSection(target_lanes.back())) {
    const auto goal_pose = route_handler.getGoalPose();
    if (max_lane_change_length < utils::getSignedDistance(current_pose, goal_pose, target_lanes)) {
      RCLCPP_DEBUG(
        logger_, "Distance to goal has enough distance. Sample only max_acc: %f", max_acc);
      return {max_acc};
    }
  } else if (max_lane_change_length < utils::getDistanceToEndOfLane(current_pose, target_lanes)) {
    RCLCPP_DEBUG(
      logger_, "Distance to end of lane has enough distance. Sample only max_acc: %f", max_acc);
    return {max_acc};
  }

  RCLCPP_DEBUG(logger_, "Normal sampling for acc: [%f ~ %f]", min_acc, max_acc);
  return utils::lane_change::getAccelerationValues(min_acc, max_acc, longitudinal_acc_sampling_num);
}

std::vector<double> NormalLaneChange::calcPrepareDuration(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  const auto base_link2front = planner_data_->parameters.base_link2front;
  const auto threshold =
    lane_change_parameters_->min_length_for_turn_signal_activation + base_link2front;

  std::vector<double> prepare_durations;
  constexpr double step = 0.5;

  for (double duration = lane_change_parameters_->lane_change_prepare_duration; duration >= 0.0;
       duration -= step) {
    prepare_durations.push_back(duration);
    if (!isNearEndOfCurrentLanes(current_lanes, target_lanes, threshold)) {
      break;
    }
  }

  return prepare_durations;
}

PathWithLaneId NormalLaneChange::getPrepareSegment(
  const lanelet::ConstLanelets & current_lanes, const double backward_path_length,
  const double prepare_length) const
{
  if (current_lanes.empty()) {
    return PathWithLaneId();
  }

  auto prepare_segment = prev_module_output_.path;
  const size_t current_seg_idx =
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      prepare_segment.points, getEgoPose(), 3.0, 1.0);
  utils::clipPathLength(prepare_segment, current_seg_idx, prepare_length, backward_path_length);

  return prepare_segment;
}

ExtendedPredictedObjects NormalLaneChange::getTargetObjects(
  const LaneChangeLanesFilteredObjects & filtered_objects,
  const lanelet::ConstLanelets & current_lanes) const
{
  ExtendedPredictedObjects target_objects = filtered_objects.target_lane;
  const auto is_stuck = isVehicleStuck(current_lanes);
  const auto chk_obj_in_curr_lanes = lane_change_parameters_->check_objects_on_current_lanes;
  if (chk_obj_in_curr_lanes || is_stuck) {
    target_objects.insert(
      target_objects.end(), filtered_objects.current_lane.begin(),
      filtered_objects.current_lane.end());
  }

  const auto chk_obj_in_other_lanes = lane_change_parameters_->check_objects_on_other_lanes;
  if (chk_obj_in_other_lanes) {
    target_objects.insert(
      target_objects.end(), filtered_objects.other_lane.begin(), filtered_objects.other_lane.end());
  }

  return target_objects;
}

LaneChangeLanesFilteredObjects NormalLaneChange::filterObjects(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  const auto & current_pose = getEgoPose();
  const auto & route_handler = getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;
  auto objects = *planner_data_->dynamic_object;
  utils::path_safety_checker::filterObjectsByClass(
    objects, lane_change_parameters_->object_types_to_check);

  if (objects.objects.empty()) {
    return {};
  }

  filterOncomingObjects(objects);

  if (objects.objects.empty()) {
    return {};
  }

  filterAheadTerminalObjects(objects, current_lanes);

  if (objects.objects.empty()) {
    return {};
  }

  std::vector<PredictedObject> target_lane_objects;
  std::vector<PredictedObject> current_lane_objects;
  std::vector<PredictedObject> other_lane_objects;

  filterObjectsByLanelets(
    objects, current_lanes, target_lanes, current_lane_objects, target_lane_objects,
    other_lane_objects);

  const auto is_within_vel_th = [](const auto & object) -> bool {
    constexpr double min_vel_th = 1.0;
    constexpr double max_vel_th = std::numeric_limits<double>::max();
    return utils::path_safety_checker::filter::velocity_filter(object, min_vel_th, max_vel_th);
  };

  const auto path =
    route_handler->getCenterLinePath(current_lanes, 0.0, std::numeric_limits<double>::max());

  if (path.points.empty()) {
    return {};
  }

  const auto is_ahead_of_ego = [&path, &current_pose](const auto & object) {
    const auto obj_polygon = autoware::universe_utils::toPolygon2d(object).outer();

    double max_dist_ego_to_obj = std::numeric_limits<double>::lowest();
    for (const auto & polygon_p : obj_polygon) {
      const auto obj_p = autoware::universe_utils::createPoint(polygon_p.x(), polygon_p.y(), 0.0);
      const auto dist_ego_to_obj = calcSignedArcLength(path.points, current_pose.position, obj_p);
      max_dist_ego_to_obj = std::max(dist_ego_to_obj, max_dist_ego_to_obj);
    }
    return max_dist_ego_to_obj >= 0.0;
  };

  utils::path_safety_checker::filterObjects(
    target_lane_objects, [&](const PredictedObject & object) {
      return (is_within_vel_th(object) || is_ahead_of_ego(object));
    });

  utils::path_safety_checker::filterObjects(
    other_lane_objects, [&](const PredictedObject & object) {
      return is_within_vel_th(object) && is_ahead_of_ego(object);
    });

  utils::path_safety_checker::filterObjects(
    current_lane_objects, [&](const PredictedObject & object) {
      return is_within_vel_th(object) && is_ahead_of_ego(object);
    });

  LaneChangeLanesFilteredObjects lane_change_target_objects;

  const auto is_check_prepare_phase = check_prepare_phase();
  std::for_each(target_lane_objects.begin(), target_lane_objects.end(), [&](const auto & object) {
    auto extended_predicted_object = utils::lane_change::transform(
      object, common_parameters, *lane_change_parameters_, is_check_prepare_phase);
    lane_change_target_objects.target_lane.push_back(extended_predicted_object);
  });

  std::for_each(current_lane_objects.begin(), current_lane_objects.end(), [&](const auto & object) {
    auto extended_predicted_object = utils::lane_change::transform(
      object, common_parameters, *lane_change_parameters_, is_check_prepare_phase);
    lane_change_target_objects.current_lane.push_back(extended_predicted_object);
  });

  std::for_each(other_lane_objects.begin(), other_lane_objects.end(), [&](const auto & object) {
    auto extended_predicted_object = utils::lane_change::transform(
      object, common_parameters, *lane_change_parameters_, is_check_prepare_phase);
    lane_change_target_objects.other_lane.push_back(extended_predicted_object);
  });

  lane_change_debug_.filtered_objects = lane_change_target_objects;

  return lane_change_target_objects;
}

void NormalLaneChange::filterOncomingObjects(PredictedObjects & objects) const
{
  const auto & current_pose = getEgoPose();

  const auto is_same_direction = [&](const PredictedObject & object) {
    const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
    return !utils::path_safety_checker::isTargetObjectOncoming(current_pose, object_pose);
  };

  //  Perception noise could make stationary objects seem opposite the ego vehicle; check the
  //  velocity to prevent this.
  const auto is_stopped_object = [](const auto & object) -> bool {
    constexpr double min_vel_th = -0.5;
    constexpr double max_vel_th = 0.5;
    return utils::path_safety_checker::filter::velocity_filter(object, min_vel_th, max_vel_th);
  };

  utils::path_safety_checker::filterObjects(objects, [&](const PredictedObject & object) {
    const auto same_direction = is_same_direction(object);
    if (same_direction) {
      return true;
    }

    return is_stopped_object(object);
  });
}

void NormalLaneChange::filterAheadTerminalObjects(
  PredictedObjects & objects, const lanelet::ConstLanelets & current_lanes) const
{
  const auto & current_pose = getEgoPose();
  const auto & route_handler = getRouteHandler();
  const auto minimum_lane_change_length = utils::lane_change::calcMinimumLaneChangeLength(
    route_handler, current_lanes.back(), *lane_change_parameters_, direction_);

  const auto dist_ego_to_current_lanes_center =
    lanelet::utils::getLateralDistanceToClosestLanelet(current_lanes, current_pose);

  // ignore object that are ahead of terminal lane change start
  utils::path_safety_checker::filterObjects(objects, [&](const PredictedObject & object) {
    const auto obj_polygon = autoware::universe_utils::toPolygon2d(object).outer();
    // ignore object that are ahead of terminal lane change start
    auto distance_to_terminal_from_object = std::numeric_limits<double>::max();
    for (const auto & polygon_p : obj_polygon) {
      const auto obj_p = autoware::universe_utils::createPoint(polygon_p.x(), polygon_p.y(), 0.0);
      Pose polygon_pose;
      polygon_pose.position = obj_p;
      polygon_pose.orientation = object.kinematics.initial_pose_with_covariance.pose.orientation;
      const auto dist = utils::getDistanceToEndOfLane(polygon_pose, current_lanes);
      distance_to_terminal_from_object = std::min(dist_ego_to_current_lanes_center, dist);
    }

    return (minimum_lane_change_length > distance_to_terminal_from_object);
  });
}

void NormalLaneChange::filterObjectsByLanelets(
  const PredictedObjects & objects, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, std::vector<PredictedObject> & current_lane_objects,
  std::vector<PredictedObject> & target_lane_objects,
  std::vector<PredictedObject> & other_lane_objects) const
{
  const auto & current_pose = getEgoPose();
  const auto & route_handler = getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;
  const auto check_optional_polygon = [](const auto & object, const auto & polygon) {
    return polygon && isPolygonOverlapLanelet(object, *polygon);
  };

  // get backward lanes
  const auto backward_length = lane_change_parameters_->backward_lane_length;
  const auto target_backward_lanes =
    utils::getPrecedingLanelets(*route_handler, target_lanes, current_pose, backward_length);

  {
    lane_change_debug_.current_lanes = current_lanes;
    lane_change_debug_.target_lanes = target_lanes;

    // TODO(Azu) change the type to std::vector<lanelet::ConstLanelet>
    lane_change_debug_.target_backward_lanes.clear();
    std::for_each(
      target_backward_lanes.begin(), target_backward_lanes.end(),
      [&](const lanelet::ConstLanelets & target_backward_lane) {
        lane_change_debug_.target_backward_lanes.insert(
          lane_change_debug_.target_backward_lanes.end(), target_backward_lane.begin(),
          target_backward_lane.end());
      });
  }

  const auto expanded_target_lanes = utils::lane_change::generateExpandedLanelets(
    target_lanes, direction_, lane_change_parameters_->lane_expansion_left_offset,
    lane_change_parameters_->lane_expansion_right_offset);

  const auto lanes_polygon =
    createLanesPolygon(current_lanes, expanded_target_lanes, target_backward_lanes);
  const auto dist_ego_to_current_lanes_center =
    lanelet::utils::getLateralDistanceToClosestLanelet(current_lanes, current_pose);

  {
    const auto reserve_size = objects.objects.size();
    current_lane_objects.reserve(reserve_size);
    target_lane_objects.reserve(reserve_size);
    other_lane_objects.reserve(reserve_size);
  }

  for (const auto & object : objects.objects) {
    const auto is_lateral_far = std::invoke([&]() -> bool {
      const auto dist_object_to_current_lanes_center =
        lanelet::utils::getLateralDistanceToClosestLanelet(
          current_lanes, object.kinematics.initial_pose_with_covariance.pose);
      const auto lateral = dist_object_to_current_lanes_center - dist_ego_to_current_lanes_center;
      return std::abs(lateral) > (common_parameters.vehicle_width / 2);
    });

    if (check_optional_polygon(object, lanes_polygon.target) && is_lateral_far) {
      target_lane_objects.push_back(object);
      continue;
    }

    const auto is_overlap_target_backward = std::invoke([&]() -> bool {
      const auto check_backward_polygon = [&object](const auto & target_backward_polygon) {
        return isPolygonOverlapLanelet(object, target_backward_polygon);
      };
      return std::any_of(
        lanes_polygon.target_backward.begin(), lanes_polygon.target_backward.end(),
        check_backward_polygon);
    });

    // check if the object intersects with target backward lanes
    if (is_overlap_target_backward) {
      target_lane_objects.push_back(object);
      continue;
    }

    if (check_optional_polygon(object, lanes_polygon.current)) {
      // check only the objects that are in front of the ego vehicle
      current_lane_objects.push_back(object);
      continue;
    }

    other_lane_objects.push_back(object);
  }
}

PathWithLaneId NormalLaneChange::getTargetSegment(
  const lanelet::ConstLanelets & target_lanes, const Pose & lane_changing_start_pose,
  const double target_lane_length, const double lane_changing_length,
  const double lane_changing_velocity, const double buffer_for_next_lane_change) const
{
  const auto & route_handler = *getRouteHandler();
  const auto forward_path_length = planner_data_->parameters.forward_path_length;

  const double s_start = std::invoke([&lane_changing_start_pose, &target_lanes,
                                      &lane_changing_length, &target_lane_length,
                                      &buffer_for_next_lane_change]() {
    const auto arc_to_start_pose =
      lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose);
    const double dist_from_front_target_lanelet = arc_to_start_pose.length + lane_changing_length;
    const double end_of_lane_dist_without_buffer = target_lane_length - buffer_for_next_lane_change;
    return std::min(dist_from_front_target_lanelet, end_of_lane_dist_without_buffer);
  });

  const double s_end = std::invoke(
    [&s_start, &forward_path_length, &target_lane_length, &buffer_for_next_lane_change]() {
      const double dist_from_start = s_start + forward_path_length;
      const double dist_from_end = target_lane_length - buffer_for_next_lane_change;
      return std::max(
        std::min(dist_from_start, dist_from_end), s_start + std::numeric_limits<double>::epsilon());
    });

  RCLCPP_DEBUG(logger_, "in %s start: %f, end: %f", __func__, s_start, s_end);

  PathWithLaneId target_segment = route_handler.getCenterLinePath(target_lanes, s_start, s_end);
  for (auto & point : target_segment.points) {
    point.point.longitudinal_velocity_mps =
      std::min(point.point.longitudinal_velocity_mps, static_cast<float>(lane_changing_velocity));
  }

  return target_segment;
}

bool NormalLaneChange::hasEnoughLength(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes,
  const lanelet::ConstLanelets & target_lanes, const Direction direction) const
{
  if (target_lanes.empty()) {
    return false;
  }

  const auto current_pose = getEgoPose();
  const auto & route_handler = getRouteHandler();
  const auto overall_graphs_ptr = route_handler->getOverallGraphPtr();
  const auto minimum_lane_change_length_to_preferred_lane = calcMinimumLaneChangeLength(
    route_handler, target_lanes.back(), *lane_change_parameters_, direction);

  const double lane_change_length = path.info.length.sum();
  if (lane_change_length > utils::getDistanceToEndOfLane(current_pose, current_lanes)) {
    return false;
  }

  const auto goal_pose = route_handler->getGoalPose();
  if (
    route_handler->isInGoalRouteSection(current_lanes.back()) &&
    lane_change_length + minimum_lane_change_length_to_preferred_lane >
      utils::getSignedDistance(current_pose, goal_pose, current_lanes)) {
    return false;
  }

  // return if there are no target lanes
  if (
    lane_change_length + minimum_lane_change_length_to_preferred_lane >
    utils::getDistanceToEndOfLane(current_pose, target_lanes)) {
    return false;
  }

  return true;
}

bool NormalLaneChange::hasEnoughLengthToCrosswalk(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto overall_graphs_ptr = route_handler.getOverallGraphPtr();

  const double dist_to_crosswalk_from_lane_change_start_pose =
    utils::getDistanceToCrosswalk(current_pose, current_lanes, *overall_graphs_ptr) -
    path.info.length.prepare;
  // Check lane changing section includes crosswalk
  if (
    dist_to_crosswalk_from_lane_change_start_pose > 0.0 &&
    dist_to_crosswalk_from_lane_change_start_pose < path.info.length.lane_changing) {
    return false;
  }

  return true;
}

bool NormalLaneChange::hasEnoughLengthToIntersection(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto & route_handler = *getRouteHandler();
  const auto overall_graphs_ptr = route_handler.getOverallGraphPtr();

  const double dist_to_intersection_from_lane_change_start_pose =
    utils::getDistanceToNextIntersection(current_pose, current_lanes) - path.info.length.prepare;
  // Check lane changing section includes intersection
  if (
    dist_to_intersection_from_lane_change_start_pose > 0.0 &&
    dist_to_intersection_from_lane_change_start_pose < path.info.length.lane_changing) {
    return false;
  }

  return true;
}

bool NormalLaneChange::hasEnoughLengthToTrafficLight(
  const LaneChangePath & path, const lanelet::ConstLanelets & current_lanes) const
{
  const auto current_pose = getEgoPose();
  const auto dist_to_next_traffic_light =
    getDistanceToNextTrafficLight(current_pose, current_lanes);
  const auto dist_to_next_traffic_light_from_lc_start_pose =
    dist_to_next_traffic_light - path.info.length.prepare;

  return dist_to_next_traffic_light_from_lc_start_pose <= 0.0 ||
         dist_to_next_traffic_light_from_lc_start_pose >= path.info.length.lane_changing;
}

bool NormalLaneChange::getLaneChangePaths(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes,
  Direction direction, LaneChangePaths * candidate_paths,
  const utils::path_safety_checker::RSSparams rss_params, const bool is_stuck,
  const bool check_safety) const
{
  lane_change_debug_.collision_check_objects.clear();
  if (current_lanes.empty() || target_lanes.empty()) {
    RCLCPP_WARN(logger_, "target_neighbor_preferred_lane_poly_2d is empty. Not expected.");
    return false;
  }
  const auto & route_handler = *getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;

  const auto backward_path_length = common_parameters.backward_path_length;
  const auto forward_path_length = common_parameters.forward_path_length;
  const auto minimum_lane_changing_velocity =
    lane_change_parameters_->minimum_lane_changing_velocity;
  const auto lateral_acc_sampling_num = lane_change_parameters_->lateral_acc_sampling_num;

  // get velocity
  const auto current_velocity = getEgoVelocity();

  // get sampling acceleration values
  const auto longitudinal_acc_sampling_values =
    sampleLongitudinalAccValues(current_lanes, target_lanes);

  const auto is_goal_in_route = route_handler.isInGoalRouteSection(target_lanes.back());

  const double lane_change_buffer = utils::lane_change::calcMinimumLaneChangeLength(
    *lane_change_parameters_,
    route_handler.getLateralIntervalsToPreferredLane(current_lanes.back()));
  const double next_lane_change_buffer = utils::lane_change::calcMinimumLaneChangeLength(
    *lane_change_parameters_,
    route_handler.getLateralIntervalsToPreferredLane(target_lanes.back()));

  const auto dist_to_end_of_current_lanes =
    utils::getDistanceToEndOfLane(getEgoPose(), current_lanes);

  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanes);

  const auto sorted_lane_ids =
    utils::lane_change::getSortedLaneIds(route_handler, getEgoPose(), current_lanes, target_lanes);

  const auto target_neighbor_preferred_lane_poly_2d =
    utils::lane_change::getTargetNeighborLanesPolygon(route_handler, current_lanes, type_);
  if (target_neighbor_preferred_lane_poly_2d.empty()) {
    RCLCPP_WARN(logger_, "target_neighbor_preferred_lane_poly_2d is empty. Not expected.");
    return false;
  }

  const auto filtered_objects = filterObjects(current_lanes, target_lanes);
  const auto target_objects = getTargetObjects(filtered_objects, current_lanes);

  const auto prepare_durations = calcPrepareDuration(current_lanes, target_lanes);

  candidate_paths->reserve(
    longitudinal_acc_sampling_values.size() * lateral_acc_sampling_num * prepare_durations.size());

  RCLCPP_DEBUG(
    logger_, "lane change sampling start. Sampling num for prep_time: %lu, acc: %lu",
    prepare_durations.size(), longitudinal_acc_sampling_values.size());

  for (const auto & prepare_duration : prepare_durations) {
    for (const auto & sampled_longitudinal_acc : longitudinal_acc_sampling_values) {
      const auto debug_print = [&](const auto & s) {
        RCLCPP_DEBUG_STREAM(
          logger_, "  -  " << s << " : prep_time = " << prepare_duration
                           << ", lon_acc = " << sampled_longitudinal_acc);
      };

      // get path on original lanes
      const auto prepare_velocity = std::clamp(
        current_velocity + sampled_longitudinal_acc * prepare_duration,
        minimum_lane_changing_velocity, getCommonParam().max_vel);

      // compute actual longitudinal acceleration
      const double longitudinal_acc_on_prepare =
        (prepare_duration < 1e-3) ? 0.0
                                  : ((prepare_velocity - current_velocity) / prepare_duration);

      const auto prepare_length = utils::lane_change::calcPhaseLength(
        current_velocity, getCommonParam().max_vel, longitudinal_acc_on_prepare, prepare_duration);

      auto prepare_segment = getPrepareSegment(current_lanes, backward_path_length, prepare_length);

      if (prepare_segment.points.empty()) {
        debug_print("prepare segment is empty...? Unexpected.");
        continue;
      }

      // lane changing start getEgoPose() is at the end of prepare segment
      const auto & lane_changing_start_pose = prepare_segment.points.back().point.pose;
      const auto target_length_from_lane_change_start_pose = utils::getArcLengthToTargetLanelet(
        current_lanes, target_lanes.front(), lane_changing_start_pose);

      // Check if the lane changing start point is not on the lanes next to target lanes,
      if (target_length_from_lane_change_start_pose > 0.0) {
        debug_print("lane change start getEgoPose() is behind target lanelet!");
        break;
      }

      const auto shift_length =
        lanelet::utils::getLateralDistanceToClosestLanelet(target_lanes, lane_changing_start_pose);

      const auto initial_lane_changing_velocity = prepare_velocity;
      const auto max_path_velocity = prepare_segment.points.back().point.longitudinal_velocity_mps;

      // get lateral acceleration range
      const auto [min_lateral_acc, max_lateral_acc] =
        lane_change_parameters_->lane_change_lat_acc_map.find(initial_lane_changing_velocity);
      const auto lateral_acc_resolution =
        std::abs(max_lateral_acc - min_lateral_acc) / lateral_acc_sampling_num;

      std::vector<double> sample_lat_acc;
      constexpr double eps = 0.01;
      for (double a = min_lateral_acc; a < max_lateral_acc + eps; a += lateral_acc_resolution) {
        sample_lat_acc.push_back(a);
      }
      RCLCPP_DEBUG(logger_, "  -  sampling num for lat_acc: %lu", sample_lat_acc.size());

      for (const auto & lateral_acc : sample_lat_acc) {
        const auto debug_print = [&](const auto & s) {
          RCLCPP_DEBUG_STREAM(
            logger_, "    -  " << s << " : prep_time = " << prepare_duration << ", lon_acc = "
                               << sampled_longitudinal_acc << ", lat_acc = " << lateral_acc);
        };

        const auto lane_changing_time = PathShifter::calcShiftTimeFromJerk(
          shift_length, lane_change_parameters_->lane_changing_lateral_jerk, lateral_acc);
        const double longitudinal_acc_on_lane_changing =
          utils::lane_change::calcLaneChangingAcceleration(
            initial_lane_changing_velocity, max_path_velocity, lane_changing_time,
            sampled_longitudinal_acc);
        const auto lane_changing_length = utils::lane_change::calcPhaseLength(
          initial_lane_changing_velocity, getCommonParam().max_vel,
          longitudinal_acc_on_lane_changing, lane_changing_time);
        const auto terminal_lane_changing_velocity = std::min(
          initial_lane_changing_velocity + longitudinal_acc_on_lane_changing * lane_changing_time,
          getCommonParam().max_vel);
        utils::lane_change::setPrepareVelocity(
          prepare_segment, current_velocity, terminal_lane_changing_velocity);

        if (lane_changing_length + prepare_length > dist_to_end_of_current_lanes) {
          debug_print("Reject: length of lane changing path is longer than length to goal!!");
          continue;
        }

        if (is_goal_in_route) {
          const double s_start =
            lanelet::utils::getArcCoordinates(target_lanes, lane_changing_start_pose).length;
          const double s_goal =
            lanelet::utils::getArcCoordinates(target_lanes, route_handler.getGoalPose()).length;
          const auto num =
            std::abs(route_handler.getNumLaneToPreferredLane(target_lanes.back(), direction));
          const double backward_buffer =
            num == 0 ? 0.0 : lane_change_parameters_->backward_length_buffer_for_end_of_lane;
          const double finish_judge_buffer =
            lane_change_parameters_->lane_change_finish_judge_buffer;
          if (
            s_start + lane_changing_length + finish_judge_buffer + backward_buffer +
              next_lane_change_buffer >
            s_goal) {
            debug_print("Reject: length of lane changing path is longer than length to goal!!");
            continue;
          }
        }

        const auto target_segment = getTargetSegment(
          target_lanes, lane_changing_start_pose, target_lane_length, lane_changing_length,
          initial_lane_changing_velocity, next_lane_change_buffer);

        if (target_segment.points.empty()) {
          debug_print("Reject: target segment is empty!! something wrong...");
          continue;
        }

        const lanelet::BasicPoint2d lc_start_point(
          prepare_segment.points.back().point.pose.position.x,
          prepare_segment.points.back().point.pose.position.y);

        const auto target_lane_polygon = lanelet::utils::getPolygonFromArcLength(
          target_lanes, 0, std::numeric_limits<double>::max());
        const auto target_lane_poly_2d = lanelet::utils::to2D(target_lane_polygon).basicPolygon();

        const auto is_valid_start_point =
          boost::geometry::covered_by(lc_start_point, target_neighbor_preferred_lane_poly_2d) ||
          boost::geometry::covered_by(lc_start_point, target_lane_poly_2d);

        LaneChangeInfo lane_change_info;
        lane_change_info.longitudinal_acceleration =
          LaneChangePhaseInfo{longitudinal_acc_on_prepare, longitudinal_acc_on_lane_changing};
        lane_change_info.duration = LaneChangePhaseInfo{prepare_duration, lane_changing_time};
        lane_change_info.velocity =
          LaneChangePhaseInfo{prepare_velocity, initial_lane_changing_velocity};
        lane_change_info.length = LaneChangePhaseInfo{prepare_length, lane_changing_length};
        lane_change_info.current_lanes = current_lanes;
        lane_change_info.target_lanes = target_lanes;
        lane_change_info.lane_changing_start = prepare_segment.points.back().point.pose;
        lane_change_info.lane_changing_end = target_segment.points.front().point.pose;
        lane_change_info.lateral_acceleration = lateral_acc;
        lane_change_info.terminal_lane_changing_velocity = terminal_lane_changing_velocity;

        if (!is_valid_start_point) {
          debug_print(
            "Reject: lane changing points are not inside of the target preferred lanes or its "
            "neighbors");
          continue;
        }

        const auto resample_interval = utils::lane_change::calcLaneChangeResampleInterval(
          lane_changing_length, initial_lane_changing_velocity);
        const auto target_lane_reference_path = utils::lane_change::getReferencePathFromTargetLane(
          route_handler, target_lanes, lane_changing_start_pose, target_lane_length,
          lane_changing_length, forward_path_length, resample_interval, is_goal_in_route,
          next_lane_change_buffer);

        if (target_lane_reference_path.points.empty()) {
          debug_print("Reject: target_lane_reference_path is empty!!");
          continue;
        }

        lane_change_info.shift_line = utils::lane_change::getLaneChangingShiftLine(
          prepare_segment, target_segment, target_lane_reference_path, shift_length);

        const auto candidate_path = utils::lane_change::constructCandidatePath(
          lane_change_info, prepare_segment, target_segment, target_lane_reference_path,
          sorted_lane_ids);

        if (!candidate_path) {
          debug_print("Reject: failed to generate candidate path!!");
          continue;
        }

        if (!hasEnoughLength(*candidate_path, current_lanes, target_lanes, direction)) {
          debug_print("Reject: invalid candidate path!!");
          continue;
        }

        if (
          lane_change_parameters_->regulate_on_crosswalk &&
          !hasEnoughLengthToCrosswalk(*candidate_path, current_lanes)) {
          if (getStopTime() < lane_change_parameters_->stop_time_threshold) {
            debug_print("Reject: including crosswalk!!");
            continue;
          }
          RCLCPP_INFO_THROTTLE(
            logger_, clock_, 1000, "Stop time is over threshold. Allow lane change in crosswalk.");
        }

        if (
          lane_change_parameters_->regulate_on_intersection &&
          !hasEnoughLengthToIntersection(*candidate_path, current_lanes)) {
          if (getStopTime() < lane_change_parameters_->stop_time_threshold) {
            debug_print("Reject: including intersection!!");
            continue;
          }
          RCLCPP_WARN_STREAM(
            logger_, "Stop time is over threshold. Allow lane change in intersection.");
        }

        if (
          lane_change_parameters_->regulate_on_traffic_light &&
          !hasEnoughLengthToTrafficLight(*candidate_path, current_lanes)) {
          debug_print("Reject: regulate on traffic light!!");
          continue;
        }

        if (utils::traffic_light::isStoppedAtRedTrafficLightWithinDistance(
              lane_change_info.current_lanes, candidate_path.value().path, planner_data_,
              lane_change_info.length.sum())) {
          debug_print("Ego is stopping near traffic light. Do not allow lane change");
          continue;
        }
        candidate_paths->push_back(*candidate_path);

        if (
          !is_stuck && utils::lane_change::passParkedObject(
                         route_handler, *candidate_path, filtered_objects.target_lane,
                         lane_change_buffer, is_goal_in_route, *lane_change_parameters_,
                         lane_change_debug_.collision_check_objects)) {
          debug_print(
            "Reject: parking vehicle exists in the target lane, and the ego is not in stuck. Skip "
            "lane change.");
          return false;
        }

        if (!check_safety) {
          debug_print("ACCEPT!!!: it is valid (and safety check is skipped).");
          return false;
        }

        const auto [is_safe, is_object_coming_from_rear] = isLaneChangePathSafe(
          *candidate_path, target_objects, rss_params, lane_change_debug_.collision_check_objects);

        if (is_safe) {
          debug_print("ACCEPT!!!: it is valid and safe!");
          return true;
        }

        debug_print("Reject: sampled path is not safe.");
      }
    }
  }

  RCLCPP_DEBUG(logger_, "No safety path found.");
  return false;
}

std::optional<LaneChangePath> NormalLaneChange::calcTerminalLaneChangePath(
  const lanelet::ConstLanelets & current_lanes, const lanelet::ConstLanelets & target_lanes) const
{
  if (current_lanes.empty() || target_lanes.empty()) {
    RCLCPP_WARN(logger_, "target_neighbor_preferred_lane_poly_2d is empty. Not expected.");
    return {};
  }
  const auto & route_handler = *getRouteHandler();
  const auto & common_parameters = planner_data_->parameters;

  const auto forward_path_length = common_parameters.forward_path_length;
  const auto minimum_lane_changing_velocity =
    lane_change_parameters_->minimum_lane_changing_velocity;

  const auto is_goal_in_route = route_handler.isInGoalRouteSection(target_lanes.back());

  const double lane_change_buffer = utils::lane_change::calcMinimumLaneChangeLength(
    *lane_change_parameters_,
    route_handler.getLateralIntervalsToPreferredLane(current_lanes.back()));
  const double next_lane_change_buffer = utils::lane_change::calcMinimumLaneChangeLength(
    *lane_change_parameters_,
    route_handler.getLateralIntervalsToPreferredLane(target_lanes.back()));

  const auto target_lane_length = lanelet::utils::getLaneletLength2d(target_lanes);

  const auto sorted_lane_ids =
    utils::lane_change::getSortedLaneIds(route_handler, getEgoPose(), current_lanes, target_lanes);

  const auto target_neighbor_preferred_lane_poly_2d =
    utils::lane_change::getTargetNeighborLanesPolygon(route_handler, current_lanes, type_);
  if (target_neighbor_preferred_lane_poly_2d.empty()) {
    RCLCPP_WARN(logger_, "target_neighbor_preferred_lane_poly_2d is empty. Not expected.");
    return {};
  }

  // lane changing start getEgoPose() is at the end of prepare segment
  const auto current_lane_terminal_point =
    lanelet::utils::conversion::toGeomMsgPt(current_lanes.back().centerline3d().back());

  double distance_to_terminal_from_goal = 0;
  if (is_goal_in_route) {
    distance_to_terminal_from_goal =
      utils::getDistanceToEndOfLane(route_handler.getGoalPose(), current_lanes);
  }

  const auto lane_changing_start_pose = autoware::motion_utils::calcLongitudinalOffsetPose(
    prev_module_output_.path.points, current_lane_terminal_point,
    -(lane_change_buffer + next_lane_change_buffer + distance_to_terminal_from_goal));

  if (!lane_changing_start_pose) {
    RCLCPP_DEBUG(logger_, "Reject: lane changing start pose not found!!!");
    return {};
  }

  const auto target_length_from_lane_change_start_pose = utils::getArcLengthToTargetLanelet(
    current_lanes, target_lanes.front(), lane_changing_start_pose.value());

  // Check if the lane changing start point is not on the lanes next to target lanes,
  if (target_length_from_lane_change_start_pose > 0.0) {
    RCLCPP_DEBUG(logger_, "lane change start getEgoPose() is behind target lanelet!");
    return {};
  }

  const auto shift_length = lanelet::utils::getLateralDistanceToClosestLanelet(
    target_lanes, lane_changing_start_pose.value());

  const auto [min_lateral_acc, max_lateral_acc] =
    lane_change_parameters_->lane_change_lat_acc_map.find(minimum_lane_changing_velocity);

  const auto lane_changing_time = PathShifter::calcShiftTimeFromJerk(
    shift_length, lane_change_parameters_->lane_changing_lateral_jerk, max_lateral_acc);

  const auto target_segment = getTargetSegment(
    target_lanes, lane_changing_start_pose.value(), target_lane_length, lane_change_buffer,
    minimum_lane_changing_velocity, next_lane_change_buffer);

  if (target_segment.points.empty()) {
    RCLCPP_DEBUG(logger_, "Reject: target segment is empty!! something wrong...");
    return {};
  }

  const lanelet::BasicPoint2d lc_start_point(
    lane_changing_start_pose->position.x, lane_changing_start_pose->position.y);

  const auto target_lane_polygon =
    lanelet::utils::getPolygonFromArcLength(target_lanes, 0, std::numeric_limits<double>::max());
  const auto target_lane_poly_2d = lanelet::utils::to2D(target_lane_polygon).basicPolygon();

  const auto is_valid_start_point =
    boost::geometry::covered_by(lc_start_point, target_neighbor_preferred_lane_poly_2d) ||
    boost::geometry::covered_by(lc_start_point, target_lane_poly_2d);

  LaneChangeInfo lane_change_info;
  lane_change_info.longitudinal_acceleration = LaneChangePhaseInfo{0.0, 0.0};
  lane_change_info.duration = LaneChangePhaseInfo{0.0, lane_changing_time};
  lane_change_info.velocity =
    LaneChangePhaseInfo{minimum_lane_changing_velocity, minimum_lane_changing_velocity};
  lane_change_info.length = LaneChangePhaseInfo{0.0, lane_change_buffer};
  lane_change_info.current_lanes = current_lanes;
  lane_change_info.target_lanes = target_lanes;
  lane_change_info.lane_changing_start = lane_changing_start_pose.value();
  lane_change_info.lane_changing_end = target_segment.points.front().point.pose;
  lane_change_info.lateral_acceleration = max_lateral_acc;
  lane_change_info.terminal_lane_changing_velocity = minimum_lane_changing_velocity;

  if (!is_valid_start_point) {
    RCLCPP_DEBUG(
      logger_,
      "Reject: lane changing points are not inside of the target preferred lanes or its "
      "neighbors");
    return {};
  }

  const auto resample_interval = utils::lane_change::calcLaneChangeResampleInterval(
    lane_change_buffer, minimum_lane_changing_velocity);
  const auto target_lane_reference_path = utils::lane_change::getReferencePathFromTargetLane(
    route_handler, target_lanes, lane_changing_start_pose.value(), target_lane_length,
    lane_change_buffer, forward_path_length, resample_interval, is_goal_in_route,
    next_lane_change_buffer);

  if (target_lane_reference_path.points.empty()) {
    RCLCPP_DEBUG(logger_, "Reject: target_lane_reference_path is empty!!");
    return {};
  }

  lane_change_info.shift_line = utils::lane_change::getLaneChangingShiftLine(
    lane_changing_start_pose.value(), target_segment.points.front().point.pose,
    target_lane_reference_path, shift_length);

  auto reference_segment = prev_module_output_.path;
  const double length_to_lane_changing_start = autoware::motion_utils::calcSignedArcLength(
    reference_segment.points, reference_segment.points.front().point.pose.position,
    lane_changing_start_pose->position);
  utils::clipPathLength(reference_segment, 0, length_to_lane_changing_start, 0.0);
  // remove terminal points because utils::clipPathLength() calculates extra long path
  reference_segment.points.pop_back();
  reference_segment.points.back().point.longitudinal_velocity_mps = minimum_lane_changing_velocity;

  const auto terminal_lane_change_path = utils::lane_change::constructCandidatePath(
    lane_change_info, reference_segment, target_segment, target_lane_reference_path,
    sorted_lane_ids);

  return terminal_lane_change_path;
}

PathSafetyStatus NormalLaneChange::isApprovedPathSafe() const
{
  const auto & path = status_.lane_change_path;
  const auto & current_lanes = status_.current_lanes;
  const auto & target_lanes = status_.target_lanes;

  const auto filtered_objects = filterObjects(current_lanes, target_lanes);
  const auto target_objects = getTargetObjects(filtered_objects, current_lanes);

  CollisionCheckDebugMap debug_data;
  const auto safety_status = isLaneChangePathSafe(
    path, target_objects, lane_change_parameters_->rss_params_for_abort, debug_data);
  {
    // only for debug purpose
    lane_change_debug_.collision_check_objects.clear();
    lane_change_debug_.collision_check_object_debug_lifetime +=
      (stop_watch_.toc(getModuleTypeStr()) / 1000);
    if (lane_change_debug_.collision_check_object_debug_lifetime > 2.0) {
      stop_watch_.toc(getModuleTypeStr(), true);
      lane_change_debug_.collision_check_object_debug_lifetime = 0.0;
      lane_change_debug_.collision_check_objects_after_approval.clear();
    }

    if (!safety_status.is_safe) {
      lane_change_debug_.collision_check_objects_after_approval = debug_data;
    }
  }

  return safety_status;
}

PathSafetyStatus NormalLaneChange::evaluateApprovedPathWithUnsafeHysteresis(
  PathSafetyStatus approved_path_safety_status)
{
  if (!approved_path_safety_status.is_safe) {
    ++unsafe_hysteresis_count_;
    RCLCPP_DEBUG(
      logger_, "%s: Increasing hysteresis count to %d.", __func__, unsafe_hysteresis_count_);
  } else {
    if (unsafe_hysteresis_count_ > 0) {
      RCLCPP_DEBUG(logger_, "%s: Lane change is now SAFE. Resetting hysteresis count.", __func__);
    }
    unsafe_hysteresis_count_ = 0;
  }
  if (unsafe_hysteresis_count_ > lane_change_parameters_->cancel.unsafe_hysteresis_threshold) {
    RCLCPP_DEBUG(
      logger_, "%s: hysteresis count exceed threshold. lane change is now %s", __func__,
      (approved_path_safety_status.is_safe ? "safe" : "UNSAFE"));
    return approved_path_safety_status;
  }
  return {};
}

bool NormalLaneChange::isValidPath(const PathWithLaneId & path) const
{
  const auto & route_handler = planner_data_->route_handler;
  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  // check lane departure
  const auto drivable_lanes = utils::lane_change::generateDrivableLanes(
    *route_handler, utils::extendLanes(route_handler, status_.current_lanes),
    utils::extendLanes(route_handler, status_.target_lanes));
  const auto expanded_lanes = utils::expandLanelets(
    drivable_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);

  const auto lanelets = utils::transformToLanelets(expanded_lanes);

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
      return false;
    }
  }

  // check relative angle
  if (!utils::checkPathRelativeAngle(path, M_PI)) {
    return false;
  }

  return true;
}

bool NormalLaneChange::isRequiredStop(const bool is_object_coming_from_rear)
{
  const auto threshold = lane_change_parameters_->backward_length_buffer_for_end_of_lane;
  if (
    isNearEndOfCurrentLanes(status_.current_lanes, status_.target_lanes, threshold) &&
    isAbleToStopSafely() && is_object_coming_from_rear) {
    current_lane_change_state_ = LaneChangeStates::Stop;
    return true;
  }
  current_lane_change_state_ = LaneChangeStates::Normal;
  return false;
}

bool NormalLaneChange::calcAbortPath()
{
  const auto & route_handler = getRouteHandler();
  const auto & common_param = getCommonParam();
  const auto current_velocity =
    std::max(lane_change_parameters_->minimum_lane_changing_velocity, getEgoVelocity());
  const auto current_pose = getEgoPose();
  const auto & selected_path = status_.lane_change_path;
  const auto reference_lanelets = selected_path.info.current_lanes;

  const auto ego_nearest_dist_threshold = common_param.ego_nearest_dist_threshold;
  const auto ego_nearest_yaw_threshold = common_param.ego_nearest_yaw_threshold;

  const auto direction = getDirection();
  const auto minimum_lane_change_length = utils::lane_change::calcMinimumLaneChangeLength(
    route_handler, selected_path.info.current_lanes.back(), *lane_change_parameters_, direction);

  const auto & lane_changing_path = selected_path.path;
  const auto lane_changing_end_pose_idx = std::invoke([&]() {
    constexpr double s_start = 0.0;
    const double s_end = std::max(
      lanelet::utils::getLaneletLength2d(reference_lanelets) - minimum_lane_change_length, 0.0);

    const auto ref = route_handler->getCenterLinePath(reference_lanelets, s_start, s_end);
    return autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
      lane_changing_path.points, ref.points.back().point.pose, ego_nearest_dist_threshold,
      ego_nearest_yaw_threshold);
  });

  const auto ego_pose_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    lane_changing_path.points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold);

  const auto get_abort_idx_and_distance = [&](const double param_time) {
    if (ego_pose_idx > lane_changing_end_pose_idx) {
      return std::make_pair(ego_pose_idx, 0.0);
    }

    const auto desired_distance = current_velocity * param_time;
    const auto & points = lane_changing_path.points;

    for (size_t idx = ego_pose_idx; idx < lane_changing_end_pose_idx; ++idx) {
      const double distance =
        utils::getSignedDistance(current_pose, points.at(idx).point.pose, reference_lanelets);
      if (distance > desired_distance) {
        return std::make_pair(idx, distance);
      }
    }

    return std::make_pair(ego_pose_idx, 0.0);
  };

  const auto [abort_start_idx, abort_start_dist] =
    get_abort_idx_and_distance(lane_change_parameters_->cancel.delta_time);
  const auto [abort_return_idx, abort_return_dist] = get_abort_idx_and_distance(
    lane_change_parameters_->cancel.delta_time + lane_change_parameters_->cancel.duration);

  if (abort_start_idx >= abort_return_idx) {
    RCLCPP_ERROR(logger_, "abort start idx and return idx is equal. can't compute abort path.");
    return false;
  }

  if (!utils::lane_change::hasEnoughLengthToLaneChangeAfterAbort(
        route_handler, reference_lanelets, current_pose, abort_return_dist,
        *lane_change_parameters_, direction)) {
    RCLCPP_ERROR(logger_, "insufficient distance to abort.");
    return false;
  }

  const auto abort_start_pose = lane_changing_path.points.at(abort_start_idx).point.pose;
  const auto abort_return_pose = lane_changing_path.points.at(abort_return_idx).point.pose;
  const auto shift_length =
    lanelet::utils::getArcCoordinates(reference_lanelets, abort_return_pose).distance;

  ShiftLine shift_line;
  shift_line.start = abort_start_pose;
  shift_line.end = abort_return_pose;
  shift_line.end_shift_length = -shift_length;
  shift_line.start_idx = abort_start_idx;
  shift_line.end_idx = abort_return_idx;

  PathShifter path_shifter;
  path_shifter.setPath(lane_changing_path);
  path_shifter.addShiftLine(shift_line);
  const auto lateral_jerk =
    autoware::behavior_path_planner::PathShifter::calcJerkFromLatLonDistance(
      shift_line.end_shift_length, abort_start_dist, current_velocity);
  path_shifter.setVelocity(current_velocity);
  const auto lateral_acc_range =
    lane_change_parameters_->lane_change_lat_acc_map.find(current_velocity);
  const double & max_lateral_acc = lateral_acc_range.second;
  path_shifter.setLateralAccelerationLimit(max_lateral_acc);

  if (lateral_jerk > lane_change_parameters_->cancel.max_lateral_jerk) {
    RCLCPP_ERROR(logger_, "Aborting jerk is too strong. lateral_jerk = %f", lateral_jerk);
    return false;
  }

  ShiftedPath shifted_path;
  // offset front side
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(logger_, "failed to generate abort shifted path.");
  }

  auto reference_lane_segment = prev_module_output_.path;
  {
    // const auto terminal_path =
    //   calcTerminalLaneChangePath(reference_lanelets, selected_path.info.target_lanes);
    // if (terminal_path) {
    //   reference_lane_segment = terminal_path->path;
    // }
    const auto return_pose = shifted_path.path.points.at(abort_return_idx).point.pose;
    const auto seg_idx = autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      reference_lane_segment.points, return_pose, common_param.ego_nearest_dist_threshold,
      common_param.ego_nearest_yaw_threshold);
    reference_lane_segment.points = autoware::motion_utils::cropPoints(
      reference_lane_segment.points, return_pose.position, seg_idx,
      common_param.forward_path_length, 0.0);
  }

  auto abort_path = selected_path;
  abort_path.shifted_path = shifted_path;
  abort_path.info.shift_line = shift_line;

  {
    PathWithLaneId aborting_path;
    aborting_path.points.insert(
      aborting_path.points.begin(), shifted_path.path.points.begin(),
      shifted_path.path.points.begin() + abort_return_idx);

    if (!reference_lane_segment.points.empty()) {
      abort_path.path = utils::combinePath(aborting_path, reference_lane_segment);
    } else {
      abort_path.path = aborting_path;
    }
  }

  abort_path_ = std::make_shared<LaneChangePath>(abort_path);
  return true;
}

PathSafetyStatus NormalLaneChange::isLaneChangePathSafe(
  const LaneChangePath & lane_change_path, const ExtendedPredictedObjects & collision_check_objects,
  const utils::path_safety_checker::RSSparams & rss_params,
  CollisionCheckDebugMap & debug_data) const
{
  PathSafetyStatus path_safety_status;

  if (collision_check_objects.empty()) {
    RCLCPP_DEBUG(logger_, "There is nothing to check.");
    return path_safety_status;
  }

  const auto & path = lane_change_path.path;
  const auto & common_parameters = planner_data_->parameters;
  const auto current_pose = getEgoPose();
  const auto current_twist = getEgoTwist();

  if (path.points.empty()) {
    path_safety_status.is_safe = false;
    return path_safety_status;
  }

  const double & time_resolution = lane_change_parameters_->prediction_time_resolution;

  const auto ego_predicted_path = utils::lane_change::convertToPredictedPath(
    lane_change_path, current_twist, current_pose, common_parameters, *lane_change_parameters_,
    time_resolution);
  const auto debug_predicted_path =
    utils::path_safety_checker::convertToPredictedPath(ego_predicted_path, time_resolution);

  const auto current_lanes = getCurrentLanes();

  const auto expanded_target_lanes = utils::lane_change::generateExpandedLanelets(
    lane_change_path.info.target_lanes, direction_,
    lane_change_parameters_->lane_expansion_left_offset,
    lane_change_parameters_->lane_expansion_right_offset);

  constexpr double collision_check_yaw_diff_threshold{M_PI};

  for (const auto & obj : collision_check_objects) {
    auto current_debug_data = utils::path_safety_checker::createObjectDebug(obj);
    const auto obj_predicted_paths = utils::path_safety_checker::getPredictedPathFromObj(
      obj, lane_change_parameters_->use_all_predicted_path);
    auto is_safe = true;
    for (const auto & obj_path : obj_predicted_paths) {
      const auto collided_polygons = utils::path_safety_checker::getCollidedPolygons(
        path, ego_predicted_path, obj, obj_path, common_parameters, rss_params, 1.0,
        get_max_velocity_for_safety_check(), collision_check_yaw_diff_threshold,
        current_debug_data.second);

      if (collided_polygons.empty()) {
        utils::path_safety_checker::updateCollisionCheckDebugMap(
          debug_data, current_debug_data, is_safe);
        continue;
      }

      const auto collision_in_current_lanes = utils::lane_change::isCollidedPolygonsInLanelet(
        collided_polygons, lane_change_path.info.current_lanes);
      const auto collision_in_target_lanes =
        utils::lane_change::isCollidedPolygonsInLanelet(collided_polygons, expanded_target_lanes);

      if (!collision_in_current_lanes && !collision_in_target_lanes) {
        utils::path_safety_checker::updateCollisionCheckDebugMap(
          debug_data, current_debug_data, is_safe);
        continue;
      }

      is_safe = false;
      path_safety_status.is_safe = false;
      utils::path_safety_checker::updateCollisionCheckDebugMap(
        debug_data, current_debug_data, is_safe);
      const auto & obj_pose = obj.initial_pose.pose;
      const auto obj_polygon = autoware::universe_utils::toPolygon2d(obj_pose, obj.shape);
      path_safety_status.is_object_coming_from_rear |=
        !utils::path_safety_checker::isTargetObjectFront(
          path, current_pose, common_parameters.vehicle_info, obj_polygon);
    }
    utils::path_safety_checker::updateCollisionCheckDebugMap(
      debug_data, current_debug_data, is_safe);
  }

  return path_safety_status;
}

// Check if the ego vehicle is in stuck by a stationary obstacle or by the terminal of current lanes
bool NormalLaneChange::isVehicleStuck(
  const lanelet::ConstLanelets & current_lanes, const double obstacle_check_distance) const
{
  // Ego is still moving, not in stuck
  if (std::abs(getEgoVelocity()) > lane_change_parameters_->stop_velocity_threshold) {
    RCLCPP_DEBUG(logger_, "Ego is still moving, not in stuck");
    return false;
  }

  // Ego is just stopped, not sure it is in stuck yet.
  if (getStopTime() < lane_change_parameters_->stop_time_threshold) {
    RCLCPP_DEBUG(logger_, "Ego is just stopped, counting for stuck judge... (%f)", getStopTime());
    return false;
  }

  // Check if any stationary object exist in obstacle_check_distance
  using lanelet::utils::getArcCoordinates;
  const auto base_distance = getArcCoordinates(current_lanes, getEgoPose()).length;

  for (const auto & object : lane_change_debug_.filtered_objects.current_lane) {
    const auto & p = object.initial_pose.pose;  // TODO(Horibe): consider footprint point

    // Note: it needs chattering prevention.
    if (std::abs(object.initial_twist.twist.linear.x) > 0.3) {  // check if stationary
      continue;
    }

    const auto ego_to_obj_dist = getArcCoordinates(current_lanes, p).length - base_distance;
    if (0 < ego_to_obj_dist && ego_to_obj_dist < obstacle_check_distance) {
      RCLCPP_DEBUG(logger_, "Stationary object is in front of ego.");
      return true;  // Stationary object is in front of ego.
    }
  }

  // Check if Ego is in terminal of current lanes
  const auto & route_handler = getRouteHandler();
  const double distance_to_terminal =
    route_handler->isInGoalRouteSection(current_lanes.back())
      ? utils::getSignedDistance(getEgoPose(), route_handler->getGoalPose(), current_lanes)
      : utils::getDistanceToEndOfLane(getEgoPose(), current_lanes);
  const auto lane_change_buffer = calcMinimumLaneChangeLength(
    route_handler, current_lanes.back(), *lane_change_parameters_, Direction::NONE);
  const double stop_point_buffer = lane_change_parameters_->backward_length_buffer_for_end_of_lane;
  const double terminal_judge_buffer = lane_change_buffer + stop_point_buffer + 1.0;
  if (distance_to_terminal < terminal_judge_buffer) {
    return true;
  }

  // No stationary objects found in obstacle_check_distance and Ego is not in terminal of current
  RCLCPP_DEBUG(
    logger_,
    "No stationary objects found in obstacle_check_distance and Ego is not in "
    "terminal of current lanes");
  return false;
}

double NormalLaneChange::get_max_velocity_for_safety_check() const
{
  const auto external_velocity_limit_ptr = planner_data_->external_limit_max_velocity;
  if (external_velocity_limit_ptr) {
    return std::min(
      static_cast<double>(external_velocity_limit_ptr->max_velocity), getCommonParam().max_vel);
  }

  return getCommonParam().max_vel;
}

bool NormalLaneChange::isVehicleStuck(const lanelet::ConstLanelets & current_lanes) const
{
  if (current_lanes.empty()) {
    lane_change_debug_.is_stuck = false;
    return false;  // can not check
  }

  const auto [min_acc, max_acc] = calcCurrentMinMaxAcceleration();
  const auto max_lane_change_length = calcMaximumLaneChangeLength(current_lanes.back(), max_acc);
  const auto rss_dist = calcRssDistance(
    0.0, lane_change_parameters_->minimum_lane_changing_velocity,
    lane_change_parameters_->rss_params);

  // It is difficult to define the detection range. If it is too short, the stuck will not be
  // determined, even though you are stuck by an obstacle. If it is too long,
  // the ego will be judged to be stuck by a distant vehicle, even though the ego is only
  // stopped at a traffic light. Essentially, the calculation should be based on the information of
  // the stop reason, but this is outside the scope of one module. I keep it as a TODO.
  constexpr double DETECTION_DISTANCE_MARGIN = 10.0;
  const auto detection_distance = max_lane_change_length + rss_dist +
                                  getCommonParam().base_link2front + DETECTION_DISTANCE_MARGIN;
  RCLCPP_DEBUG(logger_, "max_lane_change_length: %f, max_acc: %f", max_lane_change_length, max_acc);

  auto is_vehicle_stuck = isVehicleStuck(current_lanes, detection_distance);

  lane_change_debug_.is_stuck = is_vehicle_stuck;
  return is_vehicle_stuck;
}

void NormalLaneChange::setStopPose(const Pose & stop_pose)
{
  lane_change_stop_pose_ = stop_pose;
}

void NormalLaneChange::updateStopTime()
{
  const auto current_vel = getEgoVelocity();

  if (std::abs(current_vel) > lane_change_parameters_->stop_velocity_threshold) {
    stop_time_ = 0.0;
  } else {
    const double duration = stop_watch_.toc("stop_time");
    // clip stop time
    if (stop_time_ + duration * 0.001 > lane_change_parameters_->stop_time_threshold) {
      constexpr double eps = 0.1;
      stop_time_ = lane_change_parameters_->stop_time_threshold + eps;
    } else {
      stop_time_ += duration * 0.001;
    }
  }

  stop_watch_.tic("stop_time");
}

bool NormalLaneChange::check_prepare_phase() const
{
  const auto & route_handler = getRouteHandler();
  const auto & vehicle_info = getCommonParam().vehicle_info;

  const auto check_in_general_lanes =
    lane_change_parameters_->enable_collision_check_for_prepare_phase_in_general_lanes;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(getEgoPose(), &current_lane)) {
    RCLCPP_DEBUG(
      logger_, "Unable to get current lane. Default to %s.",
      (check_in_general_lanes ? "true" : "false"));
    return check_in_general_lanes;
  }

  const auto ego_footprint = utils::lane_change::getEgoCurrentFootprint(getEgoPose(), vehicle_info);

  const auto check_in_intersection = std::invoke([&]() {
    if (!lane_change_parameters_->enable_collision_check_for_prepare_phase_in_intersection) {
      return false;
    }

    return utils::lane_change::isWithinIntersection(route_handler, current_lane, ego_footprint);
  });

  const auto check_in_turns = std::invoke([&]() {
    if (!lane_change_parameters_->enable_collision_check_for_prepare_phase_in_turns) {
      return false;
    }

    return utils::lane_change::isWithinTurnDirectionLanes(current_lane, ego_footprint);
  });

  return check_in_intersection || check_in_turns || check_in_general_lanes;
}
}  // namespace autoware::behavior_path_planner
