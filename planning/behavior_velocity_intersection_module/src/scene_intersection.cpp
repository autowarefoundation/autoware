// Copyright 2020 Tier IV, Inc.
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

#include "scene_intersection.hpp"

#include "util.hpp"

#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <magic_enum.hpp>
#include <opencv2/imgproc.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

static bool isTargetCollisionVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object)
{
  if (
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE ||
    object.classification.at(0).label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BICYCLE) {
    return true;
  }
  return false;
}

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const std::set<int> & associative_ids,
  const bool is_private_area, const bool enable_occlusion_detection, rclcpp::Node & node,
  const rclcpp::Logger logger, const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  node_(node),
  lane_id_(lane_id),
  associative_ids_(associative_ids),
  enable_occlusion_detection_(enable_occlusion_detection),
  occlusion_attention_divisions_(std::nullopt),
  is_private_area_(is_private_area),
  occlusion_uuid_(tier4_autoware_utils::generateUUID())
{
  velocity_factor_.init(VelocityFactor::INTERSECTION);
  planner_param_ = planner_param;

  const auto & assigned_lanelet =
    planner_data->route_handler_->getLaneletMapPtr()->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  collision_state_machine_.setMarginTime(
    planner_param_.collision_detection.state_transit_margin_time);
  {
    before_creep_state_machine_.setMarginTime(planner_param_.occlusion.before_creep_stop_time);
    before_creep_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    occlusion_stop_state_machine_.setMarginTime(planner_param_.occlusion.stop_release_margin_time);
    occlusion_stop_state_machine_.setState(StateMachine::State::GO);
  }
  {
    stuck_private_area_timeout_.setMarginTime(planner_param_.stuck_vehicle.timeout_private_area);
    stuck_private_area_timeout_.setState(StateMachine::State::STOP);
  }

  decision_state_pub_ =
    node_.create_publisher<std_msgs::msg::String>("~/debug/intersection/decision_state", 1);
}

void IntersectionModule::initializeRTCStatus()
{
  setSafe(true);
  setDistance(std::numeric_limits<double>::lowest());
  // occlusion
  occlusion_safety_ = true;
  occlusion_stop_distance_ = std::numeric_limits<double>::lowest();
  occlusion_first_stop_required_ = false;
  // activated_ and occlusion_activated_ must be set from manager's RTC callback
}

// template-specification based visitor pattern
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct VisitorSwitch : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
VisitorSwitch(Ts...) -> VisitorSwitch<Ts...>;

template <typename T>
void prepareRTCByDecisionResult(
  const T & result, const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  bool * default_safety, double * default_distance, bool * occlusion_safety,
  double * occlusion_distance, bool * occlusion_first_stop_required)
{
  static_assert("Unsupported type passed to prepareRTCByDecisionResult");
  return;
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const IntersectionModule::Indecisive & result,
  [[maybe_unused]] const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance,
  [[maybe_unused]] bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::StuckStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "StuckStop");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto stop_line_idx = result.stop_line_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stop_line_idx);
  *occlusion_safety = true;
  if (!result.is_detection_area_empty) {
    const auto occlusion_stop_line_idx = result.stop_lines.occlusion_peeking_stop_line;
    *occlusion_distance =
      motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stop_line_idx);
  }
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::NonOccludedCollisionStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "NonOccludedCollisionStop");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto stop_line_idx = result.stop_line_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stop_line_idx);
  const auto occlusion_stop_line = result.stop_lines.occlusion_peeking_stop_line;
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stop_line);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::FirstWaitBeforeOcclusion & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FirstWaitBeforeOcclusion");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto first_stop_line_idx = result.first_stop_line_idx;
  const auto occlusion_stop_line_idx = result.occlusion_stop_line_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, first_stop_line_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stop_line_idx);
  *occlusion_first_stop_required = true;
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::PeekingTowardOcclusion & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "PeekingTowardOcclusion");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto stop_line_idx = result.stop_line_idx;
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stop_line_idx);
  const auto default_stop_line_idx = result.stop_lines.default_stop_line;
  *default_safety = true;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, default_stop_line_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::OccludedCollisionStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedCollisionStop");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto stop_line_idx = result.stop_line_idx;
  const auto occlusion_stop_line_idx = result.occlusion_stop_line_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stop_line_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stop_line_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::Safe & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "Safe");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto default_stop_line_idx = result.stop_lines.default_stop_line;
  const auto occlusion_stop_line_idx = result.stop_lines.occlusion_peeking_stop_line;
  *default_safety = true;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, default_stop_line_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stop_line_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::TrafficLightArrowSolidOn & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance,
  [[maybe_unused]] bool * occlusion_first_stop_required)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "TrafficLightArrowSolidOn");
  const auto closest_idx = result.stop_lines.closest_idx;
  const auto default_stop_line_idx = result.stop_lines.default_stop_line;
  const auto occlusion_stop_line_idx = result.stop_lines.occlusion_peeking_stop_line;
  *default_safety = !result.collision_detected;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, default_stop_line_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stop_line_idx);
  return;
}

void IntersectionModule::prepareRTCStatus(
  const DecisionResult & decision_result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path)
{
  bool default_safety = true;
  double default_distance = std::numeric_limits<double>::lowest();
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      prepareRTCByDecisionResult(
        decision, path, &default_safety, &default_distance, &occlusion_safety_,
        &occlusion_stop_distance_, &occlusion_first_stop_required_);
    }},
    decision_result);
  setSafe(default_safety);
  setDistance(default_distance);
}

template <typename T>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved, const T & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  static_assert("Unsupported type passed to reactRTCByDecisionResult");
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  [[maybe_unused]] const bool rtc_default_approved,
  [[maybe_unused]] const bool rtc_occlusion_approved,
  [[maybe_unused]] const IntersectionModule::Indecisive & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  [[maybe_unused]] const double baselink2front,
  [[maybe_unused]] autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  [[maybe_unused]] StopReason * stop_reason,
  [[maybe_unused]] VelocityFactorInterface * velocity_factor,
  [[maybe_unused]] util::DebugData * debug_data)
{
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::StuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "StuckStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stop_line_idx = decision_result.stop_line_idx;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data->conflicting_targets);
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  if (
    !rtc_occlusion_approved && !decision_result.is_detection_area_empty &&
    planner_param.occlusion.enable) {
    const auto occlusion_stop_line_idx = decision_result.stop_lines.occlusion_peeking_stop_line;
    planning_utils::setVelocityFromIndex(occlusion_stop_line_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(occlusion_stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(occlusion_stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::NonOccludedCollisionStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "NonOccludedCollisionStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stop_line_idx = decision_result.stop_line_idx;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stop_line_idx = decision_result.stop_lines.occlusion_peeking_stop_line;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::FirstWaitBeforeOcclusion & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "FirstWaitBeforeOcclusion, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stop_line_idx = decision_result.first_stop_line_idx;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->occlusion_first_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    if (planner_param.occlusion.enable_creeping) {
      const size_t occlusion_peeking_stop_line = decision_result.occlusion_stop_line_idx;
      const size_t closest_idx = decision_result.stop_lines.closest_idx;
      for (size_t i = closest_idx; i < occlusion_peeking_stop_line; i++) {
        planning_utils::setVelocityFromIndex(
          i, planner_param.occlusion.occlusion_creep_velocity, path);
      }
    }
    const auto stop_line_idx = decision_result.occlusion_stop_line_idx;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::PeekingTowardOcclusion & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "PeekingTowardOcclusion, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  // NOTE: creep_velocity should be inserted first at closest_idx if !rtc_default_approved

  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const size_t occlusion_peeking_stop_line =
      decision_result.stop_lines.occlusion_peeking_stop_line;
    if (planner_param.occlusion.enable_creeping) {
      const size_t closest_idx = decision_result.stop_lines.closest_idx;
      for (size_t i = closest_idx; i < occlusion_peeking_stop_line; i++) {
        planning_utils::setVelocityFromIndex(
          i, planner_param.occlusion.occlusion_creep_velocity, path);
      }
    }
    planning_utils::setVelocityFromIndex(occlusion_peeking_stop_line, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_peeking_stop_line, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(occlusion_peeking_stop_line).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(occlusion_peeking_stop_line).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  if (!rtc_default_approved && planner_param.occlusion.enable) {
    const auto stop_line_idx = decision_result.stop_lines.default_stop_line;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::OccludedCollisionStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "OccludedCollisionStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stop_line_idx = decision_result.stop_line_idx;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stop_line_idx = decision_result.occlusion_stop_line_idx;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::Safe & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "Safe, approval = (default: %d, occlusion: %d)", rtc_default_approved, rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stop_line_idx = decision_result.stop_lines.default_stop_line;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stop_line_idx = decision_result.stop_lines.occlusion_peeking_stop_line;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::TrafficLightArrowSolidOn & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "TrafficLightArrowSolidOn, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stop_line_idx = decision_result.stop_lines.default_stop_line;
    planning_utils::setVelocityFromIndex(stop_line_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stop_line_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stop_line_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.stop_lines.closest_idx).point.pose,
        path->points.at(stop_line_idx).point.pose, VelocityFactor::INTERSECTION);
    }
  }
  return;
}

void reactRTCApproval(
  const bool rtc_default_approval, const bool rtc_occlusion_approval,
  const IntersectionModule::DecisionResult & decision_result,
  const IntersectionModule::PlannerParam & planner_param, const double baselink2front,
  autoware_auto_planning_msgs::msg::PathWithLaneId * path, StopReason * stop_reason,
  VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  std::visit(
    VisitorSwitch{[&](const auto & decision) {
      reactRTCApprovalByDecisionResult(
        rtc_default_approval, rtc_occlusion_approval, decision, planner_param, baselink2front, path,
        stop_reason, velocity_factor, debug_data);
    }},
    decision_result);
  return;
}

static std::string formatDecisionResult(const IntersectionModule::DecisionResult & decision_result)
{
  if (std::holds_alternative<IntersectionModule::Indecisive>(decision_result)) {
    return "Indecisive";
  }
  if (std::holds_alternative<IntersectionModule::Safe>(decision_result)) {
    return "Safe";
  }
  if (std::holds_alternative<IntersectionModule::StuckStop>(decision_result)) {
    return "StuckStop";
  }
  if (std::holds_alternative<IntersectionModule::NonOccludedCollisionStop>(decision_result)) {
    return "NonOccludedCollisionStop";
  }
  if (std::holds_alternative<IntersectionModule::FirstWaitBeforeOcclusion>(decision_result)) {
    return "FirstWaitBeforeOcclusion";
  }
  if (std::holds_alternative<IntersectionModule::PeekingTowardOcclusion>(decision_result)) {
    return "PeekingTowardOcclusion";
  }
  if (std::holds_alternative<IntersectionModule::OccludedCollisionStop>(decision_result)) {
    return "OccludedCollisionStop";
  }
  if (std::holds_alternative<IntersectionModule::TrafficLightArrowSolidOn>(decision_result)) {
    return "TrafficLightArrowSolidOn";
  }
  return "";
}

bool IntersectionModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  debug_data_ = util::DebugData();
  *stop_reason = planning_utils::initializeStopReason(StopReason::INTERSECTION);

  // set default RTC
  initializeRTCStatus();

  // calculate the
  const auto decision_result = modifyPathVelocityDetail(path, stop_reason);

  const std::string decision_type =
    "intersection" + std::to_string(module_id_) + " : " + formatDecisionResult(decision_result);
  std_msgs::msg::String decision_result_msg;
  decision_result_msg.data = decision_type;
  decision_state_pub_->publish(decision_result_msg);

  prepareRTCStatus(decision_result, *path);

  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  reactRTCApproval(
    activated_, occlusion_activated_, decision_result, planner_param_, baselink2front, path,
    stop_reason, &velocity_factor_, &debug_data_);

  if (!activated_ || !occlusion_activated_) {
    is_go_out_ = false;
  } else {
    is_go_out_ = true;
  }
  RCLCPP_DEBUG(logger_, "===== plan end =====");
  return true;
}

IntersectionModule::DecisionResult IntersectionModule::modifyPathVelocityDetail(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");

  // spline interpolation
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, *path, planner_param_.common.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    RCLCPP_DEBUG(logger_, "splineInterpolate failed");
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }
  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    RCLCPP_WARN(logger_, "Path has no interval on intersection lane %ld", lane_id_);
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }

  const auto & current_pose = planner_data_->current_odometry->pose;
  const auto lanelets_on_path =
    planning_utils::getLaneletsOnPath(*path, lanelet_map_ptr, current_pose);
  if (!intersection_lanelets_) {
    intersection_lanelets_ = util::getObjectiveLanelets(
      lanelet_map_ptr, routing_graph_ptr, assigned_lanelet, lanelets_on_path, associative_ids_,
      planner_param_.common.attention_area_length,
      planner_param_.occlusion.occlusion_attention_area_length,
      planner_param_.common.consider_wrong_direction_vehicle);
  }
  const bool tl_arrow_solid_on =
    util::isTrafficLightArrowActivated(assigned_lanelet, planner_data_->traffic_light_id_map);
  intersection_lanelets_.value().update(tl_arrow_solid_on, interpolated_path_info);

  const auto & conflicting_lanelets = intersection_lanelets_.value().conflicting();
  const auto & first_conflicting_area = intersection_lanelets_.value().first_conflicting_area();
  if (conflicting_lanelets.empty() || !first_conflicting_area) {
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }

  const auto & first_attention_area = intersection_lanelets_.value().first_attention_area();
  const auto & dummy_first_attention_area =
    first_attention_area ? first_attention_area.value() : first_conflicting_area.value();
  const auto intersection_stop_lines_opt = util::generateIntersectionStopLines(
    first_conflicting_area.value(), dummy_first_attention_area, planner_data_,
    interpolated_path_info, planner_param_.stuck_vehicle.use_stuck_stopline,
    planner_param_.common.stop_line_margin, planner_param_.occlusion.peeking_offset, path);
  if (!intersection_stop_lines_opt) {
    RCLCPP_DEBUG(logger_, "failed to generate intersection_stop_lines");
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }
  const auto & intersection_stop_lines = intersection_stop_lines_opt.value();
  const auto
    [closest_idx, stuck_stop_line_idx, default_stop_line_idx, occlusion_peeking_stop_line_idx,
     pass_judge_line_idx] = intersection_stop_lines;

  const auto path_lanelets_opt = util::generatePathLanelets(
    lanelets_on_path, *path, associative_ids_, closest_idx,
    planner_data_->vehicle_info_.vehicle_width_m);
  if (!path_lanelets_opt.has_value()) {
    RCLCPP_DEBUG(logger_, "failed to generate PathLanelets");
    return IntersectionModule::Indecisive{};
  }
  const auto path_lanelets = path_lanelets_opt.value();

  const auto ego_lane_with_next_lane =
    path_lanelets.next.has_value()
      ? std::vector<
          lanelet::ConstLanelet>{path_lanelets.ego_or_entry2exit, path_lanelets.next.value()}
      : std::vector<lanelet::ConstLanelet>{path_lanelets.ego_or_entry2exit};
  const bool stuck_detected =
    checkStuckVehicle(planner_data_, ego_lane_with_next_lane, *path, intersection_stop_lines);

  if (stuck_detected) {
    const double dist_stopline = motion_utils::calcSignedArcLength(
      path->points, path->points.at(closest_idx).point.pose.position,
      path->points.at(stuck_stop_line_idx).point.pose.position);
    const bool approached_stop_line =
      (std::fabs(dist_stopline) < planner_param_.common.stop_overshoot_margin);
    const bool is_stopped = planner_data_->isVehicleStopped();
    if (is_stopped && approached_stop_line) {
      stuck_private_area_timeout_.setStateWithMarginTime(
        StateMachine::State::GO, logger_.get_child("stuck_private_area_timeout"), *clock_);
    }
    const bool timeout =
      (is_private_area_ && stuck_private_area_timeout_.getState() == StateMachine::State::GO);
    if (!timeout) {
      is_peeking_ = false;
      return IntersectionModule::StuckStop{
        stuck_stop_line_idx, !first_attention_area.has_value(), intersection_stop_lines};
    }
  }

  if (!first_attention_area) {
    RCLCPP_DEBUG(logger_, "attention area is empty");
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }

  if (default_stop_line_idx == 0) {
    RCLCPP_DEBUG(logger_, "stop line index is 0");
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }

  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  debug_data_.pass_judge_wall_pose =
    planning_utils::getAheadPose(pass_judge_line_idx, baselink2front, *path);
  const bool is_over_pass_judge_line =
    util::isOverTargetIndex(*path, closest_idx, current_pose, pass_judge_line_idx);
  const bool is_over_default_stop_line =
    util::isOverTargetIndex(*path, closest_idx, current_pose, default_stop_line_idx);
  const double vel_norm = std::hypot(
    planner_data_->current_velocity->twist.linear.x,
    planner_data_->current_velocity->twist.linear.y);
  const bool keep_detection =
    (vel_norm < planner_param_.collision_detection.keep_detection_vel_thr);
  // if ego is over the pass judge line and not stopped
  if (is_peeking_) {
    // do nothing
    RCLCPP_DEBUG(logger_, "peeking now");
  } else if (is_over_default_stop_line && !is_over_pass_judge_line && keep_detection) {
    RCLCPP_DEBUG(
      logger_, "is_over_default_stop_line && !is_over_pass_judge_line && keep_detection");
    // do nothing
  } else if (
    (is_over_default_stop_line && is_over_pass_judge_line && is_go_out_) || is_permanent_go_) {
    // is_go_out_: previous RTC approval
    // activated_: current RTC approval
    is_permanent_go_ = true;
    RCLCPP_DEBUG(logger_, "over the pass judge line. no plan needed.");
    is_peeking_ = false;
    return IntersectionModule::Indecisive{};
  }

  const auto & attention_lanelets = intersection_lanelets_.value().attention();
  const auto & adjacent_lanelets = intersection_lanelets_.value().adjacent();
  const auto & occlusion_attention_lanelets = intersection_lanelets_.value().occlusion_attention();
  const auto & occlusion_attention_area = intersection_lanelets_.value().occlusion_attention_area();
  debug_data_.attention_area = intersection_lanelets_.value().attention_area();
  debug_data_.adjacent_area = intersection_lanelets_.value().adjacent_area();

  // get intersection area
  const auto intersection_area = planner_param_.common.use_intersection_area
                                   ? util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr)
                                   : std::nullopt;
  if (intersection_area) {
    const auto intersection_area_2d = intersection_area.value();
    debug_data_.intersection_area = toGeomPoly(intersection_area_2d);
  }

  // calculate dynamic collision around detection area
  const double time_delay = (is_go_out_ || tl_arrow_solid_on)
                              ? 0.0
                              : (planner_param_.collision_detection.state_transit_margin_time -
                                 collision_state_machine_.getDuration());
  const auto target_objects =
    filterTargetObjects(attention_lanelets, adjacent_lanelets, intersection_area);

  const bool has_collision = checkCollision(
    *path, target_objects, path_lanelets, closest_idx, time_delay, tl_arrow_solid_on);
  collision_state_machine_.setStateWithMarginTime(
    has_collision ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);
  const bool has_collision_with_margin =
    collision_state_machine_.getState() == StateMachine::State::STOP;

  if (tl_arrow_solid_on) {
    is_peeking_ = false;
    return TrafficLightArrowSolidOn{has_collision, intersection_stop_lines};
  }

  // check occlusion on detection lane
  if (!occlusion_attention_divisions_) {
    occlusion_attention_divisions_ = util::generateDetectionLaneDivisions(
      occlusion_attention_lanelets, routing_graph_ptr,
      planner_data_->occupancy_grid->info.resolution / std::sqrt(2.0));
  }

  const double occlusion_dist_thr = std::fabs(
    std::pow(planner_param_.occlusion.max_vehicle_velocity_for_rss, 2) /
    (2 * planner_param_.occlusion.min_vehicle_brake_for_rss));
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> parked_attention_objects;
  std::copy_if(
    target_objects.objects.begin(), target_objects.objects.end(),
    std::back_inserter(parked_attention_objects),
    [thresh = planner_param_.occlusion.ignore_parked_vehicle_speed_threshold](const auto & object) {
      return std::hypot(
               object.kinematics.initial_twist_with_covariance.twist.linear.x,
               object.kinematics.initial_twist_with_covariance.twist.linear.y) <= thresh;
    });
  const bool is_occlusion_cleared =
    (enable_occlusion_detection_ && !occlusion_attention_lanelets.empty() && !tl_arrow_solid_on)
      ? isOcclusionCleared(
          *planner_data_->occupancy_grid, occlusion_attention_area, adjacent_lanelets,
          first_attention_area.value(), interpolated_path_info,
          occlusion_attention_divisions_.value(), parked_attention_objects, occlusion_dist_thr)
      : true;
  occlusion_stop_state_machine_.setStateWithMarginTime(
    is_occlusion_cleared ? StateMachine::State::GO : StateMachine::STOP,
    logger_.get_child("occlusion_stop"), *clock_);

  // check safety
  const bool ext_occlusion_requested = (is_occlusion_cleared && !occlusion_activated_);
  if (
    occlusion_stop_state_machine_.getState() == StateMachine::State::STOP ||
    ext_occlusion_requested) {
    const double dist_stopline = motion_utils::calcSignedArcLength(
      path->points, path->points.at(closest_idx).point.pose.position,
      path->points.at(default_stop_line_idx).point.pose.position);
    const bool approached_stop_line =
      (std::fabs(dist_stopline) < planner_param_.common.stop_overshoot_margin);
    const bool over_stop_line = (dist_stopline < 0.0);
    const bool is_stopped =
      planner_data_->isVehicleStopped(planner_param_.occlusion.before_creep_stop_time);
    if (over_stop_line) {
      before_creep_state_machine_.setState(StateMachine::State::GO);
    }
    if (before_creep_state_machine_.getState() == StateMachine::State::GO) {
      if (has_collision) {
        is_peeking_ = true;
        return IntersectionModule::OccludedCollisionStop{
          default_stop_line_idx, occlusion_peeking_stop_line_idx, is_occlusion_cleared,
          intersection_stop_lines};
      } else {
        is_peeking_ = true;
        return IntersectionModule::PeekingTowardOcclusion{
          occlusion_peeking_stop_line_idx, is_occlusion_cleared, intersection_stop_lines};
      }
    } else {
      if (is_stopped && approached_stop_line) {
        // start waiting at the first stop line
        before_creep_state_machine_.setState(StateMachine::State::GO);
      }
      is_peeking_ = true;
      return IntersectionModule::FirstWaitBeforeOcclusion{
        default_stop_line_idx, occlusion_peeking_stop_line_idx, is_occlusion_cleared,
        intersection_stop_lines};
    }
  } else if (has_collision_with_margin) {
    const bool is_over_default_stopLine =
      util::isOverTargetIndex(*path, closest_idx, current_pose, default_stop_line_idx);
    const auto stop_line_idx = is_over_default_stopLine ? closest_idx : default_stop_line_idx;
    is_peeking_ = false;
    return IntersectionModule::NonOccludedCollisionStop{stop_line_idx, intersection_stop_lines};
  }

  is_peeking_ = false;
  return IntersectionModule::Safe{intersection_stop_lines};
}

bool IntersectionModule::checkStuckVehicle(
  const std::shared_ptr<const PlannerData> & planner_data,
  const lanelet::ConstLanelets & ego_lane_with_next_lane,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path,
  const util::IntersectionStopLines & intersection_stop_lines)
{
  const auto & objects_ptr = planner_data->predicted_objects;
  const geometry_msgs::msg::Pose & current_pose = planner_data->current_odometry->pose;
  const auto closest_idx = intersection_stop_lines.closest_idx;
  const auto stuck_line_idx = intersection_stop_lines.stuck_stop_line;

  // considering lane change in the intersection, these lanelets are generated from the path
  const auto ego_lane = ego_lane_with_next_lane.front();
  debug_data_.ego_lane = ego_lane.polygon3d();
  const auto stuck_vehicle_detect_area = util::generateStuckVehicleDetectAreaPolygon(
    input_path, ego_lane_with_next_lane, closest_idx,
    planner_param_.stuck_vehicle.stuck_vehicle_detect_dist,
    planner_param_.stuck_vehicle.stuck_vehicle_ignore_dist,
    planner_data->vehicle_info_.vehicle_length_m);
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);

  const double dist_stuck_stopline = motion_utils::calcSignedArcLength(
    input_path.points, input_path.points.at(stuck_line_idx).point.pose.position,
    input_path.points.at(closest_idx).point.pose.position);
  const bool is_over_stuck_stopline =
    util::isOverTargetIndex(input_path, closest_idx, current_pose, stuck_line_idx) &&
    (dist_stuck_stopline > planner_param_.common.stop_overshoot_margin);

  bool is_stuck = false;
  if (!is_over_stuck_stopline) {
    is_stuck = util::checkStuckVehicleInIntersection(
      objects_ptr, stuck_vehicle_detect_area, planner_param_.stuck_vehicle.stuck_vehicle_vel_thr,
      &debug_data_);
  }
  return is_stuck;
}

autoware_auto_perception_msgs::msg::PredictedObjects IntersectionModule::filterTargetObjects(
  const lanelet::ConstLanelets & attention_area_lanelets,
  const lanelet::ConstLanelets & adjacent_lanelets,
  const std::optional<Polygon2d> & intersection_area) const
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  const auto & objects_ptr = planner_data_->predicted_objects;
  // extract target objects
  autoware_auto_perception_msgs::msg::PredictedObjects target_objects;
  target_objects.header = objects_ptr->header;
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetCollisionVehicleType(object)) {
      continue;
    }

    // check direction of objects
    const auto object_direction = util::getObjectPoseWithVelocityDirection(object.kinematics);
    const auto is_in_adjacent_lanelets = util::checkAngleForTargetLanelets(
      object_direction, adjacent_lanelets, planner_param_.common.attention_area_angle_thr,
      planner_param_.common.consider_wrong_direction_vehicle,
      planner_param_.common.attention_area_margin);
    if (is_in_adjacent_lanelets) {
      continue;
    }

    if (intersection_area) {
      const auto obj_poly = tier4_autoware_utils::toPolygon2d(object);
      const auto intersection_area_2d = intersection_area.value();
      const auto is_in_intersection_area = bg::within(obj_poly, intersection_area_2d);
      if (is_in_intersection_area) {
        target_objects.objects.push_back(object);
      } else if (util::checkAngleForTargetLanelets(
                   object_direction, attention_area_lanelets,
                   planner_param_.common.attention_area_angle_thr,
                   planner_param_.common.consider_wrong_direction_vehicle,
                   planner_param_.common.attention_area_margin)) {
        target_objects.objects.push_back(object);
      }
    } else if (util::checkAngleForTargetLanelets(
                 object_direction, attention_area_lanelets,
                 planner_param_.common.attention_area_angle_thr,
                 planner_param_.common.consider_wrong_direction_vehicle,
                 planner_param_.common.attention_area_margin)) {
      // intersection_area is not available, use detection_area_with_margin as before
      target_objects.objects.push_back(object);
    }
  }
  return target_objects;
}

bool IntersectionModule::checkCollision(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const autoware_auto_perception_msgs::msg::PredictedObjects & objects,
  const util::PathLanelets & path_lanelets, const int closest_idx, const double time_delay,
  const bool tl_arrow_solid_on)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  // check collision between target_objects predicted path and ego lane
  // cut the predicted path at passing_time
  const auto time_distance_array = util::calcIntersectionPassingTime(
    path, planner_data_, associative_ids_, closest_idx, time_delay,
    planner_param_.common.intersection_velocity,
    planner_param_.collision_detection.minimum_ego_predicted_velocity);
  const double passing_time = time_distance_array.back().first;
  auto target_objects = objects;
  util::cutPredictPathWithDuration(&target_objects, clock_, passing_time);

  const auto & concat_lanelets = path_lanelets.all;
  const auto closest_arc_coords = getArcCoordinates(
    concat_lanelets, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const auto & ego_lane = path_lanelets.ego_or_entry2exit;

  const auto ego_poly = ego_lane.polygon2d().basicPolygon();
  // check collision between predicted_path and ego_area
  const double collision_start_margin_time =
    tl_arrow_solid_on ? planner_param_.collision_detection.relaxed.collision_start_margin_time
                      : planner_param_.collision_detection.normal.collision_start_margin_time;
  const double collision_end_margin_time =
    tl_arrow_solid_on ? planner_param_.collision_detection.relaxed.collision_end_margin_time
                      : planner_param_.collision_detection.normal.collision_end_margin_time;
  bool collision_detected = false;
  for (const auto & object : target_objects.objects) {
    for (const auto & predicted_path : object.kinematics.predicted_paths) {
      if (
        predicted_path.confidence <
        planner_param_.collision_detection.min_predicted_path_confidence) {
        // ignore the predicted path with too low confidence
        continue;
      }

      // collision point
      const auto first_itr = std::adjacent_find(
        predicted_path.path.cbegin(), predicted_path.path.cend(),
        [&ego_poly](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, LineString2d{to_bg2d(a), to_bg2d(b)});
        });
      if (first_itr == predicted_path.path.cend()) continue;
      const auto last_itr = std::adjacent_find(
        predicted_path.path.crbegin(), predicted_path.path.crend(),
        [&ego_poly](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, LineString2d{to_bg2d(a), to_bg2d(b)});
        });
      if (last_itr == predicted_path.path.crend()) continue;

      // possible collision time interval
      const double ref_object_enter_time =
        static_cast<double>(first_itr - predicted_path.path.begin()) *
        rclcpp::Duration(predicted_path.time_step).seconds();
      auto start_time_distance_itr = time_distance_array.begin();
      if (ref_object_enter_time - collision_start_margin_time > 0) {
        // start of possible ego position in the intersection
        start_time_distance_itr = std::lower_bound(
          time_distance_array.begin(), time_distance_array.end(),
          ref_object_enter_time - collision_start_margin_time,
          [](const auto & a, const double b) { return a.first < b; });
        if (start_time_distance_itr == time_distance_array.end()) {
          // ego is already at the exit of intersection when npc is at collision point even if npc
          // accelerates so ego's position interval is empty
          continue;
        }
      }
      const double ref_object_exit_time =
        static_cast<double>(last_itr.base() - predicted_path.path.begin()) *
        rclcpp::Duration(predicted_path.time_step).seconds();
      auto end_time_distance_itr = std::lower_bound(
        time_distance_array.begin(), time_distance_array.end(),
        ref_object_exit_time + collision_end_margin_time,
        [](const auto & a, const double b) { return a.first < b; });
      if (end_time_distance_itr == time_distance_array.end()) {
        // ego is already passing the intersection, when npc is is at collision point
        // so ego's position interval is up to the end of intersection lane
        end_time_distance_itr = time_distance_array.end() - 1;
      }
      const double start_arc_length = std::max(
        0.0, closest_arc_coords.length + (*start_time_distance_itr).second -
               planner_data_->vehicle_info_.rear_overhang_m);
      const double end_arc_length = std::min(
        closest_arc_coords.length + (*end_time_distance_itr).second +
          planner_data_->vehicle_info_.max_longitudinal_offset_m,
        lanelet::utils::getLaneletLength2d(concat_lanelets));

      const auto trimmed_ego_polygon =
        getPolygonFromArcLength(concat_lanelets, start_arc_length, end_arc_length);

      if (trimmed_ego_polygon.empty()) {
        continue;
      }

      Polygon2d polygon{};
      for (const auto & p : trimmed_ego_polygon) {
        polygon.outer().emplace_back(p.x(), p.y());
      }
      bg::correct(polygon);
      debug_data_.candidate_collision_ego_lane_polygon = toGeomPoly(polygon);

      for (auto itr = first_itr; itr != last_itr.base(); ++itr) {
        const auto footprint_polygon = tier4_autoware_utils::toPolygon2d(*itr, object.shape);
        if (bg::intersects(polygon, footprint_polygon)) {
          collision_detected = true;
          break;
        }
      }
      if (collision_detected) {
        debug_data_.conflicting_targets.objects.push_back(object);
        break;
      }
    }
  }

  return collision_detected;
}

bool IntersectionModule::isOcclusionCleared(
  const nav_msgs::msg::OccupancyGrid & occ_grid,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
  const lanelet::ConstLanelets & adjacent_lanelets,
  const lanelet::CompoundPolygon3d & first_attention_area,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const std::vector<util::DiscretizedLane> & lane_divisions,
  const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & parked_attention_objects,
  const double occlusion_dist_thr)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();

  const auto first_attention_area_idx =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_attention_area);
  if (!first_attention_area_idx) {
    return false;
  }

  const auto first_inside_attention_idx_ip_opt =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_attention_area);
  const std::pair<size_t, size_t> lane_attention_interval_ip =
    first_inside_attention_idx_ip_opt
      ? std::make_pair(first_inside_attention_idx_ip_opt.value(), std::get<1>(lane_interval_ip))
      : lane_interval_ip;

  const int width = occ_grid.info.width;
  const int height = occ_grid.info.height;
  const double resolution = occ_grid.info.resolution;
  const auto & origin = occ_grid.info.origin.position;

  // NOTE: interesting area is set to 0 for later masking
  cv::Mat attention_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::Mat unknown_mask(width, height, CV_8UC1, cv::Scalar(0));

  // (1) prepare detection area mask
  // attention: 255
  // non-attention: 0
  Polygon2d grid_poly;
  grid_poly.outer().emplace_back(origin.x, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * resolution, origin.y);
  grid_poly.outer().emplace_back(
    origin.x + (width - 1) * resolution, origin.y + (height - 1) * resolution);
  grid_poly.outer().emplace_back(origin.x, origin.y + (height - 1) * resolution);
  grid_poly.outer().emplace_back(origin.x, origin.y);
  bg::correct(grid_poly);

  std::vector<std::vector<cv::Point>> attention_area_cv_polygons;
  for (const auto & attention_area : attention_areas) {
    const auto area2d = lanelet::utils::to2D(attention_area);
    Polygon2d area2d_poly;
    for (const auto & p : area2d) {
      area2d_poly.outer().emplace_back(p.x(), p.y());
    }
    area2d_poly.outer().push_back(area2d_poly.outer().front());
    bg::correct(area2d_poly);
    std::vector<Polygon2d> common_areas;
    bg::intersection(area2d_poly, grid_poly, common_areas);
    if (common_areas.empty()) {
      continue;
    }
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> attention_area_cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = static_cast<int>((p.x() - origin.x) / resolution);
        const int idx_y = static_cast<int>((p.y() - origin.y) / resolution);
        attention_area_cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      attention_area_cv_polygons.push_back(attention_area_cv_polygon);
    }
  }
  for (const auto & poly : attention_area_cv_polygons) {
    cv::fillPoly(attention_mask, poly, cv::Scalar(255), cv::LINE_AA);
  }
  // (1.1)
  // reset adjacent_lanelets area to 0 on attention_mask
  std::vector<std::vector<cv::Point>> adjacent_lane_cv_polygons;
  for (const auto & adjacent_lanelet : adjacent_lanelets) {
    const auto area2d = adjacent_lanelet.polygon2d().basicPolygon();
    Polygon2d area2d_poly;
    for (const auto & p : area2d) {
      area2d_poly.outer().emplace_back(p.x(), p.y());
    }
    area2d_poly.outer().push_back(area2d_poly.outer().front());
    bg::correct(area2d_poly);
    std::vector<Polygon2d> common_areas;
    bg::intersection(area2d_poly, grid_poly, common_areas);
    if (common_areas.empty()) {
      continue;
    }
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> adjacent_lane_cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = std::floor<int>((p.x() - origin.x) / resolution);
        const int idx_y = std::floor<int>((p.y() - origin.y) / resolution);
        adjacent_lane_cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      adjacent_lane_cv_polygons.push_back(adjacent_lane_cv_polygon);
    }
  }
  for (const auto & poly : adjacent_lane_cv_polygons) {
    cv::fillPoly(attention_mask, poly, cv::Scalar(0), cv::LINE_AA);
  }

  // (2) prepare unknown mask
  // In OpenCV the pixel at (X=x, Y=y) (with left-upper origin) is accessed by img[y, x]
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      const int idx = y * width + x;
      const unsigned char intensity = occ_grid.data.at(idx);
      if (
        planner_param_.occlusion.free_space_max <= intensity &&
        intensity < planner_param_.occlusion.occupied_min) {
        unknown_mask.at<unsigned char>(height - 1 - y, x) = 255;
      }
    }
  }

  // (3) occlusion mask
  cv::Mat occlusion_mask_raw(width, height, CV_8UC1, cv::Scalar(0));
  cv::bitwise_and(attention_mask, unknown_mask, occlusion_mask_raw);
  // (3.1) apply morphologyEx
  cv::Mat occlusion_mask;
  const int morph_size = static_cast<int>(planner_param_.occlusion.denoise_kernel / resolution);
  cv::morphologyEx(
    occlusion_mask_raw, occlusion_mask, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_size, morph_size)));

  // (4) create distance grid
  // value: 0 - 254: signed distance representing [distance_min, distance_max]
  // 255: undefined value
  const double distance_max = std::hypot(width * resolution / 2, height * resolution / 2);
  const double distance_min = -distance_max;
  const int undef_pixel = 255;
  const int max_cost_pixel = 254;
  auto dist2pixel = [=](const double dist) {
    return std::min(
      max_cost_pixel,
      static_cast<int>((dist - distance_min) / (distance_max - distance_min) * max_cost_pixel));
  };
  auto pixel2dist = [=](const int pixel) {
    return pixel * 1.0 / max_cost_pixel * (distance_max - distance_min) + distance_min;
  };
  const int zero_dist_pixel = dist2pixel(0.0);
  const int parked_vehicle_pixel = zero_dist_pixel - 1;  // magic

  auto coord2index = [&](const double x, const double y) {
    const int idx_x = (x - origin.x) / resolution;
    const int idx_y = (y - origin.y) / resolution;
    if (idx_x < 0 || idx_x >= width) return std::make_tuple(false, -1, -1);
    if (idx_y < 0 || idx_y >= height) return std::make_tuple(false, -1, -1);
    return std::make_tuple(true, idx_x, idx_y);
  };

  cv::Mat distance_grid(width, height, CV_8UC1, cv::Scalar(undef_pixel));
  cv::Mat projection_ind_grid(width, height, CV_32S, cv::Scalar(-1));

  // (4.1) fill zero_dist_pixel on the path
  const auto [lane_start, lane_end] = lane_attention_interval_ip;
  for (int i = static_cast<int>(lane_end); i >= static_cast<int>(lane_start); i--) {
    const auto & path_pos = path_ip.points.at(i).point.pose.position;
    const int idx_x = (path_pos.x - origin.x) / resolution;
    const int idx_y = (path_pos.y - origin.y) / resolution;
    if (idx_x < 0 || idx_x >= width) continue;
    if (idx_y < 0 || idx_y >= height) continue;
    distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = zero_dist_pixel;
    projection_ind_grid.at<int>(height - 1 - idx_y, idx_x) = i;
  }

  // (4.2) fill parked_vehicle_pixel to parked_vehicles (both positive and negative)
  for (const auto & parked_attention_object : parked_attention_objects) {
    const auto obj_poly = tier4_autoware_utils::toPolygon2d(parked_attention_object);
    std::vector<Polygon2d> common_areas;
    bg::intersection(obj_poly, grid_poly, common_areas);
    if (common_areas.empty()) continue;
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    std::vector<std::vector<cv::Point>> parked_attention_object_area_cv_polygons;
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> parked_attention_object_area_cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = static_cast<int>((p.x() - origin.x) / resolution);
        const int idx_y = static_cast<int>((p.y() - origin.y) / resolution);
        parked_attention_object_area_cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      parked_attention_object_area_cv_polygons.push_back(parked_attention_object_area_cv_polygon);
    }
    for (const auto & poly : parked_attention_object_area_cv_polygons) {
      cv::fillPoly(distance_grid, poly, cv::Scalar(parked_vehicle_pixel), cv::LINE_AA);
    }
  }

  for (const auto & lane_division : lane_divisions) {
    const auto & divisions = lane_division.divisions;
    for (const auto & division : divisions) {
      bool is_in_grid = false;
      bool zero_dist_cell_found = false;
      int projection_ind = -1;
      std::optional<std::tuple<double, double, double, int>> cost_prev_checkpoint =
        std::nullopt;  // cost, x, y, projection_ind
      for (auto point = division.begin(); point != division.end(); point++) {
        const double x = point->x(), y = point->y();
        const auto [valid, idx_x, idx_y] = coord2index(x, y);
        // exited grid just now
        if (is_in_grid && !valid) break;

        // still not entering grid
        if (!is_in_grid && !valid) continue;

        // From here, "valid"
        const int pixel = distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x);

        // entered grid for 1st time
        if (!is_in_grid) {
          assert(pixel == undef_pixel || pixel == zero_dist_pixel);
          is_in_grid = true;
          if (pixel == undef_pixel) {
            continue;
          }
        }

        if (pixel == zero_dist_pixel) {
          zero_dist_cell_found = true;
          projection_ind = projection_ind_grid.at<int>(height - 1 - idx_y, idx_x);
          assert(projection_ind >= 0);
          cost_prev_checkpoint =
            std::make_optional<std::tuple<double, double, double, int>>(0.0, x, y, projection_ind);
          continue;
        }

        // hit positive parked vehicle
        if (zero_dist_cell_found && pixel == parked_vehicle_pixel) {
          while (point != division.end()) {
            const double x = point->x(), y = point->y();
            const auto [valid, idx_x, idx_y] = coord2index(x, y);
            if (valid) occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x) = 0;
            point++;
          }
          break;
        }

        if (zero_dist_cell_found) {
          // finally traversed to defined cell (first half)
          const auto [prev_cost, prev_checkpoint_x, prev_checkpoint_y, prev_projection_ind] =
            cost_prev_checkpoint.value();
          const double dy = y - prev_checkpoint_y, dx = x - prev_checkpoint_x;
          double new_dist = prev_cost + std::hypot(dy, dx);
          const int new_projection_ind = projection_ind_grid.at<int>(height - 1 - idx_y, idx_x);
          const double cur_dist = pixel2dist(pixel);
          if (planner_param_.occlusion.do_dp && cur_dist < new_dist) {
            new_dist = cur_dist;
            if (new_projection_ind > 0) {
              projection_ind = std::min<int>(prev_projection_ind, new_projection_ind);
            }
          }
          projection_ind_grid.at<int>(height - 1 - idx_y, idx_x) = projection_ind;
          distance_grid.at<unsigned char>(height - 1 - idx_y, idx_x) = dist2pixel(new_dist);
          cost_prev_checkpoint = std::make_optional<std::tuple<double, double, double, int>>(
            new_dist, x, y, projection_ind);
        }
      }
    }
  }

  const auto & possible_object_bbox = planner_param_.occlusion.possible_object_bbox;
  const double possible_object_bbox_x = possible_object_bbox.at(0) / resolution;
  const double possible_object_bbox_y = possible_object_bbox.at(1) / resolution;
  const double possible_object_area = possible_object_bbox_x * possible_object_bbox_y;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(occlusion_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::vector<std::vector<cv::Point>> valid_contours;
  for (const auto & contour : contours) {
    std::vector<cv::Point> valid_contour;
    for (const auto & p : contour) {
      if (distance_grid.at<unsigned char>(p.y, p.x) == undef_pixel) {
        continue;
      }
      valid_contour.push_back(p);
    }
    if (valid_contour.size() <= 2) {
      continue;
    }
    std::vector<cv::Point> approx_contour;
    cv::approxPolyDP(
      valid_contour, approx_contour,
      std::round(std::min(possible_object_bbox_x, possible_object_bbox_y) / std::sqrt(2.0)), true);
    if (approx_contour.size() <= 2) continue;
    // check area
    const double poly_area = cv::contourArea(approx_contour);
    if (poly_area < possible_object_area) continue;
    // check bounding box size
    const auto bbox = cv::minAreaRect(approx_contour);
    if (const auto size = bbox.size; std::min(size.height, size.width) <
                                       std::min(possible_object_bbox_x, possible_object_bbox_y) ||
                                     std::max(size.height, size.width) <
                                       std::max(possible_object_bbox_x, possible_object_bbox_y)) {
      continue;
    }
    valid_contours.push_back(approx_contour);
    geometry_msgs::msg::Polygon polygon_msg;
    geometry_msgs::msg::Point32 point_msg;
    for (const auto & p : approx_contour) {
      const double glob_x = (p.x + 0.5) * resolution + origin.x;
      const double glob_y = (height - 0.5 - p.y) * resolution + origin.y;
      point_msg.x = glob_x;
      point_msg.y = glob_y;
      point_msg.z = origin.z;
      polygon_msg.points.push_back(point_msg);
    }
    debug_data_.occlusion_polygons.push_back(polygon_msg);
  }

  const int min_cost_thr = dist2pixel(occlusion_dist_thr);
  int min_cost = undef_pixel - 1;
  geometry_msgs::msg::Point nearest_occlusion_point;
  for (const auto & occlusion_contour : valid_contours) {
    for (const auto & p : occlusion_contour) {
      const int pixel = static_cast<int>(distance_grid.at<unsigned char>(p.y, p.x));
      const bool occluded = (occlusion_mask.at<unsigned char>(p.y, p.x) == 255);
      if (pixel == undef_pixel || !occluded) {
        continue;
      }
      if (pixel < min_cost) {
        min_cost = pixel;
        nearest_occlusion_point.x = origin.x + p.x * resolution;
        nearest_occlusion_point.y = origin.y + (height - 1 - p.y) * resolution;
        nearest_occlusion_point.z = origin.z;
      }
    }
  }

  if (min_cost > min_cost_thr) {
    return true;
  } else {
    debug_data_.nearest_occlusion_point = nearest_occlusion_point;
    return false;
  }
}

/*
bool IntersectionModule::checkFrontVehicleDeceleration(
  lanelet::ConstLanelets & ego_lane_with_next_lane, lanelet::ConstLanelet & closest_lanelet,
  const Polygon2d & stuck_vehicle_detect_area,
  const autoware_auto_perception_msgs::msg::PredictedObject & object,
  const double assumed_front_car_decel)
{
  const auto & object_pose = object.kinematics.initial_pose_with_covariance.pose;
  // consider vehicle in ego-lane && in front of ego
  const auto lon_vel = object.kinematics.initial_twist_with_covariance.twist.linear.x;
  const double object_decel =
    planner_param_.stuck_vehicle.assumed_front_car_decel;  // NOTE: this is positive
  const double stopping_distance = lon_vel * lon_vel / (2 * object_decel);

  std::vector<geometry_msgs::msg::Point> center_points;
  for (auto && p : ego_lane_with_next_lane[0].centerline())
    center_points.push_back(std::move(lanelet::utils::conversion::toGeomMsgPt(p)));
  for (auto && p : ego_lane_with_next_lane[1].centerline())
    center_points.push_back(std::move(lanelet::utils::conversion::toGeomMsgPt(p)));
  const double lat_offset =
    std::fabs(motion_utils::calcLateralOffset(center_points, object_pose.position));
  // get the nearest centerpoint to object
  std::vector<double> dist_obj_center_points;
  for (const auto & p : center_points)
    dist_obj_center_points.push_back(tier4_autoware_utils::calcDistance2d(object_pose.position,
p)); const int obj_closest_centerpoint_idx = std::distance( dist_obj_center_points.begin(),
    std::min_element(dist_obj_center_points.begin(), dist_obj_center_points.end()));
  // find two center_points whose distances from `closest_centerpoint` cross stopping_distance
  double acc_dist_prev = 0.0, acc_dist = 0.0;
  auto p1 = center_points[obj_closest_centerpoint_idx];
  auto p2 = center_points[obj_closest_centerpoint_idx];
  for (unsigned i = obj_closest_centerpoint_idx; i < center_points.size() - 1; ++i) {
    p1 = center_points[i];
    p2 = center_points[i + 1];
    acc_dist_prev = acc_dist;
    const auto arc_position_p1 =
      lanelet::utils::getArcCoordinates(ego_lane_with_next_lane, toPose(p1));
    const auto arc_position_p2 =
      lanelet::utils::getArcCoordinates(ego_lane_with_next_lane, toPose(p2));
    const double delta = arc_position_p2.length - arc_position_p1.length;
    acc_dist += delta;
    if (acc_dist > stopping_distance) {
      break;
    }
  }
  // if stopping_distance >= center_points, stopping_point is center_points[end]
  const double ratio = (acc_dist <= stopping_distance)
                         ? 0.0
                         : (acc_dist - stopping_distance) / (stopping_distance - acc_dist_prev);
  // linear interpolation
  geometry_msgs::msg::Point stopping_point;
  stopping_point.x = (p1.x * ratio + p2.x) / (1 + ratio);
  stopping_point.y = (p1.y * ratio + p2.y) / (1 + ratio);
  stopping_point.z = (p1.z * ratio + p2.z) / (1 + ratio);
  const double lane_yaw = lanelet::utils::getLaneletAngle(closest_lanelet, stopping_point);
  stopping_point.x += lat_offset * std::cos(lane_yaw + M_PI / 2.0);
  stopping_point.y += lat_offset * std::sin(lane_yaw + M_PI / 2.0);

  // calculate footprint of predicted stopping pose
  autoware_auto_perception_msgs::msg::PredictedObject predicted_object = object;
  predicted_object.kinematics.initial_pose_with_covariance.pose.position = stopping_point;
  predicted_object.kinematics.initial_pose_with_covariance.pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(0, 0, lane_yaw);
  auto predicted_obj_footprint = tier4_autoware_utils::toPolygon2d(predicted_object);
  const bool is_in_stuck_area = !bg::disjoint(predicted_obj_footprint, stuck_vehicle_detect_area);
  debug_data_.predicted_obj_pose.position = stopping_point;
  debug_data_.predicted_obj_pose.orientation =
    tier4_autoware_utils::createQuaternionFromRPY(0, 0, lane_yaw);

  if (is_in_stuck_area) {
    return true;
  }
  return false;
}
*/

}  // namespace behavior_velocity_planner
