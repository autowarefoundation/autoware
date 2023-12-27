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
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <opencv2/imgproc.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <vector>
namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

namespace
{
Polygon2d createOneStepPolygon(
  const geometry_msgs::msg::Pose & prev_pose, const geometry_msgs::msg::Pose & next_pose,
  const autoware_auto_perception_msgs::msg::Shape & shape)
{
  const auto prev_poly = tier4_autoware_utils::toPolygon2d(prev_pose, shape);
  const auto next_poly = tier4_autoware_utils::toPolygon2d(next_pose, shape);

  Polygon2d one_step_poly;
  for (const auto & point : prev_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }
  for (const auto & point : next_poly.outer()) {
    one_step_poly.outer().push_back(point);
  }

  bg::correct(one_step_poly);

  Polygon2d convex_one_step_poly;
  bg::convex_hull(one_step_poly, convex_one_step_poly);

  return convex_one_step_poly;
}
}  // namespace

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
  const int64_t module_id, const int64_t lane_id,
  [[maybe_unused]] std::shared_ptr<const PlannerData> planner_data,
  const PlannerParam & planner_param, const std::set<lanelet::Id> & associative_ids,
  const std::string & turn_direction, const bool has_traffic_light,
  const bool enable_occlusion_detection, rclcpp::Node & node, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  node_(node),
  lane_id_(lane_id),
  associative_ids_(associative_ids),
  turn_direction_(turn_direction),
  has_traffic_light_(has_traffic_light),
  enable_occlusion_detection_(enable_occlusion_detection),
  occlusion_attention_divisions_(std::nullopt),
  occlusion_uuid_(tier4_autoware_utils::generateUUID())
{
  velocity_factor_.init(PlanningBehavior::INTERSECTION);
  planner_param_ = planner_param;

  {
    collision_state_machine_.setMarginTime(
      planner_param_.collision_detection.collision_detection_hold_time);
  }
  {
    before_creep_state_machine_.setMarginTime(
      planner_param_.occlusion.temporal_stop_time_before_peeking);
    before_creep_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    occlusion_stop_state_machine_.setMarginTime(
      planner_param_.occlusion.occlusion_detection_hold_time);
    occlusion_stop_state_machine_.setState(StateMachine::State::GO);
  }
  {
    temporal_stop_before_attention_state_machine_.setMarginTime(
      planner_param_.occlusion.occlusion_detection_hold_time);
    temporal_stop_before_attention_state_machine_.setState(StateMachine::State::STOP);
  }
  {
    static_occlusion_timeout_state_machine_.setMarginTime(
      planner_param_.occlusion.static_occlusion_with_traffic_light_timeout);
    static_occlusion_timeout_state_machine_.setState(StateMachine::State::STOP);
  }

  decision_state_pub_ =
    node_.create_publisher<std_msgs::msg::String>("~/debug/intersection/decision_state", 1);
  ego_ttc_pub_ = node_.create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>(
    "~/debug/intersection/ego_ttc", 1);
  object_ttc_pub_ = node_.create_publisher<tier4_debug_msgs::msg::Float64MultiArrayStamped>(
    "~/debug/intersection/object_ttc", 1);
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
  double * occlusion_distance)
{
  static_assert("Unsupported type passed to prepareRTCByDecisionResult");
  return;
}

template <>
void prepareRTCByDecisionResult(
  [[maybe_unused]] const IntersectionModule::Indecisive & result,
  [[maybe_unused]] const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  [[maybe_unused]] bool * default_safety, [[maybe_unused]] double * default_distance,
  [[maybe_unused]] bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::StuckStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "StuckStop");
  const auto closest_idx = result.closest_idx;
  const auto stopline_idx = result.stuck_stopline_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stopline_idx);
  *occlusion_safety = true;
  if (result.occlusion_stopline_idx) {
    const auto occlusion_stopline_idx = result.occlusion_stopline_idx.value();
    *occlusion_distance =
      motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  }
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::YieldStuckStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, [[maybe_unused]] double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "YieldStuckStop");
  const auto closest_idx = result.closest_idx;
  const auto stopline_idx = result.stuck_stopline_idx;
  *default_safety = false;
  *default_distance = motion_utils::calcSignedArcLength(path.points, closest_idx, stopline_idx);
  *occlusion_safety = true;
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::NonOccludedCollisionStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "NonOccludedCollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  const auto occlusion_stopline = result.occlusion_stopline_idx;
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::FirstWaitBeforeOcclusion & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FirstWaitBeforeOcclusion");
  const auto closest_idx = result.closest_idx;
  const auto first_stopline_idx = result.first_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, first_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::PeekingTowardOcclusion & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "PeekingTowardOcclusion");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = true;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::OccludedAbsenceTrafficLight & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedAbsenceTrafficLight");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.closest_idx;
  *default_safety = !result.collision_detected;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance = 0;
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::OccludedCollisionStop & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "OccludedCollisionStop");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = false;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = result.is_actually_occlusion_cleared;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::Safe & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "Safe");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = true;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
  return;
}

template <>
void prepareRTCByDecisionResult(
  const IntersectionModule::FullyPrioritized & result,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, bool * default_safety,
  double * default_distance, bool * occlusion_safety, double * occlusion_distance)
{
  RCLCPP_DEBUG(rclcpp::get_logger("prepareRTCByDecisionResult"), "FullyPrioritized");
  const auto closest_idx = result.closest_idx;
  const auto collision_stopline_idx = result.collision_stopline_idx;
  const auto occlusion_stopline_idx = result.occlusion_stopline_idx;
  *default_safety = !result.collision_detected;
  *default_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, collision_stopline_idx);
  *occlusion_safety = true;
  *occlusion_distance =
    motion_utils::calcSignedArcLength(path.points, closest_idx, occlusion_stopline_idx);
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
        &occlusion_stop_distance_);
    }},
    decision_result);
  setSafe(default_safety);
  setDistance(default_distance);
  occlusion_first_stop_required_ =
    std::holds_alternative<IntersectionModule::FirstWaitBeforeOcclusion>(decision_result);
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
  const auto closest_idx = decision_result.closest_idx;
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stopline_idx = decision_result.stuck_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data->conflicting_targets);
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (
    !rtc_occlusion_approved && decision_result.occlusion_stopline_idx &&
    planner_param.occlusion.enable) {
    const auto occlusion_stopline_idx = decision_result.occlusion_stopline_idx.value();
    planning_utils::setVelocityFromIndex(occlusion_stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(occlusion_stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(closest_idx).point.pose,
        path->points.at(occlusion_stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::YieldStuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "YieldStuckStop, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  const auto closest_idx = decision_result.closest_idx;
  if (!rtc_default_approved) {
    // use default_rtc uuid for stuck vehicle detection
    const auto stopline_idx = decision_result.stuck_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      stop_factor.stop_factor_points = planning_utils::toRosPoints(debug_data->conflicting_targets);
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
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
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
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
    const auto stopline_idx = decision_result.first_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_first_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    if (planner_param.occlusion.creep_during_peeking.enable) {
      const size_t occlusion_peeking_stopline = decision_result.occlusion_stopline_idx;
      const size_t closest_idx = decision_result.closest_idx;
      for (size_t i = closest_idx; i < occlusion_peeking_stopline; i++) {
        planning_utils::setVelocityFromIndex(
          i, planner_param.occlusion.creep_during_peeking.creep_velocity, path);
      }
    }
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
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
    const size_t occlusion_peeking_stopline =
      decision_result.temporal_stop_before_attention_required
        ? decision_result.first_attention_stopline_idx
        : decision_result.occlusion_stopline_idx;
    if (planner_param.occlusion.creep_during_peeking.enable) {
      const size_t closest_idx = decision_result.closest_idx;
      for (size_t i = closest_idx; i < occlusion_peeking_stopline; i++) {
        planning_utils::setVelocityFromIndex(
          i, planner_param.occlusion.creep_during_peeking.creep_velocity, path);
      }
    }
    planning_utils::setVelocityFromIndex(occlusion_peeking_stopline, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(occlusion_peeking_stopline, baselink2front, *path);
    debug_data->static_occlusion_with_traffic_light_timeout =
      decision_result.static_occlusion_timeout;
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(occlusion_peeking_stopline).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(occlusion_peeking_stopline).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
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
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.temporal_stop_before_attention_required
                                ? decision_result.first_attention_stopline_idx
                                : decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    debug_data->static_occlusion_with_traffic_light_timeout =
      decision_result.static_occlusion_timeout;
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::OccludedAbsenceTrafficLight & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "OccludedAbsenceTrafficLight, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.closest_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && decision_result.temporal_stop_before_attention_required) {
    const auto stopline_idx = decision_result.first_attention_area_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && !decision_result.temporal_stop_before_attention_required) {
    const auto closest_idx = decision_result.closest_idx;
    const auto peeking_limit_line = decision_result.peeking_limit_line_idx;
    for (auto i = closest_idx; i <= peeking_limit_line; ++i) {
      planning_utils::setVelocityFromIndex(
        i, planner_param.occlusion.creep_velocity_without_traffic_light, path);
    }
    debug_data->absence_traffic_light_creep_wall =
      planning_utils::getAheadPose(closest_idx, baselink2front, *path);
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
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::FullyPrioritized & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, util::DebugData * debug_data)
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("reactRTCApprovalByDecisionResult"),
    "FullyPrioritized, approval = (default: %d, occlusion: %d)", rtc_default_approved,
    rtc_occlusion_approved);
  if (!rtc_default_approved) {
    const auto stopline_idx = decision_result.collision_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->collision_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
    }
  }
  if (!rtc_occlusion_approved && planner_param.occlusion.enable) {
    const auto stopline_idx = decision_result.occlusion_stopline_idx;
    planning_utils::setVelocityFromIndex(stopline_idx, 0.0, path);
    debug_data->occlusion_stop_wall_pose =
      planning_utils::getAheadPose(stopline_idx, baselink2front, *path);
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = path->points.at(stopline_idx).point.pose;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor->set(
        path->points, path->points.at(decision_result.closest_idx).point.pose,
        path->points.at(stopline_idx).point.pose, VelocityFactor::UNKNOWN);
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
    const auto indecisive = std::get<IntersectionModule::Indecisive>(decision_result);
    return "Indecisive because " + indecisive.error;
  }
  if (std::holds_alternative<IntersectionModule::Safe>(decision_result)) {
    return "Safe";
  }
  if (std::holds_alternative<IntersectionModule::StuckStop>(decision_result)) {
    return "StuckStop";
  }
  if (std::holds_alternative<IntersectionModule::YieldStuckStop>(decision_result)) {
    return "YieldStuckStop";
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
  if (std::holds_alternative<IntersectionModule::OccludedAbsenceTrafficLight>(decision_result)) {
    return "OccludedAbsenceTrafficLight";
  }
  if (std::holds_alternative<IntersectionModule::FullyPrioritized>(decision_result)) {
    return "FullyPrioritized";
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
  prev_decision_result_ = decision_result;

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

static bool isGreenSolidOn(
  lanelet::ConstLanelet lane, const std::map<int, TrafficSignalStamped> & tl_infos)
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;

  std::optional<int> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  if (!tl_id) {
    // this lane has no traffic light
    return false;
  }
  const auto tl_info_it = tl_infos.find(tl_id.value());
  if (tl_info_it == tl_infos.end()) {
    // the info of this traffic light is not available
    return false;
  }
  const auto & tl_info = tl_info_it->second;
  for (auto && tl_light : tl_info.signal.elements) {
    if (
      tl_light.color == TrafficSignalElement::GREEN &&
      tl_light.shape == TrafficSignalElement::CIRCLE) {
      return true;
    }
  }
  return false;
}

IntersectionModule::DecisionResult IntersectionModule::modifyPathVelocityDetail(
  PathWithLaneId * path, [[maybe_unused]] StopReason * stop_reason)
{
  const auto lanelet_map_ptr = planner_data_->route_handler_->getLaneletMapPtr();
  const auto routing_graph_ptr = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id_);
  const std::string turn_direction = assigned_lanelet.attributeOr("turn_direction", "else");
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  // spline interpolation
  const auto interpolated_path_info_opt = util::generateInterpolatedPath(
    lane_id_, associative_ids_, *path, planner_param_.common.path_interpolation_ds, logger_);
  if (!interpolated_path_info_opt) {
    return IntersectionModule::Indecisive{"splineInterpolate failed"};
  }
  const auto & interpolated_path_info = interpolated_path_info_opt.value();
  if (!interpolated_path_info.lane_id_interval) {
    return IntersectionModule::Indecisive{
      "Path has no interval on intersection lane " + std::to_string(lane_id_)};
  }

  // cache intersection lane information because it is invariant
  const auto & current_pose = planner_data_->current_odometry->pose;
  const auto lanelets_on_path =
    planning_utils::getLaneletsOnPath(*path, lanelet_map_ptr, current_pose);
  if (!intersection_lanelets_) {
    intersection_lanelets_ = util::getObjectiveLanelets(
      lanelet_map_ptr, routing_graph_ptr, assigned_lanelet, lanelets_on_path, associative_ids_,
      planner_param_.common.attention_area_length,
      planner_param_.occlusion.occlusion_attention_area_length,
      planner_param_.collision_detection.consider_wrong_direction_vehicle);
  }
  auto & intersection_lanelets = intersection_lanelets_.value();

  // at the very first time of registration of this module, the path may not be conflicting with the
  // attention area, so update() is called to update the internal data as well as traffic light info
  const auto traffic_prioritized_level =
    util::getTrafficPrioritizedLevel(assigned_lanelet, planner_data_->traffic_light_id_map);
  const bool is_prioritized =
    traffic_prioritized_level == util::TrafficPrioritizedLevel::FULLY_PRIORITIZED;
  const auto footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  intersection_lanelets.update(is_prioritized, interpolated_path_info, footprint, baselink2front);

  // this is abnormal
  const auto & conflicting_lanelets = intersection_lanelets.conflicting();
  const auto & first_conflicting_area_opt = intersection_lanelets.first_conflicting_area();
  const auto & first_conflicting_lane_opt = intersection_lanelets.first_conflicting_lane();
  if (conflicting_lanelets.empty() || !first_conflicting_area_opt || !first_conflicting_lane_opt) {
    return IntersectionModule::Indecisive{"conflicting area is empty"};
  }
  const auto & first_conflicting_lane = first_conflicting_lane_opt.value();
  const auto & first_conflicting_area = first_conflicting_area_opt.value();

  // generate all stop line candidates
  // see the doc for struct IntersectionStopLines
  const auto & first_attention_area_opt = intersection_lanelets.first_attention_area();
  /// even if the attention area is null, stuck vehicle stop line needs to be generated from
  /// conflicting lanes
  const auto & dummy_first_attention_area =
    first_attention_area_opt ? first_attention_area_opt.value() : first_conflicting_area;
  const auto & dummy_first_attention_lane_centerline =
    intersection_lanelets.first_attention_lane()
      ? intersection_lanelets.first_attention_lane().value().centerline2d()
      : first_conflicting_lane.centerline2d();
  const auto intersection_stoplines_opt = util::generateIntersectionStopLines(
    first_conflicting_area, dummy_first_attention_area, dummy_first_attention_lane_centerline,
    planner_data_, interpolated_path_info, planner_param_.stuck_vehicle.use_stuck_stopline,
    planner_param_.common.default_stopline_margin, planner_param_.common.max_accel,
    planner_param_.common.max_jerk, planner_param_.common.delay_response_time,
    planner_param_.occlusion.peeking_offset, path);
  if (!intersection_stoplines_opt) {
    return IntersectionModule::Indecisive{"failed to generate intersection_stoplines"};
  }
  const auto & intersection_stoplines = intersection_stoplines_opt.value();
  const auto
    [closest_idx, stuck_stopline_idx_opt, default_stopline_idx_opt,
     first_attention_stopline_idx_opt, occlusion_peeking_stopline_idx_opt,
     default_pass_judge_line_idx, occlusion_wo_tl_pass_judge_line_idx] = intersection_stoplines;

  // see the doc for struct PathLanelets
  const auto & conflicting_area = intersection_lanelets.conflicting_area();
  const auto path_lanelets_opt = util::generatePathLanelets(
    lanelets_on_path, interpolated_path_info, associative_ids_, first_conflicting_area,
    conflicting_area, first_attention_area_opt, intersection_lanelets.attention_area(), closest_idx,
    planner_data_->vehicle_info_.vehicle_width_m);
  if (!path_lanelets_opt.has_value()) {
    return IntersectionModule::Indecisive{"failed to generate PathLanelets"};
  }
  const auto path_lanelets = path_lanelets_opt.value();

  // utility functions
  auto fromEgoDist = [&](const size_t index) {
    return motion_utils::calcSignedArcLength(path->points, closest_idx, index);
  };
  auto stoppedForDuration =
    [&](const size_t pos, const double duration, StateMachine & state_machine) {
      const double dist_stopline = fromEgoDist(pos);
      const bool approached_dist_stopline =
        (std::fabs(dist_stopline) < planner_param_.common.stopline_overshoot_margin);
      const bool over_stopline = (dist_stopline < 0.0);
      const bool is_stopped_duration = planner_data_->isVehicleStopped(duration);
      if (over_stopline) {
        state_machine.setState(StateMachine::State::GO);
      } else if (is_stopped_duration && approached_dist_stopline) {
        state_machine.setState(StateMachine::State::GO);
      }
      return state_machine.getState() == StateMachine::State::GO;
    };
  auto stoppedAtPosition = [&](const size_t pos, const double duration) {
    const double dist_stopline = fromEgoDist(pos);
    const bool approached_dist_stopline =
      (std::fabs(dist_stopline) < planner_param_.common.stopline_overshoot_margin);
    const bool over_stopline = (dist_stopline < -planner_param_.common.stopline_overshoot_margin);
    const bool is_stopped = planner_data_->isVehicleStopped(duration);
    if (over_stopline) {
      return true;
    } else if (is_stopped && approached_dist_stopline) {
      return true;
    }
    return false;
  };

  // stuck vehicle detection is viable even if attention area is empty
  // so this needs to be checked before attention area validation
  const bool stuck_detected = checkStuckVehicle(planner_data_, path_lanelets);
  const bool is_first_conflicting_lane_private =
    (std::string(first_conflicting_lane.attributeOr("location", "else")).compare("private") == 0);
  if (stuck_detected) {
    if (
      is_first_conflicting_lane_private &&
      planner_param_.stuck_vehicle.disable_against_private_lane) {
      // do nothing
    } else {
      std::optional<size_t> stopline_idx = std::nullopt;
      if (stuck_stopline_idx_opt) {
        const bool is_over_stuck_stopline = fromEgoDist(stuck_stopline_idx_opt.value()) <
                                            -planner_param_.common.stopline_overshoot_margin;
        if (!is_over_stuck_stopline) {
          stopline_idx = stuck_stopline_idx_opt.value();
        }
      }
      if (!stopline_idx) {
        if (default_stopline_idx_opt && fromEgoDist(default_stopline_idx_opt.value()) >= 0.0) {
          stopline_idx = default_stopline_idx_opt.value();
        } else if (
          first_attention_stopline_idx_opt &&
          fromEgoDist(first_attention_stopline_idx_opt.value()) >= 0.0) {
          stopline_idx = closest_idx;
        }
      }
      if (stopline_idx) {
        return IntersectionModule::StuckStop{
          closest_idx, stopline_idx.value(), occlusion_peeking_stopline_idx_opt};
      }
    }
  }

  // if attention area is empty, collision/occlusion detection is impossible
  if (!first_attention_area_opt) {
    return IntersectionModule::Indecisive{"attention area is empty"};
  }
  const auto first_attention_area = first_attention_area_opt.value();

  // if attention area is not null but default stop line is not available, ego/backward-path has
  // already passed the stop line
  if (!default_stopline_idx_opt) {
    return IntersectionModule::Indecisive{"default stop line is null"};
  }
  // occlusion stop line is generated from the intersection of ego footprint along the path with the
  // attention area, so if this is null, eog has already passed the intersection
  if (!first_attention_stopline_idx_opt || !occlusion_peeking_stopline_idx_opt) {
    return IntersectionModule::Indecisive{"occlusion stop line is null"};
  }
  const auto default_stopline_idx = default_stopline_idx_opt.value();
  const bool is_over_default_stopline =
    util::isOverTargetIndex(*path, closest_idx, current_pose, default_stopline_idx);
  const auto collision_stopline_idx = is_over_default_stopline ? closest_idx : default_stopline_idx;
  const auto first_attention_stopline_idx = first_attention_stopline_idx_opt.value();
  const auto occlusion_stopline_idx = occlusion_peeking_stopline_idx_opt.value();

  const auto & adjacent_lanelets = intersection_lanelets.adjacent();
  const auto & occlusion_attention_lanelets = intersection_lanelets.occlusion_attention();
  const auto & occlusion_attention_area = intersection_lanelets.occlusion_attention_area();
  debug_data_.attention_area = intersection_lanelets.attention_area();
  debug_data_.occlusion_attention_area = occlusion_attention_area;
  debug_data_.adjacent_area = intersection_lanelets.adjacent_area();

  // check occlusion on detection lane
  if (!occlusion_attention_divisions_) {
    occlusion_attention_divisions_ = util::generateDetectionLaneDivisions(
      occlusion_attention_lanelets, routing_graph_ptr,
      planner_data_->occupancy_grid->info.resolution,
      planner_param_.occlusion.attention_lane_crop_curvature_threshold,
      planner_param_.occlusion.attention_lane_curvature_calculation_ds);
  }
  const auto & occlusion_attention_divisions = occlusion_attention_divisions_.value();

  // get intersection area
  const auto intersection_area = util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr);
  // filter objects
  auto target_objects = generateTargetObjects(intersection_lanelets, intersection_area);

  const bool yield_stuck_detected = checkYieldStuckVehicle(
    target_objects, interpolated_path_info, intersection_lanelets.attention_non_preceding_);
  if (yield_stuck_detected) {
    std::optional<size_t> stopline_idx = std::nullopt;
    const bool is_before_default_stopline = fromEgoDist(default_stopline_idx) >= 0.0;
    const bool is_before_first_attention_stopline =
      fromEgoDist(first_attention_stopline_idx) >= 0.0;
    if (stuck_stopline_idx_opt) {
      const bool is_over_stuck_stopline = fromEgoDist(stuck_stopline_idx_opt.value()) <
                                          -planner_param_.common.stopline_overshoot_margin;
      if (!is_over_stuck_stopline) {
        stopline_idx = stuck_stopline_idx_opt.value();
      }
    }
    if (!stopline_idx) {
      if (is_before_default_stopline) {
        stopline_idx = default_stopline_idx;
      } else if (is_before_first_attention_stopline) {
        stopline_idx = closest_idx;
      }
    }
    if (stopline_idx) {
      return IntersectionModule::YieldStuckStop{closest_idx, stopline_idx.value()};
    }
  }

  const double is_amber_or_red =
    (traffic_prioritized_level == util::TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) ||
    (traffic_prioritized_level == util::TrafficPrioritizedLevel::FULLY_PRIORITIZED);
  auto occlusion_status =
    (enable_occlusion_detection_ && !occlusion_attention_lanelets.empty() && !is_amber_or_red)
      ? getOcclusionStatus(
          *planner_data_->occupancy_grid, occlusion_attention_area, adjacent_lanelets,
          first_attention_area, interpolated_path_info, occlusion_attention_divisions,
          target_objects, current_pose,
          planner_param_.occlusion.occlusion_required_clearance_distance)
      : OcclusionType::NOT_OCCLUDED;
  occlusion_stop_state_machine_.setStateWithMarginTime(
    occlusion_status == OcclusionType::NOT_OCCLUDED ? StateMachine::State::GO : StateMachine::STOP,
    logger_.get_child("occlusion_stop"), *clock_);
  const bool is_occlusion_cleared_with_margin =
    (occlusion_stop_state_machine_.getState() == StateMachine::State::GO);
  // distinguish if ego detected occlusion or RTC detects occlusion
  const bool ext_occlusion_requested = (is_occlusion_cleared_with_margin && !occlusion_activated_);
  if (ext_occlusion_requested) {
    occlusion_status = OcclusionType::RTC_OCCLUDED;
  }
  const bool is_occlusion_state = (!is_occlusion_cleared_with_margin || ext_occlusion_requested);
  if (is_occlusion_state && occlusion_status == OcclusionType::NOT_OCCLUDED) {
    occlusion_status = prev_occlusion_status_;
  } else {
    prev_occlusion_status_ = occlusion_status;
  }

  // TODO(Mamoru Sobue): this part needs more formal handling
  const size_t pass_judge_line_idx = [=]() {
    if (enable_occlusion_detection_) {
      // if occlusion detection is enabled, pass_judge position is beyond the boundary of first
      // attention area
      if (has_traffic_light_) {
        return occlusion_stopline_idx;
      } else if (is_occlusion_state) {
        // if there is no traffic light and occlusion is detected, pass_judge position is beyond
        // the boundary of first attention area
        return occlusion_wo_tl_pass_judge_line_idx;
      } else {
        // if there is no traffic light and occlusion is not detected, pass_judge position is
        // default
        return default_pass_judge_line_idx;
      }
    }
    return default_pass_judge_line_idx;
  }();
  debug_data_.pass_judge_wall_pose =
    planning_utils::getAheadPose(pass_judge_line_idx, baselink2front, *path);
  const bool is_over_pass_judge_line =
    util::isOverTargetIndex(*path, closest_idx, current_pose, pass_judge_line_idx);
  const double vel_norm = std::hypot(
    planner_data_->current_velocity->twist.linear.x,
    planner_data_->current_velocity->twist.linear.y);
  const bool keep_detection =
    (vel_norm < planner_param_.collision_detection.keep_detection_velocity_threshold);
  const bool was_safe = std::holds_alternative<IntersectionModule::Safe>(prev_decision_result_);
  // if ego is over the pass judge line and not stopped
  if (is_over_default_stopline && !is_over_pass_judge_line && keep_detection) {
    RCLCPP_DEBUG(logger_, "is_over_default_stopline && !is_over_pass_judge_line && keep_detection");
    // do nothing
  } else if (
    (was_safe && is_over_default_stopline && is_over_pass_judge_line && is_go_out_) ||
    is_permanent_go_) {
    // is_go_out_: previous RTC approval
    // activated_: current RTC approval
    is_permanent_go_ = true;
    return IntersectionModule::Indecisive{"over the pass judge line. no plan needed"};
  }

  // If there are any vehicles on the attention area when ego entered the intersection on green
  // light, do pseudo collision detection because the vehicles are very slow and no collisions may
  // be detected. check if ego vehicle entered assigned lanelet
  const bool is_green_solid_on =
    isGreenSolidOn(assigned_lanelet, planner_data_->traffic_light_id_map);
  if (is_green_solid_on) {
    if (!initial_green_light_observed_time_) {
      const auto assigned_lane_begin_point = assigned_lanelet.centerline().front();
      const bool approached_assigned_lane =
        motion_utils::calcSignedArcLength(
          path->points, closest_idx,
          tier4_autoware_utils::createPoint(
            assigned_lane_begin_point.x(), assigned_lane_begin_point.y(),
            assigned_lane_begin_point.z())) <
        planner_param_.collision_detection.yield_on_green_traffic_light
          .distance_to_assigned_lanelet_start;
      if (approached_assigned_lane) {
        initial_green_light_observed_time_ = clock_->now();
      }
    }
    if (initial_green_light_observed_time_) {
      const auto now = clock_->now();
      const bool exist_close_vehicles = std::any_of(
        target_objects.all_attention_objects.begin(), target_objects.all_attention_objects.end(),
        [&](const auto & object) {
          return object.dist_to_stopline.has_value() &&
                 object.dist_to_stopline.value() <
                   planner_param_.collision_detection.yield_on_green_traffic_light
                     .object_dist_to_stopline;
        });
      if (
        exist_close_vehicles &&
        rclcpp::Duration((now - initial_green_light_observed_time_.value())).seconds() <
          planner_param_.collision_detection.yield_on_green_traffic_light.duration) {
        return IntersectionModule::NonOccludedCollisionStop{
          closest_idx, collision_stopline_idx, occlusion_stopline_idx};
      }
    }
  }

  // calculate dynamic collision around attention area
  const double time_to_restart =
    (is_go_out_ || is_prioritized)
      ? 0.0
      : (planner_param_.collision_detection.collision_detection_hold_time -
         collision_state_machine_.getDuration());

  const bool has_collision = checkCollision(
    *path, &target_objects, path_lanelets, closest_idx,
    std::min<size_t>(occlusion_stopline_idx, path->points.size() - 1), time_to_restart,
    traffic_prioritized_level);
  collision_state_machine_.setStateWithMarginTime(
    has_collision ? StateMachine::State::STOP : StateMachine::State::GO,
    logger_.get_child("collision state_machine"), *clock_);
  const bool has_collision_with_margin =
    collision_state_machine_.getState() == StateMachine::State::STOP;

  if (is_prioritized) {
    return FullyPrioritized{
      has_collision_with_margin, closest_idx, collision_stopline_idx, occlusion_stopline_idx};
  }

  // Safe
  if (!is_occlusion_state && !has_collision_with_margin) {
    return IntersectionModule::Safe{closest_idx, collision_stopline_idx, occlusion_stopline_idx};
  }
  // Only collision
  if (!is_occlusion_state && has_collision_with_margin) {
    return IntersectionModule::NonOccludedCollisionStop{
      closest_idx, collision_stopline_idx, occlusion_stopline_idx};
  }
  // Occluded
  // occlusion_status is assured to be not NOT_OCCLUDED
  const bool stopped_at_default_line = stoppedForDuration(
    default_stopline_idx, planner_param_.occlusion.temporal_stop_time_before_peeking,
    before_creep_state_machine_);
  if (stopped_at_default_line) {
    // if specified the parameter occlusion.temporal_stop_before_attention_area OR
    // has_no_traffic_light_, ego will temporarily stop before entering attention area
    const bool temporal_stop_before_attention_required =
      (planner_param_.occlusion.temporal_stop_before_attention_area || !has_traffic_light_)
        ? !stoppedForDuration(
            first_attention_stopline_idx,
            planner_param_.occlusion.temporal_stop_time_before_peeking,
            temporal_stop_before_attention_state_machine_)
        : false;
    if (!has_traffic_light_) {
      if (fromEgoDist(occlusion_wo_tl_pass_judge_line_idx) < 0) {
        return IntersectionModule::Indecisive{
          "already passed maximum peeking line in the absence of traffic light"};
      }
      return IntersectionModule::OccludedAbsenceTrafficLight{
        is_occlusion_cleared_with_margin,
        has_collision_with_margin,
        temporal_stop_before_attention_required,
        closest_idx,
        first_attention_stopline_idx,
        occlusion_wo_tl_pass_judge_line_idx};
    }
    // following remaining block is "has_traffic_light_"
    // if ego is stuck by static occlusion in the presence of traffic light, start timeout count
    const bool is_static_occlusion = occlusion_status == OcclusionType::STATICALLY_OCCLUDED;
    const bool is_stuck_by_static_occlusion =
      stoppedAtPosition(
        occlusion_stopline_idx, planner_param_.occlusion.temporal_stop_time_before_peeking) &&
      is_static_occlusion;
    if (has_collision_with_margin) {
      // if collision is detected, timeout is reset
      static_occlusion_timeout_state_machine_.setState(StateMachine::State::STOP);
    } else if (is_stuck_by_static_occlusion) {
      static_occlusion_timeout_state_machine_.setStateWithMarginTime(
        StateMachine::State::GO, logger_.get_child("static_occlusion"), *clock_);
    }
    const bool release_static_occlusion_stuck =
      (static_occlusion_timeout_state_machine_.getState() == StateMachine::State::GO);
    if (!has_collision_with_margin && release_static_occlusion_stuck) {
      return IntersectionModule::Safe{closest_idx, collision_stopline_idx, occlusion_stopline_idx};
    }
    // occlusion_status is either STATICALLY_OCCLUDED or DYNAMICALLY_OCCLUDED
    const double max_timeout =
      planner_param_.occlusion.static_occlusion_with_traffic_light_timeout +
      planner_param_.occlusion.occlusion_detection_hold_time;
    const std::optional<double> static_occlusion_timeout =
      is_stuck_by_static_occlusion
        ? std::make_optional<double>(
            max_timeout - static_occlusion_timeout_state_machine_.getDuration() -
            occlusion_stop_state_machine_.getDuration())
        : (is_static_occlusion ? std::make_optional<double>(max_timeout) : std::nullopt);
    if (has_collision_with_margin) {
      return IntersectionModule::OccludedCollisionStop{
        is_occlusion_cleared_with_margin,
        temporal_stop_before_attention_required,
        closest_idx,
        collision_stopline_idx,
        first_attention_stopline_idx,
        occlusion_stopline_idx,
        static_occlusion_timeout};
    } else {
      return IntersectionModule::PeekingTowardOcclusion{
        is_occlusion_cleared_with_margin,
        temporal_stop_before_attention_required,
        closest_idx,
        collision_stopline_idx,
        first_attention_stopline_idx,
        occlusion_stopline_idx,
        static_occlusion_timeout};
    }
  } else {
    const auto occlusion_stopline =
      (planner_param_.occlusion.temporal_stop_before_attention_area || !has_traffic_light_)
        ? first_attention_stopline_idx
        : occlusion_stopline_idx;
    return IntersectionModule::FirstWaitBeforeOcclusion{
      is_occlusion_cleared_with_margin, closest_idx, default_stopline_idx, occlusion_stopline};
  }
}

bool IntersectionModule::checkStuckVehicle(
  const std::shared_ptr<const PlannerData> & planner_data, const util::PathLanelets & path_lanelets)
{
  const bool stuck_detection_direction = [&]() {
    return (turn_direction_ == "left" && planner_param_.stuck_vehicle.turn_direction.left) ||
           (turn_direction_ == "right" && planner_param_.stuck_vehicle.turn_direction.right) ||
           (turn_direction_ == "straight" && planner_param_.stuck_vehicle.turn_direction.straight);
  }();
  if (!stuck_detection_direction) {
    return false;
  }

  const auto & objects_ptr = planner_data->predicted_objects;

  // considering lane change in the intersection, these lanelets are generated from the path
  const auto stuck_vehicle_detect_area = util::generateStuckVehicleDetectAreaPolygon(
    path_lanelets, planner_param_.stuck_vehicle.stuck_vehicle_detect_dist);
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);

  return util::checkStuckVehicleInIntersection(
    objects_ptr, stuck_vehicle_detect_area,
    planner_param_.stuck_vehicle.stuck_vehicle_velocity_threshold, &debug_data_);
}

bool IntersectionModule::checkYieldStuckVehicle(
  const util::TargetObjects & target_objects,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const lanelet::ConstLanelets & attention_lanelets)
{
  const bool yield_stuck_detection_direction = [&]() {
    return (turn_direction_ == "left" && planner_param_.yield_stuck.turn_direction.left) ||
           (turn_direction_ == "right" && planner_param_.yield_stuck.turn_direction.right) ||
           (turn_direction_ == "straight" && planner_param_.yield_stuck.turn_direction.straight);
  }();
  if (!yield_stuck_detection_direction) {
    return false;
  }

  return util::checkYieldStuckVehicleInIntersection(
    target_objects, interpolated_path_info, attention_lanelets, turn_direction_,
    planner_data_->vehicle_info_.vehicle_width_m,
    planner_param_.stuck_vehicle.stuck_vehicle_velocity_threshold,
    planner_param_.yield_stuck.distance_threshold, &debug_data_);
}

util::TargetObjects IntersectionModule::generateTargetObjects(
  const util::IntersectionLanelets & intersection_lanelets,
  const std::optional<Polygon2d> & intersection_area) const
{
  const auto & objects_ptr = planner_data_->predicted_objects;
  // extract target objects
  util::TargetObjects target_objects;
  target_objects.header = objects_ptr->header;
  const auto & attention_lanelets = intersection_lanelets.attention();
  const auto & attention_lanelet_stoplines = intersection_lanelets.attention_stoplines();
  const auto & adjacent_lanelets = intersection_lanelets.adjacent();
  for (const auto & object : objects_ptr->objects) {
    // ignore non-vehicle type objects, such as pedestrian.
    if (!isTargetCollisionVehicleType(object)) {
      continue;
    }

    // check direction of objects
    const auto object_direction = util::getObjectPoseWithVelocityDirection(object.kinematics);
    const auto belong_adjacent_lanelet_id = util::checkAngleForTargetLanelets(
      object_direction, adjacent_lanelets, planner_param_.common.attention_area_angle_threshold,
      planner_param_.collision_detection.consider_wrong_direction_vehicle,
      planner_param_.common.attention_area_margin, false);
    if (belong_adjacent_lanelet_id) {
      continue;
    }

    const auto is_parked_vehicle =
      std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x) <
      planner_param_.occlusion.ignore_parked_vehicle_speed_threshold;
    auto & container = is_parked_vehicle ? target_objects.parked_attention_objects
                                         : target_objects.attention_objects;
    if (intersection_area) {
      const auto & obj_pos = object.kinematics.initial_pose_with_covariance.pose.position;
      const auto obj_poly = tier4_autoware_utils::toPolygon2d(object);
      const auto intersection_area_2d = intersection_area.value();
      const auto belong_attention_lanelet_id = util::checkAngleForTargetLanelets(
        object_direction, attention_lanelets, planner_param_.common.attention_area_angle_threshold,
        planner_param_.collision_detection.consider_wrong_direction_vehicle,
        planner_param_.common.attention_area_margin, is_parked_vehicle);
      if (belong_attention_lanelet_id) {
        const auto id = belong_attention_lanelet_id.value();
        util::TargetObject target_object;
        target_object.object = object;
        target_object.attention_lanelet = attention_lanelets.at(id);
        target_object.stopline = attention_lanelet_stoplines.at(id);
        container.push_back(target_object);
      } else if (bg::within(Point2d{obj_pos.x, obj_pos.y}, intersection_area_2d)) {
        util::TargetObject target_object;
        target_object.object = object;
        target_object.attention_lanelet = std::nullopt;
        target_object.stopline = std::nullopt;
        target_objects.intersection_area_objects.push_back(target_object);
      }
    } else if (const auto belong_attention_lanelet_id = util::checkAngleForTargetLanelets(
                 object_direction, attention_lanelets,
                 planner_param_.common.attention_area_angle_threshold,
                 planner_param_.collision_detection.consider_wrong_direction_vehicle,
                 planner_param_.common.attention_area_margin, is_parked_vehicle);
               belong_attention_lanelet_id.has_value()) {
      // intersection_area is not available, use detection_area_with_margin as before
      const auto id = belong_attention_lanelet_id.value();
      util::TargetObject target_object;
      target_object.object = object;
      target_object.attention_lanelet = attention_lanelets.at(id);
      target_object.stopline = attention_lanelet_stoplines.at(id);
      container.push_back(target_object);
    }
  }
  for (const auto & object : target_objects.attention_objects) {
    target_objects.all_attention_objects.push_back(object);
  }
  for (const auto & object : target_objects.parked_attention_objects) {
    target_objects.all_attention_objects.push_back(object);
  }
  for (auto & object : target_objects.all_attention_objects) {
    object.calc_dist_to_stopline();
  }
  return target_objects;
}

bool IntersectionModule::checkCollision(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  util::TargetObjects * target_objects, const util::PathLanelets & path_lanelets,
  const size_t closest_idx, const size_t last_intersection_stopline_candidate_idx,
  const double time_delay, const util::TrafficPrioritizedLevel & traffic_prioritized_level)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  // check collision between target_objects predicted path and ego lane
  // cut the predicted path at passing_time
  tier4_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array = util::calcIntersectionPassingTime(
    path, planner_data_, lane_id_, associative_ids_, closest_idx,
    last_intersection_stopline_candidate_idx, time_delay,
    planner_param_.collision_detection.velocity_profile.default_velocity,
    planner_param_.collision_detection.velocity_profile.minimum_default_velocity,
    planner_param_.collision_detection.velocity_profile.use_upstream,
    planner_param_.collision_detection.velocity_profile.minimum_upstream_velocity,
    &ego_ttc_time_array);

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    ego_ttc_time_array.stamp = path.header.stamp;
    ego_ttc_pub_->publish(ego_ttc_time_array);
  }

  const double passing_time = time_distance_array.back().first;
  util::cutPredictPathWithDuration(target_objects, clock_, passing_time);

  const auto & concat_lanelets = path_lanelets.all;
  const auto closest_arc_coords = getArcCoordinates(
    concat_lanelets, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const auto & ego_lane = path_lanelets.ego_or_entry2exit;
  debug_data_.ego_lane = ego_lane.polygon3d();
  const auto ego_poly = ego_lane.polygon2d().basicPolygon();

  // change TTC margin based on ego traffic light color
  const auto [collision_start_margin_time, collision_end_margin_time] = [&]() {
    if (traffic_prioritized_level == util::TrafficPrioritizedLevel::FULLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.fully_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.fully_prioritized.collision_end_margin_time);
    }
    if (traffic_prioritized_level == util::TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.partially_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.partially_prioritized.collision_end_margin_time);
    }
    return std::make_pair(
      planner_param_.collision_detection.not_prioritized.collision_start_margin_time,
      planner_param_.collision_detection.not_prioritized.collision_end_margin_time);
  }();
  const auto expectedToStopBeforeStopLine = [&](const util::TargetObject & target_object) {
    if (!target_object.dist_to_stopline) {
      return false;
    }
    const double dist_to_stopline = target_object.dist_to_stopline.value();
    if (dist_to_stopline < 0) {
      return false;
    }
    const double v = target_object.object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const double braking_distance =
      v * v /
      (2.0 * std::fabs(planner_param_.collision_detection.ignore_on_amber_traffic_light
                         .object_expected_deceleration));
    return dist_to_stopline > braking_distance;
  };
  const auto isTolerableOvershoot = [&](const util::TargetObject & target_object) {
    if (
      !target_object.attention_lanelet || !target_object.dist_to_stopline ||
      !target_object.stopline) {
      return false;
    }
    const double dist_to_stopline = target_object.dist_to_stopline.value();
    const double v = target_object.object.kinematics.initial_twist_with_covariance.twist.linear.x;
    const double braking_distance =
      v * v /
      (2.0 * std::fabs(planner_param_.collision_detection.ignore_on_amber_traffic_light
                         .object_expected_deceleration));
    if (dist_to_stopline > braking_distance) {
      return false;
    }
    const auto stopline_front = target_object.stopline.value().front();
    const auto stopline_back = target_object.stopline.value().back();
    tier4_autoware_utils::LineString2d object_line;
    object_line.emplace_back(
      (stopline_front.x() + stopline_back.x()) / 2.0,
      (stopline_front.y() + stopline_back.y()) / 2.0);
    const auto stopline_mid = object_line.front();
    const auto endpoint = target_object.attention_lanelet.value().centerline().back();
    object_line.emplace_back(endpoint.x(), endpoint.y());
    std::vector<tier4_autoware_utils::Point2d> intersections;
    bg::intersection(object_line, ego_lane.centerline2d().basicLineString(), intersections);
    if (intersections.empty()) {
      return false;
    }
    const auto collision_point = intersections.front();
    // distance from object expected stop position to collision point
    const double stopline_to_object = -1.0 * dist_to_stopline + braking_distance;
    const double stopline_to_collision =
      std::hypot(collision_point.x() - stopline_mid.x(), collision_point.y() - stopline_mid.y());
    const double object2collision = stopline_to_collision - stopline_to_object;
    const double margin =
      planner_param_.collision_detection.ignore_on_red_traffic_light.object_margin_to_path;
    return (object2collision > margin) || (object2collision < 0);
  };
  // check collision between predicted_path and ego_area
  tier4_debug_msgs::msg::Float64MultiArrayStamped object_ttc_time_array;
  object_ttc_time_array.layout.dim.resize(3);
  object_ttc_time_array.layout.dim.at(0).label = "objects";
  object_ttc_time_array.layout.dim.at(0).size = 1;  // incremented in the loop, first row is lane_id
  object_ttc_time_array.layout.dim.at(1).label =
    "[x, y, th, length, width, speed, dangerous, ref_obj_enter_time, ref_obj_exit_time, "
    "start_time, start_dist, "
    "end_time, end_dist, first_collision_x, first_collision_y, last_collision_x, last_collision_y, "
    "prd_x[0], ... pred_x[19], pred_y[0], ... pred_y[19]]";
  object_ttc_time_array.layout.dim.at(1).size = 57;
  for (unsigned i = 0; i < object_ttc_time_array.layout.dim.at(1).size; ++i) {
    object_ttc_time_array.data.push_back(lane_id_);
  }
  bool collision_detected = false;
  for (const auto & target_object : target_objects->all_attention_objects) {
    const auto & object = target_object.object;
    // If the vehicle is expected to stop before their stopline, ignore
    const bool expected_to_stop_before_stopline = expectedToStopBeforeStopLine(target_object);
    if (
      traffic_prioritized_level == util::TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED &&
      expected_to_stop_before_stopline) {
      debug_data_.amber_ignore_targets.objects.push_back(object);
      continue;
    }
    const bool is_tolerable_overshoot = isTolerableOvershoot(target_object);
    if (
      traffic_prioritized_level == util::TrafficPrioritizedLevel::FULLY_PRIORITIZED &&
      !expected_to_stop_before_stopline && is_tolerable_overshoot) {
      debug_data_.red_overshoot_ignore_targets.objects.push_back(object);
      continue;
    }
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
        [&ego_poly, &object](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, createOneStepPolygon(a, b, object.shape));
        });
      if (first_itr == predicted_path.path.cend()) continue;
      const auto last_itr = std::adjacent_find(
        predicted_path.path.crbegin(), predicted_path.path.crend(),
        [&ego_poly, &object](const auto & a, const auto & b) {
          return bg::intersects(ego_poly, createOneStepPolygon(a, b, object.shape));
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
      object_ttc_time_array.layout.dim.at(0).size++;
      const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
      const auto & shape = object.shape;
      object_ttc_time_array.data.insert(
        object_ttc_time_array.data.end(),
        {pos.x, pos.y, tf2::getYaw(object.kinematics.initial_pose_with_covariance.pose.orientation),
         shape.dimensions.x, shape.dimensions.y,
         object.kinematics.initial_twist_with_covariance.twist.linear.x,
         1.0 * static_cast<int>(collision_detected), ref_object_enter_time, ref_object_exit_time,
         start_time_distance_itr->first, start_time_distance_itr->second,
         end_time_distance_itr->first, end_time_distance_itr->second, first_itr->position.x,
         first_itr->position.y, last_itr->position.x, last_itr->position.y});
      for (unsigned i = 0; i < 20; i++) {
        const auto & pos =
          predicted_path.path.at(std::min<size_t>(i, predicted_path.path.size() - 1)).position;
        object_ttc_time_array.data.push_back(pos.x);
        object_ttc_time_array.data.push_back(pos.y);
      }
      if (collision_detected) {
        debug_data_.conflicting_targets.objects.push_back(object);
        break;
      }
    }
  }

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    object_ttc_time_array.stamp = path.header.stamp;
    object_ttc_pub_->publish(object_ttc_time_array);
  }

  return collision_detected;
}

IntersectionModule::OcclusionType IntersectionModule::getOcclusionStatus(
  const nav_msgs::msg::OccupancyGrid & occ_grid,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
  const lanelet::ConstLanelets & adjacent_lanelets,
  const lanelet::CompoundPolygon3d & first_attention_area,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const std::vector<lanelet::ConstLineString3d> & lane_divisions,
  const util::TargetObjects & target_objects, const geometry_msgs::msg::Pose & current_pose,
  const double occlusion_dist_thr)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();

  const auto first_attention_area_idx =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_attention_area);
  if (!first_attention_area_idx) {
    return OcclusionType::NOT_OCCLUDED;
  }

  const auto first_inside_attention_idx_ip_opt =
    util::getFirstPointInsidePolygon(path_ip, lane_interval_ip, first_attention_area);
  const std::pair<size_t, size_t> lane_attention_interval_ip =
    first_inside_attention_idx_ip_opt
      ? std::make_pair(first_inside_attention_idx_ip_opt.value(), std::get<1>(lane_interval_ip))
      : lane_interval_ip;
  const auto [lane_start_idx, lane_end_idx] = lane_attention_interval_ip;

  const int width = occ_grid.info.width;
  const int height = occ_grid.info.height;
  const double resolution = occ_grid.info.resolution;
  const auto & origin = occ_grid.info.origin.position;
  auto coord2index = [&](const double x, const double y) {
    const int idx_x = (x - origin.x) / resolution;
    const int idx_y = (y - origin.y) / resolution;
    if (idx_x < 0 || idx_x >= width) return std::make_tuple(false, -1, -1);
    if (idx_y < 0 || idx_y >= height) return std::make_tuple(false, -1, -1);
    return std::make_tuple(true, idx_x, idx_y);
  };

  Polygon2d grid_poly;
  grid_poly.outer().emplace_back(origin.x, origin.y);
  grid_poly.outer().emplace_back(origin.x + (width - 1) * resolution, origin.y);
  grid_poly.outer().emplace_back(
    origin.x + (width - 1) * resolution, origin.y + (height - 1) * resolution);
  grid_poly.outer().emplace_back(origin.x, origin.y + (height - 1) * resolution);
  grid_poly.outer().emplace_back(origin.x, origin.y);
  bg::correct(grid_poly);

  auto findCommonCvPolygons =
    [&](const auto & area2d, std::vector<std::vector<cv::Point>> & cv_polygons) -> void {
    tier4_autoware_utils::Polygon2d area2d_poly;
    for (const auto & p : area2d) {
      area2d_poly.outer().emplace_back(p.x(), p.y());
    }
    area2d_poly.outer().push_back(area2d_poly.outer().front());
    bg::correct(area2d_poly);
    std::vector<Polygon2d> common_areas;
    bg::intersection(area2d_poly, grid_poly, common_areas);
    if (common_areas.empty()) {
      return;
    }
    for (size_t i = 0; i < common_areas.size(); ++i) {
      common_areas[i].outer().push_back(common_areas[i].outer().front());
      bg::correct(common_areas[i]);
    }
    for (const auto & common_area : common_areas) {
      std::vector<cv::Point> cv_polygon;
      for (const auto & p : common_area.outer()) {
        const int idx_x = static_cast<int>((p.x() - origin.x) / resolution);
        const int idx_y = static_cast<int>((p.y() - origin.y) / resolution);
        cv_polygon.emplace_back(idx_x, height - 1 - idx_y);
      }
      cv_polygons.push_back(cv_polygon);
    }
  };

  // (1) prepare detection area mask
  // attention: 255
  // non-attention: 0
  // NOTE: interesting area is set to 255 for later masking
  cv::Mat attention_mask(width, height, CV_8UC1, cv::Scalar(0));
  std::vector<std::vector<cv::Point>> attention_area_cv_polygons;
  for (const auto & attention_area : attention_areas) {
    const auto area2d = lanelet::utils::to2D(attention_area);
    findCommonCvPolygons(area2d, attention_area_cv_polygons);
  }
  for (const auto & poly : attention_area_cv_polygons) {
    cv::fillPoly(attention_mask, poly, cv::Scalar(255), cv::LINE_AA);
  }
  // (1.1)
  // reset adjacent_lanelets area to 0 on attention_mask
  std::vector<std::vector<cv::Point>> adjacent_lane_cv_polygons;
  for (const auto & adjacent_lanelet : adjacent_lanelets) {
    const auto area2d = adjacent_lanelet.polygon2d().basicPolygon();
    findCommonCvPolygons(area2d, adjacent_lane_cv_polygons);
  }
  for (const auto & poly : adjacent_lane_cv_polygons) {
    cv::fillPoly(attention_mask, poly, cv::Scalar(0), cv::LINE_AA);
  }

  // (2) prepare unknown mask
  // In OpenCV the pixel at (X=x, Y=y) (with left-upper origin) is accessed by img[y, x]
  // unknown: 255
  // not-unknown: 0
  cv::Mat unknown_mask_raw(width, height, CV_8UC1, cv::Scalar(0));
  cv::Mat unknown_mask(width, height, CV_8UC1, cv::Scalar(0));
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      const int idx = y * width + x;
      const unsigned char intensity = occ_grid.data.at(idx);
      if (
        planner_param_.occlusion.free_space_max <= intensity &&
        intensity < planner_param_.occlusion.occupied_min) {
        unknown_mask_raw.at<unsigned char>(height - 1 - y, x) = 255;
      }
    }
  }
  // (2.1) apply morphologyEx
  const int morph_size = static_cast<int>(planner_param_.occlusion.denoise_kernel / resolution);
  cv::morphologyEx(
    unknown_mask_raw, unknown_mask, cv::MORPH_OPEN,
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_size, morph_size)));

  // (3) occlusion mask
  static constexpr unsigned char OCCLUDED = 255;
  static constexpr unsigned char BLOCKED = 127;
  cv::Mat occlusion_mask(width, height, CV_8UC1, cv::Scalar(0));
  cv::bitwise_and(attention_mask, unknown_mask, occlusion_mask);
  // re-use attention_mask
  attention_mask = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
  // (3.1) draw all cells on attention_mask behind blocking vehicles as not occluded
  const auto & blocking_attention_objects = target_objects.parked_attention_objects;
  for (const auto & blocking_attention_object : blocking_attention_objects) {
    debug_data_.blocking_attention_objects.objects.push_back(blocking_attention_object.object);
  }
  std::vector<std::vector<cv::Point>> blocking_polygons;
  for (const auto & blocking_attention_object : blocking_attention_objects) {
    const Polygon2d obj_poly = tier4_autoware_utils::toPolygon2d(blocking_attention_object.object);
    findCommonCvPolygons(obj_poly.outer(), blocking_polygons);
  }
  for (const auto & blocking_polygon : blocking_polygons) {
    cv::fillPoly(attention_mask, blocking_polygon, cv::Scalar(BLOCKED), cv::LINE_AA);
  }
  for (const auto & division : lane_divisions) {
    bool blocking_vehicle_found = false;
    for (const auto & point_it : division) {
      const auto [valid, idx_x, idx_y] = coord2index(point_it.x(), point_it.y());
      if (!valid) continue;
      if (blocking_vehicle_found) {
        occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x) = 0;
        continue;
      }
      if (attention_mask.at<unsigned char>(height - 1 - idx_y, idx_x) == BLOCKED) {
        blocking_vehicle_found = true;
        occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x) = 0;
      }
    }
  }

  // (4) extract occlusion polygons
  const auto & possible_object_bbox = planner_param_.occlusion.possible_object_bbox;
  const double possible_object_bbox_x = possible_object_bbox.at(0) / resolution;
  const double possible_object_bbox_y = possible_object_bbox.at(1) / resolution;
  const double possible_object_area = possible_object_bbox_x * possible_object_bbox_y;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(occlusion_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  std::vector<std::vector<cv::Point>> valid_contours;
  for (const auto & contour : contours) {
    if (contour.size() <= 2) {
      continue;
    }
    std::vector<cv::Point> approx_contour;
    cv::approxPolyDP(
      contour, approx_contour,
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
  // (4.1) re-draw occluded cells using valid_contours
  occlusion_mask = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
  for (const auto & valid_contour : valid_contours) {
    // NOTE: drawContour does not work well
    cv::fillPoly(occlusion_mask, valid_contour, cv::Scalar(OCCLUDED), cv::LINE_AA);
  }

  // (5) find distance
  // (5.1) discretize path_ip with resolution for computational cost
  LineString2d path_linestring;
  path_linestring.emplace_back(
    path_ip.points.at(lane_start_idx).point.pose.position.x,
    path_ip.points.at(lane_start_idx).point.pose.position.y);
  {
    auto prev_path_linestring_point = path_ip.points.at(lane_start_idx).point.pose.position;
    for (auto i = lane_start_idx + 1; i <= lane_end_idx; i++) {
      const auto path_linestring_point = path_ip.points.at(i).point.pose.position;
      if (
        tier4_autoware_utils::calcDistance2d(prev_path_linestring_point, path_linestring_point) <
        1.0 /* rough tick for computational cost */) {
        continue;
      }
      path_linestring.emplace_back(path_linestring_point.x, path_linestring_point.y);
      prev_path_linestring_point = path_linestring_point;
    }
  }

  auto findNearestPointToProjection = [](
                                        const lanelet::ConstLineString3d & division,
                                        const Point2d & projection, const double dist_thresh) {
    double min_dist = std::numeric_limits<double>::infinity();
    auto nearest = division.end();
    for (auto it = division.begin(); it != division.end(); it++) {
      const double dist = std::hypot(it->x() - projection.x(), it->y() - projection.y());
      if (dist < min_dist) {
        min_dist = dist;
        nearest = it;
      }
      if (dist < dist_thresh) {
        break;
      }
    }
    return nearest;
  };
  struct NearestOcclusionPoint
  {
    int64 division_index{0};
    int64 point_index{0};
    double dist{0.0};
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Point projection;
  } nearest_occlusion_point;
  double min_dist = std::numeric_limits<double>::infinity();
  for (unsigned division_index = 0; division_index < lane_divisions.size(); ++division_index) {
    const auto & division = lane_divisions.at(division_index);
    LineString2d division_linestring;
    auto division_point_it = division.begin();
    division_linestring.emplace_back(division_point_it->x(), division_point_it->y());
    for (auto point_it = division.begin(); point_it != division.end(); point_it++) {
      if (
        std::hypot(point_it->x() - division_point_it->x(), point_it->y() - division_point_it->y()) <
        3.0 /* rough tick for computational cost */) {
        continue;
      }
      division_linestring.emplace_back(point_it->x(), point_it->y());
      division_point_it = point_it;
    }

    // find the intersection point of lane_line and path
    std::vector<Point2d> intersection_points;
    boost::geometry::intersection(division_linestring, path_linestring, intersection_points);
    if (intersection_points.empty()) {
      continue;
    }
    const auto & projection_point = intersection_points.at(0);
    const auto projection_it = findNearestPointToProjection(division, projection_point, resolution);
    if (projection_it == division.end()) {
      continue;
    }
    double acc_dist = 0.0;
    auto acc_dist_it = projection_it;
    for (auto point_it = projection_it; point_it != division.end(); point_it++) {
      const double dist =
        std::hypot(point_it->x() - acc_dist_it->x(), point_it->y() - acc_dist_it->y());
      acc_dist += dist;
      acc_dist_it = point_it;
      const auto [valid, idx_x, idx_y] = coord2index(point_it->x(), point_it->y());
      if (!valid) continue;
      const auto pixel = occlusion_mask.at<unsigned char>(height - 1 - idx_y, idx_x);
      if (pixel == BLOCKED) {
        break;
      }
      if (pixel == OCCLUDED) {
        if (acc_dist < min_dist) {
          min_dist = acc_dist;
          nearest_occlusion_point = {
            division_index, std::distance(division.begin(), point_it), acc_dist,
            tier4_autoware_utils::createPoint(point_it->x(), point_it->y(), origin.z),
            tier4_autoware_utils::createPoint(projection_it->x(), projection_it->y(), origin.z)};
        }
      }
    }
  }

  if (min_dist == std::numeric_limits<double>::infinity() || min_dist > occlusion_dist_thr) {
    return OcclusionType::NOT_OCCLUDED;
  }

  debug_data_.nearest_occlusion_projection =
    std::make_pair(nearest_occlusion_point.point, nearest_occlusion_point.projection);
  LineString2d ego_occlusion_line;
  ego_occlusion_line.emplace_back(current_pose.position.x, current_pose.position.y);
  ego_occlusion_line.emplace_back(nearest_occlusion_point.point.x, nearest_occlusion_point.point.y);
  for (const auto & attention_object : target_objects.all_attention_objects) {
    const auto obj_poly = tier4_autoware_utils::toPolygon2d(attention_object.object);
    if (bg::intersects(obj_poly, ego_occlusion_line)) {
      return OcclusionType::DYNAMICALLY_OCCLUDED;
    }
  }
  for (const auto & attention_object : target_objects.intersection_area_objects) {
    const auto obj_poly = tier4_autoware_utils::toPolygon2d(attention_object.object);
    if (bg::intersects(obj_poly, ego_occlusion_line)) {
      return OcclusionType::DYNAMICALLY_OCCLUDED;
    }
  }
  return OcclusionType::STATICALLY_OCCLUDED;
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
