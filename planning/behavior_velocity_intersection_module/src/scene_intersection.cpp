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
#include <behavior_velocity_planner_common/utilization/trajectory_utils.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <interpolation/spline_interpolation_points_2d.hpp>
#include <lanelet2_extension/regulatory_elements/road_marking.hpp>
#include <lanelet2_extension/utility/query.hpp>
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
#include <list>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

namespace tier4_autoware_utils
{

template <>
inline geometry_msgs::msg::Point getPoint(const lanelet::ConstPoint3d & p)
{
  geometry_msgs::msg::Point point;
  point.x = p.x();
  point.y = p.y();
  point.z = p.z();
  return point;
}

}  // namespace tier4_autoware_utils

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

static bool isTargetStuckVehicleType(
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
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}

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
  VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  [[maybe_unused]] DebugData * debug_data)
{
  return;
}

template <>
void reactRTCApprovalByDecisionResult(
  const bool rtc_default_approved, const bool rtc_occlusion_approved,
  const IntersectionModule::StuckStop & decision_result,
  [[maybe_unused]] const IntersectionModule::PlannerParam & planner_param,
  const double baselink2front, autoware_auto_planning_msgs::msg::PathWithLaneId * path,
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  StopReason * stop_reason, VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  VelocityFactorInterface * velocity_factor, DebugData * debug_data)
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
  debug_data_ = DebugData();
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

bool IntersectionModule::isGreenSolidOn(lanelet::ConstLanelet lane)
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;

  std::optional<lanelet::Id> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  if (!tl_id) {
    // this lane has no traffic light
    return false;
  }
  const auto tl_info_opt = planner_data_->getTrafficSignal(
    tl_id.value(), true /* traffic light module keeps last observation*/);
  if (!tl_info_opt) {
    // the info of this traffic light is not available
    return false;
  }
  const auto & tl_info = tl_info_opt.value();
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
  const auto footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  const auto & current_pose = planner_data_->current_odometry->pose;

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
  const auto lanelets_on_path =
    planning_utils::getLaneletsOnPath(*path, lanelet_map_ptr, current_pose);
  if (!intersection_lanelets_) {
    intersection_lanelets_ =
      getObjectiveLanelets(lanelet_map_ptr, routing_graph_ptr, assigned_lanelet, lanelets_on_path);
  }
  auto & intersection_lanelets = intersection_lanelets_.value();

  // at the very first time of regisTration of this module, the path may not be conflicting with the
  // attention area, so update() is called to update the internal data as well as traffic light info
  const auto traffic_prioritized_level = getTrafficPrioritizedLevel(assigned_lanelet);
  const bool is_prioritized =
    traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED;
  intersection_lanelets.update(
    is_prioritized, interpolated_path_info, footprint, baselink2front, routing_graph_ptr);

  // this is abnormal
  const auto & conflicting_lanelets = intersection_lanelets.conflicting();
  const auto & first_conflicting_area_opt = intersection_lanelets.first_conflicting_area();
  const auto & first_conflicting_lane_opt = intersection_lanelets.first_conflicting_lane();
  if (conflicting_lanelets.empty() || !first_conflicting_area_opt || !first_conflicting_lane_opt) {
    return IntersectionModule::Indecisive{"conflicting area is empty"};
  }
  const auto & first_conflicting_lane = first_conflicting_lane_opt.value();
  const auto & first_conflicting_area = first_conflicting_area_opt.value();
  const auto & second_attention_area_opt = intersection_lanelets.second_attention_area();

  // generate all stop line candidates
  // see the doc for struct IntersectionStopLines
  /// even if the attention area is null, stuck vehicle stop line needs to be generated from
  /// conflicting lanes
  const auto & dummy_first_attention_lane = intersection_lanelets.first_attention_lane()
                                              ? intersection_lanelets.first_attention_lane().value()
                                              : first_conflicting_lane;

  const auto intersection_stoplines_opt = generateIntersectionStopLines(
    assigned_lanelet, first_conflicting_area, dummy_first_attention_lane, second_attention_area_opt,
    interpolated_path_info, path);
  if (!intersection_stoplines_opt) {
    return IntersectionModule::Indecisive{"failed to generate intersection_stoplines"};
  }
  const auto & intersection_stoplines = intersection_stoplines_opt.value();
  const auto closest_idx = intersection_stoplines.closest_idx;
  const auto stuck_stopline_idx_opt = intersection_stoplines.stuck_stopline;
  const auto default_stopline_idx_opt = intersection_stoplines.default_stopline;
  const auto first_attention_stopline_idx_opt = intersection_stoplines.first_attention_stopline;
  const auto occlusion_peeking_stopline_idx_opt = intersection_stoplines.occlusion_peeking_stopline;
  const auto first_pass_judge_line_idx = intersection_stoplines.first_pass_judge_line;
  const auto second_pass_judge_line_idx = intersection_stoplines.second_pass_judge_line;
  const auto occlusion_wo_tl_pass_judge_line_idx =
    intersection_stoplines.occlusion_wo_tl_pass_judge_line;

  // see the doc for struct PathLanelets
  const auto & first_attention_area_opt = intersection_lanelets.first_attention_area();
  const auto & conflicting_area = intersection_lanelets.conflicting_area();
  const auto path_lanelets_opt = generatePathLanelets(
    lanelets_on_path, interpolated_path_info, first_conflicting_area, conflicting_area,
    first_attention_area_opt, intersection_lanelets.attention_area(), closest_idx);
  if (!path_lanelets_opt.has_value()) {
    return IntersectionModule::Indecisive{"failed to generate PathLanelets"};
  }
  const auto & path_lanelets = path_lanelets_opt.value();

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
  const bool stuck_detected = checkStuckVehicleInIntersection(path_lanelets, &debug_data_);
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
  debug_data_.first_attention_area = intersection_lanelets.first_attention_area();
  debug_data_.second_attention_area = intersection_lanelets.second_attention_area();

  // check occlusion on detection lane
  if (!occlusion_attention_divisions_) {
    occlusion_attention_divisions_ = generateDetectionLaneDivisions(
      occlusion_attention_lanelets, routing_graph_ptr,
      planner_data_->occupancy_grid->info.resolution);
  }
  const auto & occlusion_attention_divisions = occlusion_attention_divisions_.value();

  // get intersection area
  const auto intersection_area = util::getIntersectionArea(assigned_lanelet, lanelet_map_ptr);
  // filter objects
  auto target_objects = generateTargetObjects(intersection_lanelets, intersection_area);

  const bool yield_stuck_detected = checkYieldStuckVehicleInIntersection(
    target_objects, interpolated_path_info, intersection_lanelets.attention_non_preceding(),
    &debug_data_);
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

  const bool is_amber_or_red =
    (traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) ||
    (traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED);
  auto occlusion_status =
    (enable_occlusion_detection_ && !occlusion_attention_lanelets.empty() && !is_amber_or_red)
      ? getOcclusionStatus(
          occlusion_attention_area, adjacent_lanelets, first_attention_area, interpolated_path_info,
          occlusion_attention_divisions, target_objects)
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

  const size_t pass_judge_line_idx = [&]() {
    if (enable_occlusion_detection_) {
      if (has_traffic_light_) {
        // if ego passed the first_pass_judge_line while it is peeking to occlusion, then its
        // position is occlusion_stopline_idx. Otherwise it is first_pass_judge_line
        if (passed_1st_judge_line_while_peeking_) {
          return occlusion_stopline_idx;
        }
        const bool is_over_first_pass_judge_line =
          util::isOverTargetIndex(*path, closest_idx, current_pose, first_pass_judge_line_idx);
        if (is_occlusion_state && is_over_first_pass_judge_line) {
          passed_1st_judge_line_while_peeking_ = true;
          return occlusion_stopline_idx;
        } else {
          return first_pass_judge_line_idx;
        }
      } else if (is_occlusion_state) {
        // if there is no traffic light and occlusion is detected, pass_judge position is beyond
        // the boundary of first attention area
        return occlusion_wo_tl_pass_judge_line_idx;
      } else {
        // if there is no traffic light and occlusion is not detected, pass_judge position is
        // default
        return first_pass_judge_line_idx;
      }
    }
    return first_pass_judge_line_idx;
  }();

  const bool was_safe = std::holds_alternative<IntersectionModule::Safe>(prev_decision_result_);

  const bool is_over_1st_pass_judge_line =
    util::isOverTargetIndex(*path, closest_idx, current_pose, pass_judge_line_idx);
  if (is_over_1st_pass_judge_line && was_safe && !safely_passed_1st_judge_line_time_) {
    safely_passed_1st_judge_line_time_ = clock_->now();
  }
  debug_data_.first_pass_judge_wall_pose =
    planning_utils::getAheadPose(pass_judge_line_idx, baselink2front, *path);
  debug_data_.passed_first_pass_judge = safely_passed_1st_judge_line_time_.has_value();
  const bool is_over_2nd_pass_judge_line =
    util::isOverTargetIndex(*path, closest_idx, current_pose, second_pass_judge_line_idx);
  if (is_over_2nd_pass_judge_line && was_safe && !safely_passed_2nd_judge_line_time_) {
    safely_passed_2nd_judge_line_time_ = clock_->now();
  }
  debug_data_.second_pass_judge_wall_pose =
    planning_utils::getAheadPose(second_pass_judge_line_idx, baselink2front, *path);
  debug_data_.passed_second_pass_judge = safely_passed_2nd_judge_line_time_.has_value();

  if (
    ((is_over_default_stopline ||
      planner_param_.common.enable_pass_judge_before_default_stopline) &&
     is_over_2nd_pass_judge_line && was_safe) ||
    is_permanent_go_) {
    /*
     * This body is active if ego is
     * - over the default stopline AND
     * - over the 1st && 2nd pass judge line AND
     * - previously safe
     * ,
     * which means ego can stop even if it is over the 1st pass judge line but
     * - before default stopline OR
     * - before the 2nd pass judge line OR
     * - or previously unsafe
     * .
     * In order for ego to continue peeking or collision detection when occlusion is detected after
     * ego passed the 1st pass judge line, it needs to be
     * - before the default stopline OR
     * - before the 2nd pass judge line OR
     * - previously unsafe
     */
    is_permanent_go_ = true;
    return IntersectionModule::Indecisive{"over the pass judge line. no plan needed"};
  }

  // If there are any vehicles on the attention area when ego entered the intersection on green
  // light, do pseudo collision detection because the vehicles are very slow and no collisions may
  // be detected. check if ego vehicle entered assigned lanelet
  const bool is_green_solid_on = isGreenSolidOn(assigned_lanelet);
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

  // TODO(Mamoru Sobue): if ego is over 1st/2nd pass judge line and collision is expected at 1st/2nd
  // pass judge line respectively, ego should go
  const auto second_attention_stopline_idx = intersection_stoplines.second_attention_stopline;
  const auto last_intersection_stopline_candidate_idx =
    second_attention_stopline_idx ? second_attention_stopline_idx.value() : occlusion_stopline_idx;
  const bool has_collision = checkCollision(
    *path, &target_objects, path_lanelets, closest_idx, last_intersection_stopline_candidate_idx,
    time_to_restart, traffic_prioritized_level);
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

TrafficPrioritizedLevel IntersectionModule::getTrafficPrioritizedLevel(lanelet::ConstLanelet lane)
{
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;

  std::optional<lanelet::Id> tl_id = std::nullopt;
  for (auto && tl_reg_elem : lane.regulatoryElementsAs<lanelet::TrafficLight>()) {
    tl_id = tl_reg_elem->id();
    break;
  }
  if (!tl_id) {
    // this lane has no traffic light
    return TrafficPrioritizedLevel::NOT_PRIORITIZED;
  }
  const auto tl_info_opt = planner_data_->getTrafficSignal(
    tl_id.value(), true /* traffic light module keeps last observation*/);
  if (!tl_info_opt) {
    return TrafficPrioritizedLevel::NOT_PRIORITIZED;
  }
  const auto & tl_info = tl_info_opt.value();
  bool has_amber_signal{false};
  for (auto && tl_light : tl_info.signal.elements) {
    if (tl_light.color == TrafficSignalElement::AMBER) {
      has_amber_signal = true;
    }
    if (tl_light.color == TrafficSignalElement::RED) {
      // NOTE: Return here since the red signal has the highest priority.
      return TrafficPrioritizedLevel::FULLY_PRIORITIZED;
    }
  }
  if (has_amber_signal) {
    return TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED;
  }
  return TrafficPrioritizedLevel::NOT_PRIORITIZED;
}

static std::vector<lanelet::CompoundPolygon3d> getPolygon3dFromLanelets(
  const lanelet::ConstLanelets & ll_vec)
{
  std::vector<lanelet::CompoundPolygon3d> polys;
  for (auto && ll : ll_vec) {
    polys.push_back(ll.polygon3d());
  }
  return polys;
}

IntersectionLanelets IntersectionModule::getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const lanelet::ConstLanelet assigned_lanelet, const lanelet::ConstLanelets & lanelets_on_path)
{
  const double detection_area_length = planner_param_.common.attention_area_length;
  const double occlusion_detection_area_length =
    planner_param_.occlusion.occlusion_attention_area_length;
  const bool consider_wrong_direction_vehicle =
    planner_param_.collision_detection.consider_wrong_direction_vehicle;

  // retrieve a stopline associated with a traffic light
  bool has_traffic_light = false;
  if (const auto tl_reg_elems = assigned_lanelet.regulatoryElementsAs<lanelet::TrafficLight>();
      tl_reg_elems.size() != 0) {
    const auto tl_reg_elem = tl_reg_elems.front();
    const auto stopline_opt = tl_reg_elem->stopLine();
    if (!!stopline_opt) has_traffic_light = true;
  }

  // for low priority lane
  // If ego_lane has right of way (i.e. is high priority),
  // ignore yieldLanelets (i.e. low priority lanes)
  lanelet::ConstLanelets yield_lanelets{};
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    if (lanelet::utils::contains(right_of_way->rightOfWayLanelets(), assigned_lanelet)) {
      for (const auto & yield_lanelet : right_of_way->yieldLanelets()) {
        yield_lanelets.push_back(yield_lanelet);
        for (const auto & previous_lanelet : routing_graph_ptr->previous(yield_lanelet)) {
          yield_lanelets.push_back(previous_lanelet);
        }
      }
    }
  }

  // get all following lanes of previous lane
  lanelet::ConstLanelets ego_lanelets = lanelets_on_path;
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    ego_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(ego_lanelets, following_lanelet)) {
        continue;
      }
      ego_lanelets.push_back(following_lanelet);
    }
  }

  // get conflicting lanes on assigned lanelet
  const auto & conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
  std::vector<lanelet::ConstLanelet> adjacent_followings;

  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    for (const auto & following_lanelet : routing_graph_ptr->following(conflicting_lanelet)) {
      adjacent_followings.push_back(following_lanelet);
    }
    for (const auto & following_lanelet : routing_graph_ptr->previous(conflicting_lanelet)) {
      adjacent_followings.push_back(following_lanelet);
    }
  }

  // final objective lanelets
  lanelet::ConstLanelets detection_lanelets;
  lanelet::ConstLanelets conflicting_ex_ego_lanelets;
  // conflicting lanes is necessary to get stopline for stuck vehicle
  for (auto && conflicting_lanelet : conflicting_lanelets) {
    if (!lanelet::utils::contains(ego_lanelets, conflicting_lanelet))
      conflicting_ex_ego_lanelets.push_back(conflicting_lanelet);
  }

  // exclude yield lanelets and ego lanelets from detection_lanelets
  if (turn_direction_ == std::string("straight") && has_traffic_light) {
    // if assigned lanelet is "straight" with traffic light, detection area is not necessary
  } else {
    if (consider_wrong_direction_vehicle) {
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (lanelet::utils::contains(yield_lanelets, conflicting_lanelet)) {
          continue;
        }
        detection_lanelets.push_back(conflicting_lanelet);
      }
      for (const auto & adjacent_following : adjacent_followings) {
        detection_lanelets.push_back(adjacent_following);
      }
    } else {
      // otherwise we need to know the priority from RightOfWay
      for (const auto & conflicting_lanelet : conflicting_lanelets) {
        if (
          lanelet::utils::contains(yield_lanelets, conflicting_lanelet) ||
          lanelet::utils::contains(ego_lanelets, conflicting_lanelet)) {
          continue;
        }
        detection_lanelets.push_back(conflicting_lanelet);
      }
    }
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  lanelet::ConstLanelets detection_and_preceding_lanelets;
  {
    const double length = detection_area_length;
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : detection_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
        routing_graph_ptr, ll, length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inserted = detection_ids.insert(l.id());
          if (inserted.second) detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }

  lanelet::ConstLanelets occlusion_detection_and_preceding_lanelets;
  {
    const double length = occlusion_detection_area_length;
    std::set<lanelet::Id> detection_ids;
    for (const auto & ll : detection_lanelets) {
      // Preceding lanes does not include detection_lane so add them at the end
      const auto & inserted = detection_ids.insert(ll.id());
      if (inserted.second) occlusion_detection_and_preceding_lanelets.push_back(ll);
      // get preceding lanelets without ego_lanelets
      // to prevent the detection area from including the ego lanes and its' preceding lanes.
      const auto lanelet_sequences = lanelet::utils::query::getPrecedingLaneletSequences(
        routing_graph_ptr, ll, length, ego_lanelets);
      for (const auto & ls : lanelet_sequences) {
        for (const auto & l : ls) {
          const auto & inserted = detection_ids.insert(l.id());
          if (inserted.second) occlusion_detection_and_preceding_lanelets.push_back(l);
        }
      }
    }
  }
  lanelet::ConstLanelets occlusion_detection_and_preceding_lanelets_wo_turn_direction;
  for (const auto & ll : occlusion_detection_and_preceding_lanelets) {
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (turn_direction == "left" || turn_direction == "right") {
      continue;
    }
    occlusion_detection_and_preceding_lanelets_wo_turn_direction.push_back(ll);
  }

  auto [attention_lanelets, original_attention_lanelet_sequences] =
    util::mergeLaneletsByTopologicalSort(detection_and_preceding_lanelets, routing_graph_ptr);

  IntersectionLanelets result;
  result.attention_ = std::move(attention_lanelets);
  for (const auto & original_attention_lanelet_seq : original_attention_lanelet_sequences) {
    // NOTE: in mergeLaneletsByTopologicalSort(), sub_ids are empty checked, so it is ensured that
    // back() exists.
    std::optional<lanelet::ConstLineString3d> stopline{std::nullopt};
    for (auto it = original_attention_lanelet_seq.rbegin();
         it != original_attention_lanelet_seq.rend(); ++it) {
      const auto traffic_lights = it->regulatoryElementsAs<lanelet::TrafficLight>();
      for (const auto & traffic_light : traffic_lights) {
        const auto stopline_opt = traffic_light->stopLine();
        if (!stopline_opt) continue;
        stopline = stopline_opt.get();
        break;
      }
      if (stopline) break;
    }
    result.attention_stoplines_.push_back(stopline);
  }
  result.attention_non_preceding_ = std::move(detection_lanelets);
  for (unsigned i = 0; i < result.attention_non_preceding_.size(); ++i) {
    std::optional<lanelet::ConstLineString3d> stopline = std::nullopt;
    const auto & ll = result.attention_non_preceding_.at(i);
    const auto traffic_lights = ll.regulatoryElementsAs<lanelet::TrafficLight>();
    for (const auto & traffic_light : traffic_lights) {
      const auto stopline_opt = traffic_light->stopLine();
      if (!stopline_opt) continue;
      stopline = stopline_opt.get();
    }
    result.attention_non_preceding_stoplines_.push_back(stopline);
  }
  result.conflicting_ = std::move(conflicting_ex_ego_lanelets);
  result.adjacent_ = planning_utils::getConstLaneletsFromIds(lanelet_map_ptr, associative_ids_);
  // NOTE: occlusion_attention is not inverted here
  // TODO(Mamoru Sobue): apply mergeLaneletsByTopologicalSort for occlusion lanelets as well and
  // then trim part of them based on curvature threshold
  result.occlusion_attention_ =
    std::move(occlusion_detection_and_preceding_lanelets_wo_turn_direction);

  // NOTE: to properly update(), each element in conflicting_/conflicting_area_,
  // attention_non_preceding_/attention_non_preceding_area_ need to be matched
  result.attention_area_ = getPolygon3dFromLanelets(result.attention_);
  result.attention_non_preceding_area_ = getPolygon3dFromLanelets(result.attention_non_preceding_);
  result.conflicting_area_ = getPolygon3dFromLanelets(result.conflicting_);
  result.adjacent_area_ = getPolygon3dFromLanelets(result.adjacent_);
  result.occlusion_attention_area_ = getPolygon3dFromLanelets(result.occlusion_attention_);
  return result;
}

std::optional<IntersectionStopLines> IntersectionModule::generateIntersectionStopLines(
  lanelet::ConstLanelet assigned_lanelet, const lanelet::CompoundPolygon3d & first_conflicting_area,
  const lanelet::ConstLanelet & first_attention_lane,
  const std::optional<lanelet::CompoundPolygon3d> & second_attention_area_opt,
  const util::InterpolatedPathInfo & interpolated_path_info,
  autoware_auto_planning_msgs::msg::PathWithLaneId * original_path)
{
  const bool use_stuck_stopline = planner_param_.stuck_vehicle.use_stuck_stopline;
  const double stopline_margin = planner_param_.common.default_stopline_margin;
  const double max_accel = planner_param_.common.max_accel;
  const double max_jerk = planner_param_.common.max_jerk;
  const double delay_response_time = planner_param_.common.delay_response_time;
  const double peeking_offset = planner_param_.occlusion.peeking_offset;

  const auto first_attention_area = first_attention_lane.polygon3d();
  const auto first_attention_lane_centerline = first_attention_lane.centerline2d();
  const auto & path_ip = interpolated_path_info.path;
  const double ds = interpolated_path_info.ds;
  const auto & lane_interval_ip = interpolated_path_info.lane_id_interval.value();
  const double baselink2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;

  const int stopline_margin_idx_dist = std::ceil(stopline_margin / ds);
  const int base2front_idx_dist =
    std::ceil(planner_data_->vehicle_info_.max_longitudinal_offset_m / ds);

  // find the index of the first point whose vehicle footprint on it intersects with attention_area
  const auto local_footprint = planner_data_->vehicle_info_.createFootprint(0.0, 0.0);
  const std::optional<size_t> first_footprint_inside_1st_attention_ip_opt =
    getFirstPointInsidePolygonByFootprint(
      first_attention_area, interpolated_path_info, local_footprint, baselink2front);
  if (!first_footprint_inside_1st_attention_ip_opt) {
    return std::nullopt;
  }
  const auto first_footprint_inside_1st_attention_ip =
    first_footprint_inside_1st_attention_ip_opt.value();

  std::optional<size_t> first_footprint_attention_centerline_ip_opt = std::nullopt;
  for (auto i = std::get<0>(lane_interval_ip); i < std::get<1>(lane_interval_ip); ++i) {
    const auto & base_pose = path_ip.points.at(i).point.pose;
    const auto path_footprint = tier4_autoware_utils::transformVector(
      local_footprint, tier4_autoware_utils::pose2transform(base_pose));
    if (bg::intersects(path_footprint, first_attention_lane_centerline.basicLineString())) {
      // NOTE: maybe consideration of braking dist is necessary
      first_footprint_attention_centerline_ip_opt = i;
      break;
    }
  }
  if (!first_footprint_attention_centerline_ip_opt) {
    return std::nullopt;
  }
  const size_t first_footprint_attention_centerline_ip =
    first_footprint_attention_centerline_ip_opt.value();

  // (1) default stop line position on interpolated path
  bool default_stopline_valid = true;
  int stop_idx_ip_int = -1;
  if (const auto map_stop_idx_ip =
        getStopLineIndexFromMap(interpolated_path_info, assigned_lanelet);
      map_stop_idx_ip) {
    stop_idx_ip_int = static_cast<int>(map_stop_idx_ip.value()) - base2front_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    stop_idx_ip_int = first_footprint_inside_1st_attention_ip - stopline_margin_idx_dist;
  }
  if (stop_idx_ip_int < 0) {
    default_stopline_valid = false;
  }
  const auto default_stopline_ip = stop_idx_ip_int >= 0 ? static_cast<size_t>(stop_idx_ip_int) : 0;

  // (2) ego front stop line position on interpolated path
  const geometry_msgs::msg::Pose & current_pose = planner_data_->current_odometry->pose;
  const auto closest_idx_ip = motion_utils::findFirstNearestIndexWithSoftConstraints(
    path_ip.points, current_pose, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);

  // (3) occlusion peeking stop line position on interpolated path
  int occlusion_peeking_line_ip_int = static_cast<int>(default_stopline_ip);
  bool occlusion_peeking_line_valid = true;
  // NOTE: if footprints[0] is already inside the attention area, invalid
  {
    const auto & base_pose0 = path_ip.points.at(default_stopline_ip).point.pose;
    const auto path_footprint0 = tier4_autoware_utils::transformVector(
      local_footprint, tier4_autoware_utils::pose2transform(base_pose0));
    if (bg::intersects(
          path_footprint0, lanelet::utils::to2D(first_attention_area).basicPolygon())) {
      occlusion_peeking_line_valid = false;
    }
  }
  if (occlusion_peeking_line_valid) {
    occlusion_peeking_line_ip_int =
      first_footprint_inside_1st_attention_ip + std::ceil(peeking_offset / ds);
  }
  const auto occlusion_peeking_line_ip = static_cast<size_t>(
    std::clamp<int>(occlusion_peeking_line_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));

  // (4) first attention stopline position on interpolated path
  const auto first_attention_stopline_ip = first_footprint_inside_1st_attention_ip;
  const bool first_attention_stopline_valid = true;

  // (5) 1st pass judge line position on interpolated path
  const double velocity = planner_data_->current_velocity->twist.linear.x;
  const double acceleration = planner_data_->current_acceleration->accel.accel.linear.x;
  const double braking_dist = planning_utils::calcJudgeLineDistWithJerkLimit(
    velocity, acceleration, max_accel, max_jerk, delay_response_time);
  int first_pass_judge_ip_int =
    static_cast<int>(first_footprint_inside_1st_attention_ip) - std::ceil(braking_dist / ds);
  const auto first_pass_judge_line_ip = static_cast<size_t>(
    std::clamp<int>(first_pass_judge_ip_int, 0, static_cast<int>(path_ip.points.size()) - 1));
  const auto occlusion_wo_tl_pass_judge_line_ip = static_cast<size_t>(std::max<int>(
    0, static_cast<int>(first_footprint_attention_centerline_ip) - std::ceil(braking_dist / ds)));

  // (6) stuck vehicle stopline position on interpolated path
  int stuck_stopline_ip_int = 0;
  bool stuck_stopline_valid = true;
  if (use_stuck_stopline) {
    // NOTE: when ego vehicle is approaching attention area and already passed
    // first_conflicting_area, this could be null.
    const auto stuck_stopline_idx_ip_opt = getFirstPointInsidePolygonByFootprint(
      first_conflicting_area, interpolated_path_info, local_footprint, baselink2front);
    if (!stuck_stopline_idx_ip_opt) {
      stuck_stopline_valid = false;
      stuck_stopline_ip_int = 0;
    } else {
      stuck_stopline_ip_int = stuck_stopline_idx_ip_opt.value() - stopline_margin_idx_dist;
    }
  } else {
    stuck_stopline_ip_int =
      std::get<0>(lane_interval_ip) - (stopline_margin_idx_dist + base2front_idx_dist);
  }
  if (stuck_stopline_ip_int < 0) {
    stuck_stopline_valid = false;
  }
  const auto stuck_stopline_ip = static_cast<size_t>(std::max(0, stuck_stopline_ip_int));

  // (7) second attention stopline position on interpolated path
  int second_attention_stopline_ip_int = -1;
  bool second_attention_stopline_valid = false;
  if (second_attention_area_opt) {
    const auto & second_attention_area = second_attention_area_opt.value();
    std::optional<size_t> first_footprint_inside_2nd_attention_ip_opt =
      getFirstPointInsidePolygonByFootprint(
        second_attention_area, interpolated_path_info, local_footprint, baselink2front);
    if (first_footprint_inside_2nd_attention_ip_opt) {
      second_attention_stopline_ip_int = first_footprint_inside_2nd_attention_ip_opt.value();
      second_attention_stopline_valid = true;
    }
  }
  const auto second_attention_stopline_ip =
    second_attention_stopline_ip_int >= 0 ? static_cast<size_t>(second_attention_stopline_ip_int)
                                          : 0;

  // (8) second pass judge line position on interpolated path. It is the same as first pass judge
  // line if second_attention_lane is null
  int second_pass_judge_ip_int = occlusion_wo_tl_pass_judge_line_ip;
  const auto second_pass_judge_line_ip =
    second_attention_area_opt ? static_cast<size_t>(std::max<int>(second_pass_judge_ip_int, 0))
                              : first_pass_judge_line_ip;

  struct IntersectionStopLinesTemp
  {
    size_t closest_idx{0};
    size_t stuck_stopline{0};
    size_t default_stopline{0};
    size_t first_attention_stopline{0};
    size_t second_attention_stopline{0};
    size_t occlusion_peeking_stopline{0};
    size_t first_pass_judge_line{0};
    size_t second_pass_judge_line{0};
    size_t occlusion_wo_tl_pass_judge_line{0};
  };

  IntersectionStopLinesTemp intersection_stoplines_temp;
  std::list<std::pair<const size_t *, size_t *>> stoplines = {
    {&closest_idx_ip, &intersection_stoplines_temp.closest_idx},
    {&stuck_stopline_ip, &intersection_stoplines_temp.stuck_stopline},
    {&default_stopline_ip, &intersection_stoplines_temp.default_stopline},
    {&first_attention_stopline_ip, &intersection_stoplines_temp.first_attention_stopline},
    {&second_attention_stopline_ip, &intersection_stoplines_temp.second_attention_stopline},
    {&occlusion_peeking_line_ip, &intersection_stoplines_temp.occlusion_peeking_stopline},
    {&first_pass_judge_line_ip, &intersection_stoplines_temp.first_pass_judge_line},
    {&second_pass_judge_line_ip, &intersection_stoplines_temp.second_pass_judge_line},
    {&occlusion_wo_tl_pass_judge_line_ip,
     &intersection_stoplines_temp.occlusion_wo_tl_pass_judge_line}};
  stoplines.sort(
    [](const auto & it1, const auto & it2) { return *(std::get<0>(it1)) < *(std::get<0>(it2)); });
  for (const auto & [stop_idx_ip, stop_idx] : stoplines) {
    const auto & insert_point = path_ip.points.at(*stop_idx_ip).point.pose;
    const auto insert_idx = util::insertPointIndex(
      insert_point, original_path, planner_data_->ego_nearest_dist_threshold,
      planner_data_->ego_nearest_yaw_threshold);
    if (!insert_idx) {
      return std::nullopt;
    }
    *stop_idx = insert_idx.value();
  }
  if (
    intersection_stoplines_temp.occlusion_peeking_stopline <
    intersection_stoplines_temp.default_stopline) {
    intersection_stoplines_temp.occlusion_peeking_stopline =
      intersection_stoplines_temp.default_stopline;
  }

  IntersectionStopLines intersection_stoplines;
  intersection_stoplines.closest_idx = intersection_stoplines_temp.closest_idx;
  if (stuck_stopline_valid) {
    intersection_stoplines.stuck_stopline = intersection_stoplines_temp.stuck_stopline;
  }
  if (default_stopline_valid) {
    intersection_stoplines.default_stopline = intersection_stoplines_temp.default_stopline;
  }
  if (first_attention_stopline_valid) {
    intersection_stoplines.first_attention_stopline =
      intersection_stoplines_temp.first_attention_stopline;
  }
  if (second_attention_stopline_valid) {
    intersection_stoplines.second_attention_stopline =
      intersection_stoplines_temp.second_attention_stopline;
  }
  if (occlusion_peeking_line_valid) {
    intersection_stoplines.occlusion_peeking_stopline =
      intersection_stoplines_temp.occlusion_peeking_stopline;
  }
  intersection_stoplines.first_pass_judge_line = intersection_stoplines_temp.first_pass_judge_line;
  intersection_stoplines.second_pass_judge_line =
    intersection_stoplines_temp.second_pass_judge_line;
  intersection_stoplines.occlusion_wo_tl_pass_judge_line =
    intersection_stoplines_temp.occlusion_wo_tl_pass_judge_line;
  return intersection_stoplines;
}

/**
 * @brief Get stop point from map if exists
 * @param stop_pose stop point defined on map
 * @return true when the stop point is defined on map.
 */
std::optional<size_t> IntersectionModule::getStopLineIndexFromMap(
  const util::InterpolatedPathInfo & interpolated_path_info, lanelet::ConstLanelet assigned_lanelet)
{
  const auto & path = interpolated_path_info.path;
  const auto & lane_interval = interpolated_path_info.lane_id_interval.value();

  const auto road_markings =
    assigned_lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stopline;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stopline.push_back(road_marking->roadMarking());
      break;  // only one stopline exists.
    }
  }
  if (stopline.empty()) {
    return std::nullopt;
  }

  const auto p_start = stopline.front().front();
  const auto p_end = stopline.front().back();
  const LineString2d extended_stopline =
    planning_utils::extendLine(p_start, p_end, planner_data_->stop_line_extend_length);

  for (size_t i = lane_interval.first; i < lane_interval.second; i++) {
    const auto & p_front = path.points.at(i).point.pose.position;
    const auto & p_back = path.points.at(i + 1).point.pose.position;

    const LineString2d path_segment = {{p_front.x, p_front.y}, {p_back.x, p_back.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(extended_stopline, path_segment, collision_points);

    if (collision_points.empty()) {
      continue;
    }

    return i;
  }

  geometry_msgs::msg::Pose stop_point_from_map;
  stop_point_from_map.position.x = 0.5 * (p_start.x() + p_end.x());
  stop_point_from_map.position.y = 0.5 * (p_start.y() + p_end.y());
  stop_point_from_map.position.z = 0.5 * (p_start.z() + p_end.z());

  return motion_utils::findFirstNearestIndexWithSoftConstraints(
    path.points, stop_point_from_map, planner_data_->ego_nearest_dist_threshold,
    planner_data_->ego_nearest_yaw_threshold);
}

static lanelet::ConstLanelets getPrevLanelets(
  const lanelet::ConstLanelets & lanelets_on_path, const std::set<lanelet::Id> & associative_ids)
{
  lanelet::ConstLanelets previous_lanelets;
  for (const auto & ll : lanelets_on_path) {
    if (associative_ids.find(ll.id()) != associative_ids.end()) {
      return previous_lanelets;
    }
    previous_lanelets.push_back(ll);
  }
  return previous_lanelets;
}

// end inclusive
static lanelet::ConstLanelet generatePathLanelet(
  const PathWithLaneId & path, const size_t start_idx, const size_t end_idx, const double width,
  const double interval)
{
  lanelet::Points3d lefts;
  lanelet::Points3d rights;
  size_t prev_idx = start_idx;
  for (size_t i = start_idx; i <= end_idx; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto & p_prev = path.points.at(prev_idx).point.pose;
    if (i != start_idx && tier4_autoware_utils::calcDistance2d(p_prev, p) < interval) {
      continue;
    }
    prev_idx = i;
    const double yaw = tf2::getYaw(p.orientation);
    const double x = p.position.x;
    const double y = p.position.y;
    // NOTE: maybe this is opposite
    const double left_x = x + width / 2 * std::sin(yaw);
    const double left_y = y - width / 2 * std::cos(yaw);
    const double right_x = x - width / 2 * std::sin(yaw);
    const double right_y = y + width / 2 * std::cos(yaw);
    lefts.emplace_back(lanelet::InvalId, left_x, left_y, p.position.z);
    rights.emplace_back(lanelet::InvalId, right_x, right_y, p.position.z);
  }
  lanelet::LineString3d left = lanelet::LineString3d(lanelet::InvalId, lefts);
  lanelet::LineString3d right = lanelet::LineString3d(lanelet::InvalId, rights);

  return lanelet::Lanelet(lanelet::InvalId, left, right);
}

static std::optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>
getFirstPointInsidePolygons(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const std::pair<size_t, size_t> lane_interval,
  const std::vector<lanelet::CompoundPolygon3d> & polygons, const bool search_forward = true)
{
  if (search_forward) {
    for (size_t i = lane_interval.first; i <= lane_interval.second; ++i) {
      bool is_in_lanelet = false;
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
        if (is_in_lanelet) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (is_in_lanelet) {
        break;
      }
    }
  } else {
    for (size_t i = lane_interval.second; i >= lane_interval.first; --i) {
      bool is_in_lanelet = false;
      const auto & p = path.points.at(i).point.pose.position;
      for (const auto & polygon : polygons) {
        const auto polygon_2d = lanelet::utils::to2D(polygon);
        is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
        if (is_in_lanelet) {
          return std::make_optional<std::pair<size_t, const lanelet::CompoundPolygon3d &>>(
            i, polygon);
        }
      }
      if (is_in_lanelet) {
        break;
      }
      if (i == 0) {
        break;
      }
    }
  }
  return std::nullopt;
}

std::optional<PathLanelets> IntersectionModule::generatePathLanelets(
  const lanelet::ConstLanelets & lanelets_on_path,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const lanelet::CompoundPolygon3d & first_conflicting_area,
  const std::vector<lanelet::CompoundPolygon3d> & conflicting_areas,
  const std::optional<lanelet::CompoundPolygon3d> & first_attention_area,
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas, const size_t closest_idx)
{
  const double width = planner_data_->vehicle_info_.vehicle_width_m;
  static constexpr double path_lanelet_interval = 1.5;

  const auto & assigned_lane_interval_opt = interpolated_path_info.lane_id_interval;
  if (!assigned_lane_interval_opt) {
    return std::nullopt;
  }
  const auto assigned_lane_interval = assigned_lane_interval_opt.value();
  const auto & path = interpolated_path_info.path;

  PathLanelets path_lanelets;
  // prev
  path_lanelets.prev = getPrevLanelets(lanelets_on_path, associative_ids_);
  path_lanelets.all = path_lanelets.prev;

  // entry2ego if exist
  const auto [assigned_lane_start, assigned_lane_end] = assigned_lane_interval;
  if (closest_idx > assigned_lane_start) {
    path_lanelets.all.push_back(
      generatePathLanelet(path, assigned_lane_start, closest_idx, width, path_lanelet_interval));
  }

  // ego_or_entry2exit
  const auto ego_or_entry_start = std::max(closest_idx, assigned_lane_start);
  path_lanelets.ego_or_entry2exit =
    generatePathLanelet(path, ego_or_entry_start, assigned_lane_end, width, path_lanelet_interval);
  path_lanelets.all.push_back(path_lanelets.ego_or_entry2exit);

  // next
  if (assigned_lane_end < path.points.size() - 1) {
    const int next_id = path.points.at(assigned_lane_end).lane_ids.at(0);
    const auto next_lane_interval_opt = util::findLaneIdsInterval(path, {next_id});
    if (next_lane_interval_opt) {
      const auto [next_start, next_end] = next_lane_interval_opt.value();
      path_lanelets.next =
        generatePathLanelet(path, next_start, next_end, width, path_lanelet_interval);
      path_lanelets.all.push_back(path_lanelets.next.value());
    }
  }

  const auto first_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? util::getFirstPointInsidePolygon(path, assigned_lane_interval, first_attention_area.value())
      : util::getFirstPointInsidePolygon(path, assigned_lane_interval, first_conflicting_area);
  const auto last_inside_conflicting_idx_opt =
    first_attention_area.has_value()
      ? getFirstPointInsidePolygons(path, assigned_lane_interval, attention_areas, false)
      : getFirstPointInsidePolygons(path, assigned_lane_interval, conflicting_areas, false);
  if (first_inside_conflicting_idx_opt && last_inside_conflicting_idx_opt) {
    const auto first_inside_conflicting_idx = first_inside_conflicting_idx_opt.value();
    const auto last_inside_conflicting_idx = last_inside_conflicting_idx_opt.value().first;
    lanelet::ConstLanelet conflicting_interval = generatePathLanelet(
      path, first_inside_conflicting_idx, last_inside_conflicting_idx, width,
      path_lanelet_interval);
    path_lanelets.conflicting_interval_and_remaining.push_back(std::move(conflicting_interval));
    if (last_inside_conflicting_idx < assigned_lane_end) {
      lanelet::ConstLanelet remaining_interval = generatePathLanelet(
        path, last_inside_conflicting_idx, assigned_lane_end, width, path_lanelet_interval);
      path_lanelets.conflicting_interval_and_remaining.push_back(std::move(remaining_interval));
    }
  }
  return path_lanelets;
}

bool IntersectionModule::checkStuckVehicleInIntersection(
  const PathLanelets & path_lanelets, DebugData * debug_data)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getLaneletLength3d;
  using lanelet::utils::getPolygonFromArcLength;
  using lanelet::utils::to2D;

  const bool stuck_detection_direction = [&]() {
    return (turn_direction_ == "left" && planner_param_.stuck_vehicle.turn_direction.left) ||
           (turn_direction_ == "right" && planner_param_.stuck_vehicle.turn_direction.right) ||
           (turn_direction_ == "straight" && planner_param_.stuck_vehicle.turn_direction.straight);
  }();
  if (!stuck_detection_direction) {
    return false;
  }

  const auto & objects_ptr = planner_data_->predicted_objects;

  // considering lane change in the intersection, these lanelets are generated from the path
  const double stuck_vehicle_detect_dist = planner_param_.stuck_vehicle.stuck_vehicle_detect_dist;
  Polygon2d stuck_vehicle_detect_area{};
  if (path_lanelets.conflicting_interval_and_remaining.size() == 0) {
    return false;
  }

  double target_polygon_length =
    getLaneletLength3d(path_lanelets.conflicting_interval_and_remaining);
  lanelet::ConstLanelets targets = path_lanelets.conflicting_interval_and_remaining;
  if (path_lanelets.next) {
    targets.push_back(path_lanelets.next.value());
    const double next_arc_length =
      std::min(stuck_vehicle_detect_dist, getLaneletLength3d(path_lanelets.next.value()));
    target_polygon_length += next_arc_length;
  }
  const auto target_polygon =
    to2D(getPolygonFromArcLength(targets, 0, target_polygon_length)).basicPolygon();

  if (target_polygon.empty()) {
    return false;
  }

  for (const auto & p : target_polygon) {
    stuck_vehicle_detect_area.outer().emplace_back(p.x(), p.y());
  }

  stuck_vehicle_detect_area.outer().emplace_back(stuck_vehicle_detect_area.outer().front());
  bg::correct(stuck_vehicle_detect_area);

  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v_norm = std::hypot(
      object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.kinematics.initial_twist_with_covariance.twist.linear.y);
    if (obj_v_norm > planner_param_.stuck_vehicle.stuck_vehicle_velocity_threshold) {
      continue;  // not stop vehicle
    }

    // check if the footprint is in the stuck detect area
    const auto obj_footprint = tier4_autoware_utils::toPolygon2d(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, stuck_vehicle_detect_area);
    if (is_in_stuck_area && debug_data) {
      debug_data->stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

static lanelet::LineString3d getLineStringFromArcLength(
  const lanelet::ConstLineString3d & linestring, const double s1, const double s2)
{
  lanelet::Points3d points;
  double accumulated_length = 0;
  size_t start_index = linestring.size();
  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = boost::geometry::distance(p1.basicPoint(), p2.basicPoint());
    if (accumulated_length + length > s1) {
      start_index = i;
      break;
    }
    accumulated_length += length;
  }
  if (start_index < linestring.size() - 1) {
    const auto & p1 = linestring[start_index];
    const auto & p2 = linestring[start_index + 1];
    const double residue = s1 - accumulated_length;
    const auto direction_vector = (p2.basicPoint() - p1.basicPoint()).normalized();
    const auto start_basic_point = p1.basicPoint() + residue * direction_vector;
    const auto start_point = lanelet::Point3d(lanelet::InvalId, start_basic_point);
    points.push_back(start_point);
  }

  accumulated_length = 0;
  size_t end_index = linestring.size();
  for (size_t i = 0; i < linestring.size() - 1; i++) {
    const auto & p1 = linestring[i];
    const auto & p2 = linestring[i + 1];
    const double length = boost::geometry::distance(p1.basicPoint(), p2.basicPoint());
    if (accumulated_length + length > s2) {
      end_index = i;
      break;
    }
    accumulated_length += length;
  }

  for (size_t i = start_index + 1; i < end_index; i++) {
    const auto p = lanelet::Point3d(linestring[i]);
    points.push_back(p);
  }
  if (end_index < linestring.size() - 1) {
    const auto & p1 = linestring[end_index];
    const auto & p2 = linestring[end_index + 1];
    const double residue = s2 - accumulated_length;
    const auto direction_vector = (p2.basicPoint() - p1.basicPoint()).normalized();
    const auto end_basic_point = p1.basicPoint() + residue * direction_vector;
    const auto end_point = lanelet::Point3d(lanelet::InvalId, end_basic_point);
    points.push_back(end_point);
  }
  return lanelet::LineString3d{lanelet::InvalId, points};
}

static lanelet::ConstLanelet createLaneletFromArcLength(
  const lanelet::ConstLanelet & lanelet, const double s1, const double s2)
{
  const double total_length = boost::geometry::length(lanelet.centerline2d().basicLineString());
  // make sure that s1, and s2 are between [0, lane_length]
  const auto s1_saturated = std::max(0.0, std::min(s1, total_length));
  const auto s2_saturated = std::max(0.0, std::min(s2, total_length));

  const auto ratio_s1 = s1_saturated / total_length;
  const auto ratio_s2 = s2_saturated / total_length;

  const auto s1_left =
    static_cast<double>(ratio_s1 * boost::geometry::length(lanelet.leftBound().basicLineString()));
  const auto s2_left =
    static_cast<double>(ratio_s2 * boost::geometry::length(lanelet.leftBound().basicLineString()));
  const auto s1_right =
    static_cast<double>(ratio_s1 * boost::geometry::length(lanelet.rightBound().basicLineString()));
  const auto s2_right =
    static_cast<double>(ratio_s2 * boost::geometry::length(lanelet.rightBound().basicLineString()));

  const auto left_bound = getLineStringFromArcLength(lanelet.leftBound(), s1_left, s2_left);
  const auto right_bound = getLineStringFromArcLength(lanelet.rightBound(), s1_right, s2_right);

  return lanelet::Lanelet(lanelet::InvalId, left_bound, right_bound);
}

bool IntersectionModule::checkYieldStuckVehicleInIntersection(
  const TargetObjects & target_objects, const util::InterpolatedPathInfo & interpolated_path_info,
  const lanelet::ConstLanelets & attention_lanelets, DebugData * debug_data)
{
  const bool yield_stuck_detection_direction = [&]() {
    return (turn_direction_ == "left" && planner_param_.yield_stuck.turn_direction.left) ||
           (turn_direction_ == "right" && planner_param_.yield_stuck.turn_direction.right) ||
           (turn_direction_ == "straight" && planner_param_.yield_stuck.turn_direction.straight);
  }();
  if (!yield_stuck_detection_direction) {
    return false;
  }

  const double width = planner_data_->vehicle_info_.vehicle_width_m;
  const double stuck_vehicle_vel_thr =
    planner_param_.stuck_vehicle.stuck_vehicle_velocity_threshold;
  const double yield_stuck_distance_thr = planner_param_.yield_stuck.distance_threshold;

  LineString2d sparse_intersection_path;
  const auto [start, end] = interpolated_path_info.lane_id_interval.value();
  for (unsigned i = start; i < end; ++i) {
    const auto & point = interpolated_path_info.path.points.at(i).point.pose.position;
    const auto yaw = tf2::getYaw(interpolated_path_info.path.points.at(i).point.pose.orientation);
    if (turn_direction_ == "right") {
      const double right_x = point.x - width / 2 * std::sin(yaw);
      const double right_y = point.y + width / 2 * std::cos(yaw);
      sparse_intersection_path.emplace_back(right_x, right_y);
    } else if (turn_direction_ == "left") {
      const double left_x = point.x + width / 2 * std::sin(yaw);
      const double left_y = point.y - width / 2 * std::cos(yaw);
      sparse_intersection_path.emplace_back(left_x, left_y);
    } else {
      // straight
      sparse_intersection_path.emplace_back(point.x, point.y);
    }
  }
  lanelet::ConstLanelets yield_stuck_detect_lanelets;
  for (const auto & attention_lanelet : attention_lanelets) {
    const auto centerline = attention_lanelet.centerline2d().basicLineString();
    std::vector<Point2d> intersects;
    bg::intersection(sparse_intersection_path, centerline, intersects);
    if (intersects.empty()) {
      continue;
    }
    const auto intersect = intersects.front();
    const auto intersect_arc_coords = lanelet::geometry::toArcCoordinates(
      centerline, lanelet::BasicPoint2d(intersect.x(), intersect.y()));
    const double yield_stuck_start =
      std::max(0.0, intersect_arc_coords.length - yield_stuck_distance_thr);
    const double yield_stuck_end = intersect_arc_coords.length;
    yield_stuck_detect_lanelets.push_back(
      createLaneletFromArcLength(attention_lanelet, yield_stuck_start, yield_stuck_end));
  }
  debug_data->yield_stuck_detect_area = getPolygon3dFromLanelets(yield_stuck_detect_lanelets);
  for (const auto & object : target_objects.all_attention_objects) {
    const auto obj_v_norm = std::hypot(
      object.object.kinematics.initial_twist_with_covariance.twist.linear.x,
      object.object.kinematics.initial_twist_with_covariance.twist.linear.y);

    if (obj_v_norm > stuck_vehicle_vel_thr) {
      continue;
    }
    for (const auto & yield_stuck_detect_lanelet : yield_stuck_detect_lanelets) {
      const bool is_in_lanelet = lanelet::utils::isInLanelet(
        object.object.kinematics.initial_pose_with_covariance.pose, yield_stuck_detect_lanelet);
      if (is_in_lanelet) {
        debug_data->yield_stuck_targets.objects.push_back(object.object);
        return true;
      }
    }
  }
  return false;
}

TargetObjects IntersectionModule::generateTargetObjects(
  const IntersectionLanelets & intersection_lanelets,
  const std::optional<Polygon2d> & intersection_area) const
{
  const auto & objects_ptr = planner_data_->predicted_objects;
  // extract target objects
  TargetObjects target_objects;
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
    const auto belong_adjacent_lanelet_id =
      checkAngleForTargetLanelets(object_direction, adjacent_lanelets, false);
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
      const auto belong_attention_lanelet_id =
        checkAngleForTargetLanelets(object_direction, attention_lanelets, is_parked_vehicle);
      if (belong_attention_lanelet_id) {
        const auto id = belong_attention_lanelet_id.value();
        TargetObject target_object;
        target_object.object = object;
        target_object.attention_lanelet = attention_lanelets.at(id);
        target_object.stopline = attention_lanelet_stoplines.at(id);
        container.push_back(target_object);
      } else if (bg::within(Point2d{obj_pos.x, obj_pos.y}, intersection_area_2d)) {
        TargetObject target_object;
        target_object.object = object;
        target_object.attention_lanelet = std::nullopt;
        target_object.stopline = std::nullopt;
        target_objects.intersection_area_objects.push_back(target_object);
      }
    } else if (const auto belong_attention_lanelet_id = checkAngleForTargetLanelets(
                 object_direction, attention_lanelets, is_parked_vehicle);
               belong_attention_lanelet_id.has_value()) {
      // intersection_area is not available, use detection_area_with_margin as before
      const auto id = belong_attention_lanelet_id.value();
      TargetObject target_object;
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
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, TargetObjects * target_objects,
  const PathLanelets & path_lanelets, const size_t closest_idx,
  const size_t last_intersection_stopline_candidate_idx, const double time_delay,
  const TrafficPrioritizedLevel & traffic_prioritized_level)
{
  using lanelet::utils::getArcCoordinates;
  using lanelet::utils::getPolygonFromArcLength;

  // check collision between target_objects predicted path and ego lane
  // cut the predicted path at passing_time
  tier4_debug_msgs::msg::Float64MultiArrayStamped ego_ttc_time_array;
  const auto time_distance_array = calcIntersectionPassingTime(
    path, closest_idx, last_intersection_stopline_candidate_idx, time_delay, &ego_ttc_time_array);

  if (
    std::find(planner_param_.debug.ttc.begin(), planner_param_.debug.ttc.end(), lane_id_) !=
    planner_param_.debug.ttc.end()) {
    ego_ttc_time_array.stamp = path.header.stamp;
    ego_ttc_pub_->publish(ego_ttc_time_array);
  }

  const double passing_time = time_distance_array.back().first;
  cutPredictPathWithDuration(target_objects, passing_time);

  const auto & concat_lanelets = path_lanelets.all;
  const auto closest_arc_coords = getArcCoordinates(
    concat_lanelets, tier4_autoware_utils::getPose(path.points.at(closest_idx).point));
  const auto & ego_lane = path_lanelets.ego_or_entry2exit;
  debug_data_.ego_lane = ego_lane.polygon3d();
  const auto ego_poly = ego_lane.polygon2d().basicPolygon();

  // change TTC margin based on ego traffic light color
  const auto [collision_start_margin_time, collision_end_margin_time] = [&]() {
    if (traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.fully_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.fully_prioritized.collision_end_margin_time);
    }
    if (traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED) {
      return std::make_pair(
        planner_param_.collision_detection.partially_prioritized.collision_start_margin_time,
        planner_param_.collision_detection.partially_prioritized.collision_end_margin_time);
    }
    return std::make_pair(
      planner_param_.collision_detection.not_prioritized.collision_start_margin_time,
      planner_param_.collision_detection.not_prioritized.collision_end_margin_time);
  }();
  const auto expectedToStopBeforeStopLine = [&](const TargetObject & target_object) {
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
  const auto isTolerableOvershoot = [&](const TargetObject & target_object) {
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
      traffic_prioritized_level == TrafficPrioritizedLevel::PARTIALLY_PRIORITIZED &&
      expected_to_stop_before_stopline) {
      debug_data_.amber_ignore_targets.objects.push_back(object);
      continue;
    }
    const bool is_tolerable_overshoot = isTolerableOvershoot(target_object);
    if (
      traffic_prioritized_level == TrafficPrioritizedLevel::FULLY_PRIORITIZED &&
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

std::optional<size_t> IntersectionModule::checkAngleForTargetLanelets(

  const geometry_msgs::msg::Pose & pose, const lanelet::ConstLanelets & target_lanelets,
  const bool is_parked_vehicle) const
{
  const double detection_area_angle_thr = planner_param_.common.attention_area_angle_threshold;
  const bool consider_wrong_direction_vehicle =
    planner_param_.common.attention_area_angle_threshold;
  const double dist_margin = planner_param_.common.attention_area_margin;

  for (unsigned i = 0; i < target_lanelets.size(); ++i) {
    const auto & ll = target_lanelets.at(i);
    if (!lanelet::utils::isInLanelet(pose, ll, dist_margin)) {
      continue;
    }
    const double ll_angle = lanelet::utils::getLaneletAngle(ll, pose.position);
    const double pose_angle = tf2::getYaw(pose.orientation);
    const double angle_diff = tier4_autoware_utils::normalizeRadian(ll_angle - pose_angle, -M_PI);
    if (consider_wrong_direction_vehicle) {
      if (std::fabs(angle_diff) > 1.57 || std::fabs(angle_diff) < detection_area_angle_thr) {
        return std::make_optional<size_t>(i);
      }
    } else {
      if (std::fabs(angle_diff) < detection_area_angle_thr) {
        return std::make_optional<size_t>(i);
      }
      // NOTE: sometimes parked vehicle direction is reversed even if its longitudinal velocity is
      // positive
      if (
        is_parked_vehicle && (std::fabs(angle_diff) < detection_area_angle_thr ||
                              (std::fabs(angle_diff + M_PI) < detection_area_angle_thr))) {
        return std::make_optional<size_t>(i);
      }
    }
  }
  return std::nullopt;
}

void IntersectionModule::cutPredictPathWithDuration(
  TargetObjects * target_objects, const double time_thr)
{
  const rclcpp::Time current_time = clock_->now();
  for (auto & target_object : target_objects->all_attention_objects) {  // each objects
    for (auto & predicted_path :
         target_object.object.kinematics.predicted_paths) {  // each predicted paths
      const auto origin_path = predicted_path;
      predicted_path.path.clear();

      for (size_t k = 0; k < origin_path.path.size(); ++k) {  // each path points
        const auto & predicted_pose = origin_path.path.at(k);
        const auto predicted_time =
          rclcpp::Time(target_objects->header.stamp) +
          rclcpp::Duration(origin_path.time_step) * static_cast<double>(k);
        if ((predicted_time - current_time).seconds() < time_thr) {
          predicted_path.path.push_back(predicted_pose);
        }
      }
    }
  }
}

TimeDistanceArray IntersectionModule::calcIntersectionPassingTime(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const size_t closest_idx,
  const size_t last_intersection_stopline_candidate_idx, const double time_delay,
  tier4_debug_msgs::msg::Float64MultiArrayStamped * debug_ttc_array)
{
  const double intersection_velocity =
    planner_param_.collision_detection.velocity_profile.default_velocity;
  const double minimum_ego_velocity =
    planner_param_.collision_detection.velocity_profile.minimum_default_velocity;
  const bool use_upstream_velocity =
    planner_param_.collision_detection.velocity_profile.use_upstream;
  const double minimum_upstream_velocity =
    planner_param_.collision_detection.velocity_profile.minimum_upstream_velocity;
  const double current_velocity = planner_data_->current_velocity->twist.linear.x;

  int assigned_lane_found = false;

  // crop intersection part of the path, and set the reference velocity to intersection_velocity
  // for ego's ttc
  PathWithLaneId reference_path;
  std::optional<size_t> upstream_stopline{std::nullopt};
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    auto reference_point = path.points.at(i);
    // assume backward velocity is current ego velocity
    if (i < closest_idx) {
      reference_point.point.longitudinal_velocity_mps = current_velocity;
    }
    if (
      i > last_intersection_stopline_candidate_idx &&
      std::fabs(reference_point.point.longitudinal_velocity_mps) <
        std::numeric_limits<double>::epsilon() &&
      !upstream_stopline) {
      upstream_stopline = i;
    }
    if (!use_upstream_velocity) {
      reference_point.point.longitudinal_velocity_mps = intersection_velocity;
    }
    reference_path.points.push_back(reference_point);
    bool has_objective_lane_id = util::hasLaneIds(path.points.at(i), associative_ids_);
    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) {
    return {{0.0, 0.0}};  // has already passed the intersection.
  }

  std::vector<std::pair<double, double>> original_path_xy;
  for (size_t i = 0; i < reference_path.points.size(); ++i) {
    const auto & p = reference_path.points.at(i).point.pose.position;
    original_path_xy.emplace_back(p.x, p.y);
  }

  // apply smoother to reference velocity
  PathWithLaneId smoothed_reference_path = reference_path;
  if (!smoothPath(reference_path, smoothed_reference_path, planner_data_)) {
    smoothed_reference_path = reference_path;
  }

  // calculate when ego is going to reach each (interpolated) points on the path
  TimeDistanceArray time_distance_array{};
  double dist_sum = 0.0;
  double passing_time = time_delay;
  time_distance_array.emplace_back(passing_time, dist_sum);

  // NOTE: `reference_path` is resampled in `reference_smoothed_path`, so
  // `last_intersection_stopline_candidate_idx` makes no sense
  const auto smoothed_path_closest_idx = motion_utils::findFirstNearestIndexWithSoftConstraints(
    smoothed_reference_path.points, path.points.at(closest_idx).point.pose,
    planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);

  const std::optional<size_t> upstream_stopline_idx_opt = [&]() -> std::optional<size_t> {
    if (upstream_stopline) {
      const auto upstream_stopline_point = path.points.at(upstream_stopline.value()).point.pose;
      return motion_utils::findFirstNearestIndexWithSoftConstraints(
        smoothed_reference_path.points, upstream_stopline_point,
        planner_data_->ego_nearest_dist_threshold, planner_data_->ego_nearest_yaw_threshold);
    } else {
      return std::nullopt;
    }
  }();

  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size() - 1; ++i) {
    const auto & p1 = smoothed_reference_path.points.at(i);
    const auto & p2 = smoothed_reference_path.points.at(i + 1);

    const double dist = tier4_autoware_utils::calcDistance2d(p1, p2);
    dist_sum += dist;

    // use average velocity between p1 and p2
    const double average_velocity =
      (p1.point.longitudinal_velocity_mps + p2.point.longitudinal_velocity_mps) / 2.0;
    const double passing_velocity = [=]() {
      if (use_upstream_velocity) {
        if (upstream_stopline_idx_opt && i > upstream_stopline_idx_opt.value()) {
          return minimum_upstream_velocity;
        }
        return std::max<double>(average_velocity, minimum_ego_velocity);
      } else {
        return std::max<double>(average_velocity, minimum_ego_velocity);
      }
    }();
    passing_time += (dist / passing_velocity);

    time_distance_array.emplace_back(passing_time, dist_sum);
  }
  debug_ttc_array->layout.dim.resize(3);
  debug_ttc_array->layout.dim.at(0).label = "lane_id_@[0][0], ttc_time, ttc_dist, path_x, path_y";
  debug_ttc_array->layout.dim.at(0).size = 5;
  debug_ttc_array->layout.dim.at(1).label = "values";
  debug_ttc_array->layout.dim.at(1).size = time_distance_array.size();
  debug_ttc_array->data.reserve(
    time_distance_array.size() * debug_ttc_array->layout.dim.at(0).size);
  for (unsigned i = 0; i < time_distance_array.size(); ++i) {
    debug_ttc_array->data.push_back(lane_id_);
  }
  for (const auto & [t, d] : time_distance_array) {
    debug_ttc_array->data.push_back(t);
  }
  for (const auto & [t, d] : time_distance_array) {
    debug_ttc_array->data.push_back(d);
  }
  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p = smoothed_reference_path.points.at(i).point.pose.position;
    debug_ttc_array->data.push_back(p.x);
  }
  for (size_t i = smoothed_path_closest_idx; i < smoothed_reference_path.points.size(); ++i) {
    const auto & p = smoothed_reference_path.points.at(i).point.pose.position;
    debug_ttc_array->data.push_back(p.y);
  }
  return time_distance_array;
}

static double getHighestCurvature(const lanelet::ConstLineString3d & centerline)
{
  std::vector<lanelet::ConstPoint3d> points;
  for (auto point = centerline.begin(); point != centerline.end(); point++) {
    points.push_back(*point);
  }

  SplineInterpolationPoints2d interpolation(points);
  const std::vector<double> curvatures = interpolation.getSplineInterpolatedCurvatures();
  std::vector<double> curvatures_positive;
  for (const auto & curvature : curvatures) {
    curvatures_positive.push_back(std::fabs(curvature));
  }
  return *std::max_element(curvatures_positive.begin(), curvatures_positive.end());
}

std::vector<lanelet::ConstLineString3d> IntersectionModule::generateDetectionLaneDivisions(
  lanelet::ConstLanelets detection_lanelets_all,
  const lanelet::routing::RoutingGraphPtr routing_graph_ptr, const double resolution)
{
  const double curvature_threshold =
    planner_param_.occlusion.attention_lane_crop_curvature_threshold;
  const double curvature_calculation_ds =
    planner_param_.occlusion.attention_lane_curvature_calculation_ds;

  using lanelet::utils::getCenterlineWithOffset;

  // (0) remove left/right lanelet
  lanelet::ConstLanelets detection_lanelets;
  for (const auto & detection_lanelet : detection_lanelets_all) {
    // TODO(Mamoru Sobue): instead of ignoring, only trim straight part of lanelet
    const auto fine_centerline =
      lanelet::utils::generateFineCenterline(detection_lanelet, curvature_calculation_ds);
    const double highest_curvature = getHighestCurvature(fine_centerline);
    if (highest_curvature > curvature_threshold) {
      continue;
    }
    detection_lanelets.push_back(detection_lanelet);
  }

  // (1) tsort detection_lanelets
  const auto [merged_detection_lanelets, originals] =
    util::mergeLaneletsByTopologicalSort(detection_lanelets, routing_graph_ptr);

  // (2) merge each branch to one lanelet
  // NOTE: somehow bg::area() for merged lanelet does not work, so calculate it here
  std::vector<std::pair<lanelet::ConstLanelet, double>> merged_lanelet_with_area;
  for (unsigned i = 0; i < merged_detection_lanelets.size(); ++i) {
    const auto & merged_detection_lanelet = merged_detection_lanelets.at(i);
    const auto & original = originals.at(i);
    double area = 0;
    for (const auto & partition : original) {
      area += bg::area(partition.polygon2d().basicPolygon());
    }
    merged_lanelet_with_area.emplace_back(merged_detection_lanelet, area);
  }

  // (3) discretize each merged lanelet
  std::vector<lanelet::ConstLineString3d> detection_divisions;
  for (const auto & [merged_lanelet, area] : merged_lanelet_with_area) {
    const double length = bg::length(merged_lanelet.centerline());
    const double width = area / length;
    for (int i = 0; i < static_cast<int>(width / resolution); ++i) {
      const double offset = resolution * i - width / 2;
      detection_divisions.push_back(
        getCenterlineWithOffset(merged_lanelet, offset, resolution).invert());
    }
    detection_divisions.push_back(
      getCenterlineWithOffset(merged_lanelet, width / 2, resolution).invert());
  }
  return detection_divisions;
}

IntersectionModule::OcclusionType IntersectionModule::getOcclusionStatus(
  const std::vector<lanelet::CompoundPolygon3d> & attention_areas,
  const lanelet::ConstLanelets & adjacent_lanelets,
  const lanelet::CompoundPolygon3d & first_attention_area,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const std::vector<lanelet::ConstLineString3d> & lane_divisions,
  const TargetObjects & target_objects)
{
  const auto & occ_grid = *planner_data_->occupancy_grid;
  const auto & current_pose = planner_data_->current_odometry->pose;
  const double occlusion_dist_thr = planner_param_.occlusion.occlusion_required_clearance_distance;

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

static std::optional<std::pair<
  size_t /* the index of interpolated PathPoint*/, size_t /* the index of corresponding Polygon */>>
getFirstPointInsidePolygonsByFootprint(
  const std::vector<lanelet::CompoundPolygon3d> & polygons,
  const util::InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length)
{
  const auto & path_ip = interpolated_path_info.path;
  const auto [lane_start, lane_end] = interpolated_path_info.lane_id_interval.value();
  const size_t vehicle_length_idx = static_cast<size_t>(vehicle_length / interpolated_path_info.ds);
  const size_t start =
    static_cast<size_t>(std::max<int>(0, static_cast<int>(lane_start) - vehicle_length_idx));

  for (size_t i = start; i <= lane_end; ++i) {
    const auto & pose = path_ip.points.at(i).point.pose;
    const auto path_footprint =
      tier4_autoware_utils::transformVector(footprint, tier4_autoware_utils::pose2transform(pose));
    for (size_t j = 0; j < polygons.size(); ++j) {
      const auto area_2d = lanelet::utils::to2D(polygons.at(j)).basicPolygon();
      const bool is_in_polygon = bg::intersects(area_2d, path_footprint);
      if (is_in_polygon) {
        return std::make_optional<std::pair<size_t, size_t>>(i, j);
      }
    }
  }
  return std::nullopt;
}

void IntersectionLanelets::update(
  const bool is_prioritized, const util::InterpolatedPathInfo & interpolated_path_info,
  const tier4_autoware_utils::LinearRing2d & footprint, const double vehicle_length,
  lanelet::routing::RoutingGraphPtr routing_graph_ptr)
{
  is_prioritized_ = is_prioritized;
  // find the first conflicting/detection area polygon intersecting the path
  if (!first_conflicting_area_) {
    auto first = getFirstPointInsidePolygonsByFootprint(
      conflicting_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_conflicting_lane_ = conflicting_.at(first.value().second);
      first_conflicting_area_ = conflicting_area_.at(first.value().second);
    }
  }
  if (!first_attention_area_) {
    const auto first = getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_area_, interpolated_path_info, footprint, vehicle_length);
    if (first) {
      first_attention_lane_ = attention_non_preceding_.at(first.value().second);
      first_attention_area_ = attention_non_preceding_area_.at(first.value().second);
    }
  }
  if (first_attention_lane_ && !second_attention_lane_ && !second_attention_lane_empty_) {
    const auto first_attention_lane = first_attention_lane_.value();
    // remove first_attention_area_ and non-straight lanelets from attention_non_preceding
    lanelet::ConstLanelets attention_non_preceding_ex_first;
    lanelet::ConstLanelets sibling_first_attention_lanelets;
    for (const auto & previous : routing_graph_ptr->previous(first_attention_lane)) {
      for (const auto & following : routing_graph_ptr->following(previous)) {
        sibling_first_attention_lanelets.push_back(following);
      }
    }
    for (const auto & ll : attention_non_preceding_) {
      // the sibling lanelets of first_attention_lanelet are ruled out
      if (lanelet::utils::contains(sibling_first_attention_lanelets, ll)) {
        continue;
      }
      if (std::string(ll.attributeOr("turn_direction", "else")).compare("straight") == 0) {
        attention_non_preceding_ex_first.push_back(ll);
      }
    }
    if (attention_non_preceding_ex_first.empty()) {
      second_attention_lane_empty_ = true;
    }
    const auto attention_non_preceding_ex_first_area =
      getPolygon3dFromLanelets(attention_non_preceding_ex_first);
    const auto second = getFirstPointInsidePolygonsByFootprint(
      attention_non_preceding_ex_first_area, interpolated_path_info, footprint, vehicle_length);
    if (second) {
      second_attention_lane_ = attention_non_preceding_ex_first.at(second.value().second);
      second_attention_area_ = second_attention_lane_.value().polygon3d();
    }
  }
}

void TargetObject::calc_dist_to_stopline()
{
  if (!attention_lanelet || !stopline) {
    return;
  }
  const auto attention_lanelet_val = attention_lanelet.value();
  const auto object_arc_coords = lanelet::utils::getArcCoordinates(
    {attention_lanelet_val}, object.kinematics.initial_pose_with_covariance.pose);
  const auto stopline_val = stopline.value();
  geometry_msgs::msg::Pose stopline_center;
  stopline_center.position.x = (stopline_val.front().x() + stopline_val.back().x()) / 2.0;
  stopline_center.position.y = (stopline_val.front().y() + stopline_val.back().y()) / 2.0;
  stopline_center.position.z = (stopline_val.front().z() + stopline_val.back().z()) / 2.0;
  const auto stopline_arc_coords =
    lanelet::utils::getArcCoordinates({attention_lanelet_val}, stopline_center);
  dist_to_stopline = (stopline_arc_coords.length - object_arc_coords.length);
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
