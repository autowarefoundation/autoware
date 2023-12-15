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

#include "scene_no_stopping_area.hpp"

#include <behavior_velocity_planner_common/utilization/arc_lane_util.hpp>
#include <behavior_velocity_planner_common/utilization/path_utilization.hpp>
#include <behavior_velocity_planner_common/utilization/util.hpp>
#include <interpolation/spline_interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/boost_polygon_utils.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/utility/Optional.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_velocity_planner
{
namespace bg = boost::geometry;

NoStoppingAreaModule::NoStoppingAreaModule(
  const int64_t module_id, const int64_t lane_id,
  const lanelet::autoware::NoStoppingArea & no_stopping_area_reg_elem,
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock)
: SceneModuleInterface(module_id, logger, clock),
  lane_id_(lane_id),
  no_stopping_area_reg_elem_(no_stopping_area_reg_elem),
  planner_param_(planner_param)
{
  velocity_factor_.init(PlanningBehavior::NO_STOPPING_AREA);
  state_machine_.setState(StateMachine::State::GO);
  state_machine_.setMarginTime(planner_param_.state_clear_time);
}

boost::optional<LineString2d> NoStoppingAreaModule::getStopLineGeometry2d(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const double stop_line_margin) const
{
  // get stop line from map
  {
    const auto & stop_line = no_stopping_area_reg_elem_.stopLine();
    if (stop_line) {
      return planning_utils::extendLine(
        stop_line.value()[0], stop_line.value()[1], planner_data_->stop_line_extend_length);
    }
  }
  // auto gen stop line
  {
    LineString2d stop_line;
    /**
     * @brief auto gen no stopping area stop line from area polygon if stop line is not set
     *        ---------------
     * ------col-------------|--> ego path
     *        |     Area     |
     *        ---------------
     **/

    for (const auto & no_stopping_area : no_stopping_area_reg_elem_.noStoppingAreas()) {
      const auto & area_poly = lanelet::utils::to2D(no_stopping_area).basicPolygon();
      lanelet::BasicLineString2d path_line;
      for (size_t i = 0; i < path.points.size() - 1; ++i) {
        const auto p0 = path.points.at(i).point.pose.position;
        const auto p1 = path.points.at(i + 1).point.pose.position;
        const LineString2d line{{p0.x, p0.y}, {p1.x, p1.y}};
        std::vector<Point2d> collision_points;
        bg::intersection(area_poly, line, collision_points);
        if (collision_points.empty()) {
          continue;
        }
        const double yaw = tier4_autoware_utils::calcAzimuthAngle(p0, p1);
        if (!collision_points.empty()) {
          geometry_msgs::msg::Point left_point;
          const double w = planner_data_->vehicle_info_.vehicle_width_m;
          const double l = stop_line_margin;
          stop_line.emplace_back(
            -l * std::cos(yaw) + collision_points.front().x() + w * std::cos(yaw + M_PI_2),
            collision_points.front().y() + w * std::sin(yaw + M_PI_2));
          stop_line.emplace_back(
            -l * std::cos(yaw) + collision_points.front().x() + w * std::cos(yaw - M_PI_2),
            collision_points.front().y() + w * std::sin(yaw - M_PI_2));
          return stop_line;
        }
      }
    }
  }
  return {};
}

bool NoStoppingAreaModule::modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason)
{
  // Store original path
  const auto original_path = *path;
  const auto & predicted_obj_arr_ptr = planner_data_->predicted_objects;
  const auto & current_pose = planner_data_->current_odometry;
  if (path->points.size() <= 2) {
    return true;
  }
  // Reset data
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data_->vehicle_info_.max_longitudinal_offset_m;
  *stop_reason = planning_utils::initializeStopReason(StopReason::NO_STOPPING_AREA);

  // Get stop line geometry
  const auto stop_line = getStopLineGeometry2d(original_path, planner_param_.stop_line_margin);
  if (!stop_line) {
    setSafe(true);
    return true;
  }
  const auto stop_point = arc_lane_utils::createTargetPoint(
    original_path, stop_line.value(), lane_id_, planner_param_.stop_margin,
    planner_data_->vehicle_info_.max_longitudinal_offset_m);
  if (!stop_point) {
    setSafe(true);
    return true;
  }
  const auto & stop_pose = stop_point->second;
  setDistance(motion_utils::calcSignedArcLength(
    original_path.points, current_pose->pose.position, stop_pose.position));
  if (planning_utils::isOverLine(
        original_path, current_pose->pose, stop_pose, planner_param_.dead_line_margin)) {
    // ego can't stop in front of no stopping area -> GO or OR
    state_machine_.setState(StateMachine::State::GO);
    setSafe(true);
    return true;
  }
  const auto & vi = planner_data_->vehicle_info_;
  const double margin = planner_param_.stop_line_margin;
  const double ego_space_in_front_of_stuck_vehicle =
    margin + vi.vehicle_length_m + planner_param_.stuck_vehicle_front_margin;
  const Polygon2d stuck_vehicle_detect_area = generateEgoNoStoppingAreaLanePolygon(
    *path, current_pose->pose, ego_space_in_front_of_stuck_vehicle,
    planner_param_.detection_area_length);
  const double ego_space_in_front_of_stop_line =
    margin + planner_param_.stop_margin + vi.rear_overhang_m;
  const Polygon2d stop_line_detect_area = generateEgoNoStoppingAreaLanePolygon(
    *path, current_pose->pose, ego_space_in_front_of_stop_line,
    planner_param_.detection_area_length);
  if (stuck_vehicle_detect_area.outer().empty() && stop_line_detect_area.outer().empty()) {
    setSafe(true);
    return true;
  }
  debug_data_.stuck_vehicle_detect_area = toGeomPoly(stuck_vehicle_detect_area);
  debug_data_.stop_line_detect_area = toGeomPoly(stop_line_detect_area);
  // Find stuck vehicle in no stopping area
  const bool is_entry_prohibited_by_stuck_vehicle =
    checkStuckVehiclesInNoStoppingArea(stuck_vehicle_detect_area, predicted_obj_arr_ptr);
  // Find stop line in no stopping area
  const bool is_entry_prohibited_by_stop_line =
    checkStopLinesInNoStoppingArea(*path, stop_line_detect_area);
  const bool is_entry_prohibited =
    is_entry_prohibited_by_stuck_vehicle || is_entry_prohibited_by_stop_line;
  if (!isStoppable(current_pose->pose, stop_point->second)) {
    state_machine_.setState(StateMachine::State::GO);
    setSafe(true);
    return false;
  } else {
    state_machine_.setStateWithMarginTime(
      is_entry_prohibited ? StateMachine::State::STOP : StateMachine::State::GO,
      logger_.get_child("state_machine"), *clock_);
  }

  setSafe(state_machine_.getState() != StateMachine::State::STOP);
  if (!isActivated()) {
    // ----------------stop reason and stop point--------------------------
    insertStopPoint(*path, *stop_point);
    // For virtual wall
    debug_data_.stop_poses.push_back(stop_pose);

    // Create StopReason
    {
      tier4_planning_msgs::msg::StopFactor stop_factor;
      stop_factor.stop_pose = stop_point->second;
      stop_factor.stop_factor_points = debug_data_.stuck_points;
      planning_utils::appendStopReason(stop_factor, stop_reason);
      velocity_factor_.set(
        path->points, planner_data_->current_odometry->pose, stop_point->second,
        VelocityFactor::UNKNOWN);
    }

    // Create legacy StopReason
    {
      const auto insert_idx = stop_point->first + 1;
      if (
        !first_stop_path_point_index_ ||
        static_cast<int>(insert_idx) < first_stop_path_point_index_) {
        debug_data_.first_stop_pose = stop_pose;
        first_stop_path_point_index_ = static_cast<int>(insert_idx);
      }
    }
  } else if (state_machine_.getState() == StateMachine::State::GO) {
    // reset pass judge if current state is go
    is_stoppable_ = true;
    pass_judged_ = false;
  }
  return true;
}

bool NoStoppingAreaModule::checkStuckVehiclesInNoStoppingArea(
  const Polygon2d & poly,
  const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr &
    predicted_obj_arr_ptr)
{
  // stuck points by predicted objects
  for (const auto & object : predicted_obj_arr_ptr->objects) {
    if (!isTargetStuckVehicleType(object)) {
      continue;  // not target vehicle type
    }
    const auto obj_v = std::fabs(object.kinematics.initial_twist_with_covariance.twist.linear.x);
    if (obj_v > planner_param_.stuck_vehicle_vel_thr) {
      setObjectsOfInterestData(
        object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::GREEN);
      continue;  // not stop vehicle
    }
    // check if the footprint is in the stuck detect area
    const Polygon2d obj_footprint = tier4_autoware_utils::toPolygon2d(object);
    const bool is_in_stuck_area = !bg::disjoint(obj_footprint, poly);
    if (is_in_stuck_area) {
      RCLCPP_DEBUG(logger_, "stuck vehicle found.");
      setObjectsOfInterestData(
        object.kinematics.initial_pose_with_covariance.pose, object.shape, ColorName::RED);
      for (const auto & p : obj_footprint.outer()) {
        geometry_msgs::msg::Point point;
        point.x = p.x();
        point.y = p.y();
        point.z = 0.0;
        debug_data_.stuck_points.emplace_back(point);
      }
      return true;
    }
  }
  return false;
}
bool NoStoppingAreaModule::checkStopLinesInNoStoppingArea(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const Polygon2d & poly)
{
  const double stop_vel = std::numeric_limits<float>::min();
  // stuck points by stop line
  for (size_t i = 0; i < path.points.size() - 1; ++i) {
    const auto p0 = path.points.at(i).point.pose.position;
    const auto p1 = path.points.at(i + 1).point.pose.position;
    const auto v0 = path.points.at(i).point.longitudinal_velocity_mps;
    const auto v1 = path.points.at(i + 1).point.longitudinal_velocity_mps;
    if (v0 > stop_vel && v1 > stop_vel) {
      continue;
    }
    const LineString2d line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point2d> collision_points;
    bg::intersection(poly, line, collision_points);
    if (!collision_points.empty()) {
      geometry_msgs::msg::Point point;
      point.x = collision_points.front().x();
      point.y = collision_points.front().y();
      point.z = 0.0;
      debug_data_.stuck_points.emplace_back(point);
      return true;
    }
  }
  return false;
}

Polygon2d NoStoppingAreaModule::generateEgoNoStoppingAreaLanePolygon(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Pose & ego_pose, const double margin, const double extra_dist) const
{
  Polygon2d ego_area;  // open polygon
  double dist_from_start_sum = 0.0;
  const double interpolation_interval = 0.5;
  bool is_in_area = false;
  autoware_auto_planning_msgs::msg::PathWithLaneId interpolated_path;
  if (!splineInterpolate(path, interpolation_interval, interpolated_path, logger_)) {
    return ego_area;
  }
  auto & pp = interpolated_path.points;
  /* calc closest index */
  const auto closest_idx_opt =
    motion_utils::findNearestIndex(interpolated_path.points, ego_pose, 3.0, M_PI_4);
  if (!closest_idx_opt) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      logger_, *clock_, 1000 /* ms */, "motion_utils::findNearestIndex fail");
    return ego_area;
  }
  const size_t closest_idx = closest_idx_opt.value();

  const int num_ignore_nearest = 1;  // Do not consider nearest lane polygon
  size_t ego_area_start_idx = closest_idx + num_ignore_nearest;
  size_t ego_area_end_idx = ego_area_start_idx;
  // return if area size is not intentional
  if (no_stopping_area_reg_elem_.noStoppingAreas().size() != 1) {
    return ego_area;
  }
  const auto no_stopping_area = no_stopping_area_reg_elem_.noStoppingAreas().front();
  for (size_t i = closest_idx + num_ignore_nearest; i < pp.size() - 1; ++i) {
    dist_from_start_sum += tier4_autoware_utils::calcDistance2d(pp.at(i), pp.at(i - 1));
    const auto & p = pp.at(i).point.pose.position;
    if (bg::within(Point2d{p.x, p.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon())) {
      is_in_area = true;
      break;
    }
    if (dist_from_start_sum > extra_dist) {
      return ego_area;
    }
    ++ego_area_start_idx;
  }
  if (ego_area_start_idx > num_ignore_nearest) {
    ego_area_start_idx--;
  }
  if (!is_in_area) {
    return ego_area;
  }
  double dist_from_area_sum = 0.0;
  // decide end idx with extract distance
  ego_area_end_idx = ego_area_start_idx;
  for (size_t i = ego_area_start_idx; i < pp.size() - 1; ++i) {
    dist_from_start_sum += tier4_autoware_utils::calcDistance2d(pp.at(i), pp.at(i - 1));
    const auto & p = pp.at(i).point.pose.position;
    if (!bg::within(Point2d{p.x, p.y}, lanelet::utils::to2D(no_stopping_area).basicPolygon())) {
      dist_from_area_sum += tier4_autoware_utils::calcDistance2d(pp.at(i), pp.at(i - 1));
    }
    if (dist_from_start_sum > extra_dist || dist_from_area_sum > margin) {
      break;
    }
    ++ego_area_end_idx;
  }

  const auto width = planner_param_.path_expand_width;
  ego_area = planning_utils::generatePathPolygon(
    interpolated_path, ego_area_start_idx, ego_area_end_idx, width);
  return ego_area;
}

bool NoStoppingAreaModule::isTargetStuckVehicleType(
  const autoware_auto_perception_msgs::msg::PredictedObject & object) const
{
  if (
    object.classification.front().label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::CAR ||
    object.classification.front().label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::BUS ||
    object.classification.front().label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRUCK ||
    object.classification.front().label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::TRAILER ||
    object.classification.front().label ==
      autoware_auto_perception_msgs::msg::ObjectClassification::MOTORCYCLE) {
    return true;
  }
  return false;
}

bool NoStoppingAreaModule::isStoppable(
  const geometry_msgs::msg::Pose & self_pose, const geometry_msgs::msg::Pose & line_pose) const
{
  // get vehicle info and compute pass_judge_line_distance
  const auto current_velocity = planner_data_->current_velocity->twist.linear.x;
  const auto current_acceleration = planner_data_->current_acceleration->accel.accel.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold;
  const double max_jerk = planner_data_->max_stop_jerk_threshold;
  const double delay_response_time = planner_data_->delay_response_time;
  const double stoppable_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    current_velocity, current_acceleration, max_acc, max_jerk, delay_response_time);
  const double signed_arc_length =
    arc_lane_utils::calcSignedDistance(self_pose, line_pose.position);
  const bool distance_stoppable = stoppable_distance < signed_arc_length;
  const bool slow_velocity = planner_data_->current_velocity->twist.linear.x < 2.0;
  // ego vehicle is high speed and can't stop before stop line -> GO
  const bool not_stoppable = !distance_stoppable && !slow_velocity;
  // stoppable or not is judged only once
  RCLCPP_DEBUG(
    logger_, "stoppable_dist: %lf signed_arc_length: %lf", stoppable_distance, signed_arc_length);
  if (!distance_stoppable && !pass_judged_) {
    pass_judged_ = true;
    // can't stop using maximum brake consider jerk limit
    if (not_stoppable) {
      // pass through
      is_stoppable_ = false;
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[NoStoppingArea] can't stop in front of no stopping area");
    } else {
      is_stoppable_ = true;
    }
  }
  return is_stoppable_;
}

void NoStoppingAreaModule::insertStopPoint(
  autoware_auto_planning_msgs::msg::PathWithLaneId & path, const PathIndexWithPose & stop_point)
{
  size_t insert_idx = static_cast<size_t>(stop_point.first + 1);
  const auto stop_pose = stop_point.second;

  // To PathPointWithLaneId
  autoware_auto_planning_msgs::msg::PathPointWithLaneId stop_point_with_lane_id;
  stop_point_with_lane_id = path.points.at(insert_idx);
  stop_point_with_lane_id.point.pose = stop_pose;
  stop_point_with_lane_id.point.longitudinal_velocity_mps = 0.0;

  // Insert stop point or replace with zero velocity
  planning_utils::insertVelocity(path, stop_point_with_lane_id, 0.0, insert_idx);
}
}  // namespace behavior_velocity_planner
