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

#ifndef AUTOWARE_BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__HELPER_HPP_
#define AUTOWARE_BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__HELPER_HPP_

#include "autoware_behavior_path_planner_common/utils/utils.hpp"
#include "autoware_behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware_behavior_path_static_obstacle_avoidance_module/type_alias.hpp"
#include "autoware_behavior_path_static_obstacle_avoidance_module/utils.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace behavior_path_planner::helper::static_obstacle_avoidance
{

using behavior_path_planner::PathShifter;
using behavior_path_planner::PlannerData;

class AvoidanceHelper
{
public:
  explicit AvoidanceHelper(const std::shared_ptr<AvoidanceParameters> & parameters)
  : parameters_{parameters}
  {
  }

  void validate() const
  {
    const auto reference_points_size = prev_reference_path_.points.size();
    const auto linear_shift_size = prev_linear_shift_path_.shift_length.size();
    const auto spline_shift_size = prev_spline_shift_path_.shift_length.size();

    if (reference_points_size != linear_shift_size) {
      throw std::logic_error("there is an inconsistency among the previous data.");
    }

    if (reference_points_size != spline_shift_size) {
      throw std::logic_error("there is an inconsistency among the previous data.");
    }
  }

  double getEgoSpeed() const { return std::abs(data_->self_odometry->twist.twist.linear.x); }

  geometry_msgs::msg::Pose getEgoPose() const { return data_->self_odometry->pose.pose; }

  geometry_msgs::msg::Point getEgoPosition() const
  {
    return data_->self_odometry->pose.pose.position;
  }

  static size_t getConstraintsMapIndex(const double velocity, const std::vector<double> & values)
  {
    const auto itr = std::find_if(
      values.begin(), values.end(), [&](const auto value) { return velocity < value; });
    const auto idx = std::distance(values.begin(), itr);
    return 0 < idx ? idx - 1 : 0;
  }

  double getLateralMinJerkLimit() const
  {
    const auto idx = getConstraintsMapIndex(getEgoSpeed(), parameters_->velocity_map);
    return parameters_->lateral_min_jerk_map.at(idx);
  }

  double getLateralMaxJerkLimit() const
  {
    const auto idx = getConstraintsMapIndex(getEgoSpeed(), parameters_->velocity_map);
    return parameters_->lateral_max_jerk_map.at(idx);
  }

  double getLateralMaxAccelLimit() const
  {
    const auto idx = getConstraintsMapIndex(getEgoSpeed(), parameters_->velocity_map);
    return parameters_->lateral_max_accel_map.at(idx);
  }

  double getAvoidanceEgoSpeed() const
  {
    const auto & values = parameters_->velocity_map;
    const auto idx = getConstraintsMapIndex(getEgoSpeed(), values);
    return std::max(getEgoSpeed(), values.at(idx));
  }

  double getNominalPrepareDistance(const double velocity) const
  {
    const auto & p = parameters_;
    const auto & values = p->velocity_map;
    const auto idx = getConstraintsMapIndex(velocity, values);
    return std::max(values.at(idx) * p->max_prepare_time, p->min_prepare_distance);
  }

  double getNominalPrepareDistance() const
  {
    const auto & p = parameters_;
    return std::max(getAvoidanceEgoSpeed() * p->max_prepare_time, p->min_prepare_distance);
  }

  double getNominalAvoidanceDistance(const double shift_length) const
  {
    const auto & p = parameters_;
    const auto nominal_speed = std::max(getEgoSpeed(), p->nominal_avoidance_speed);
    const auto nominal_jerk =
      p->lateral_min_jerk_map.at(getConstraintsMapIndex(nominal_speed, p->velocity_map));
    return PathShifter::calcLongitudinalDistFromJerk(shift_length, nominal_jerk, nominal_speed);
  }

  double getMinAvoidanceDistance(const double shift_length) const
  {
    const auto & p = parameters_;
    return PathShifter::calcLongitudinalDistFromJerk(
      shift_length, p->lateral_max_jerk_map.front(), p->velocity_map.front());
  }

  double getMaxAvoidanceDistance(const double shift_length) const
  {
    const auto distance_from_jerk = PathShifter::calcLongitudinalDistFromJerk(
      shift_length, getLateralMinJerkLimit(), getAvoidanceEgoSpeed());
    return std::max(getNominalAvoidanceDistance(shift_length), distance_from_jerk);
  }

  double getSharpAvoidanceDistance(const double shift_length) const
  {
    return PathShifter::calcLongitudinalDistFromJerk(
      shift_length, getLateralMaxJerkLimit(), getAvoidanceEgoSpeed());
  }

  double getFrontConstantDistance(const ObjectData & object) const
  {
    const auto object_type = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters_->object_parameters.at(object_type);
    if (!parameters_->consider_front_overhang) {
      return object_parameter.longitudinal_margin;
    }
    return object_parameter.longitudinal_margin + data_->parameters.base_link2front;
  }

  double getRearConstantDistance(const ObjectData & object) const
  {
    const auto object_type = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters_->object_parameters.at(object_type);
    if (!parameters_->consider_rear_overhang) {
      return object_parameter.longitudinal_margin;
    }
    return object_parameter.longitudinal_margin + data_->parameters.base_link2rear + object.length;
  }

  double getEgoShift() const
  {
    validate();
    const auto idx = data_->findEgoIndex(prev_reference_path_.points);
    return prev_spline_shift_path_.shift_length.at(idx);
  }

  double getEgoLinearShift() const
  {
    validate();
    const auto idx = data_->findEgoIndex(prev_reference_path_.points);
    return prev_linear_shift_path_.shift_length.at(idx);
  }

  double getShift(const Point & p) const
  {
    validate();
    const auto idx = motion_utils::findNearestIndex(prev_reference_path_.points, p);
    return prev_spline_shift_path_.shift_length.at(idx);
  }

  double getLinearShift(const Point & p) const
  {
    validate();
    const auto idx = motion_utils::findNearestIndex(prev_reference_path_.points, p);
    return prev_linear_shift_path_.shift_length.at(idx);
  }

  double getRelativeShiftToPath(const AvoidLine & line) const
  {
    return line.end_shift_length - getLinearShift(line.end.position);
  }

  double getRightShiftBound() const { return -parameters_->max_right_shift_length; }

  double getLeftShiftBound() const { return parameters_->max_left_shift_length; }

  double getShiftLength(
    const ObjectData & object, const bool & is_on_right, const double & margin) const
  {
    using utils::static_obstacle_avoidance::calcShiftLength;

    const auto shift_length =
      calcShiftLength(is_on_right, object.overhang_points.front().first, margin);
    return is_on_right ? std::min(shift_length, getLeftShiftBound())
                       : std::max(shift_length, getRightShiftBound());
  }

  double getForwardDetectionRange() const
  {
    if (parameters_->use_static_detection_area) {
      return parameters_->object_check_max_forward_distance;
    }

    const auto & route_handler = data_->route_handler;

    lanelet::ConstLanelet closest_lane;
    if (!route_handler->getClosestLaneletWithinRoute(getEgoPose(), &closest_lane)) {
      return parameters_->object_check_max_forward_distance;
    }

    const auto limit = route_handler->getTrafficRulesPtr()->speedLimit(closest_lane);
    const auto speed = isShifted() ? limit.speedLimit.value() : getEgoSpeed();

    const auto max_shift_length = std::max(
      std::abs(parameters_->max_right_shift_length), std::abs(parameters_->max_left_shift_length));
    const auto dynamic_distance =
      PathShifter::calcLongitudinalDistFromJerk(max_shift_length, getLateralMinJerkLimit(), speed);

    return std::clamp(
      1.5 * dynamic_distance + getNominalPrepareDistance(),
      parameters_->object_check_min_forward_distance,
      parameters_->object_check_max_forward_distance);
  }

  void alignShiftLinesOrder(AvoidLineArray & lines, const bool align_shift_length = true) const
  {
    if (lines.empty()) {
      return;
    }

    std::sort(lines.begin(), lines.end(), [](auto a, auto b) {
      return a.end_longitudinal < b.end_longitudinal;
    });

    if (!align_shift_length) {
      return;
    }

    lines.front().start_shift_length = getEgoLinearShift();
    for (size_t i = 1; i < lines.size(); ++i) {
      lines.at(i).start_shift_length = lines.at(i - 1).end_shift_length;
    }
  }

  AvoidLine getMainShiftLine(const AvoidLineArray & lines) const
  {
    const auto itr =
      std::max_element(lines.begin(), lines.end(), [this](const auto & a, const auto & b) {
        return std::abs(getRelativeShiftToPath(a)) < std::abs(getRelativeShiftToPath(b));
      });

    if (itr == lines.end()) {
      return {};
    }

    return *itr;
  }

  std::pair<double, double> getDistanceToAccelEndPoint(const PathWithLaneId & path)
  {
    updateAccelEndPoint(path);

    if (!max_v_point_.has_value()) {
      return std::make_pair(0.0, std::numeric_limits<double>::max());
    }

    const auto start_idx = data_->findEgoIndex(path.points);
    const auto distance = motion_utils::calcSignedArcLength(
      path.points, start_idx, max_v_point_.value().first.position);
    return std::make_pair(distance, max_v_point_.value().second);
  }

  double getFeasibleDecelDistance(
    const double target_velocity, const bool use_hard_constraints = true) const
  {
    const auto & p = parameters_;
    const auto & a_now = data_->self_acceleration->accel.accel.linear.x;
    const auto & a_lim = use_hard_constraints ? p->max_deceleration : p->nominal_deceleration;
    const auto & j_lim = use_hard_constraints ? p->max_jerk : p->nominal_jerk;
    const auto ret = motion_utils::calcDecelDistWithJerkAndAccConstraints(
      getEgoSpeed(), target_velocity, a_now, a_lim, j_lim, -1.0 * j_lim);

    if (!!ret) {
      return ret.value();
    }

    return std::numeric_limits<double>::max();
  }

  bool isEnoughPrepareDistance(const double prepare_distance) const
  {
    const auto & p = parameters_;
    return prepare_distance >
           std::max(getEgoSpeed() * p->min_prepare_distance, p->min_prepare_distance);
  }

  bool isComfortable(const AvoidLineArray & shift_lines) const
  {
    const auto JERK_BUFFER = 0.1;  // [m/sss]
    return std::all_of(shift_lines.begin(), shift_lines.end(), [&](const auto & line) {
      return PathShifter::calcJerkFromLatLonDistance(
               line.getRelativeLength(), line.getRelativeLongitudinal(), getAvoidanceEgoSpeed()) <
             getLateralMaxJerkLimit() + JERK_BUFFER;
    });
  }

  bool isReady(const ObjectDataArray & objects) const
  {
    if (objects.empty()) {
      return true;
    }

    const auto object = objects.front();

    if (!object.is_ambiguous) {
      return true;
    }

    if (!object.avoid_margin.has_value()) {
      return true;
    }

    const auto is_object_on_right = utils::static_obstacle_avoidance::isOnRight(object);
    const auto desire_shift_length =
      getShiftLength(object, is_object_on_right, object.avoid_margin.value());

    const auto prepare_distance = getNominalPrepareDistance(0.0);
    const auto constant_distance = getFrontConstantDistance(object);
    const auto avoidance_distance = getMinAvoidanceDistance(desire_shift_length);

    return object.longitudinal <
           prepare_distance + constant_distance + avoidance_distance +
             parameters_->closest_distance_to_wait_and_see_for_ambiguous_vehicle;
  }

  static bool isAbsolutelyNotAvoidable(const ObjectData & object)
  {
    if (object.is_avoidable) {
      return false;
    }

    // can avoid it after deceleration.
    if (object.info == ObjectInfo::NEED_DECELERATION) {
      return false;
    }

    // can avoid it after relax drivable space limitation.
    if (object.info == ObjectInfo::LIMIT_DRIVABLE_SPACE_TEMPORARY) {
      return false;
    }

    return true;
  }

  bool isReady(const AvoidLineArray & new_shift_lines, const double current_shift_length) const
  {
    if (std::abs(current_shift_length) < 1e-3) {
      return true;
    }

    if (new_shift_lines.empty()) {
      return true;
    }

    const auto front_shift_relative_length = new_shift_lines.front().getRelativeLength();

    // same direction shift.
    if (current_shift_length > 0.0 && front_shift_relative_length > 0.0) {
      return true;
    }

    // same direction shift.
    if (current_shift_length < 0.0 && front_shift_relative_length < 0.0) {
      return true;
    }

    // keep waiting the other side shift approval until the ego reaches shift length threshold.
    const auto ego_shift_ratio = (current_shift_length - getEgoShift()) / current_shift_length;
    return ego_shift_ratio < parameters_->ratio_for_return_shift_approval + 1e-3;
  }

  bool isShifted() const
  {
    return std::abs(getEgoShift()) > parameters_->lateral_execution_threshold;
  }

  bool isInitialized() const
  {
    if (prev_spline_shift_path_.path.points.empty()) {
      return false;
    }

    if (prev_linear_shift_path_.path.points.empty()) {
      return false;
    }

    if (prev_reference_path_.points.empty()) {
      return false;
    }

    return true;
  }

  void updateAccelEndPoint(const PathWithLaneId & path)
  {
    const auto & p = parameters_;
    const auto & a_now = data_->self_acceleration->accel.accel.linear.x;
    if (a_now < 0.0) {
      max_v_point_ = std::nullopt;
      return;
    }

    if (getEgoSpeed() < p->min_velocity_to_limit_max_acceleration) {
      max_v_point_ = std::nullopt;
      return;
    }

    if (max_v_point_.has_value()) {
      return;
    }

    const auto v0 = getEgoSpeed() + p->buf_slow_down_speed;

    const auto t_neg_jerk = std::max(0.0, a_now - p->max_acceleration) / p->max_jerk;
    const auto v_neg_jerk = v0 + a_now * t_neg_jerk + std::pow(t_neg_jerk, 2.0) / 2.0;
    const auto x_neg_jerk = v0 * t_neg_jerk + a_now * std::pow(t_neg_jerk, 2.0) / 2.0 +
                            p->max_jerk * std::pow(t_neg_jerk, 3.0) / 6.0;

    const auto & v_max = data_->parameters.max_vel;
    if (v_max < v_neg_jerk) {
      max_v_point_ = std::nullopt;
      return;
    }

    const auto t_max_accel = (v_max - v_neg_jerk) / p->max_acceleration;
    const auto x_max_accel =
      v_neg_jerk * t_max_accel + p->max_acceleration * std::pow(t_max_accel, 2.0) / 2.0;

    const auto point = motion_utils::calcLongitudinalOffsetPose(
      path.points, getEgoPosition(), x_neg_jerk + x_max_accel);
    if (point.has_value()) {
      max_v_point_ = std::make_pair(point.value(), v_max);
      return;
    }

    const auto x_end =
      motion_utils::calcSignedArcLength(path.points, getEgoPosition(), path.points.size() - 1);
    const auto t_end =
      (std::sqrt(v0 * v0 + 2.0 * p->max_acceleration * x_end) - v0) / p->max_acceleration;
    const auto v_end = v0 + p->max_acceleration * t_end;

    max_v_point_ = std::make_pair(getPose(path.points.back()), v_end);
  }

  void reset()
  {
    prev_reference_path_ = PathWithLaneId();

    prev_spline_shift_path_ = ShiftedPath();

    prev_linear_shift_path_ = ShiftedPath();

    prev_driving_lanes_.clear();
  }

  void setData(const std::shared_ptr<const PlannerData> & data) { data_ = data; }

  void setPreviousReferencePath(const PathWithLaneId & path) { prev_reference_path_ = path; }

  void setPreviousSplineShiftPath(const ShiftedPath & shift_path)
  {
    prev_spline_shift_path_ = shift_path;
  }

  void setPreviousLinearShiftPath(const ShiftedPath & shift_path)
  {
    prev_linear_shift_path_ = shift_path;
  }

  void setPreviousDrivingLanes(const lanelet::ConstLanelets & lanes)
  {
    prev_driving_lanes_ = lanes;
  }

  PathWithLaneId getPreviousReferencePath() const { return prev_reference_path_; }

  ShiftedPath getPreviousSplineShiftPath() const { return prev_spline_shift_path_; }

  ShiftedPath getPreviousLinearShiftPath() const { return prev_linear_shift_path_; }

  lanelet::ConstLanelets getPreviousDrivingLanes() const { return prev_driving_lanes_; }

private:
  std::shared_ptr<const PlannerData> data_;

  std::shared_ptr<AvoidanceParameters> parameters_;

  PathWithLaneId prev_reference_path_;

  ShiftedPath prev_spline_shift_path_;

  ShiftedPath prev_linear_shift_path_;

  lanelet::ConstLanelets prev_driving_lanes_;

  std::optional<std::pair<Pose, double>> max_v_point_;
};
}  // namespace behavior_path_planner::helper::static_obstacle_avoidance

#endif  // AUTOWARE_BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__HELPER_HPP_
