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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__HELPER_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__HELPER_HPP_

#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/utils/avoidance/utils.hpp"

#include <motion_utils/distance/distance.hpp>

#include <algorithm>
#include <memory>

namespace behavior_path_planner::helper::avoidance
{

using behavior_path_planner::PathShifter;
using behavior_path_planner::PlannerData;
using motion_utils::calcDecelDistWithJerkAndAccConstraints;
using motion_utils::findNearestIndex;

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

  double getNominalAvoidanceEgoSpeed() const
  {
    return std::max(getEgoSpeed(), parameters_->min_nominal_avoidance_speed);
  }

  double getSharpAvoidanceEgoSpeed() const
  {
    return std::max(getEgoSpeed(), parameters_->min_sharp_avoidance_speed);
  }

  float getMaximumAvoidanceEgoSpeed() const
  {
    return parameters_->target_velocity_matrix.at(parameters_->col_size - 1);
  }

  float getMinimumAvoidanceEgoSpeed() const { return parameters_->target_velocity_matrix.front(); }

  double getNominalPrepareDistance() const
  {
    const auto & p = parameters_;
    const auto epsilon_m = 0.01;  // for floating error to pass "has_enough_distance" check.
    const auto nominal_distance =
      std::max(getEgoSpeed() * p->prepare_time, p->min_prepare_distance);
    return nominal_distance + epsilon_m;
  }

  double getNominalAvoidanceDistance(const double shift_length) const
  {
    const auto & p = parameters_;
    const auto distance_by_jerk = PathShifter::calcLongitudinalDistFromJerk(
      shift_length, p->nominal_lateral_jerk, getNominalAvoidanceEgoSpeed());

    return std::max(p->min_avoidance_distance, distance_by_jerk);
  }

  double getMinimumAvoidanceDistance(const double shift_length) const
  {
    const auto & p = parameters_;
    const auto distance_by_jerk = PathShifter::calcLongitudinalDistFromJerk(
      shift_length, p->nominal_lateral_jerk, getMinimumAvoidanceEgoSpeed());

    return std::max(p->min_avoidance_distance, distance_by_jerk);
  }

  double getSharpAvoidanceDistance(const double shift_length) const
  {
    const auto & p = parameters_;
    const auto distance_by_jerk = PathShifter::calcLongitudinalDistFromJerk(
      shift_length, p->max_lateral_jerk, getSharpAvoidanceEgoSpeed());

    return std::max(p->min_avoidance_distance, distance_by_jerk);
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
    const auto idx = findNearestIndex(prev_reference_path_.points, p);
    return prev_spline_shift_path_.shift_length.at(idx);
  }

  double getLinearShift(const Point & p) const
  {
    validate();
    const auto idx = findNearestIndex(prev_reference_path_.points, p);
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
    using utils::avoidance::calcShiftLength;

    const auto shift_length = calcShiftLength(is_on_right, object.overhang_dist, margin);
    return is_on_right ? std::min(shift_length, getLeftShiftBound())
                       : std::max(shift_length, getRightShiftBound());
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

  boost::optional<double> getFeasibleDecelDistance(const double target_velocity) const
  {
    const auto & a_now = data_->self_acceleration->accel.accel.linear.x;
    const auto & a_lim = parameters_->max_deceleration;
    const auto & j_lim = parameters_->max_jerk;
    return calcDecelDistWithJerkAndAccConstraints(
      getEgoSpeed(), target_velocity, a_now, a_lim, j_lim, -1.0 * j_lim);
  }

  boost::optional<double> getMildDecelDistance(const double target_velocity) const
  {
    const auto & a_now = data_->self_acceleration->accel.accel.linear.x;
    const auto & a_lim = parameters_->nominal_deceleration;
    const auto & j_lim = parameters_->nominal_jerk;
    return calcDecelDistWithJerkAndAccConstraints(
      getEgoSpeed(), target_velocity, a_now, a_lim, j_lim, -1.0 * j_lim);
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
};
}  // namespace behavior_path_planner::helper::avoidance

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__AVOIDANCE__HELPER_HPP_
