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

#include "behavior_path_planner/path_shifter/path_shifter.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{
// for debug
std::string toStr(const geometry_msgs::msg::Point & p)
{
  return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " + std::to_string(p.z) + ")";
}
std::string toStr(const behavior_path_planner::ShiftPoint & p)
{
  return "start point = " + toStr(p.start.position) + ", end point = " + toStr(p.end.position) +
         ", start idx = " + std::to_string(p.start_idx) +
         ", end idx = " + std::to_string(p.end_idx) + ", length = " + std::to_string(p.length);
}
}  // namespace

namespace behavior_path_planner
{
void PathShifter::setPath(const PathWithLaneId & path)
{
  reference_path_ = path;
  is_index_aligned_ = false;  // shift_point index has to be updated for new path.
}
void PathShifter::addShiftPoint(const ShiftPoint & point)
{
  shift_points_.push_back(point);
  is_index_aligned_ = false;  // shift_point index has to be updated for new shift points.
}

void PathShifter::setShiftPoints(const std::vector<ShiftPoint> & points)
{
  shift_points_ = points;
  is_index_aligned_ = false;  // shift_point index has to be updated for new shift points.
}

bool PathShifter::generate(
  ShiftedPath * shifted_path, const bool offset_back, const SHIFT_TYPE type)
{
  RCLCPP_DEBUG_STREAM(logger_, "PathShifter::generate start!");

  // Guard
  if (reference_path_.points.empty()) {
    RCLCPP_ERROR_STREAM(logger_, "reference path is empty.");
    return false;
  }

  shifted_path->path = reference_path_;
  shifted_path->shift_length.resize(reference_path_.points.size(), 0.0);

  if (shift_points_.empty()) {
    RCLCPP_DEBUG_STREAM(logger_, "shift_points_ is empty. Return reference with base offset.");
    shiftBaseLength(shifted_path, base_offset_);
    return true;
  }

  if (!is_index_aligned_) {
    updateShiftPointIndices();
    for (const auto & shift_point : shift_points_) {
      int idx_gap = shift_point.end_idx - shift_point.start_idx;
      if (idx_gap <= 1) {
        RCLCPP_WARN_STREAM(
          logger_,
          "shift start point and end point can't be adjoining "
          "Maybe shift length is too short?");
        return false;
      }
    }
  }

  // Sort shift points since applyShift function only supports sorted points
  if (!sortShiftPointsAlongPath(reference_path_)) {
    RCLCPP_ERROR_STREAM(logger_, "has duplicated points. Failed!");
    return false;
  }

  if (shift_points_.front().start_idx == 0) {
    // if offset is applied on front side, shifting from first point is no problem
    if (offset_back) {
      RCLCPP_WARN_STREAM(
        logger_,
        "shift start point is at the edge of path. It could cause undesired result."
        " Maybe path is too short for backward?");
    }
  }

  // Calculate shifted path
  type == SHIFT_TYPE::SPLINE ? applySplineShifter(shifted_path, offset_back)
                             : applyLinearShifter(shifted_path);

  // DEBUG
  RCLCPP_DEBUG_STREAM(
    logger_, "PathShifter::generate end. shift_points_.size = " << shift_points_.size());

  return true;
}

void PathShifter::applyLinearShifter(ShiftedPath * shifted_path)
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  shiftBaseLength(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_points,
  for (const auto & shift_point : shift_points_) {
    const auto current_shift = shifted_path->shift_length.at(shift_point.end_idx);
    const auto delta_shift = shift_point.length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_point.end_idx) - arclength_arr.at(shift_point.start_idx), epsilon);

    // For all path.points,
    for (size_t i = 0; i < shifted_path->path.points.size(); ++i) {
      // Set shift length.
      double ith_shift_length;
      if (i < shift_point.start_idx) {
        ith_shift_length = 0.0;
      } else if (shift_point.start_idx <= i && i <= shift_point.end_idx) {
        auto dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_point.start_idx);
        ith_shift_length = (dist_from_start / shifting_arclength) * delta_shift;
      } else {
        ith_shift_length = delta_shift;
      }

      // Apply shifting.
      addLateralOffsetOnIndexPoint(shifted_path, ith_shift_length, i);
    }
  }
}

void PathShifter::applySplineShifter(ShiftedPath * shifted_path, const bool offset_back)
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  shiftBaseLength(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_points,
  for (const auto & shift_point : shift_points_) {
    // calc delta shift at the sp.end_idx so that the sp.end_idx on the path will have
    // the desired shift length.
    const auto current_shift = shifted_path->shift_length.at(shift_point.end_idx);
    const auto delta_shift = shift_point.length - current_shift;

    RCLCPP_DEBUG(logger_, "current_shift = %f, sp.length = %f", current_shift, shift_point.length);

    if (std::abs(delta_shift) < 0.01) {
      RCLCPP_DEBUG(logger_, "delta shift is zero. skip for this shift point.");
    }

    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_point.end_idx) - arclength_arr.at(shift_point.start_idx), epsilon);

    // TODO(Watanabe) write docs.
    // These points are defined to achieve the constant-jerk shifting (see the header description).
    const std::vector<double> base_distance = {
      0.0, shifting_arclength / 4.0, shifting_arclength * 3.0 / 4.0, shifting_arclength};
    const auto base_length =
      offset_back
        ? std::vector<double>{0.0, delta_shift / 12.0, delta_shift * 11.0 / 12.0, delta_shift}
        : std::vector<double>{delta_shift, delta_shift * 11.0 / 12.0, delta_shift / 12.0, 0.0};

    std::vector<double> query_distance, query_length;

    // For all path.points,
    // Note: start_idx is not included since shift = 0,
    //       end_idx is not included since shift is considered out of spline.
    for (size_t i = shift_point.start_idx + 1; i < shift_point.end_idx; ++i) {
      const double dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_point.start_idx);
      query_distance.push_back(dist_from_start);
    }
    if (!query_distance.empty()) {
      query_length = interpolation::slerp(base_distance, base_length, query_distance);
    }

    // Apply shifting.
    {
      size_t i = shift_point.start_idx + 1;
      for (const auto & itr : query_length) {
        addLateralOffsetOnIndexPoint(shifted_path, itr, i);
        ++i;
      }
    }

    if (offset_back == true) {
      // Apply shifting after shift
      for (size_t i = shift_point.end_idx; i < shifted_path->path.points.size(); ++i) {
        addLateralOffsetOnIndexPoint(shifted_path, delta_shift, i);
      }
    } else {
      // Apply shifting before shift
      for (size_t i = 0; i < shift_point.start_idx + 1; ++i) {
        addLateralOffsetOnIndexPoint(shifted_path, query_length.front(), i);
      }
    }
  }
}

std::vector<double> PathShifter::calcLateralJerk()
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  std::vector<double> lateral_jerk;

  // TODO(Watanabe) write docs.
  double current_shift = base_offset_;
  for (const auto & shift_point : shift_points_) {
    const double delta_shift = shift_point.length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_point.end_idx) - arclength_arr.at(shift_point.start_idx), epsilon);
    lateral_jerk.push_back((delta_shift / 2.0) / std::pow(shifting_arclength / 4.0, 3.0));
    current_shift = shift_point.length;
  }

  return lateral_jerk;
}

bool PathShifter::calcShiftPointFromArcLength(
  const PathWithLaneId & path, const Point & origin, double dist_to_start, double dist_to_end,
  double shift_length, ShiftPoint * shift_point)
{
  if (dist_to_end < dist_to_start) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("path_shifter"),
      "dist_to_end must be longer than dist_to_start. Failed to create shift point");
    return false;
  }

  if (path.points.empty()) {
    return false;
  }

  const auto origin_idx = tier4_autoware_utils::findNearestIndex(path.points, origin);
  const auto arclength_from_origin = util::calcPathArcLengthArray(path, origin_idx);

  if (dist_to_end > arclength_from_origin.back()) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("path_shifter"),
      "path is too short to calculate shift point: path length from origin = "
        << arclength_from_origin.back() << ", desired dist_to_end = " << dist_to_end);
  }

  bool is_start_found = false;
  bool is_end_found = false;
  const auto getPathPointFromOrigin = [&](size_t idx_from_origin) {
    return path.points.at(idx_from_origin + origin_idx).point.pose;
  };
  for (size_t i = 0; i < arclength_from_origin.size() - 1; ++i) {
    if (!is_start_found && arclength_from_origin.at(i + 1) > dist_to_start) {
      shift_point->start = getPathPointFromOrigin(i);
      is_start_found = true;
    }
    if (!is_end_found && arclength_from_origin.at(i + 1) > dist_to_end) {
      shift_point->end = getPathPointFromOrigin(i);
      is_end_found = true;
      break;
    }
  }

  if (!is_start_found) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("path_shifter"),
      "Even start point has not found. You should stop the shifting. Failed to create shift point");
    return false;
  }

  if (is_start_found && !is_end_found) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("behavior_path_planner").get_child("path_shifter"),
      "failed to find end point. Set end point to path.back().");
    shift_point->end = path.points.back().point.pose;
  }

  shift_point->length = shift_length;
  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner").get_child("path_shifter"),
    "Shift Point is generated from arclength: shift_length = %f", shift_length);
  return true;
}

void PathShifter::updateShiftPointIndices()
{
  for (auto & p : shift_points_) {
    p.start_idx = tier4_autoware_utils::findNearestIndex(reference_path_.points, p.start.position);
    p.end_idx = tier4_autoware_utils::findNearestIndex(reference_path_.points, p.end.position);
  }
  is_index_aligned_ = true;
}

bool PathShifter::checkShiftPointsAlignment(const ShiftPointArray & shift_points) const
{
  for (const auto & p : shift_points) {
    RCLCPP_DEBUG(logger_, "shift point = %s", toStr(p).c_str());
  }

  for (const auto & c : shift_points) {
    if (c.start_idx > c.end_idx) {
      RCLCPP_ERROR(logger_, "shift_point must satisfy start_idx <= end_idx.");
      return false;
    }
  }

  return true;
}

bool PathShifter::sortShiftPointsAlongPath([[maybe_unused]] const PathWithLaneId & path)
{
  if (shift_points_.empty()) {
    RCLCPP_DEBUG_STREAM(logger_, "shift_points_ is empty. do nothing.");
    return true;
  }

  const auto & unsorted_shift_points = shift_points_;

  // Calc indices sorted by "shift start point index" order
  std::vector<size_t> sorted_indices(unsorted_shift_points.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(), [&](size_t i1, size_t i2) {
    return unsorted_shift_points[i1].start_idx < unsorted_shift_points[i2].start_idx;
  });

  // Set shift points and index by sorted_indices
  ShiftPointArray sorted_shift_points;
  for (const auto sorted_i : sorted_indices) {
    sorted_shift_points.push_back(unsorted_shift_points.at(sorted_i));
  }

  // Check if the shift points are sorted correctly
  if (!checkShiftPointsAlignment(sorted_shift_points)) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to sort shift points..!!");
    return false;
  }

  // set to member
  setShiftPoints(sorted_shift_points);

  // Debug
  for (const auto & p : unsorted_shift_points) {
    RCLCPP_DEBUG_STREAM(logger_, "unsorted_shift_points: " << toStr(p));
  }
  for (const auto & i : sorted_indices) {
    RCLCPP_DEBUG_STREAM(logger_, "sorted_indices i = " << i);
  }
  for (const auto & p : sorted_shift_points) {
    RCLCPP_DEBUG_STREAM(logger_, "sorted_shift_points: in order: " << toStr(p));
  }
  RCLCPP_DEBUG(logger_, "PathShifter::sortShiftPointsAlongPath end.");

  return true;
}

void PathShifter::removeBehindShiftPointAndSetBaseOffset(const Point & base_point)
{
  const auto base_idx = tier4_autoware_utils::findNearestIndex(reference_path_.points, base_point);

  // If shift_point.end is behind the base_point, remove the shift_point and
  // set its shift_length to the base_offset.
  ShiftPointArray new_shift_points;
  ShiftPointArray removed_shift_points;
  for (const auto & sp : shift_points_) {
    (sp.end_idx > base_idx) ? new_shift_points.push_back(sp) : removed_shift_points.push_back(sp);
  }

  double new_base_offset = base_offset_;
  if (!removed_shift_points.empty()) {
    const auto last_removed_sp = std::max_element(
      removed_shift_points.begin(), removed_shift_points.end(),
      [](auto & a, auto & b) { return a.end_idx > b.end_idx; });
    new_base_offset = last_removed_sp->length;
  }

  // remove accumulated floating noise
  if (std::abs(new_base_offset) < 1.0e-4) {
    new_base_offset = 0.0;
  }

  RCLCPP_DEBUG(
    logger_, "shift_points_ size: %lu -> %lu", shift_points_.size(), new_shift_points.size());

  setShiftPoints(new_shift_points);

  setBaseOffset(new_base_offset);
}

void PathShifter::addLateralOffsetOnIndexPoint(
  ShiftedPath * path, double offset, size_t index) const
{
  if (fabs(offset) < 1.0e-8) {
    return;
  }

  auto & p = path->path.points.at(index).point.pose;
  double yaw = tf2::getYaw(p.orientation);
  p.position.x -= std::sin(yaw) * offset;
  p.position.y += std::cos(yaw) * offset;

  path->shift_length.at(index) += offset;
}

void PathShifter::shiftBaseLength(ShiftedPath * path, double offset) const
{
  constexpr double BASE_OFFSET_THR = 1.0e-4;
  if (std::abs(offset) > BASE_OFFSET_THR) {
    for (size_t i = 0; i < path->path.points.size(); ++i) {
      addLateralOffsetOnIndexPoint(path, offset, i);
    }
  }
}

}  // namespace behavior_path_planner
