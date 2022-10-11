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

#include "behavior_path_planner/scene_module/utils/path_shifter.hpp"

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
std::string toStr(const behavior_path_planner::ShiftLine & p)
{
  return "start point = " + toStr(p.start.position) + ", end point = " + toStr(p.end.position) +
         ", start idx = " + std::to_string(p.start_idx) +
         ", end idx = " + std::to_string(p.end_idx) +
         ", length = " + std::to_string(p.end_shift_length);
}
}  // namespace

namespace behavior_path_planner
{

using motion_utils::findNearestIndex;
using motion_utils::insertOrientation;

void PathShifter::setPath(const PathWithLaneId & path)
{
  reference_path_ = path;

  updateShiftLinesIndices(shift_lines_);
  sortShiftLinesAlongPath(shift_lines_);
}
void PathShifter::addShiftLine(const ShiftLine & line)
{
  shift_lines_.push_back(line);

  updateShiftLinesIndices(shift_lines_);
  sortShiftLinesAlongPath(shift_lines_);
}

void PathShifter::setShiftLines(const std::vector<ShiftLine> & lines)
{
  shift_lines_ = lines;

  updateShiftLinesIndices(shift_lines_);
  sortShiftLinesAlongPath(shift_lines_);
}

bool PathShifter::generate(
  ShiftedPath * shifted_path, const bool offset_back, const SHIFT_TYPE type) const
{
  RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "PathShifter::generate start!");

  // Guard
  if (reference_path_.points.empty()) {
    RCLCPP_ERROR_STREAM(logger_, "reference path is empty.");
    return false;
  }

  shifted_path->path = reference_path_;
  shifted_path->shift_length.resize(reference_path_.points.size(), 0.0);

  if (shift_lines_.empty()) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, clock_, 3000, "shift_lines_ is empty. Return reference with base offset.");
    shiftBaseLength(shifted_path, base_offset_);
    return true;
  }

  for (const auto & shift_line : shift_lines_) {
    int idx_gap = shift_line.end_idx - shift_line.start_idx;
    if (idx_gap <= 1) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, clock_, 3000,
        "shift start point and end point can't be adjoining "
        "Maybe shift length is too short?");
      return false;
    }
  }

  // Check if the shift points are sorted correctly
  if (!checkShiftLinesAlignment(shift_lines_)) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to sort shift points..!!");
    return false;
  }

  if (shift_lines_.front().start_idx == 0) {
    // if offset is applied on front side, shifting from first point is no problem
    if (offset_back) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, clock_, 3000,
        "shift start point is at the edge of path. It could cause undesired result."
        " Maybe path is too short for backward?");
    }
  }

  // Calculate shifted path
  type == SHIFT_TYPE::SPLINE ? applySplineShifter(shifted_path, offset_back)
                             : applyLinearShifter(shifted_path);

  const bool is_driving_forward = true;
  insertOrientation(shifted_path->path.points, is_driving_forward);

  // DEBUG
  RCLCPP_DEBUG_STREAM_THROTTLE(
    logger_, clock_, 3000,
    "PathShifter::generate end. shift_lines_.size = " << shift_lines_.size());

  return true;
}

void PathShifter::applyLinearShifter(ShiftedPath * shifted_path) const
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  shiftBaseLength(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_lines_,
  for (const auto & shift_line : shift_lines_) {
    const auto current_shift = shifted_path->shift_length.at(shift_line.end_idx);
    const auto delta_shift = shift_line.end_shift_length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_line.end_idx) - arclength_arr.at(shift_line.start_idx), epsilon);

    // For all path.points,
    for (size_t i = 0; i < shifted_path->path.points.size(); ++i) {
      // Set shift length.
      double ith_shift_length;
      if (i < shift_line.start_idx) {
        ith_shift_length = 0.0;
      } else if (shift_line.start_idx <= i && i <= shift_line.end_idx) {
        auto dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_line.start_idx);
        ith_shift_length = (dist_from_start / shifting_arclength) * delta_shift;
      } else {
        ith_shift_length = delta_shift;
      }

      // Apply shifting.
      addLateralOffsetOnIndexPoint(shifted_path, ith_shift_length, i);
    }
  }
}

void PathShifter::applySplineShifter(ShiftedPath * shifted_path, const bool offset_back) const
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  shiftBaseLength(shifted_path, base_offset_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  // For all shift_lines,
  for (const auto & shift_line : shift_lines_) {
    // calc delta shift at the sp.end_idx so that the sp.end_idx on the path will have
    // the desired shift length.
    const auto current_shift = shifted_path->shift_length.at(shift_line.end_idx);
    const auto delta_shift = shift_line.end_shift_length - current_shift;

    RCLCPP_DEBUG(
      logger_, "current_shift = %f, sp.length = %f", current_shift, shift_line.end_shift_length);

    if (std::abs(delta_shift) < 0.01) {
      RCLCPP_DEBUG(logger_, "delta shift is zero. skip for this shift point.");
    }

    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_line.end_idx) - arclength_arr.at(shift_line.start_idx), epsilon);

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
    for (size_t i = shift_line.start_idx + 1; i < shift_line.end_idx; ++i) {
      const double dist_from_start = arclength_arr.at(i) - arclength_arr.at(shift_line.start_idx);
      query_distance.push_back(dist_from_start);
    }
    if (!query_distance.empty()) {
      query_length = interpolation::spline(base_distance, base_length, query_distance);
    }

    // Apply shifting.
    {
      size_t i = shift_line.start_idx + 1;
      for (const auto & itr : query_length) {
        addLateralOffsetOnIndexPoint(shifted_path, itr, i);
        ++i;
      }
    }

    if (offset_back == true) {
      // Apply shifting after shift
      for (size_t i = shift_line.end_idx; i < shifted_path->path.points.size(); ++i) {
        addLateralOffsetOnIndexPoint(shifted_path, delta_shift, i);
      }
    } else {
      // Apply shifting before shift
      for (size_t i = 0; i < shift_line.start_idx + 1; ++i) {
        addLateralOffsetOnIndexPoint(shifted_path, query_length.front(), i);
      }
    }
  }
}

std::vector<double> PathShifter::calcLateralJerk() const
{
  const auto arclength_arr = util::calcPathArcLengthArray(reference_path_);

  constexpr double epsilon = 1.0e-8;  // to avoid 0 division

  std::vector<double> lateral_jerk{};

  // TODO(Watanabe) write docs.
  double current_shift = base_offset_;
  for (const auto & shift_line : shift_lines_) {
    const double delta_shift = shift_line.end_shift_length - current_shift;
    const auto shifting_arclength = std::max(
      arclength_arr.at(shift_line.end_idx) - arclength_arr.at(shift_line.start_idx), epsilon);
    lateral_jerk.push_back((delta_shift / 2.0) / std::pow(shifting_arclength / 4.0, 3.0));
    current_shift = shift_line.end_shift_length;
  }

  return lateral_jerk;
}

void PathShifter::updateShiftLinesIndices(ShiftLineArray & shift_lines) const
{
  if (reference_path_.points.empty()) {
    RCLCPP_ERROR(
      logger_, "reference path is empty, setPath is needed before addShiftLine/setShiftLines.");
  }

  for (auto & l : shift_lines) {
    // TODO(murooka) remove findNearestIndex for except
    // lane_following to support u-turn & crossing path
    l.start_idx = findNearestIndex(reference_path_.points, l.start.position);
    // TODO(murooka) remove findNearestIndex for except
    // lane_following to support u-turn & crossing path
    l.end_idx = findNearestIndex(reference_path_.points, l.end.position);
  }
}

bool PathShifter::checkShiftLinesAlignment(const ShiftLineArray & shift_lines) const
{
  for (const auto & l : shift_lines) {
    RCLCPP_DEBUG(logger_, "shift point = %s", toStr(l).c_str());
  }

  for (const auto & l : shift_lines) {
    if (l.start_idx > l.end_idx) {
      RCLCPP_ERROR(logger_, "shift_line must satisfy start_idx <= end_idx.");
      return false;
    }
  }

  return true;
}

void PathShifter::sortShiftLinesAlongPath(ShiftLineArray & shift_lines) const
{
  if (shift_lines.empty()) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "shift_lines is empty. do nothing.");
    return;
  }

  const auto unsorted_shift_lines = shift_lines;

  // Calc indices sorted by "shift start point index" order
  std::vector<size_t> sorted_indices(unsorted_shift_lines.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(), [&](size_t i, size_t j) {
    return unsorted_shift_lines.at(i).start_idx < unsorted_shift_lines.at(j).start_idx;
  });

  // Set shift points and index by sorted_indices
  ShiftLineArray sorted_shift_lines;
  for (const auto sorted_i : sorted_indices) {
    sorted_shift_lines.push_back(unsorted_shift_lines.at(sorted_i));
  }

  shift_lines = sorted_shift_lines;

  // Debug
  for (const auto & l : unsorted_shift_lines) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "unsorted_shift_lines: " << toStr(l));
  }
  for (const auto & i : sorted_indices) {
    RCLCPP_DEBUG_STREAM_THROTTLE(logger_, clock_, 3000, "sorted_indices i = " << i);
  }
  for (const auto & l : sorted_shift_lines) {
    RCLCPP_DEBUG_STREAM_THROTTLE(
      logger_, clock_, 3000, "sorted_shift_lines: in order: " << toStr(l));
  }
  RCLCPP_DEBUG(logger_, "PathShifter::sortShiftLinesAlongPath end.");
}

void PathShifter::removeBehindShiftLineAndSetBaseOffset(const size_t nearest_idx)
{
  // If shift_line.end is behind the ego_pose, remove the shift_line and
  // set its shift_length to the base_offset.
  ShiftLineArray new_shift_lines;
  ShiftLineArray removed_shift_lines;
  for (const auto & sl : shift_lines_) {
    (sl.end_idx > nearest_idx) ? new_shift_lines.push_back(sl) : removed_shift_lines.push_back(sl);
  }

  double new_base_offset = base_offset_;
  if (!removed_shift_lines.empty()) {
    const auto last_removed_sl = std::max_element(
      removed_shift_lines.begin(), removed_shift_lines.end(),
      [](auto & a, auto & b) { return a.end_idx > b.end_idx; });
    new_base_offset = last_removed_sl->end_shift_length;
  }

  // remove accumulated floating noise
  if (std::abs(new_base_offset) < 1.0e-4) {
    new_base_offset = 0.0;
  }

  RCLCPP_DEBUG(
    logger_, "shift_lines size: %lu -> %lu", shift_lines_.size(), new_shift_lines.size());

  setShiftLines(new_shift_lines);

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
