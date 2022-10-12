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

#ifndef BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__PATH_SHIFTER_HPP_
#define BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__PATH_SHIFTER_HPP_

#include "behavior_path_planner/parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <boost/optional.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using autoware_auto_planning_msgs::msg::PathPointWithLaneId;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;

struct ShiftLine
{
  Pose start{};  // shift start point in absolute coordinate
  Pose end{};    // shift start point in absolute coordinate

  // relative shift length at the start point related to the reference path
  double start_shift_length{};

  // relative shift length at the end point related to the reference path
  double end_shift_length{};

  size_t start_idx{};  // associated start-point index for the reference path
  size_t end_idx{};    // associated end-point index for the reference path
};
using ShiftLineArray = std::vector<ShiftLine>;

struct ShiftedPath
{
  PathWithLaneId path{};
  std::vector<double> shift_length{};
};

enum class SHIFT_TYPE {
  LINEAR = 0,
  SPLINE = 1,
};

class PathShifter
{
public:
  // setter & getter

  /**
   * @brief  Set reference path.
   */
  void setPath(const PathWithLaneId & path);

  /**
   * @brief  Add shift point. You don't have to care about the start/end_idx.
   */
  void addShiftLine(const ShiftLine & point);

  /**
   * @brief  Set new shift point. You don't have to care about the start/end_idx.
   */
  void setShiftLines(const std::vector<ShiftLine> & lines);

  /**
   * @brief  Get shift points.
   */
  std::vector<ShiftLine> getShiftLines() const { return shift_lines_; }

  /**
   * @brief  Get shift points size.
   */
  size_t getShiftLinesSize() const { return shift_lines_.size(); }

  /**
   * @brief  Get base offset.
   */
  double getBaseOffset() const { return base_offset_; }

  /**
   * @brief  Get reference path.
   */
  PathWithLaneId getReferencePath() const { return reference_path_; }

  /**
   * @brief  Generate a shifted path according to the given reference path and shift points.
   * @return False if the path is empty or shift points have conflicts.
   */
  bool generate(
    ShiftedPath * shifted_path, const bool offset_back = true,
    const SHIFT_TYPE type = SHIFT_TYPE::SPLINE) const;

  /**
   * @brief Remove behind shift points and add the removed offset to the base_offset_.
   * @details The previous offset information is stored in the base_offset_.
   *          This should be called after generate().
   */
  void removeBehindShiftLineAndSetBaseOffset(const size_t nearest_idx);

  ////////////////////////////////////////
  // Utility Functions
  ////////////////////////////////////////

  static double calcLongitudinalDistFromJerk(
    const double lateral, const double jerk, const double velocity)
  {
    const double j = std::abs(jerk);
    const double l = std::abs(lateral);
    const double v = std::abs(velocity);
    if (j < 1.0e-8) {
      return 1.0e10;  // TODO(Horibe) maybe invalid arg?
    }
    return 4.0 * std::pow(0.5 * l / j, 1.0 / 3.0) * v;
  }

  static double calcJerkFromLatLonDistance(
    const double lateral, const double longitudinal, const double velocity)
  {
    constexpr double ep = 1.0e-3;
    const double lat = std::abs(lateral);
    const double lon = std::max(std::abs(longitudinal), ep);
    const double v = std::abs(velocity);
    return 0.5 * lat * std::pow(4.0 * v / lon, 3);
  }

  double getTotalShiftLength() const
  {
    double sum = base_offset_;
    for (const auto & l : shift_lines_) {
      sum += l.end_shift_length;
    }
    return sum;
  }

  double getLastShiftLength() const
  {
    if (shift_lines_.empty()) {
      return base_offset_;
    }

    const auto furthest = std::max_element(
      shift_lines_.begin(), shift_lines_.end(),
      [](auto & a, auto & b) { return a.end_idx < b.end_idx; });

    return furthest->end_shift_length;
  }

  boost::optional<ShiftLine> getLastShiftLine() const
  {
    if (shift_lines_.empty()) {
      return {};
    }

    const auto furthest = std::max_element(
      shift_lines_.begin(), shift_lines_.end(),
      [](auto & a, auto & b) { return a.end_idx > b.end_idx; });

    return *furthest;
  }

  /**
   * @brief  Calculate the theoretical lateral jerk by spline shifting for current shift_lines_.
   * @return Jerk array. THe size is same as the shift points.
   */
  std::vector<double> calcLateralJerk() const;

private:
  // The reference path along which the shift will be performed.
  PathWithLaneId reference_path_;

  // Shift points used for shifted-path generation.
  ShiftLineArray shift_lines_;

  // The amount of shift length to the entire path.
  double base_offset_{0.0};

  // Logger
  mutable rclcpp::Logger logger_{
    rclcpp::get_logger("behavior_path_planner").get_child("path_shifter")};

  // Clock
  mutable rclcpp::Clock clock_{RCL_ROS_TIME};

  /**
   * @brief Calculate path index for shift_lines and set is_index_aligned_ to true.
   */
  void updateShiftLinesIndices(ShiftLineArray & shift_lines) const;

  /**
   * @brief Sort the points in order from the front of the path.
   */
  void sortShiftLinesAlongPath(ShiftLineArray & shift_lines) const;

  /**
   * @brief Generate shifted path from reference_path_ and shift_lines_ with linear shifting.
   */
  void applyLinearShifter(ShiftedPath * shifted_path) const;

  /**
   * @brief Generate shifted path from reference_path_ and shift_lines_ with spline_based shifting.
   * @details Calculate the shift so that the horizontal jerk remains constant. This is achieved by
   *          dividing the shift interval into four parts and apply a cubic spline to them.
   *          The resultant shifting shape is closed to the Clothoid curve.
   */
  void applySplineShifter(ShiftedPath * shifted_path, const bool offset_back) const;

  ////////////////////////////////////////
  // Helper Functions
  ////////////////////////////////////////

  /**
   * @brief Check if the shift points are aligned in order and have no conflict range.
   */
  bool checkShiftLinesAlignment(const ShiftLineArray & shift_lines) const;

  void addLateralOffsetOnIndexPoint(ShiftedPath * path, double offset, size_t index) const;

  void shiftBaseLength(ShiftedPath * path, double offset) const;

  void setBaseOffset(const double val)
  {
    RCLCPP_DEBUG(logger_, "base_offset is changed: %f -> %f", base_offset_, val);
    base_offset_ = val;
  }
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__SCENE_MODULE__UTILS__PATH_SHIFTER_HPP_
