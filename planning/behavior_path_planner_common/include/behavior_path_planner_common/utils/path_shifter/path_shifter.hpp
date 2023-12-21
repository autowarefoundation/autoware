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

#ifndef BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SHIFTER__PATH_SHIFTER_HPP_
#define BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SHIFTER__PATH_SHIFTER_HPP_

#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <optional>
#include <string>
#include <utility>
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
   * @brief  Set velocity used to apply a lateral acceleration limit.
   */
  void setVelocity(const double velocity);

  /**
   * @brief  Set lateral acceleration limit
   */
  void setLateralAccelerationLimit(const double lateral_acc);

  /**
   * @brief  Set longitudinal acceleration
   */
  void setLongitudinalAcceleration(const double longitudinal_acc);

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

  static double calcFeasibleVelocityFromJerk(
    const double lateral, const double jerk, const double distance);

  static double calcLateralDistFromJerk(
    const double longitudinal, const double jerk, const double velocity);

  static double calcLongitudinalDistFromJerk(
    const double lateral, const double jerk, const double velocity);

  static double calcShiftTimeFromJerk(const double lateral, const double jerk, const double acc);

  static double calcJerkFromLatLonDistance(
    const double lateral, const double longitudinal, const double velocity);

  double getTotalShiftLength() const;

  double getLastShiftLength() const;

  std::optional<ShiftLine> getLastShiftLine() const;

  /**
   * @brief  Calculate the theoretical lateral jerk by spline shifting for current shift_lines_.
   * @return Jerk array. The size is same as the shift points.
   */
  std::vector<double> calcLateralJerk() const;

private:
  // The reference path along which the shift will be performed.
  PathWithLaneId reference_path_;

  // Shift points used for shifted-path generation.
  ShiftLineArray shift_lines_;

  // The amount of shift length to the entire path.
  double base_offset_{0.0};

  // Used to apply a lateral acceleration limit
  double velocity_{0.0};

  // lateral acceleration limit considered in the path planning
  double lateral_acc_limit_{-1.0};

  double longitudinal_acc_{0.0};

  // Logger
  mutable rclcpp::Logger logger_{
    rclcpp::get_logger("behavior_path_planner").get_child("path_shifter")};

  // Clock
  mutable rclcpp::Clock clock_{RCL_ROS_TIME};

  std::pair<std::vector<double>, std::vector<double>> calcBaseLengths(
    const double arclength, const double shift_length, const bool offset_back) const;

  std::pair<std::vector<double>, std::vector<double>> getBaseLengthsWithoutAccelLimit(
    const double arclength, const double shift_length, const bool offset_back) const;

  std::pair<std::vector<double>, std::vector<double>> getBaseLengthsWithoutAccelLimit(
    const double arclength, const double shift_length, const double velocity,
    const double longitudinal_acc, const double total_time, const bool offset_back) const;

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

#endif  // BEHAVIOR_PATH_PLANNER_COMMON__UTILS__PATH_SHIFTER__PATH_SHIFTER_HPP_
