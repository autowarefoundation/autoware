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

#ifndef AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__SHIFT_LINE_GENERATOR_HPP_
#define AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__SHIFT_LINE_GENERATOR_HPP_

#include "autoware/behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/helper.hpp"
#include "autoware/behavior_path_static_obstacle_avoidance_module/type_alias.hpp"

#include <memory>

namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance
{

using autoware::behavior_path_planner::PathShifter;
using autoware::behavior_path_planner::helper::static_obstacle_avoidance::AvoidanceHelper;

class ShiftLineGenerator
{
public:
  explicit ShiftLineGenerator(const std::shared_ptr<AvoidanceParameters> & parameters)
  : parameters_{parameters}
  {
  }

  void update(AvoidancePlanningData & data, DebugData & debug);

  void setRawRegisteredShiftLine(
    const AvoidLineArray & shift_lines, const AvoidancePlanningData & data);

  void setData(const std::shared_ptr<const PlannerData> & data) { data_ = data; }

  void setHelper(const std::shared_ptr<AvoidanceHelper> & helper) { helper_ = helper; }

  void setPathShifter(const PathShifter & path_shifter)
  {
    base_offset_ = path_shifter.getBaseOffset();
    if (path_shifter.getShiftLines().empty()) {
      last_ = std::nullopt;
    } else {
      last_ = path_shifter.getLastShiftLine().value();
    }
  }

  void reset()
  {
    last_ = std::nullopt;
    base_offset_ = 0.0;
    raw_.clear();
    raw_registered_.clear();
  }

  AvoidLineArray generate(const AvoidancePlanningData & data, DebugData & debug) const;

  AvoidLineArray getRawRegisteredShiftLine() const { return raw_registered_; }

private:
  /**
   * @brief Calculate the shift points (start/end point, shift length) from the object lateral
   * and longitudinal positions in the Frenet coordinate. The jerk limit is also considered here.
   * @param avoidance data.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidOutlines generateAvoidOutline(AvoidancePlanningData & data, DebugData & debug) const;

  /*
   * @brief merge avoid outlines.
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidOutlines applyMergeProcess(
    const AvoidOutlines & outlines, const AvoidancePlanningData & data, DebugData & debug) const;

  /*
   * @brief fill gap between two shift lines.
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidOutlines applyFillGapProcess(
    const AvoidOutlines & outlines, const AvoidancePlanningData & data, DebugData & debug) const;

  /*
   * @brief generate candidate shift lines.
   * @param one-shot shift lines.
   * @param debug data.
   */
  AvoidLineArray generateCandidateShiftLine(
    const AvoidLineArray & shift_lines, const AvoidancePlanningData & data,
    DebugData & debug) const;

  /**
   * @brief clean up raw shift lines.
   * @param target shift lines.
   * @param debug data.
   * @return processed shift lines.
   * @details process flow:
   * 1. combine raw shirt lines and previous registered shift lines.
   * 2. add return shift line.
   * 3. merge raw shirt lines.
   * 4. trim unnecessary shift lines.
   */
  AvoidLineArray applyPreProcess(
    const AvoidOutlines & outlines, const AvoidancePlanningData & data, DebugData & debug) const;

  /*
   * @brief fill gap among shift lines.
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidLineArray applyFillGapProcess(
    const AvoidLineArray & shift_lines, const AvoidancePlanningData & data,
    DebugData & debug) const;

  /*
   * @brief merge negative & positive shift lines.
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   */
  AvoidLineArray applyMergeProcess(
    const AvoidLineArray & shift_lines, const AvoidancePlanningData & data,
    DebugData & debug) const;

  /*
   * @brief add return shift line from ego position.
   * @param current raw shift line.
   * @param current registered shift line.
   * @param debug data.
   */
  AvoidLineArray applyCombineProcess(
    const AvoidLineArray & shift_lines, const AvoidLineArray & registered_lines,
    [[maybe_unused]] DebugData & debug) const;

  /*
   * @brief add return shift line from ego position.
   * @param shift lines which the return shift is added.
   * Pick up the last shift point, which is the most farthest from ego, from the current candidate
   * avoidance points and registered points in the shifter. If the last shift length of the point is
   * non-zero, add a return-shift to center line from the point. If there is no shift point in
   * candidate avoidance points nor registered points, and base_shift > 0, add a return-shift to
   * center line from ego.
   */
  AvoidLineArray addReturnShiftLine(
    const AvoidLineArray & shift_lines, const AvoidancePlanningData & data,
    DebugData & debug) const;

  /*
   * @brief extract shift lines from total shift lines based on their gradient.
   * @param shift length data.
   * @return extracted shift lines.
   */
  AvoidLineArray extractShiftLinesFromLine(
    const AvoidancePlanningData & data, ShiftLineData & shift_line_data) const;

  /*
   * @brief remove unnecessary avoid points
   * @param original shift lines.
   * @param debug data.
   * @return processed shift lines.
   * @details
   * - Combine avoid points that have almost same gradient
   * - Quantize the shift length to reduce the shift point noise
   * - Change the shift length to the previous one if the deviation is small.
   * - Remove unnecessary return shift (back to the center line).
   */
  AvoidLineArray applyTrimProcess(const AvoidLineArray & shift_lines, DebugData & debug) const;

  /*
   * @brief extract new shift lines based on current shifted path. the module makes a RTC request
   * for those new shift lines.
   * @param candidate shift lines.
   * @return new shift lines.
   */
  AvoidLineArray findNewShiftLine(const AvoidLineArray & shift_lines, DebugData & debug) const;

  /*
   * @brief generate total shift line. total shift line has shift length and gradient array.
   * @param raw shift lines.
   * @param total shift line.
   */
  void generateTotalShiftLine(
    const AvoidLineArray & avoid_points, const AvoidancePlanningData & data,
    ShiftLineData & shift_line_data) const;

  /*
   * @brief quantize shift line length.
   * @param target shift lines.
   * @param threshold. shift length is quantized by this value. (if it is 0.3[m], the output shift
   * length is 0.0, 0.3, 0.6...)
   */
  void applyQuantizeProcess(AvoidLineArray & shift_lines, const double threshold) const;

  /*
   * @brief trim shift line whose relative longitudinal distance is less than threshold.
   * @param target shift lines.
   * @param threshold.
   */
  void applySmallShiftFilter(AvoidLineArray & shift_lines, const double threshold) const;

  /*
   * @brief merge multiple shift lines whose relative gradient is less than threshold.
   * @param target shift lines.
   * @param threshold.
   */
  void applySimilarGradFilter(AvoidLineArray & shift_lines, const double threshold) const;

  /**
   * @brief update path index of the registered objects. remove old objects whose end point is
   * behind ego pose.
   */
  void updateRegisteredRawShiftLines(const AvoidancePlanningData & data);

  std::shared_ptr<const PlannerData> data_;

  std::shared_ptr<AvoidanceParameters> parameters_;

  std::shared_ptr<AvoidanceHelper> helper_;

  std::optional<ShiftLine> last_{std::nullopt};

  AvoidLineArray raw_;

  AvoidLineArray raw_registered_;

  double base_offset_{0.0};
};

}  // namespace autoware::behavior_path_planner::utils::static_obstacle_avoidance

#endif  // AUTOWARE__BEHAVIOR_PATH_STATIC_OBSTACLE_AVOIDANCE_MODULE__SHIFT_LINE_GENERATOR_HPP_
