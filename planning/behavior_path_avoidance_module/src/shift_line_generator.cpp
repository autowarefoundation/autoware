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

#include "behavior_path_avoidance_module/shift_line_generator.hpp"

#include "behavior_path_avoidance_module/utils.hpp"
#include "behavior_path_planner_common/utils/utils.hpp"

#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <tier4_planning_msgs/msg/detail/avoidance_debug_factor__struct.hpp>

namespace behavior_path_planner::utils::avoidance
{

using tier4_autoware_utils::generateUUID;
using tier4_autoware_utils::getPoint;
using tier4_planning_msgs::msg::AvoidanceDebugFactor;

namespace
{
bool isBestEffort(const std::string & policy)
{
  return policy == "best_effort";
}

bool perManeuver(const std::string & policy)
{
  return policy == "per_avoidance_maneuver";
}

AvoidLine merge(const AvoidLine & line1, const AvoidLine & line2, const UUID id)
{
  AvoidLine ret{};

  ret.start_idx = line1.start_idx;
  ret.start_shift_length = line1.start_shift_length;
  ret.start_longitudinal = line1.start_longitudinal;

  ret.end_idx = line2.end_idx;
  ret.end_shift_length = line2.end_shift_length;
  ret.end_longitudinal = line2.end_longitudinal;

  ret.id = id;
  ret.object = line1.object;

  return ret;
}

AvoidLine fill(const AvoidLine & line1, const AvoidLine & line2, const UUID id)
{
  AvoidLine ret{};

  ret.start_idx = line1.end_idx;
  ret.start_shift_length = line1.end_shift_length;
  ret.start_longitudinal = line1.end_longitudinal;

  ret.end_idx = line2.start_idx;
  ret.end_shift_length = line2.start_shift_length;
  ret.end_longitudinal = line2.start_longitudinal;

  ret.id = id;
  ret.object = line1.object;

  return ret;
}

AvoidLineArray toArray(const AvoidOutlines & outlines)
{
  AvoidLineArray ret{};
  for (const auto & outline : outlines) {
    ret.push_back(outline.avoid_line);
    ret.push_back(outline.return_line);

    std::for_each(
      outline.middle_lines.begin(), outline.middle_lines.end(),
      [&ret](const auto & line) { ret.push_back(line); });
  }
  return ret;
}
}  // namespace

void ShiftLineGenerator::update(AvoidancePlanningData & data, DebugData & debug)
{
  /**
   * STEP1: Update registered shift line.
   */
  updateRegisteredRawShiftLines(data);

  /**
   * STEP2: Generate avoid outlines.
   * Basically, avoid outlines are generated per target objects.
   */
  const auto outlines = generateAvoidOutline(data, debug);

  /**
   * STEP3: Create rough shift lines.
   */
  raw_ = applyPreProcess(outlines, data, debug);
}

AvoidLineArray ShiftLineGenerator::generate(
  const AvoidancePlanningData & data, DebugData & debug) const
{
  return generateCandidateShiftLine(raw_, data, debug);
}

AvoidOutlines ShiftLineGenerator::generateAvoidOutline(
  AvoidancePlanningData & data, [[maybe_unused]] DebugData & debug) const
{
  // To be consistent with changes in the ego position, the current shift length is considered.
  const auto current_ego_shift = helper_->getEgoShift();
  const auto & base_link2rear = data_->parameters.base_link2rear;

  // Calculate feasible shift length
  const auto get_shift_profile =
    [&](
      auto & object, const auto & desire_shift_length) -> std::optional<std::pair<double, double>> {
    // use each object param
    const auto object_type = utils::getHighestProbLabel(object.object.classification);
    const auto object_parameter = parameters_->object_parameters.at(object_type);
    const auto is_object_on_right = utils::avoidance::isOnRight(object);

    // use absolute dist for return-to-center, relative dist from current for avoiding.
    const auto avoiding_shift = desire_shift_length - current_ego_shift;
    const auto nominal_avoid_distance = helper_->getMaxAvoidanceDistance(avoiding_shift);

    // calculate remaining distance.
    const auto prepare_distance = helper_->getNominalPrepareDistance();
    const auto & additional_buffer_longitudinal =
      object_parameter.use_conservative_buffer_longitudinal ? data_->parameters.base_link2front
                                                            : 0.0;
    const auto constant = object_parameter.safety_buffer_longitudinal +
                          additional_buffer_longitudinal + prepare_distance;
    const auto has_enough_distance = object.longitudinal > constant + nominal_avoid_distance;
    const auto remaining_distance = object.longitudinal - constant;
    const auto avoidance_distance =
      has_enough_distance ? nominal_avoid_distance : remaining_distance;

    // nominal case. avoidable.
    if (has_enough_distance) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    if (!isBestEffort(parameters_->policy_lateral_margin)) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    // ego already has enough positive shift.
    const auto has_enough_positive_shift = avoiding_shift < -1e-3 && desire_shift_length > 1e-3;
    if (is_object_on_right && has_enough_positive_shift) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    // ego already has enough negative shift.
    const auto has_enough_negative_shift = avoiding_shift > 1e-3 && desire_shift_length < -1e-3;
    if (!is_object_on_right && has_enough_negative_shift) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    // don't relax shift length since it can stop in front of the object.
    if (object.is_stoppable && !parameters_->use_shorten_margin_immediately) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    // the avoidance path is already approved
    const auto & object_pos = object.object.kinematics.initial_pose_with_covariance.pose.position;
    const auto is_approved = (helper_->getShift(object_pos) > 0.0 && is_object_on_right) ||
                             (helper_->getShift(object_pos) < 0.0 && !is_object_on_right);
    if (is_approved) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    // prepare distance is not enough. unavoidable.
    if (remaining_distance < 1e-3) {
      object.reason = AvoidanceDebugFactor::REMAINING_DISTANCE_LESS_THAN_ZERO;
      return std::nullopt;
    }

    // calculate lateral jerk.
    const auto required_jerk = PathShifter::calcJerkFromLatLonDistance(
      avoiding_shift, remaining_distance, helper_->getAvoidanceEgoSpeed());

    // relax lateral jerk limit. avoidable.
    if (required_jerk < helper_->getLateralMaxJerkLimit()) {
      return std::make_pair(desire_shift_length, avoidance_distance);
    }

    // avoidance distance is not enough. unavoidable.
    if (!isBestEffort(parameters_->policy_deceleration)) {
      object.reason = AvoidanceDebugFactor::TOO_LARGE_JERK;
      return std::nullopt;
    }

    // output avoidance path under lateral jerk constraints.
    const auto feasible_relative_shift_length = PathShifter::calcLateralDistFromJerk(
      remaining_distance, helper_->getLateralMaxJerkLimit(), helper_->getAvoidanceEgoSpeed());

    if (std::abs(feasible_relative_shift_length) < parameters_->lateral_execution_threshold) {
      object.reason = "LessThanExecutionThreshold";
      return std::nullopt;
    }

    const auto feasible_shift_length =
      desire_shift_length > 0.0 ? feasible_relative_shift_length + current_ego_shift
                                : -1.0 * feasible_relative_shift_length + current_ego_shift;

    const auto infeasible =
      std::abs(feasible_shift_length - object.overhang_dist) <
      0.5 * data_->parameters.vehicle_width + object_parameter.safety_buffer_lateral;
    if (infeasible) {
      RCLCPP_DEBUG(rclcpp::get_logger(""), "feasible shift length is not enough to avoid. ");
      object.reason = AvoidanceDebugFactor::TOO_LARGE_JERK;
      return std::nullopt;
    }

    return std::make_pair(feasible_shift_length, avoidance_distance);
  };

  const auto is_forward_object = [](const auto & object) { return object.longitudinal > 0.0; };

  const auto is_valid_shift_line = [](const auto & s) {
    return s.start_longitudinal > 0.0 && s.start_longitudinal < s.end_longitudinal;
  };

  AvoidOutlines outlines;
  for (auto & o : data.target_objects) {
    if (!o.avoid_margin.has_value()) {
      o.reason = AvoidanceDebugFactor::INSUFFICIENT_LATERAL_MARGIN;
      if (o.avoid_required && is_forward_object(o)) {
        break;
      } else {
        continue;
      }
    }

    const auto is_object_on_right = utils::avoidance::isOnRight(o);
    const auto desire_shift_length =
      helper_->getShiftLength(o, is_object_on_right, o.avoid_margin.value());
    if (utils::avoidance::isSameDirectionShift(is_object_on_right, desire_shift_length)) {
      o.reason = AvoidanceDebugFactor::SAME_DIRECTION_SHIFT;
      if (o.avoid_required && is_forward_object(o)) {
        break;
      } else {
        continue;
      }
    }

    // use each object param
    const auto object_type = utils::getHighestProbLabel(o.object.classification);
    const auto object_parameter = parameters_->object_parameters.at(object_type);
    const auto feasible_shift_profile = get_shift_profile(o, desire_shift_length);

    if (!feasible_shift_profile.has_value()) {
      if (o.avoid_required && is_forward_object(o)) {
        break;
      } else {
        continue;
      }
    }

    // use absolute dist for return-to-center, relative dist from current for avoiding.
    const auto feasible_return_distance =
      helper_->getMaxAvoidanceDistance(feasible_shift_profile.value().first);

    AvoidLine al_avoid;
    {
      const auto & additional_buffer_longitudinal =
        object_parameter.use_conservative_buffer_longitudinal ? data_->parameters.base_link2front
                                                              : 0.0;
      const auto offset =
        object_parameter.safety_buffer_longitudinal + additional_buffer_longitudinal;
      const auto to_shift_end = o.longitudinal - offset;
      const auto path_front_to_ego = data.arclength_from_ego.at(data.ego_closest_path_index);

      // start point (use previous linear shift length as start shift length.)
      al_avoid.start_longitudinal = [&]() {
        const auto nearest_avoid_distance =
          std::max(to_shift_end - feasible_shift_profile.value().second, 1e-3);

        if (data.to_start_point > to_shift_end) {
          return nearest_avoid_distance;
        }

        const auto minimum_avoid_distance = helper_->getMinAvoidanceDistance(
          feasible_shift_profile.value().first - current_ego_shift);
        const auto furthest_avoid_distance = std::max(to_shift_end - minimum_avoid_distance, 1e-3);

        return std::clamp(data.to_start_point, nearest_avoid_distance, furthest_avoid_distance);
      }();

      al_avoid.start_idx = utils::avoidance::findPathIndexFromArclength(
        data.arclength_from_ego, al_avoid.start_longitudinal + path_front_to_ego);
      al_avoid.start = data.reference_path.points.at(al_avoid.start_idx).point.pose;
      al_avoid.start_shift_length = helper_->getLinearShift(al_avoid.start.position);

      // end point
      al_avoid.end_shift_length = feasible_shift_profile.value().first;
      al_avoid.end_longitudinal = to_shift_end;

      // misc
      al_avoid.id = generateUUID();
      al_avoid.object = o;
      al_avoid.object_on_right = utils::avoidance::isOnRight(o);
    }

    AvoidLine al_return;
    {
      const auto offset = object_parameter.safety_buffer_longitudinal + base_link2rear + o.length;
      const auto to_shift_start = o.longitudinal + offset;

      // start point
      al_return.start_shift_length = feasible_shift_profile.value().first;
      al_return.start_longitudinal = to_shift_start;

      // end point
      al_return.end_longitudinal = [&]() {
        if (data.to_return_point > to_shift_start) {
          return std::clamp(
            data.to_return_point, to_shift_start, feasible_return_distance + to_shift_start);
        }

        return to_shift_start + feasible_return_distance;
      }();
      al_return.end_shift_length = 0.0;

      // misc
      al_return.id = generateUUID();
      al_return.object = o;
      al_return.object_on_right = utils::avoidance::isOnRight(o);
    }

    if (is_valid_shift_line(al_avoid) && is_valid_shift_line(al_return)) {
      outlines.emplace_back(al_avoid, al_return);
    } else {
      o.reason = "InvalidShiftLine";
      continue;
    }

    o.is_avoidable = true;
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, outlines);

  debug.step1_current_shift_line = toArray(outlines);

  return outlines;
}

AvoidLineArray ShiftLineGenerator::applyPreProcess(
  const AvoidOutlines & outlines, const AvoidancePlanningData & data, DebugData & debug) const
{
  AvoidOutlines processed_outlines = outlines;

  /**
   * Step1: Rough merge process.
   * Merge multiple avoid outlines. If an avoid outlines' return shift line conflicts other
   * outline's avoid shift line, those avoid outlines are merged.
   */
  processed_outlines = applyMergeProcess(processed_outlines, data, debug);

  /**
   * Step2: Fill gap process.
   * Create and add new shift line to avoid outline in order to fill gaps between avoid shift line
   * and middle shift lines, return shift line and middle shift lines.
   */
  processed_outlines = applyFillGapProcess(processed_outlines, data, debug);

  /**
   * Step3: Convert to AvoidLineArray from AvoidOutlines.
   */
  AvoidLineArray processed_raw_lines = toArray(processed_outlines);

  /**
   * Step4: Combine process.
   * Use all registered points. For the current points, if the similar one of the current
   * points are already registered, will not use it.
   */
  processed_raw_lines = applyCombineProcess(processed_raw_lines, raw_registered_, debug);

  /*
   * Step5: Add return shift line.
   * Add return-to-center shift point from the last shift point, if needed.
   * If there is no shift points, set return-to center shift from ego.
   */
  processed_raw_lines = addReturnShiftLine(processed_raw_lines, data, debug);

  /*
   * Step6: Fill gap process.
   * Create and add new shift line to avoid lines.
   */
  return applyFillGapProcess(processed_raw_lines, data, debug);
}

AvoidLineArray ShiftLineGenerator::generateCandidateShiftLine(
  const AvoidLineArray & shift_lines, const AvoidancePlanningData & data, DebugData & debug) const
{
  AvoidLineArray processed_shift_lines = shift_lines;

  /**
   * Step1: Merge process.
   * Merge positive shift avoid lines and negative shift avoid lines.
   */
  processed_shift_lines = applyMergeProcess(processed_shift_lines, data, debug);

  /**
   * Step2: Clean up process.
   * Remove noisy shift line and concat same gradient shift lines.
   */
  processed_shift_lines = applyTrimProcess(processed_shift_lines, debug);

  /**
   * Step3: Extract new shift lines.
   * Compare processed shift lines and registered shift lines in order to find new shift lines.
   */
  return findNewShiftLine(processed_shift_lines, debug);
}

void ShiftLineGenerator::generateTotalShiftLine(
  const AvoidLineArray & avoid_lines, const AvoidancePlanningData & data,
  ShiftLineData & shift_line_data) const
{
  const auto & path = data.reference_path;
  const auto & arcs = data.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  sl.shift_line = std::vector<double>(N, 0.0);
  sl.shift_line_grad = std::vector<double>(N, 0.0);

  sl.pos_shift_line = std::vector<double>(N, 0.0);
  sl.neg_shift_line = std::vector<double>(N, 0.0);

  sl.pos_shift_line_grad = std::vector<double>(N, 0.0);
  sl.neg_shift_line_grad = std::vector<double>(N, 0.0);

  // debug
  sl.shift_line_history = std::vector<std::vector<double>>(avoid_lines.size(), sl.shift_line);

  // take minmax for same directional shift length
  for (size_t j = 0; j < avoid_lines.size(); ++j) {
    const auto & al = avoid_lines.at(j);
    for (size_t i = 0; i < N; ++i) {
      // calc current interpolated shift
      const auto i_shift = utils::avoidance::lerpShiftLengthOnArc(arcs.at(i), al);

      // update maximum shift for positive direction
      if (i_shift > sl.pos_shift_line.at(i)) {
        sl.pos_shift_line.at(i) = i_shift;
        sl.pos_shift_line_grad.at(i) = al.getGradient();
      }

      // update minumum shift for negative direction
      if (i_shift < sl.neg_shift_line.at(i)) {
        sl.neg_shift_line.at(i) = i_shift;
        sl.neg_shift_line_grad.at(i) = al.getGradient();
      }

      // store for debug print
      sl.shift_line_history.at(j).at(i) = i_shift;
    }
  }

  // Merge shift length of opposite directions.
  for (size_t i = 0; i < N; ++i) {
    sl.shift_line.at(i) = sl.pos_shift_line.at(i) + sl.neg_shift_line.at(i);
    sl.shift_line_grad.at(i) = sl.pos_shift_line_grad.at(i) + sl.neg_shift_line_grad.at(i);
  }

  // overwrite shift with current_ego_shift until ego pose.
  const auto current_shift = helper_->getEgoLinearShift();
  for (size_t i = 0; i < sl.shift_line.size(); ++i) {
    if (data.ego_closest_path_index < i) {
      break;
    }
    sl.shift_line.at(i) = current_shift;
    sl.shift_line_grad.at(i) = 0.0;
  }

  // If the shift point does not have an associated object,
  // use previous value.
  for (size_t i = 1; i < N; ++i) {
    bool has_object = false;
    for (const auto & al : avoid_lines) {
      if (al.start_idx <= i && i <= al.end_idx) {
        has_object = true;
        break;
      }
    }
    if (!has_object) {
      sl.shift_line.at(i) = sl.shift_line.at(i - 1);
    }
  }

  if (avoid_lines.empty()) {
    sl.shift_line_history.push_back(sl.shift_line);
    return;
  }

  const auto grad_first_shift_line = (avoid_lines.front().start_shift_length - current_shift) /
                                     avoid_lines.front().start_longitudinal;

  for (size_t i = data.ego_closest_path_index; i <= avoid_lines.front().start_idx; ++i) {
    sl.shift_line.at(i) = helper_->getLinearShift(getPoint(path.points.at(i)));
    sl.shift_line_grad.at(i) = grad_first_shift_line;
  }

  sl.shift_line_history.push_back(sl.shift_line);
}

AvoidLineArray ShiftLineGenerator::extractShiftLinesFromLine(
  const AvoidancePlanningData & data, ShiftLineData & shift_line_data) const
{
  using utils::avoidance::setEndData;
  using utils::avoidance::setStartData;

  const auto & path = data.reference_path;
  const auto & arcs = data.arclength_from_ego;
  const auto N = path.points.size();

  auto & sl = shift_line_data;

  const auto backward_grad = [&](const size_t i) {
    if (i == 0) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arcs.at(i) - arcs.at(i - 1);
    if (ds < 1.0e-5) {
      return sl.shift_line_grad.at(i);
    }  // use theoretical value when ds is too small.
    return (sl.shift_line.at(i) - sl.shift_line.at(i - 1)) / ds;
  };

  const auto forward_grad = [&](const size_t i) {
    if (i == arcs.size() - 1) {
      return sl.shift_line_grad.at(i);
    }
    const double ds = arcs.at(i + 1) - arcs.at(i);
    if (ds < 1.0e-5) {
      return sl.shift_line_grad.at(i);
    }  // use theoretical value when ds is too small.
    return (sl.shift_line.at(i + 1) - sl.shift_line.at(i)) / ds;
  };

  // calculate forward and backward gradient of the shift length.
  // This will be used for grad-change-point check.
  sl.forward_grad = std::vector<double>(N, 0.0);
  sl.backward_grad = std::vector<double>(N, 0.0);
  for (size_t i = 0; i < N - 1; ++i) {
    sl.forward_grad.at(i) = forward_grad(i);
    sl.backward_grad.at(i) = backward_grad(i);
  }

  AvoidLineArray merged_avoid_lines;
  AvoidLine al{};
  bool found_first_start = false;
  constexpr auto CREATE_SHIFT_GRAD_THR = 0.001;
  constexpr auto IS_ALREADY_SHIFTING_THR = 0.001;
  for (size_t i = data.ego_closest_path_index; i < N - 1; ++i) {
    const auto & p = path.points.at(i).point.pose;
    const auto shift = sl.shift_line.at(i);

    // If the vehicle is already on the avoidance (checked by the first point has shift),
    // set a start point at the first path point.
    if (!found_first_start && std::abs(shift) > IS_ALREADY_SHIFTING_THR) {
      setStartData(al, 0.0, p, i, arcs.at(i));  // start length is overwritten later.
      found_first_start = true;
    }

    // find the point where the gradient of the shift is changed
    const bool set_shift_line_flag =
      std::abs(sl.forward_grad.at(i) - sl.backward_grad.at(i)) > CREATE_SHIFT_GRAD_THR;

    if (!set_shift_line_flag) {
      continue;
    }

    if (!found_first_start) {
      setStartData(al, 0.0, p, i, arcs.at(i));  // start length is overwritten later.
      found_first_start = true;
    } else {
      setEndData(al, shift, p, i, arcs.at(i));
      al.id = generateUUID();
      merged_avoid_lines.push_back(al);
      setStartData(al, 0.0, p, i, arcs.at(i));  // start length is overwritten later.
    }
  }

  if (std::abs(backward_grad(N - 1)) > CREATE_SHIFT_GRAD_THR) {
    const auto & p = path.points.at(N - 1).point.pose;
    const auto shift = sl.shift_line.at(N - 1);
    setEndData(al, shift, p, N - 1, arcs.at(N - 1));
    al.id = generateUUID();
    merged_avoid_lines.push_back(al);
  }

  return merged_avoid_lines;
}

AvoidOutlines ShiftLineGenerator::applyMergeProcess(
  const AvoidOutlines & outlines, const AvoidancePlanningData & data, DebugData & debug) const
{
  AvoidOutlines ret{};

  if (outlines.size() < 2) {
    return outlines;
  }

  const auto no_conflict = [](const auto & line1, const auto & line2) {
    return line1.end_idx < line2.start_idx || line2.end_idx < line1.start_idx;
  };

  const auto same_side_shift = [](const auto & line1, const auto & line2) {
    return line1.object_on_right == line2.object_on_right;
  };

  const auto within = [](const auto & line, const size_t idx) {
    return line.start_idx < idx && idx < line.end_idx;
  };

  ret.push_back(outlines.front());

  for (size_t i = 1; i < outlines.size(); i++) {
    auto & last_outline = ret.back();
    auto & next_outline = outlines.at(i);

    const auto & return_line = last_outline.return_line;
    const auto & avoid_line = next_outline.avoid_line;

    if (no_conflict(return_line, avoid_line)) {
      ret.push_back(outlines.at(i));
      continue;
    }

    const auto merged_shift_line = merge(return_line, avoid_line, generateUUID());

    if (!helper_->isComfortable(AvoidLineArray{merged_shift_line})) {
      ret.push_back(outlines.at(i));
      continue;
    }

    if (same_side_shift(return_line, avoid_line)) {
      last_outline.middle_lines.push_back(merged_shift_line);
      last_outline.return_line = next_outline.return_line;
      debug.step1_merged_shift_line.push_back(merged_shift_line);
      continue;
    }

    if (within(return_line, avoid_line.end_idx) && within(avoid_line, return_line.start_idx)) {
      last_outline.middle_lines.push_back(merged_shift_line);
      last_outline.return_line = next_outline.return_line;
      debug.step1_merged_shift_line.push_back(merged_shift_line);
      continue;
    }

    if (within(return_line, avoid_line.start_idx) && within(avoid_line, return_line.end_idx)) {
      last_outline.middle_lines.push_back(merged_shift_line);
      last_outline.return_line = next_outline.return_line;
      debug.step1_merged_shift_line.push_back(merged_shift_line);
      continue;
    }
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, ret);
  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, debug.step1_merged_shift_line);

  return ret;
}

AvoidOutlines ShiftLineGenerator::applyFillGapProcess(
  const AvoidOutlines & outlines, const AvoidancePlanningData & data, DebugData & debug) const
{
  AvoidOutlines ret = outlines;

  for (auto & outline : ret) {
    if (outline.middle_lines.empty()) {
      const auto new_line = fill(outline.avoid_line, outline.return_line, generateUUID());
      outline.middle_lines.push_back(new_line);
      debug.step1_filled_shift_line.push_back(new_line);
    }

    helper_->alignShiftLinesOrder(outline.middle_lines, false);

    if (outline.avoid_line.end_longitudinal < outline.middle_lines.front().start_longitudinal) {
      const auto new_line = fill(outline.avoid_line, outline.middle_lines.front(), generateUUID());
      outline.middle_lines.push_back(new_line);
      debug.step1_filled_shift_line.push_back(new_line);
    }

    helper_->alignShiftLinesOrder(outline.middle_lines, false);

    if (outline.middle_lines.back().end_longitudinal < outline.return_line.start_longitudinal) {
      const auto new_line = fill(outline.middle_lines.back(), outline.return_line, generateUUID());
      outline.middle_lines.push_back(new_line);
      debug.step1_filled_shift_line.push_back(new_line);
    }

    helper_->alignShiftLinesOrder(outline.middle_lines, false);
  }

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, ret);
  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, debug.step1_filled_shift_line);

  return ret;
}

AvoidLineArray ShiftLineGenerator::applyFillGapProcess(
  const AvoidLineArray & shift_lines, const AvoidancePlanningData & data, DebugData & debug) const
{
  AvoidLineArray sorted = shift_lines;

  helper_->alignShiftLinesOrder(sorted, false);

  AvoidLineArray ret = sorted;

  if (shift_lines.empty()) {
    return ret;
  }

  // fill gap between ego and nearest shift line.
  if (sorted.front().start_longitudinal > 0.0) {
    AvoidLine ego_line{};
    utils::avoidance::setEndData(
      ego_line, helper_->getEgoLinearShift(), data.reference_pose, data.ego_closest_path_index,
      0.0);

    const auto new_line = fill(ego_line, sorted.front(), generateUUID());
    ret.push_back(new_line);
    debug.step1_front_shift_line.push_back(new_line);
  }

  helper_->alignShiftLinesOrder(sorted, false);

  // fill gap among shift lines.
  for (size_t i = 0; i < sorted.size() - 1; ++i) {
    if (sorted.at(i + 1).start_longitudinal < sorted.at(i).end_longitudinal) {
      continue;
    }

    const auto new_line = fill(sorted.at(i), sorted.at(i + 1), generateUUID());
    ret.push_back(new_line);
    debug.step1_front_shift_line.push_back(new_line);
  }

  helper_->alignShiftLinesOrder(ret, false);

  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, ret);
  utils::avoidance::fillAdditionalInfoFromLongitudinal(data, debug.step1_front_shift_line);

  return ret;
}

AvoidLineArray ShiftLineGenerator::applyCombineProcess(
  const AvoidLineArray & shift_lines, const AvoidLineArray & registered_lines,
  DebugData & debug) const
{
  debug.step1_registered_shift_line = registered_lines;
  return utils::avoidance::combineRawShiftLinesWithUniqueCheck(registered_lines, shift_lines);
}

AvoidLineArray ShiftLineGenerator::applyMergeProcess(
  const AvoidLineArray & shift_lines, const AvoidancePlanningData & data, DebugData & debug) const
{
  // Generate shift line by merging shift_lines.
  ShiftLineData shift_line_data;
  generateTotalShiftLine(shift_lines, data, shift_line_data);

  // Re-generate shift points by detecting gradient-change point of the shift line.
  auto merged_shift_lines = extractShiftLinesFromLine(data, shift_line_data);

  // set parent id
  for (auto & al : merged_shift_lines) {
    al.parent_ids = utils::avoidance::calcParentIds(shift_lines, al);
  }

  // sort by distance from ego.
  helper_->alignShiftLinesOrder(merged_shift_lines);

  // debug visualize
  {
    debug.pos_shift = shift_line_data.pos_shift_line;
    debug.neg_shift = shift_line_data.neg_shift_line;
    debug.total_shift = shift_line_data.shift_line;
    debug.pos_shift_grad = shift_line_data.pos_shift_line_grad;
    debug.neg_shift_grad = shift_line_data.neg_shift_line_grad;
    debug.total_forward_grad = shift_line_data.forward_grad;
    debug.total_backward_grad = shift_line_data.backward_grad;
    debug.step2_merged_shift_line = merged_shift_lines;
  }

  return merged_shift_lines;
}

AvoidLineArray ShiftLineGenerator::applyTrimProcess(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  if (shift_lines.empty()) {
    return shift_lines;
  }

  AvoidLineArray sl_array_trimmed = shift_lines;

  // sort shift points from front to back.
  helper_->alignShiftLinesOrder(sl_array_trimmed);

  // - Change the shift length to the previous one if the deviation is small.
  {
    constexpr double SHIFT_DIFF_THRES = 1.0;
    applySmallShiftFilter(sl_array_trimmed, SHIFT_DIFF_THRES);
  }

  // - Combine avoid points that have almost same gradient.
  // this is to remove the noise.
  {
    const auto THRESHOLD = parameters_->same_grad_filter_1_threshold;
    applySimilarGradFilter(sl_array_trimmed, THRESHOLD);
    debug.step3_grad_filtered_1st = sl_array_trimmed;
  }

  // - Quantize the shift length to reduce the shift point noise
  // This is to remove the noise coming from detection accuracy, interpolation, resampling, etc.
  {
    const auto THRESHOLD = parameters_->quantize_filter_threshold;
    applyQuantizeProcess(sl_array_trimmed, THRESHOLD);
    debug.step3_quantize_filtered = sl_array_trimmed;
  }

  // - Change the shift length to the previous one if the deviation is small.
  {
    constexpr double SHIFT_DIFF_THRES = 1.0;
    applySmallShiftFilter(sl_array_trimmed, SHIFT_DIFF_THRES);
    debug.step3_noise_filtered = sl_array_trimmed;
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto THRESHOLD = parameters_->same_grad_filter_2_threshold;
    applySimilarGradFilter(sl_array_trimmed, THRESHOLD);
    debug.step3_grad_filtered_2nd = sl_array_trimmed;
  }

  // - Combine avoid points that have almost same gradient (again)
  {
    const auto THRESHOLD = parameters_->same_grad_filter_3_threshold;
    applySimilarGradFilter(sl_array_trimmed, THRESHOLD);
    debug.step3_grad_filtered_3rd = sl_array_trimmed;
  }

  return sl_array_trimmed;
}

void ShiftLineGenerator::applyQuantizeProcess(
  AvoidLineArray & shift_lines, const double threshold) const
{
  if (threshold < 1.0e-5) {
    return;  // no need to process
  }

  for (auto & sl : shift_lines) {
    sl.end_shift_length = std::round(sl.end_shift_length / threshold) * threshold;
  }

  helper_->alignShiftLinesOrder(shift_lines);
}

void ShiftLineGenerator::applySmallShiftFilter(
  AvoidLineArray & shift_lines, const double threshold) const
{
  if (shift_lines.empty()) {
    return;
  }

  AvoidLineArray input = shift_lines;
  shift_lines.clear();

  for (const auto & s : input) {
    if (s.getRelativeLongitudinal() < threshold) {
      continue;
    }

    if (s.start_longitudinal + 1e-3 < helper_->getMinimumPrepareDistance()) {
      continue;
    }

    shift_lines.push_back(s);
  }
}

void ShiftLineGenerator::applySimilarGradFilter(
  AvoidLineArray & avoid_lines, const double threshold) const
{
  if (avoid_lines.empty()) {
    return;
  }

  AvoidLineArray input = avoid_lines;
  avoid_lines.clear();
  avoid_lines.push_back(input.front());  // Take the first one anyway (think later)

  AvoidLine base_line = input.front();

  AvoidLineArray combine_buffer;
  combine_buffer.push_back(input.front());

  constexpr auto SHIFT_THR = 1e-3;
  const auto is_negative_shift = [&](const auto & s) {
    return s.getRelativeLength() < -1.0 * SHIFT_THR;
  };

  const auto is_positive_shift = [&](const auto & s) { return s.getRelativeLength() > SHIFT_THR; };

  for (size_t i = 1; i < input.size(); ++i) {
    AvoidLine combine{};

    utils::avoidance::setStartData(
      combine, base_line.start_shift_length, base_line.start, base_line.start_idx,
      base_line.start_longitudinal);
    utils::avoidance::setEndData(
      combine, input.at(i).end_shift_length, input.at(i).end, input.at(i).end_idx,
      input.at(i).end_longitudinal);

    combine.parent_ids =
      utils::avoidance::concatParentIds(base_line.parent_ids, input.at(i).parent_ids);

    combine_buffer.push_back(input.at(i));

    const auto violates = [&]() {
      if (is_negative_shift(input.at(i)) && is_positive_shift(base_line)) {
        return true;
      }

      if (is_negative_shift(base_line) && is_positive_shift(input.at(i))) {
        return true;
      }

      const auto lon_combine = combine.getRelativeLongitudinal();
      const auto base_length = base_line.getGradient() * lon_combine;
      return std::abs(combine.getRelativeLength() - base_length) > threshold;
    }();

    if (violates) {
      avoid_lines.push_back(input.at(i));
      base_line = input.at(i);
      combine_buffer.clear();
      combine_buffer.push_back(input.at(i));
    } else {
      avoid_lines.back() = combine;
    }
  }

  helper_->alignShiftLinesOrder(avoid_lines);
}

AvoidLineArray ShiftLineGenerator::addReturnShiftLine(
  const AvoidLineArray & shift_lines, const AvoidancePlanningData & data, DebugData & debug) const
{
  AvoidLineArray ret = shift_lines;

  constexpr double ep = 1.0e-3;
  constexpr double RETURN_SHIFT_THRESHOLD = 0.1;
  const bool has_candidate_point = !ret.empty();
  const bool has_registered_point = last_.has_value();

  const auto exist_unavoidable_object = std::any_of(
    data.target_objects.begin(), data.target_objects.end(),
    [](const auto & o) { return !o.is_avoidable && o.longitudinal > 0.0; });

  if (exist_unavoidable_object) {
    return ret;
  }

  if (last_.has_value()) {
    if (std::abs(last_.value().end_shift_length) < RETURN_SHIFT_THRESHOLD) {
      return ret;
    }
  } else {
    // If the return-to-center shift points are already registered, do nothing.
    if (std::abs(base_offset_) < ep) {
      return ret;
    }
  }

  // From here, the return-to-center is not registered. But perhaps the candidate is
  // already generated.

  // If it has a shift point, add return shift from the existing last shift point.
  // If not, add return shift from ego point. (prepare distance is considered for both.)
  ShiftLine last_sl;  // the return-shift will be generated after the last shift point.
  {
    // avoidance points: Yes, shift points: No -> select last avoidance point.
    if (has_candidate_point && !has_registered_point) {
      helper_->alignShiftLinesOrder(ret, false);
      last_sl = ret.back();
    }

    // avoidance points: No, shift points: Yes -> select last shift point.
    if (!has_candidate_point && has_registered_point) {
      last_sl = utils::avoidance::fillAdditionalInfo(data, AvoidLine{last_.value()});
    }

    // avoidance points: Yes, shift points: Yes -> select the last one from both.
    if (has_candidate_point && has_registered_point) {
      helper_->alignShiftLinesOrder(ret, false);
      const auto & al = ret.back();
      const auto & sl = utils::avoidance::fillAdditionalInfo(data, AvoidLine{last_.value()});
      last_sl = (sl.end_longitudinal > al.end_longitudinal) ? sl : al;
    }

    // avoidance points: No, shift points: No -> set the ego position to the last shift point
    // so that the return-shift will be generated from ego position.
    if (!has_candidate_point && !has_registered_point) {
      last_sl.end_idx = data.ego_closest_path_index;
      last_sl.end = data.reference_path.points.at(last_sl.end_idx).point.pose;
      last_sl.end_shift_length = base_offset_;
    }
  }

  // There already is a shift point candidates to go back to center line, but it could be too sharp
  // due to detection noise or timing.
  // Here the return-shift from ego is added for the in case.
  if (std::fabs(last_sl.end_shift_length) < RETURN_SHIFT_THRESHOLD) {
    const auto current_base_shift = helper_->getEgoShift();
    if (std::abs(current_base_shift) < ep) {
      return ret;
    }

    // Is there a shift point in the opposite direction of the current_base_shift?
    //   No  -> we can overwrite the return shift, because the other shift points that decrease
    //          the shift length are for return-shift.
    //   Yes -> we can NOT overwrite, because it might be not a return-shift, but a avoiding
    //          shift to the opposite direction which can not be overwritten by the return-shift.
    for (const auto & sl : ret) {
      if (
        (current_base_shift > 0.0 && sl.end_shift_length < -ep) ||
        (current_base_shift < 0.0 && sl.end_shift_length > ep)) {
        return ret;
      }
    }

    // If return shift already exists in candidate or registered shift lines, skip adding return
    // shift.
    if (has_candidate_point || has_registered_point) {
      return ret;
    }

    // set the return-shift from ego.
    last_sl.end_idx = data.ego_closest_path_index;
    last_sl.end = data.reference_path.points.at(last_sl.end_idx).point.pose;
    last_sl.end_shift_length = current_base_shift;
  }

  const auto & arclength_from_ego = data.arclength_from_ego;

  const auto nominal_prepare_distance = helper_->getNominalPrepareDistance();
  const auto nominal_avoid_distance = helper_->getMaxAvoidanceDistance(last_sl.end_shift_length);

  if (arclength_from_ego.empty()) {
    return ret;
  }

  const auto remaining_distance = std::min(
    arclength_from_ego.back() - parameters_->dead_line_buffer_for_goal, data.to_return_point);

  // If the avoidance point has already been set, the return shift must be set after the point.
  const auto last_sl_distance = data.arclength_from_ego.at(last_sl.end_idx);

  // check if there is enough distance for return.
  if (last_sl_distance > remaining_distance) {  // tmp: add some small number (+1.0)
    RCLCPP_DEBUG(rclcpp::get_logger(""), "No enough distance for return.");
    return ret;
  }

  // If the remaining distance is not enough, the return shift needs to be shrunk.
  // (or another option is just to ignore the return-shift.)
  // But we do not want to change the last shift point, so we will shrink the distance after
  // the last shift point.
  //
  //  The line "===" is fixed, "---" is scaled.
  //
  // [Before Scaling]
  //  ego              last_sl_end             prepare_end            path_end    avoid_end
  // ==o====================o----------------------o----------------------o------------o
  //   |            prepare_dist                   |          avoid_dist               |
  //
  // [After Scaling]
  // ==o====================o------------------o--------------------------o
  //   |        prepare_dist_scaled            |    avoid_dist_scaled     |
  //
  const double variable_prepare_distance =
    std::max(nominal_prepare_distance - last_sl_distance, 0.0);

  double prepare_distance_scaled = std::max(nominal_prepare_distance, last_sl_distance);
  double avoid_distance_scaled = nominal_avoid_distance;
  if (remaining_distance < prepare_distance_scaled + avoid_distance_scaled) {
    const auto scale = (remaining_distance - last_sl_distance) /
                       std::max(nominal_avoid_distance + variable_prepare_distance, 0.1);
    prepare_distance_scaled = last_sl_distance + scale * nominal_prepare_distance;
    avoid_distance_scaled *= scale;
  }

  // shift point for prepare distance: from last shift to return-start point.
  if (nominal_prepare_distance > last_sl_distance) {
    AvoidLine al;
    al.id = generateUUID();
    al.start_idx = last_sl.end_idx;
    al.start = last_sl.end;
    al.start_longitudinal = arclength_from_ego.at(al.start_idx);
    al.end_idx =
      utils::avoidance::findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.end = data.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = last_sl.end_shift_length;
    al.start_shift_length = last_sl.end_shift_length;
    ret.push_back(al);
    debug.step1_return_shift_line.push_back(al);
  }

  // shift point for return to center line
  {
    AvoidLine al;
    al.id = generateUUID();
    al.start_idx =
      utils::avoidance::findPathIndexFromArclength(arclength_from_ego, prepare_distance_scaled);
    al.start = data.reference_path.points.at(al.start_idx).point.pose;
    al.start_longitudinal = arclength_from_ego.at(al.start_idx);
    al.end_idx = utils::avoidance::findPathIndexFromArclength(
      arclength_from_ego, prepare_distance_scaled + avoid_distance_scaled);
    al.end = data.reference_path.points.at(al.end_idx).point.pose;
    al.end_longitudinal = arclength_from_ego.at(al.end_idx);
    al.end_shift_length = 0.0;
    al.start_shift_length = last_sl.end_shift_length;
    ret.push_back(al);
    debug.step1_return_shift_line.push_back(al);
  }

  return ret;
}

AvoidLineArray ShiftLineGenerator::findNewShiftLine(
  const AvoidLineArray & shift_lines, DebugData & debug) const
{
  if (shift_lines.empty()) {
    return {};
  }

  // add small shift lines.
  const auto add_straight_shift =
    [&, this](auto & subsequent, bool has_large_shift, const size_t start_idx) {
      for (size_t i = start_idx; i < shift_lines.size(); ++i) {
        if (
          std::abs(shift_lines.at(i).getRelativeLength()) >
          parameters_->lateral_small_shift_threshold) {
          if (has_large_shift) {
            return;
          }

          has_large_shift = true;
        }

        if (!helper_->isComfortable(AvoidLineArray{shift_lines.at(i)})) {
          return;
        }

        subsequent.push_back(shift_lines.at(i));
      }
    };

  // get subsequent shift lines.
  const auto get_subsequent_shift = [&, this](size_t i) {
    AvoidLineArray subsequent{shift_lines.at(i)};

    if (!helper_->isComfortable(subsequent)) {
      return subsequent;
    }

    if (shift_lines.size() == i + 1) {
      return subsequent;
    }

    if (!helper_->isComfortable(AvoidLineArray{shift_lines.at(i + 1)})) {
      return subsequent;
    }

    if (
      std::abs(shift_lines.at(i).getRelativeLength()) <
      parameters_->lateral_small_shift_threshold) {
      const auto has_large_shift =
        shift_lines.at(i + 1).getRelativeLength() > parameters_->lateral_small_shift_threshold;

      // candidate.at(i) is small length shift line. add large length shift line.
      subsequent.push_back(shift_lines.at(i + 1));
      add_straight_shift(subsequent, has_large_shift, i + 2);
    } else {
      // candidate.at(i) is large length shift line. add small length shift lines.
      add_straight_shift(subsequent, true, i + 1);
    }

    return subsequent;
  };

  // check ignore or not.
  const auto is_ignore_shift = [this](const auto & s) {
    return std::abs(helper_->getRelativeShiftToPath(s)) < parameters_->lateral_execution_threshold;
  };

  for (size_t i = 0; i < shift_lines.size(); ++i) {
    const auto & candidate = shift_lines.at(i);

    // prevent sudden steering.
    if (candidate.start_longitudinal < helper_->getMinimumPrepareDistance()) {
      break;
    }

    if (is_ignore_shift(candidate)) {
      continue;
    }

    if (perManeuver(parameters_->policy_approval)) {
      debug.step4_new_shift_line = shift_lines;
      return shift_lines;
    }

    const auto new_shift_lines = get_subsequent_shift(i);
    debug.step4_new_shift_line = new_shift_lines;
    return new_shift_lines;
  }

  return {};
}

void ShiftLineGenerator::updateRegisteredRawShiftLines(const AvoidancePlanningData & data)
{
  utils::avoidance::fillAdditionalInfoFromPoint(data, raw_registered_);

  AvoidLineArray avoid_lines;

  const auto has_large_offset = [this](const auto & s) {
    constexpr double THRESHOLD = 0.1;
    const auto ego_shift_length = helper_->getEgoLinearShift();

    const auto start_to_ego_longitudinal = -1.0 * s.start_longitudinal;

    if (start_to_ego_longitudinal < 0.0) {
      return false;
    }

    const auto reg_shift_length =
      s.getGradient() * start_to_ego_longitudinal + s.start_shift_length;

    return std::abs(ego_shift_length - reg_shift_length) > THRESHOLD;
  };

  const auto ego_idx = data.ego_closest_path_index;

  for (const auto & s : raw_registered_) {
    // invalid
    if (s.end_idx < ego_idx) {
      continue;
    }

    // invalid
    if (has_large_offset(s)) {
      continue;
    }

    // valid
    avoid_lines.push_back(s);
  }

  raw_registered_ = avoid_lines;
}

void ShiftLineGenerator::setRawRegisteredShiftLine(
  const AvoidLineArray & shift_lines, const AvoidancePlanningData & data)
{
  if (shift_lines.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger(""), "future is empty! return.");
    return;
  }

  auto future_with_info = shift_lines;
  utils::avoidance::fillAdditionalInfoFromPoint(data, future_with_info);

  // sort by longitudinal
  std::sort(future_with_info.begin(), future_with_info.end(), [](auto a, auto b) {
    return a.end_longitudinal < b.end_longitudinal;
  });

  // calc relative lateral length
  future_with_info.front().start_shift_length = base_offset_;
  for (size_t i = 1; i < future_with_info.size(); ++i) {
    future_with_info.at(i).start_shift_length = future_with_info.at(i - 1).end_shift_length;
  }

  const auto is_registered = [this](const auto id) {
    const auto & r = raw_registered_;
    return std::any_of(r.begin(), r.end(), [id](const auto & s) { return s.id == id; });
  };

  const auto same_id_shift_line = [this](const auto id) {
    const auto & r = raw_;
    const auto itr = std::find_if(r.begin(), r.end(), [id](const auto & s) { return s.id == id; });
    if (itr != r.end()) {
      return *itr;
    }
    throw std::logic_error("not found same id current raw shift line.");
  };

  for (const auto & s : future_with_info) {
    if (s.parent_ids.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger(""), "avoid line for path_shifter must have parent_id.");
    }

    for (const auto id : s.parent_ids) {
      if (is_registered(id)) {
        continue;
      }

      raw_registered_.push_back(same_id_shift_line(id));
    }
  }
}
}  // namespace behavior_path_planner::utils::avoidance
