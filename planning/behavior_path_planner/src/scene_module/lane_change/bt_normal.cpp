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

#include "behavior_path_planner/scene_module/lane_change/bt_normal.hpp"

#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
NormalLaneChangeBT::NormalLaneChangeBT(
  const std::shared_ptr<LaneChangeParameters> & parameters, LaneChangeModuleType type,
  Direction direction)
: NormalLaneChange(parameters, type, direction)
{
}

PathWithLaneId NormalLaneChangeBT::getReferencePath() const
{
  PathWithLaneId reference_path;
  if (!utils::lane_change::isEgoWithinOriginalLane(
        status_.current_lanes, getEgoPose(), planner_data_->parameters)) {
    return reference_path;
  }

  const auto & route_handler = getRouteHandler();
  const auto & current_pose = getEgoPose();
  const auto & common_parameters = planner_data_->parameters;

  // Set header
  reference_path.header = getRouteHeader();

  const auto current_lanes = getCurrentLanes();

  if (current_lanes.empty()) {
    return reference_path;
  }

  reference_path = utils::getCenterLinePath(
    *route_handler, current_lanes, current_pose, common_parameters.backward_path_length,
    common_parameters.forward_path_length, common_parameters);

  const auto shift_intervals =
    route_handler->getLateralIntervalsToPreferredLane(current_lanes.back());
  const double lane_change_buffer =
    utils::calcMinimumLaneChangeLength(common_parameters, shift_intervals);

  reference_path = utils::setDecelerationVelocity(
    *route_handler, reference_path, current_lanes, common_parameters.lane_change_prepare_duration,
    lane_change_buffer);

  const auto & dp = planner_data_->drivable_area_expansion_parameters;

  const auto drivable_lanes = utils::generateDrivableLanes(current_lanes);
  const auto shorten_lanes = utils::cutOverlappedLanes(reference_path, drivable_lanes);
  const auto expanded_lanes = utils::expandLanelets(
    shorten_lanes, dp.drivable_area_left_bound_offset, dp.drivable_area_right_bound_offset,
    dp.drivable_area_types_to_skip);
  utils::generateDrivableArea(
    reference_path, expanded_lanes, common_parameters.vehicle_length, planner_data_);

  return reference_path;
}

lanelet::ConstLanelets NormalLaneChangeBT::getCurrentLanes() const
{
  return utils::getCurrentLanes(planner_data_);
}

int NormalLaneChangeBT::getNumToPreferredLane(const lanelet::ConstLanelet & lane) const
{
  return std::abs(getRouteHandler()->getNumLaneToPreferredLane(lane));
}

PathWithLaneId NormalLaneChangeBT::getPrepareSegment(
  const lanelet::ConstLanelets & current_lanes, const double arc_length_from_current,
  const double backward_path_length, const double prepare_length,
  const double prepare_velocity) const
{
  if (current_lanes.empty()) {
    return PathWithLaneId();
  }

  const double s_start = arc_length_from_current - backward_path_length;
  const double s_end = arc_length_from_current + prepare_length;

  RCLCPP_DEBUG(
    rclcpp::get_logger("behavior_path_planner")
      .get_child("lane_change")
      .get_child("util")
      .get_child("getPrepareSegment"),
    "start: %f, end: %f", s_start, s_end);

  PathWithLaneId prepare_segment =
    getRouteHandler()->getCenterLinePath(current_lanes, s_start, s_end);

  prepare_segment.points.back().point.longitudinal_velocity_mps = std::min(
    prepare_segment.points.back().point.longitudinal_velocity_mps,
    static_cast<float>(prepare_velocity));

  return prepare_segment;
}

std::vector<DrivableLanes> NormalLaneChangeBT::getDrivableLanes() const
{
  return utils::lane_change::generateDrivableLanes(
    *getRouteHandler(), status_.current_lanes, status_.lane_change_lanes);
}
}  // namespace behavior_path_planner
