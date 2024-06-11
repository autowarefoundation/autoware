// Copyright 2022 TIER IV, Inc.
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

#ifndef FORWARD_PROJECTION_HPP_
#define FORWARD_PROJECTION_HPP_

#include "parameters.hpp"
#include "types.hpp"

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <vector>

namespace autoware::motion_velocity_planner::obstacle_velocity_limiter
{
/// @brief generate a segment to where ego would reach with constant velocity and heading
/// @param [in] origin origin of the segment
/// @param [in] params parameters of the forward projection
/// @return segment from the origin to its position after duration + the extra_distance
segment_t forwardSimulatedSegment(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params);

/// @brief generate lines for the forward projection using the bicycle model
/// @param [in] origin origin of the projection
/// @param [in] params parameters of the forward projection
/// @return lines from the origin to its positions after forward projection, ordered left to right
multi_linestring_t bicycleProjectionLines(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params);

/// @brief generate projection line using the bicycle model
/// @param [in] origin origin of the projection
/// @param [in] params parameters of the forward projection
/// @param [in] steering_angle steering angle used for the projection
/// @return line from the origin to its position after forward projection
linestring_t bicycleProjectionLine(
  const geometry_msgs::msg::Point & origin, const ProjectionParameters & params,
  const double steering_angle);

/// @brief generate a footprint from a segment and a lateral offset
/// @param [in] segment segment from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(const segment_t & segment, const double lateral_offset);

/// @brief generate a footprint from a linestring and a lateral offset
/// @param [in] linestring linestring from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(const linestring_t & linestring, const double lateral_offset);

/// @brief generate a footprint from multiple linestrings and a lateral offset
/// @param [in] lines linestring from which to create the footprint
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(const multi_linestring_t & lines, const double lateral_offset);

/// @brief generate a footprint from a left, right, and middle linestrings and a lateral offset
/// @param [in] left left linestring
/// @param [in] middle left linestring
/// @param [in] right left linestring
/// @param [in] lateral_offset offset around the segment used to create the footprint
/// @return footprint polygon
polygon_t generateFootprint(
  const linestring_t & left, const linestring_t & middle, const linestring_t & right,
  const double lateral_offset);
}  // namespace autoware::motion_velocity_planner::obstacle_velocity_limiter

#endif  // FORWARD_PROJECTION_HPP_
