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

#include "motion_utils/trajectory/trajectory.hpp"

namespace motion_utils
{

//
template void validateNonEmpty<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &);
template void validateNonEmpty<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &);
template void validateNonEmpty<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &);

//
template std::optional<bool>
isDrivingForward<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &);
template std::optional<bool>
isDrivingForward<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &);
template std::optional<bool>
isDrivingForward<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &);

//
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &);
template std::optional<bool>
isDrivingForwardWithTwist<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &);

//
template std::vector<autoware_auto_planning_msgs::msg::PathPoint>
removeOverlapPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t start_idx);
template std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>
removeOverlapPoints<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t start_idx);
template std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
removeOverlapPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t start_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t src_idx, const size_t dst_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t src_idx);

//
template std::optional<size_t>
searchZeroVelocityIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist);

//
template size_t findNearestIndex<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & point);
template size_t
findNearestIndex<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point);
template size_t findNearestIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & point);

//
template std::optional<size_t>
findNearestIndex<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

//
template double
calcLongitudinalOffsetToSegment<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t seg_idx,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target, const bool throw_exception);
template double
calcLongitudinalOffsetToSegment<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target, const bool throw_exception);

//
template size_t findNearestSegmentIndex<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & point);
template size_t
findNearestSegmentIndex<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & point);
template size_t
findNearestSegmentIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & point);

//
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);
template std::optional<size_t>
findNearestSegmentIndex<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

//
template double calcLateralOffset<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx, const bool throw_exception);
template double
calcLateralOffset<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx, const bool throw_exception);
template double calcLateralOffset<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & p_target, const size_t seg_idx, const bool throw_exception);

//
template double calcLateralOffset<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception);
template double
calcLateralOffset<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception);
template double calcLateralOffset<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & p_target, const bool throw_exception);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t src_idx, const size_t dst_idx);

//
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const size_t dst_idx);
template std::vector<double>
calcSignedArcLengthPartialSum<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t src_idx, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> &,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> &,
  const geometry_msgs::msg::Point & src_point, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const geometry_msgs::msg::Point & dst_point);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t src_idx, const geometry_msgs::msg::Point & dst_point);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const geometry_msgs::msg::Point & dst_point);

//
template double calcArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points);
template double calcArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points);
template double calcArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points);

//
template std::vector<double>
calcCurvature<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points);
template std::vector<double>
calcCurvature<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points);
template std::vector<double>
calcCurvature<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points);

//
template std::vector<std::pair<double, double>>
calcCurvatureAndArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points);
template std::vector<std::pair<double, double>>
calcCurvatureAndArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points);
template std::vector<std::pair<double, double>>
calcCurvatureAndArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const size_t src_idx);

//
template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const double offset, const bool throw_exception);
template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset, const bool throw_exception);
template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t src_idx, const double offset, const bool throw_exception);

//
template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);
template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);
template std::optional<geometry_msgs::msg::Point>
calcLongitudinalOffsetPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset);

//
template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);
template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t src_idx, const double offset, const bool set_orientation_from_position_direction,
  const bool throw_exception);

//
template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);
template std::optional<geometry_msgs::msg::Pose>
calcLongitudinalOffsetPose<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const double offset,
  const bool set_orientation_from_position_direction);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const size_t seg_idx, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const double insert_point_length, const geometry_msgs::msg::Point & p_target,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const size_t src_segment_idx, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const double overlap_threshold);

//
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertTargetPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double insert_point_length,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points, const double max_dist,
  const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const double overlap_threshold = 1e-3);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const size_t src_segment_idx, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold = 1e-3);

//
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const geometry_msgs::msg::Pose & src_pose, const double distance_to_stop_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double max_dist, const double max_yaw, const double overlap_threshold);

//
template std::optional<size_t>
insertStopPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const size_t stop_seg_idx, const geometry_msgs::msg::Point & stop_point,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const double overlap_threshold);

//
template std::optional<size_t>
insertDecelPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const geometry_msgs::msg::Point & src_point, const double distance_to_decel_point,
  const double velocity,
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist);

//
template void insertOrientation<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const bool is_driving_forward);
template void insertOrientation<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const bool is_driving_forward);
template void insertOrientation<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const bool is_driving_forward);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & src_point, const size_t src_seg_idx, const size_t dst_idx);

//
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points, const size_t src_idx,
  const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
template double
calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const size_t src_idx, const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);
template double calcSignedArcLength<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const size_t src_idx, const geometry_msgs::msg::Point & dst_point, const size_t dst_seg_idx);

//
template size_t
findFirstNearestIndexWithSoftConstraints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestIndexWithSoftConstraints<
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestIndexWithSoftConstraints<
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);

//
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);
template size_t findFirstNearestSegmentIndexWithSoftConstraints<
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const double dist_threshold, const double yaw_threshold);

//
template std::optional<double>
calcDistanceToForwardStopPoint<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points_with_twist,
  const geometry_msgs::msg::Pose & pose, const double max_dist, const double max_yaw);

//
template std::vector<autoware_auto_planning_msgs::msg::PathPoint>
cropForwardPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>
cropForwardPoints<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);
template std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
cropForwardPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length);

//
template std::vector<autoware_auto_planning_msgs::msg::PathPoint>
cropBackwardPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>
cropBackwardPoints<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);
template std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
cropBackwardPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double backward_length);

//
template std::vector<autoware_auto_planning_msgs::msg::PathPoint>
cropPoints<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>
cropPoints<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);
template std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>
cropPoints<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & target_pos, const size_t target_seg_idx,
  const double forward_length, const double backward_length);

//
template double calcYawDeviation<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Pose & pose, const bool throw_exception);
template double
calcYawDeviation<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Pose & pose, const bool throw_exception);
template double calcYawDeviation<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Pose & pose, const bool throw_exception);

//
template bool isTargetPointFront<std::vector<autoware_auto_planning_msgs::msg::PathPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & points,
  const geometry_msgs::msg::Point & base_point, const geometry_msgs::msg::Point & target_point,
  const double threshold);
template bool
isTargetPointFront<std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>>(
  const std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId> & points,
  const geometry_msgs::msg::Point & base_point, const geometry_msgs::msg::Point & target_point,
  const double threshold);
template bool isTargetPointFront<std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>>(
  const std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> & points,
  const geometry_msgs::msg::Point & base_point, const geometry_msgs::msg::Point & target_point,
  const double threshold);

}  // namespace motion_utils
