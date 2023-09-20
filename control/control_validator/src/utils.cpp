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

#include "control_validator/utils.hpp"

#include <motion_utils/trajectory/interpolation.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace control_validator
{

void shiftPose(Pose & pose, double longitudinal)
{
  const auto yaw = tf2::getYaw(pose.orientation);
  pose.position.x += std::cos(yaw) * longitudinal;
  pose.position.y += std::sin(yaw) * longitudinal;
}

void insertPointInPredictedTrajectory(
  TrajectoryPoints & modified_trajectory, const Pose & reference_pose,
  const TrajectoryPoints & predicted_trajectory)
{
  const auto point_to_interpolate =
    motion_utils::calcInterpolatedPoint(convertToTrajectory(predicted_trajectory), reference_pose);
  modified_trajectory.insert(modified_trajectory.begin(), point_to_interpolate);
}

TrajectoryPoints reverseTrajectoryPoints(const TrajectoryPoints & trajectory_points)
{
  TrajectoryPoints reversed_trajectory_points;
  reversed_trajectory_points.reserve(trajectory_points.size());
  std::reverse_copy(
    trajectory_points.begin(), trajectory_points.end(),
    std::back_inserter(reversed_trajectory_points));
  return reversed_trajectory_points;
}

bool removeFrontTrajectoryPoint(
  const TrajectoryPoints & trajectory_points, TrajectoryPoints & modified_trajectory_points,
  const TrajectoryPoints & predicted_trajectory_points)
{
  bool predicted_trajectory_point_removed = false;
  for (const auto & point : predicted_trajectory_points) {
    if (
      motion_utils::calcLongitudinalOffsetToSegment(trajectory_points, 0, point.pose.position) <
      0.0) {
      modified_trajectory_points.erase(modified_trajectory_points.begin());

      predicted_trajectory_point_removed = true;
    } else {
      break;
    }
  }

  return predicted_trajectory_point_removed;
}

Trajectory alignTrajectoryWithReferenceTrajectory(
  const Trajectory & trajectory, const Trajectory & predicted_trajectory)
{
  const auto last_seg_length = motion_utils::calcSignedArcLength(
    trajectory.points, trajectory.points.size() - 2, trajectory.points.size() - 1);

  // If no overlapping between trajectory and predicted_trajectory, return empty trajectory
  // predicted_trajectory:   p1------------------pN
  // trajectory:                                     t1------------------tN
  //     OR
  // predicted_trajectory:                           p1------------------pN
  // trajectory:             t1------------------tN
  const bool & is_p_n_before_t1 =
    motion_utils::calcLongitudinalOffsetToSegment(
      trajectory.points, 0, predicted_trajectory.points.back().pose.position) < 0.0;
  const bool & is_p1_behind_t_n = motion_utils::calcLongitudinalOffsetToSegment(
                                    trajectory.points, trajectory.points.size() - 2,
                                    predicted_trajectory.points.front().pose.position) -
                                    last_seg_length >
                                  0.0;
  const bool is_no_overlapping = (is_p_n_before_t1 || is_p1_behind_t_n);

  if (is_no_overlapping) {
    return Trajectory();
  }

  auto modified_trajectory_points = convertToTrajectoryPointArray(predicted_trajectory);
  auto predicted_trajectory_points = convertToTrajectoryPointArray(predicted_trajectory);
  auto trajectory_points = convertToTrajectoryPointArray(trajectory);

  // If first point of predicted_trajectory is in front of start of trajectory, erase points which
  // are in front of trajectory start point and insert pNew along the predicted_trajectory
  // predicted_trajectory:   　　　　p1-----p2-----p3----//------pN
  // trajectory:                               t1--------//------tN
  // ↓
  // predicted_trajectory:   　　　　        pNew--p3----//------pN
  // trajectory:                               t1--------//------tN
  auto predicted_trajectory_point_removed = removeFrontTrajectoryPoint(
    trajectory_points, modified_trajectory_points, predicted_trajectory_points);

  if (predicted_trajectory_point_removed) {
    insertPointInPredictedTrajectory(
      modified_trajectory_points, trajectory_points.front().pose, predicted_trajectory_points);
  }

  // If last point of predicted_trajectory is behind of end of trajectory, erase points which are
  // behind trajectory last point and insert pNew along the predicted_trajectory
  // predicted_trajectory:   　　　　p1-----//------pN-2-----pN-1-----pN
  // trajectory:                     t1-----//-----tN-1--tN
  // ↓
  // predicted_trajectory:           p1-----//------pN-2-pNew
  // trajectory:                     t1-----//-----tN-1--tN

  auto reversed_predicted_trajectory_points = reverseTrajectoryPoints(predicted_trajectory_points);
  auto reversed_trajectory_points = reverseTrajectoryPoints(trajectory_points);
  auto reversed_modified_trajectory_points = reverseTrajectoryPoints(modified_trajectory_points);

  auto reversed_predicted_trajectory_point_removed = removeFrontTrajectoryPoint(
    reversed_trajectory_points, reversed_modified_trajectory_points,
    reversed_predicted_trajectory_points);

  if (reversed_predicted_trajectory_point_removed) {
    insertPointInPredictedTrajectory(
      reversed_modified_trajectory_points, reversed_trajectory_points.front().pose,
      reversed_predicted_trajectory_points);
  }

  return convertToTrajectory(reverseTrajectoryPoints(reversed_modified_trajectory_points));
}

double calcMaxLateralDistance(
  const Trajectory & reference_trajectory, const Trajectory & predicted_trajectory)
{
  const auto alined_predicted_trajectory =
    alignTrajectoryWithReferenceTrajectory(reference_trajectory, predicted_trajectory);
  double max_dist = 0;
  for (const auto & point : alined_predicted_trajectory.points) {
    const auto p0 = tier4_autoware_utils::getPoint(point);
    // find nearest segment
    const size_t nearest_segment_idx =
      motion_utils::findNearestSegmentIndex(reference_trajectory.points, p0);
    const double temp_dist = std::abs(
      motion_utils::calcLateralOffset(reference_trajectory.points, p0, nearest_segment_idx));
    if (temp_dist > max_dist) {
      max_dist = temp_dist;
    }
  }
  return max_dist;
}

}  // namespace control_validator
