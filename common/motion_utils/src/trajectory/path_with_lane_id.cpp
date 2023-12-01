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

#include "motion_utils/trajectory/path_with_lane_id.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "motion_utils/trajectory/trajectory.hpp"

#include <algorithm>
#include <utility>
#include <vector>

namespace motion_utils
{

std::optional<std::pair<size_t, size_t>> getPathIndexRangeWithLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const int64_t target_lane_id)
{
  size_t start_idx = 0;  // NOTE: to prevent from maybe-uninitialized error
  size_t end_idx = 0;    // NOTE: to prevent from maybe-uninitialized error

  bool found_first_idx = false;
  for (size_t i = 0; i < path.points.size(); ++i) {
    const auto & p = path.points.at(i);
    for (const auto & id : p.lane_ids) {
      if (id == target_lane_id) {
        if (!found_first_idx) {
          start_idx = i;
          found_first_idx = true;
        }
        end_idx = i;
      }
    }
  }

  if (found_first_idx) {
    // NOTE: In order to fully cover lanes with target_lane_id, start_idx and end_idx are expanded.
    start_idx = static_cast<size_t>(std::max(0, static_cast<int>(start_idx) - 1));
    end_idx = std::min(path.points.size() - 1, end_idx + 1);

    return std::make_pair(start_idx, end_idx);
  }

  return {};
}

size_t findNearestIndexFromLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id)
{
  const auto opt_range = getPathIndexRangeWithLaneId(path, lane_id);
  if (opt_range) {
    const size_t start_idx = opt_range->first;
    const size_t end_idx = opt_range->second;

    validateNonEmpty(path.points);

    const auto sub_points = std::vector<autoware_auto_planning_msgs::msg::PathPointWithLaneId>{
      path.points.begin() + start_idx, path.points.begin() + end_idx + 1};
    validateNonEmpty(sub_points);

    return start_idx + findNearestIndex(sub_points, pos);
  }

  return findNearestIndex(path.points, pos);
}

size_t findNearestSegmentIndexFromLaneId(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
  const geometry_msgs::msg::Point & pos, const int64_t lane_id)
{
  const size_t nearest_idx = findNearestIndexFromLaneId(path, pos, lane_id);

  if (nearest_idx == 0) {
    return 0;
  }
  if (nearest_idx == path.points.size() - 1) {
    return path.points.size() - 2;
  }

  const double signed_length = calcLongitudinalOffsetToSegment(path.points, nearest_idx, pos);

  if (signed_length <= 0) {
    return nearest_idx - 1;
  }

  return nearest_idx;
}

// NOTE: rear_to_cog is supposed to be positive
autoware_auto_planning_msgs::msg::PathWithLaneId convertToRearWheelCenter(
  const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const double rear_to_cog,
  const bool enable_last_point_compensation)
{
  auto cog_path = path;

  // calculate curvature and yaw from spline interpolation
  const auto spline = SplineInterpolationPoints2d(path.points);
  const auto curvature_vec = spline.getSplineInterpolatedCurvatures();
  const auto yaw_vec = spline.getSplineInterpolatedYaws();

  for (size_t i = 0; i < path.points.size(); ++i) {
    // calculate beta, which is CoG's velocity direction
    const double beta = std::atan(rear_to_cog * curvature_vec.at(i));

    // apply beta to CoG pose
    geometry_msgs::msg::Pose cog_pose_with_beta;
    cog_pose_with_beta.position = tier4_autoware_utils::getPoint(path.points.at(i));
    cog_pose_with_beta.orientation =
      tier4_autoware_utils::createQuaternionFromYaw(yaw_vec.at(i) - beta);

    const auto rear_pose =
      tier4_autoware_utils::calcOffsetPose(cog_pose_with_beta, -rear_to_cog, 0.0, 0.0);

    // update pose
    tier4_autoware_utils::setPose(rear_pose, cog_path.points.at(i));
  }

  // compensate for the last pose
  if (enable_last_point_compensation) {
    cog_path.points.back() = path.points.back();
  }

  insertOrientation(cog_path.points, true);

  return cog_path;
}
}  // namespace motion_utils
