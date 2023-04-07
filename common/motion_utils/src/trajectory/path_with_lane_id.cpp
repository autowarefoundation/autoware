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

namespace motion_utils
{
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
