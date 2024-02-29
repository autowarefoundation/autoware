// Copyright 2024 TIER IV, Inc.
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

#include "perception_online_evaluator/metrics/deviation_metrics.hpp"

#include "tier4_autoware_utils/geometry/geometry.hpp"
#include "tier4_autoware_utils/geometry/pose_deviation.hpp"

#include <motion_utils/trajectory/trajectory.hpp>

namespace perception_diagnostics
{
namespace metrics
{

double calcLateralDeviation(const std::vector<Pose> & ref_path, const Pose & target_pose)
{
  if (ref_path.empty()) {
    return 0.0;
  }

  const size_t nearest_index = motion_utils::findNearestIndex(ref_path, target_pose.position);
  return std::abs(
    tier4_autoware_utils::calcLateralDeviation(ref_path[nearest_index], target_pose.position));
}

double calcYawDeviation(const std::vector<Pose> & ref_path, const Pose & target_pose)
{
  if (ref_path.empty()) {
    return 0.0;
  }

  const size_t nearest_index = motion_utils::findNearestIndex(ref_path, target_pose.position);
  return std::abs(tier4_autoware_utils::calcYawDeviation(ref_path[nearest_index], target_pose));
}

std::vector<double> calcPredictedPathDeviation(
  const std::vector<Pose> & ref_path, const PredictedPath & pred_path)
{
  std::vector<double> deviations;

  if (ref_path.empty() || pred_path.path.empty()) {
    return {};
  }
  for (const Pose & p : pred_path.path) {
    const size_t nearest_index = motion_utils::findNearestIndex(ref_path, p.position);
    deviations.push_back(
      tier4_autoware_utils::calcDistance2d(ref_path[nearest_index].position, p.position));
  }

  return deviations;
}
}  // namespace metrics
}  // namespace perception_diagnostics
