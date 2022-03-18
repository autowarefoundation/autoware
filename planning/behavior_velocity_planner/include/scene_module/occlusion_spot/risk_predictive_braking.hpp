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

#ifndef SCENE_MODULE__OCCLUSION_SPOT__RISK_PREDICTIVE_BRAKING_HPP_
#define SCENE_MODULE__OCCLUSION_SPOT__RISK_PREDICTIVE_BRAKING_HPP_

#include <scene_module/occlusion_spot/occlusion_spot_utils.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <algorithm>
#include <vector>

namespace behavior_velocity_planner
{
namespace occlusion_spot_utils
{
void applySafeVelocityConsideringPossibleCollision(
  PathWithLaneId * inout_path, std::vector<PossibleCollisionInfo> & possible_collisions,
  const PlannerParam & param);

int insertSafeVelocityToPath(
  const geometry_msgs::msg::Pose & in_pose, const double safe_vel, const PlannerParam & param,
  PathWithLaneId * inout_path);

/**
 * @param: v: ego velocity config
 * @param: ttc: time to collision
 * @return safe motion
 **/
SafeMotion calculateSafeMotion(const Velocity & v, const double ttc);

double calculateInsertVelocity(
  const double min_allowed_vel, const double safe_vel, const double min_vel,
  const double original_vel);

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__RISK_PREDICTIVE_BRAKING_HPP_
