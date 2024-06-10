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

#ifndef RISK_PREDICTIVE_BRAKING_HPP_
#define RISK_PREDICTIVE_BRAKING_HPP_

#include "occlusion_spot_utils.hpp"

#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <algorithm>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace occlusion_spot_utils
{
void applySafeVelocityConsideringPossibleCollision(
  PathWithLaneId * inout_path, std::vector<PossibleCollisionInfo> & possible_collisions,
  std::vector<geometry_msgs::msg::Pose> & debug_poses, const PlannerParam & param);

/**
 * @param: v: ego velocity config
 * @param: ttv: time to vehicle
 * @return safe motion
 **/
SafeMotion calculateSafeMotion(const Velocity & v, const double ttv);

}  // namespace occlusion_spot_utils
}  // namespace autoware::behavior_velocity_planner

#endif  // RISK_PREDICTIVE_BRAKING_HPP_
