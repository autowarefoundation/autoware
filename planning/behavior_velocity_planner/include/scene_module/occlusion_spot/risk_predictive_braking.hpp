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
void applySafeVelocityConsideringPossibleCollison(
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path,
  std::vector<PossibleCollisionInfo> & possible_collisions, const double current_vel,
  const EgoVelocity & ego, const PlannerParam & param);

int insertSafeVelocityToPath(
  const geometry_msgs::msg::Pose & in_pose, const double safe_vel, const PlannerParam & param,
  autoware_auto_planning_msgs::msg::PathWithLaneId * inout_path);

// @brief calculates the maximum velocity allowing to decelerate within the given distance
inline double calculatePredictiveBrakingVelocity(
  const double ego_vel, const double dist2col, const double pbs_decel)
{
  return std::sqrt(std::max(std::pow(ego_vel, 2.0) - 2.0 * std::abs(pbs_decel) * dist2col, 0.0));
}

/**
 * @param: safety_time: safety time buffer for reaction
 * @param: dist_to_obj: distance to virtual darting object
 * @param: v_obs: relative  velocity for virtual darting object
 * @param: ebs_decel: emergency brake
 * @return safe velocity considering rpb
 **/
inline double calculateSafeRPBVelocity(
  const double safety_time, const double dist_to_obj, const double v_obs, const double ebs_decel)
{
  const double t_vir = dist_to_obj / v_obs;
  // min safety time buffer is at least more than 0
  const double ttc_virtual = std::max(t_vir - safety_time, 0.0);
  // safe velocity consider emergency brake
  const double v_safe = std::abs(ebs_decel) * ttc_virtual;
  return v_safe;
}

inline double getPBSLimitedRPBVelocity(
  const double pbs_vel, const double rpb_vel, const double min_vel, const double original_vel)
{
  const double max_vel_noise = 0.05;
  // ensure safe velocity doesn't exceed maximum allowed pbs deceleration
  double rpb_pbs_limited_vel = std::max(pbs_vel + max_vel_noise, rpb_vel);
  // ensure safe path velocity is also above ego min velocity
  rpb_pbs_limited_vel = std::max(rpb_pbs_limited_vel, min_vel);
  // ensure we only lower the original velocity (and do not increase it)
  rpb_pbs_limited_vel = std::min(rpb_pbs_limited_vel, original_vel);
  return rpb_pbs_limited_vel;
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__RISK_PREDICTIVE_BRAKING_HPP_
