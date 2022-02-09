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

// @brief calculates the maximum velocity allowing to decelerate within the given distance
inline double calculateMinSlowDownVelocity(
  const double v0, const double len, const double a_max, const double safe_vel)
{
  // if target velocity is inserted backward return current velocity as limit
  if (len < 0) return safe_vel;
  return std::sqrt(std::max(std::pow(v0, 2.0) - 2.0 * std::abs(a_max) * len, 0.0));
}

/**
 * @param: sv: ego velocity config
 * @param: ttc: time to collision
 * @return safe motion
 **/
inline SafeMotion calculateSafeMotion(const Velocity & v, const double ttc)
{
  SafeMotion sm;
  const double j_max = v.safety_ratio * v.max_stop_jerk;
  const double a_max = v.safety_ratio * v.max_stop_accel;
  const double t1 = v.delay_time;
  double t2 = a_max / j_max;
  double & v_safe = sm.safe_velocity;
  double & stop_dist = sm.stop_dist;
  if (ttc <= t1) {
    // delay
    v_safe = 0;
    stop_dist = 0;
  } else if (ttc <= t2 + t1) {
    // delay + const jerk
    t2 = ttc - t1;
    v_safe = -0.5 * j_max * t2 * t2;
    stop_dist = v_safe * t1 - j_max * t2 * t2 * t2 / 6;
  } else {
    const double t3 = ttc - t2 - t1;
    // delay + const jerk + const accel
    const double v2 = -0.5 * j_max * t2 * t2;
    v_safe = v2 - a_max * t3;
    stop_dist = v_safe * t1 - j_max * t2 * t2 * t2 / 6 + v2 * t3 - 0.5 * a_max * t3 * t3;
  }
  stop_dist += v.safe_margin;
  return sm;
}

inline double calculateInsertVelocity(
  const double min_allowed_vel, const double safe_vel, const double min_vel,
  const double original_vel)
{
  const double max_vel_noise = 0.05;
  // ensure safe velocity doesn't exceed maximum allowed pbs deceleration
  double cmp_safe_vel = std::max(min_allowed_vel + max_vel_noise, safe_vel);
  // ensure safe path velocity is also above ego min velocity
  cmp_safe_vel = std::max(cmp_safe_vel, min_vel);
  // ensure we only lower the original velocity (and do not increase it)
  cmp_safe_vel = std::min(cmp_safe_vel, original_vel);
  return cmp_safe_vel;
}

}  // namespace occlusion_spot_utils
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__OCCLUSION_SPOT__RISK_PREDICTIVE_BRAKING_HPP_
