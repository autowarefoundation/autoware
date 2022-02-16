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
 *
 * @param: longitudinal_distance: longitudinal distance to collision
 * @param: param: planner param
 * @return lateral distance
 **/
inline double calculateLateralDistanceFromTTC(
  const double longitudinal_distance, const PlannerParam & param)
{
  const auto & v = param.v;
  const auto & p = param;
  double v_min = 1.0;
  const double lateral_buffer = 0.5;
  const double min_distance = p.half_vehicle_width + lateral_buffer;
  const double max_distance = p.detection_area.max_lateral_distance;
  if (longitudinal_distance <= 0) return min_distance;
  if (v_min < param.v.min_allowed_velocity) v_min = param.v.min_allowed_velocity;
  // use min velocity if ego velocity is below min allowed
  const double v0 = (v.v_ego > v_min) ? v.v_ego : v_min;
  // here is a part where ego t(ttc) can be replaced by calculation of velocity smoother or ?
  double t = longitudinal_distance / v0;
  double lateral_distance = t * param.pedestrian_vel + p.half_vehicle_width;
  return std::min(max_distance, std::max(min_distance, lateral_distance));
}

/**
 * @param: v: ego velocity config
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
