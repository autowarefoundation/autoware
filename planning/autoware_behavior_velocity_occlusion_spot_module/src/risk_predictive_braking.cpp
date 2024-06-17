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

#include "risk_predictive_braking.hpp"

#include "occlusion_spot_utils.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>

#include <algorithm>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace occlusion_spot_utils
{
void applySafeVelocityConsideringPossibleCollision(
  PathWithLaneId * inout_path, std::vector<PossibleCollisionInfo> & possible_collisions,
  std::vector<geometry_msgs::msg::Pose> & debug_poses, const PlannerParam & param)
{
  // return nullptr or too few points
  if (!inout_path || inout_path->points.size() < 2) {
    return;
  }
  const double v0 = param.v.v_ego;
  const double a0 = param.v.a_ego;
  const double j_min = param.v.max_slow_down_jerk;
  const double a_min = param.v.max_slow_down_accel;
  const double v_min = param.v.min_allowed_velocity;
  for (auto & possible_collision : possible_collisions) {
    const double l_obs = possible_collision.arc_lane_dist_at_collision.length;
    const double v_org = possible_collision.collision_with_margin.longitudinal_velocity_mps;

    // safe velocity : consider ego emergency braking deceleration
    const double v_safe = possible_collision.obstacle_info.safe_motion.safe_velocity;

    // safe slow down: consider ego smooth brake
    const double v_safe_slow_down =
      planning_utils::calcDecelerationVelocityFromDistanceToTarget(j_min, a_min, a0, v0, l_obs);

    // TODO(tanaka): consider edge case if ego passed safe margin
    const double v_slow_down = (l_obs < 0 && v0 <= v_safe) ? v_safe : v_safe_slow_down;

    // skip non effective velocity insertion
    if (v_org < v_safe || v_org < v_slow_down) continue;

    const double max_vel_noise = 0.05;
    // ensure safe velocity doesn't exceed maximum allowed pbs deceleration
    double safe_velocity = std::max(v_safe_slow_down + max_vel_noise, v_slow_down);
    // set safe velocity is not to stop
    safe_velocity = std::max(safe_velocity, v_min);
    possible_collision.obstacle_info.safe_motion.safe_velocity = safe_velocity;
    const auto & pose = possible_collision.collision_with_margin.pose;
    const auto & decel_pose =
      planning_utils::insertDecelPoint(pose.position, *inout_path, safe_velocity);
    if (decel_pose) debug_poses.push_back(decel_pose.value());
  }
}

SafeMotion calculateSafeMotion(const Velocity & v, const double ttv)
{
  SafeMotion sm;
  const double j_max = v.safety_ratio * v.max_stop_jerk;
  const double a_max = v.safety_ratio * v.max_stop_accel;
  const double t1 = v.delay_time;
  double t2 = a_max / j_max;
  double & v_safe = sm.safe_velocity;
  double & stop_dist = sm.stop_dist;
  if (ttv <= t1) {
    // delay
    v_safe = 0;
    stop_dist = 0;
  } else if (ttv <= t2 + t1) {
    // delay + const jerk
    t2 = ttv - t1;
    v_safe = -0.5 * j_max * t2 * t2;
    stop_dist = v_safe * t1 - j_max * t2 * t2 * t2 / 6;
  } else {
    const double t3 = ttv - t2 - t1;
    // delay + const jerk + const accel
    const double v2 = -0.5 * j_max * t2 * t2;
    v_safe = v2 - a_max * t3;
    stop_dist = v_safe * t1 - j_max * t2 * t2 * t2 / 6 + v2 * t3 - 0.5 * a_max * t3 * t3;
  }
  // Note: safe_margin controls behavior insert point
  stop_dist += v.safe_margin;
  return sm;
}
}  // namespace occlusion_spot_utils
}  // namespace autoware::behavior_velocity_planner
