// Copyright 2019 Christopher Ho
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
#include "motion_testing/motion_testing.hpp"

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <time_utils/time_utils.hpp>

#include <limits>

namespace motion
{
namespace motion_testing
{
State make_state(
  Real x0,
  Real y0,
  Real heading,
  Real v0,
  Real a0,
  Real turn_rate,
  std::chrono::system_clock::time_point t)
{
  State start_state{rosidl_runtime_cpp::MessageInitialization::ALL};
  start_state.state.pose.position.x = x0;
  start_state.state.pose.position.y = y0;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, heading);
  start_state.state.pose.orientation = tf2::toMsg(quat);
  start_state.state.longitudinal_velocity_mps = v0;
  start_state.state.acceleration_mps2 = a0;
  start_state.state.heading_rate_rps = turn_rate;
  start_state.state.lateral_velocity_mps = 0.0F;

  start_state.header.stamp = time_utils::to_message(t);

  return start_state;
}

////////////////////////////////////////////////////////////////////////////////
State generate_state(Generator & gen)
{
  State ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  // Parameters with positive and negative supports
  std::normal_distribution<decltype(ret.state.pose.position.x)> normal{0.0, 1.0};
  std::normal_distribution<decltype(ret.state.heading_rate_rps)> normalF{0.0F, 1.0F};
  ret.state.pose.position.x = 10.0 * normal(gen);
  ret.state.pose.position.y = 10.0 * normal(gen);
  ret.state.lateral_velocity_mps = 0.5F * normalF(gen);
  ret.state.acceleration_mps2 = normalF(gen);
  ret.state.heading_rate_rps = 0.1F * normalF(gen);
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, normal(gen));
  ret.state.pose.orientation = tf2::toMsg(quat);
  // Parameters with only positive supports
  std::exponential_distribution<decltype(ret.state.longitudinal_velocity_mps)>
  exponential{1.0F};
  ret.state.longitudinal_velocity_mps = 5.0F * exponential(gen);

  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory generate_trajectory(const State & start_state, Generator & gen)
{
  Trajectory ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  (void)start_state;
  (void)gen;
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_trajectory(const State & start_state, const std::chrono::nanoseconds dt)
{
  Trajectory ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  const auto capacity = 100LL;  // TEMP
  ret.points.reserve(capacity);
  const auto dt_s = std::chrono::duration_cast<std::chrono::duration<Real>>(dt).count();
  ret.points.push_back(start_state.state);
  ret.points.back().time_from_start = time_utils::to_message(std::chrono::nanoseconds::zero());
  // quaternion increments
  tf2::Quaternion orientation_increment;
  orientation_increment.setRPY(0.0, 0.0, start_state.state.heading_rate_rps * dt_s);
  // fill out trajectory
  for (auto i = 1LL; i < capacity; ++i) {
    const auto & last_state = ret.points.back();
    decltype(ret.points)::value_type next_state{last_state};
    // longitudinal velocity update; lateral assumed fixed
    next_state.longitudinal_velocity_mps += dt_s * next_state.acceleration_mps2;
    // heading update
    tf2::Quaternion last_orientation;
    tf2::fromMsg(last_state.pose.orientation, last_orientation);
    next_state.pose.orientation = tf2::toMsg(last_orientation * orientation_increment);
    // pose.position update: simplified heading effects
    const auto ds =
      static_cast<double>(dt_s *
      (last_state.longitudinal_velocity_mps + (0.5F * dt_s * last_state.acceleration_mps2)));
    const auto yaw = tf2::getYaw(next_state.pose.orientation);
    next_state.pose.position.x += std::cos(yaw) * ds;
    next_state.pose.position.y += std::sin(yaw) * ds;

    next_state.time_from_start = time_utils::to_message(dt * i);

    ret.points.push_back(next_state);
  }
  ret.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory bad_heading_trajectory(const State & start_state, const std::chrono::nanoseconds dt)
{
  Trajectory ret{rosidl_runtime_cpp::MessageInitialization::ALL};
  const auto capacity = 100LL;  // TEMP
  ret.points.reserve(capacity);
  const auto dt_s = std::chrono::duration_cast<std::chrono::duration<Real>>(dt).count();
  ret.points.push_back(start_state.state);
  ret.points.back().pose.orientation.x = 0.0;
  ret.points.back().pose.orientation.y = 0.0;
  ret.points.back().pose.orientation.z = 0.0;
  ret.points.back().pose.orientation.w = 0.0;
  ret.points.back().heading_rate_rps = 0.0F;

  ret.points.back().time_from_start = time_utils::to_message(std::chrono::nanoseconds::zero());

  // fill out trajectory
  for (auto i = 1LL; i < capacity; ++i) {
    const auto & last_state = ret.points.back();
    decltype(ret.points)::value_type next_state{last_state};

    next_state.pose.position.x += static_cast<double>(dt_s * last_state.longitudinal_velocity_mps);
    next_state.pose.position.y += 0.0;
    next_state.time_from_start = time_utils::to_message(dt * i);

    ret.points.push_back(next_state);
  }
  ret.header.stamp = time_utils::to_message(std::chrono::system_clock::now());
  return ret;
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_velocity_trajectory(
  const Real x0,
  const Real y0,
  const Real heading,
  const Real v0,
  const std::chrono::nanoseconds dt)
{
  return constant_acceleration_turn_rate_trajectory(x0, y0, heading, v0, 0.0F, 0.0F, dt);
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_acceleration_trajectory(
  const Real x0,
  const Real y0,
  const Real heading,
  const Real v0,
  const Real a0,
  const std::chrono::nanoseconds dt)
{
  return constant_acceleration_turn_rate_trajectory(x0, y0, heading, v0, a0, 0.0F, dt);
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_velocity_turn_rate_trajectory(
  const Real x0,
  const Real y0,
  const Real heading,
  const Real v0,
  const Real turn_rate,
  const std::chrono::nanoseconds dt)
{
  return constant_acceleration_turn_rate_trajectory(x0, y0, heading, v0, 0.0F, turn_rate, dt);
}

////////////////////////////////////////////////////////////////////////////////
Trajectory constant_acceleration_turn_rate_trajectory(
  const Real x0,
  const Real y0,
  const Real heading,
  const Real v0,
  const Real a0,
  const Real turn_rate,
  const std::chrono::nanoseconds dt)
{
  State start_state =
    make_state(x0, y0, heading, v0, a0, turn_rate, std::chrono::system_clock::now());

  return constant_trajectory(start_state, dt);
}

////////////////////////////////////////////////////////////////////////////////
void next_state(
  const Trajectory & trajectory,
  State & state,
  const uint32_t hint,
  Generator * const gen)
{
  (void)trajectory;
  (void)state;
  (void)gen;
  (void)hint;
}

////////////////////////////////////////////////////////////////////////////////
Index progresses_towards_target(
  const Trajectory & trajectory,
  const Point & target,
  const Real heading_tolerance)
{
  auto last_err = std::numeric_limits<double>::max();
  auto last_heading_err = -std::numeric_limits<Real>::max();
  for (auto idx = Index{}; idx < trajectory.points.size(); ++idx) {
    const auto & pt = trajectory.points[idx];
    // Pose
    const auto dx = pt.pose.position.x - target.pose.position.x;
    const auto dy = pt.pose.position.y - target.pose.position.y;
    const auto err = (dx * dx) + (dy * dy);
    if (err > last_err) {
      return idx;
    }
    last_err = err;
    // Heading: dot product should tend towards 1
    tf2::Quaternion pt_orientation;
    tf2::Quaternion target_orientation;
    tf2::fromMsg(pt.pose.orientation, pt_orientation);
    tf2::fromMsg(target.pose.orientation, target_orientation);
    const auto dot = static_cast<Real>(tf2::dot(pt_orientation, target_orientation));
    if (dot < last_heading_err - heading_tolerance) {  // Allow for some error
      return idx;
    }
    last_heading_err = dot;
  }
  return trajectory.points.size();
}

////////////////////////////////////////////////////////////////////////////////
Index dynamically_feasible(const Trajectory & trajectory, const Real tolerance)
{
  if (trajectory.points.empty()) {
    return trajectory.points.size();
  }
  auto last_pt = trajectory.points.front();
  for (auto idx = Index{1}; idx < trajectory.points.size(); ++idx) {
    const auto & pt = trajectory.points[idx];
    const auto dt_ = time_utils::from_message(pt.time_from_start) -
      time_utils::from_message(last_pt.time_from_start);
    const auto dt = std::chrono::duration_cast<std::chrono::duration<Real>>(dt_).count();
    const auto dv = last_pt.acceleration_mps2 * dt;
    const auto ds =
      (Real{0.5} *dv * dt) + (last_pt.longitudinal_velocity_mps * dt);
    const auto dn = last_pt.lateral_velocity_mps * dt;
    const auto dth = last_pt.heading_rate_rps * dt;
    const auto check_fn = [tolerance](auto expect, auto val, auto str) -> bool {
        bool success = true;
        if (static_cast<Real>(std::fabs(expect)) < Real{1}) {
          success = static_cast<Real>(std::fabs(expect - val)) < tolerance;
        } else {
          success = static_cast<Real>(std::fabs(expect - val) / expect) < tolerance;
        }
        (void)str;
        return success;
      };
    bool ok = true;
    {
      const auto v = last_pt.longitudinal_velocity_mps + dv;
      ok = check_fn(v, pt.longitudinal_velocity_mps, "vel") && ok;
    }
    {
      tf2::Quaternion th;
      tf2::fromMsg(last_pt.pose.orientation, th);
      tf2::Quaternion new_th;
      tf2::fromMsg(pt.pose.orientation, new_th);
      {
        // Dot product between angles to "check" magnitude of rotation
        const auto dot = static_cast<Real>(tf2::dot(th, new_th));
        ok = check_fn(dot, std::cos(dth), "th_mag") && ok;
        // cross product between angles to check for consistent sign of change
        if (std::fabs(dth) > Real{0.001F}) {
          const auto angle = static_cast<Real>(th.angle(new_th));
          // Negative product -> not pointing in same direction
          ok = (dth * angle > Real{}) && ok;
        }
      }
      // Check changes either from current or next heading
      const tf2::Vector3 ds_dn(ds, dn, 0.0);
      const tf2::Transform tf(th);
      const tf2::Vector3 delta = tf * ds_dn;
      const tf2::Transform tf2(new_th);
      const tf2::Vector3 delta2 = tf2 * ds_dn;
      ok = (check_fn(last_pt.pose.position.x + delta.getX(), pt.pose.position.x, "x") ||
        check_fn(last_pt.pose.position.x + delta2.getX(), pt.pose.position.x, "x2")) &&
        ok;
      ok = (check_fn(last_pt.pose.position.y + delta.getY(), pt.pose.position.y, "y") ||
        check_fn(last_pt.pose.position.y + delta2.getY(), pt.pose.position.y, "y2")) &&
        ok;
    }
    if (!ok) {
      return idx;
    }
    last_pt = pt;
  }
  return trajectory.points.size();
}
}  // namespace motion_testing
}  // namespace motion
