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
#ifndef MOTION_TESTING__MOTION_TESTING_HPP_
#define MOTION_TESTING__MOTION_TESTING_HPP_

#include <motion_testing/visibility_control.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>

#include <chrono>
#include <random>

namespace motion
{
namespace motion_testing
{
using Generator = std::mt19937;
using State = autoware_auto_vehicle_msgs::msg::VehicleKinematicState;
using Point = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using Index = decltype(Trajectory::points)::size_type;
using Real = decltype(Point::longitudinal_velocity_mps);
// TODO(c.ho) Make these more modular

/// \brief Makes a state, intended to make message generation more terse
MOTION_TESTING_PUBLIC State make_state(
  Real x0,
  Real y0,
  Real heading,
  Real v0,
  Real a0,
  Real turn_rate,
  std::chrono::system_clock::time_point t);

/// \brief Generates a state from a normal distribution with the following bounds:
///       TODO(c.ho)
MOTION_TESTING_PUBLIC State generate_state(Generator & gen);

/// \brief Generates a trajectory assuming the starting state, a bicycle model, and
/// additive noise applied to XXX
/// Note: not implemented
MOTION_TESTING_PUBLIC Trajectory generate_trajectory(const State & start_state, Generator & gen);

/// \brief Generate a trajectory given the start state, assuming the highest derivatives are held
///        constant.
/// Note: heading_rate behavior will be kind of off TODO(c.ho)
/// Note: lateral_velocity_mps will not be respected TODO(cho)
MOTION_TESTING_PUBLIC
Trajectory constant_trajectory(const State & start_state, std::chrono::nanoseconds dt);
/// \brief Generates a constant velocity trajectory

MOTION_TESTING_PUBLIC
Trajectory bad_heading_trajectory(const State & start_state, std::chrono::nanoseconds dt);
/// \brief Generates a constant velocity trajectory with invalid heading values

MOTION_TESTING_PUBLIC Trajectory constant_velocity_trajectory(
  float x0,
  float y0,
  float heading,
  float v0,
  std::chrono::nanoseconds dt);
/// \brief Generates a constant acceleration trajectory
MOTION_TESTING_PUBLIC Trajectory constant_acceleration_trajectory(
  float x0,
  float y0,
  float heading,
  float v0,
  float a0,
  std::chrono::nanoseconds dt);
/// \brief Generates a constant velocity and constant turn rate trajectory
MOTION_TESTING_PUBLIC Trajectory constant_velocity_turn_rate_trajectory(
  float x0,
  float y0,
  float heading,
  float v0,
  float turn_rate,
  std::chrono::nanoseconds dt);
/// \brief Generates a constant acceleration and constant turn rate trajectory
MOTION_TESTING_PUBLIC Trajectory constant_acceleration_turn_rate_trajectory(
  float x0,
  float y0,
  float heading,
  float v0,
  float a0,
  float turn_rate,
  std::chrono::nanoseconds dt);

/// Given a trajectory, advance state to next trajectory point, with normally distributed noise
/// Note: This version takes "hint" as gospel, and doesn't try to do any time/space matching
/// Note: not implemented
MOTION_TESTING_PUBLIC void next_state(
  const Trajectory & trajectory,
  State & state,
  uint32_t hint,
  Generator * gen = nullptr);  // TODO(c.ho) std::optional NOLINT
// TODO(c.ho) version that takes control commands

/// Checks that a trajectory makes constant progress towards a target; returns first
/// index of point that doesn't advance towards target, otherwise size of trajectory
/// heading tolerance is in dot product space of 2d quaternion
MOTION_TESTING_PUBLIC
Index progresses_towards_target(
  const Trajectory & trajectory,
  const Point & target,
  Real heading_tolerance = Real{0.006F});

/// Checks that a trajectory is more or less dynamically feasible given the derivatives;
/// tolerance is relative tolerance of trajectory, index is first point that is not dynamically
/// feasible, trajectory.size() if completely feasible
MOTION_TESTING_PUBLIC
Index dynamically_feasible(const Trajectory & trajectory, Real tolerance = 0.05F);
}  // namespace motion_testing
}  // namespace motion

#endif  // MOTION_TESTING__MOTION_TESTING_HPP_
