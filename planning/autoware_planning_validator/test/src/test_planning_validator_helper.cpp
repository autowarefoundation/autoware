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

#include "test_planning_validator_helper.hpp"

#include "autoware/universe_utils/geometry/geometry.hpp"
#include "test_parameter.hpp"

#include <math.h>

using autoware::universe_utils::createQuaternionFromYaw;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

Trajectory generateTrajectoryWithConstantAcceleration(
  const double interval_distance, const double speed, const double yaw, const size_t size,
  const double acceleration)
{
  Trajectory trajectory;
  double s = 0.0, v = speed, a = acceleration;
  constexpr auto MAX_DT = 10.0;
  for (size_t i = 0; i < size; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = s * std::cos(yaw);
    p.pose.position.y = s * std::sin(yaw);
    p.pose.orientation = createQuaternionFromYaw(yaw);
    p.longitudinal_velocity_mps = v;
    p.acceleration_mps2 = a;
    p.front_wheel_angle_rad = 0.0;
    trajectory.points.push_back(p);
    s += interval_distance;

    const auto dt = std::abs(v) > 0.1 ? interval_distance / v : MAX_DT;
    v += acceleration * dt;
    if (v < 0.0) {
      v = 0.0;
      a = 0.0;
    }
  }
  return trajectory;
}

Trajectory generateTrajectory(
  const double interval_distance, const double speed, const double yaw, const size_t size)
{
  constexpr auto acceleration = 0.0;
  return generateTrajectoryWithConstantAcceleration(
    interval_distance, speed, yaw, size, acceleration);
}

Trajectory generateTrajectoryWithConstantCurvature(
  const double interval_distance, const double speed, const double curvature, const size_t size,
  const double wheelbase)
{
  if (std::abs(curvature) < 1.0e-5) {
    return generateTrajectory(interval_distance, speed, 0.0, size);
  }

  const auto steering = std::atan(curvature * wheelbase);
  const auto radius = 1.0 / curvature;

  Trajectory trajectory;
  double x = 0.0, y = 0.0, yaw = 0.0;

  for (size_t i = 0; i <= size; ++i) {
    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = createQuaternionFromYaw(yaw);
    p.longitudinal_velocity_mps = speed;
    p.front_wheel_angle_rad = steering;
    trajectory.points.push_back(p);

    // Update x, y, yaw for the next point
    const auto prev_yaw = yaw;
    double delta_yaw = curvature * interval_distance;
    yaw += delta_yaw;
    x += radius * (std::sin(yaw) - std::sin(prev_yaw));
    y -= radius * (std::cos(yaw) - std::cos(prev_yaw));
  }
  return trajectory;
}

Trajectory generateTrajectoryWithConstantSteering(
  const double interval_distance, const double speed, const double steering_angle_rad,
  const size_t size, const double wheelbase)
{
  const auto curvature = std::tan(steering_angle_rad) / wheelbase;
  return generateTrajectoryWithConstantCurvature(
    interval_distance, speed, curvature, size, wheelbase);
}

Trajectory generateTrajectoryWithConstantSteeringRate(
  const double interval_distance, const double speed, const double steering_rate, const size_t size,
  const double wheelbase)
{
  Trajectory trajectory;
  double x = 0.0, y = 0.0, yaw = 0.0, steering_angle_rad = 0.0;

  constexpr double MAX_STEERING_ANGLE_RAD = M_PI / 3.0;

  for (size_t i = 0; i <= size; ++i) {
    // Limit the steering angle to the maximum value
    steering_angle_rad =
      std::clamp(steering_angle_rad, -MAX_STEERING_ANGLE_RAD, MAX_STEERING_ANGLE_RAD);

    TrajectoryPoint p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation = createQuaternionFromYaw(yaw);
    p.longitudinal_velocity_mps = speed;
    p.front_wheel_angle_rad = steering_angle_rad;
    p.acceleration_mps2 = 0.0;

    trajectory.points.push_back(p);

    // Update x, y, yaw, and steering_angle for the next point
    const auto curvature = std::tan(steering_angle_rad) / wheelbase;
    double delta_yaw = curvature * interval_distance;
    yaw += delta_yaw;
    x += interval_distance * cos(yaw);
    y += interval_distance * sin(yaw);
    if (std::abs(speed) > 0.01) {
      steering_angle_rad += steering_rate * interval_distance / speed;
    } else {
      steering_angle_rad = steering_rate > 0.0 ? MAX_STEERING_ANGLE_RAD : -MAX_STEERING_ANGLE_RAD;
    }
  }

  return trajectory;
}

Trajectory generateNanTrajectory()
{
  Trajectory trajectory = generateTrajectory(1.0);
  trajectory.points.front().pose.position.x = NAN;
  return trajectory;
}

Trajectory generateInfTrajectory()
{
  Trajectory trajectory = generateTrajectory(1.0);
  trajectory.points.front().pose.position.x = INFINITY;
  return trajectory;
}

Trajectory generateBadCurvatureTrajectory()
{
  Trajectory trajectory;

  double y = 1.5;
  for (double s = 0.0; s <= 10.0; s += 1.0) {
    TrajectoryPoint p;
    p.longitudinal_velocity_mps = 1.0;
    p.pose.position.x = s;
    p.pose.position.y = y;
    y *= -1.0;  // invert sign
    trajectory.points.push_back(p);
  }

  return trajectory;
}

Odometry generateDefaultOdometry(const double x, const double y, const double vx)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.twist.twist.linear.x = vx;
  return odom;
}

rclcpp::NodeOptions getNodeOptionsWithDefaultParams()
{
  rclcpp::NodeOptions node_options;

  // for planning validator
  node_options.append_parameter_override("publish_diag", true);
  node_options.append_parameter_override("invalid_trajectory_handling_type", 0);
  node_options.append_parameter_override("diag_error_count_threshold", 0);
  node_options.append_parameter_override("display_on_terminal", true);
  node_options.append_parameter_override("thresholds.interval", THRESHOLD_INTERVAL);
  node_options.append_parameter_override("thresholds.relative_angle", THRESHOLD_RELATIVE_ANGLE);
  node_options.append_parameter_override("thresholds.curvature", THRESHOLD_CURVATURE);
  node_options.append_parameter_override("thresholds.lateral_acc", THRESHOLD_LATERAL_ACC);
  node_options.append_parameter_override(
    "thresholds.longitudinal_max_acc", THRESHOLD_LONGITUDINAL_MAX_ACC);
  node_options.append_parameter_override(
    "thresholds.longitudinal_min_acc", THRESHOLD_LONGITUDINAL_MIN_ACC);
  node_options.append_parameter_override("thresholds.steering", THRESHOLD_STEERING);
  node_options.append_parameter_override("thresholds.steering_rate", THRESHOLD_STEERING_RATE);
  node_options.append_parameter_override(
    "thresholds.velocity_deviation", THRESHOLD_VELOCITY_DEVIATION);
  node_options.append_parameter_override(
    "thresholds.distance_deviation", THRESHOLD_DISTANCE_DEVIATION);
  node_options.append_parameter_override(
    "thresholds.longitudinal_distance_deviation", THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION);
  node_options.append_parameter_override(
    "parameters.forward_trajectory_length_acceleration",
    PARAMETER_FORWARD_TRAJECTORY_LENGTH_ACCELERATION);
  node_options.append_parameter_override(
    "parameters.forward_trajectory_length_margin", PARAMETER_FORWARD_TRAJECTORY_LENGTH_MARGIN);

  // for vehicle info
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", WHEELBASE);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);

  return node_options;
}
