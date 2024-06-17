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

#include "autoware/planning_validator/planning_validator.hpp"
#include "test_parameter.hpp"
#include "test_planning_validator_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

/*
 * This test checks the diagnostics message published from the planning_validator node
 */

using autoware::planning_validator::PlanningValidator;
using autoware_planning_msgs::msg::Trajectory;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;

constexpr double epsilon = 0.001;
constexpr double scale_margin = 1.1;

class PubSubManager : public rclcpp::Node
{
public:
  PubSubManager() : Node("test_pub_sub")
  {
    trajectory_pub_ = create_publisher<Trajectory>("/planning_validator/input/trajectory", 1);
    kinematics_pub_ = create_publisher<Odometry>("/planning_validator/input/kinematics", 1);
    diag_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", 1,
      [this](const DiagnosticArray::ConstSharedPtr msg) { received_diags_.push_back(msg); });
  }

  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr kinematics_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diag_sub_;

  std::vector<DiagnosticArray::ConstSharedPtr> received_diags_;
};

void spinSome(rclcpp::Node::SharedPtr node_ptr)
{
  for (int i = 0; i < 10; ++i) {
    rclcpp::spin_some(node_ptr);
    rclcpp::WallRate(100).sleep();
  }
}

bool isAllOK(const std::vector<DiagnosticArray::ConstSharedPtr> & diags)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.level != DiagnosticStatus::OK) {
        return false;
      }
    }
  }
  return true;
}

bool hasError(const std::vector<DiagnosticArray::ConstSharedPtr> & diags)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.level == DiagnosticStatus::ERROR) {
        return true;
      }
    }
  }
  return false;
}

bool hasError(const std::vector<DiagnosticArray::ConstSharedPtr> & diags, const std::string & name)
{
  for (const auto & diag : diags) {
    for (const auto & status : diag->status) {
      if (status.name == name) {
        return status.level == DiagnosticStatus::ERROR;
      }
    }
  }
  throw std::runtime_error(name + " is not contained in the diagnostic message.");
}

std::pair<
  std::shared_ptr<autoware::planning_validator::PlanningValidator>, std::shared_ptr<PubSubManager>>
prepareTest(const Trajectory & trajectory, const Odometry & ego_odom)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());
  auto manager = std::make_shared<PubSubManager>();
  EXPECT_GE(manager->trajectory_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager->trajectory_pub_->publish(trajectory);
  manager->kinematics_pub_->publish(ego_odom);
  spinSome(validator);
  spinSome(manager);

  return {validator, manager};
}

void runWithOKTrajectory(const Trajectory & trajectory, const Odometry & ego_odom)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager->received_diags_));
}

void runWithOKTrajectory(
  const Trajectory & trajectory, const Odometry & ego_odom, const std::string & name)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_FALSE(hasError(manager->received_diags_, name));
}

void runWithBadTrajectory(const Trajectory & trajectory, const Odometry & ego_odom)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(hasError(manager->received_diags_));
}

void runWithBadTrajectory(
  const Trajectory & trajectory, const Odometry & ego_odom, const std::string & name)
{
  auto [validator, manager] = prepareTest(trajectory, ego_odom);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(hasError(manager->received_diags_, name));
}

// =============================================================
//                    Overall tests
// =============================================================

// OK cases
TEST(PlanningValidator, DiagCheckForNominalTrajectory)
{
  runWithOKTrajectory(generateTrajectory(THRESHOLD_INTERVAL * 0.5), generateDefaultOdometry());
}

// Bad cases
TEST(PlanningValidator, DiagCheckForNaNTrajectory)
{
  runWithBadTrajectory(generateNanTrajectory(), generateDefaultOdometry());
}
TEST(PlanningValidator, DiagCheckForInfTrajectory)
{
  runWithBadTrajectory(generateInfTrajectory(), generateDefaultOdometry());
}
TEST(PlanningValidator, DiagCheckForTooLongIntervalTrajectory)
{
  constexpr double ep = 0.001;
  runWithBadTrajectory(generateTrajectory(THRESHOLD_INTERVAL + ep), generateDefaultOdometry());
}

// =============================================================
//                    Specific diag tests
// =============================================================

TEST(PlanningValidator, DiagCheckSize)
{
  const auto diag_name = "planning_validator: trajectory_validation_size";
  const auto odom = generateDefaultOdometry();
  runWithBadTrajectory(generateTrajectory(1.0, 1.0, 0.0, 0), odom, diag_name);
  runWithBadTrajectory(generateTrajectory(1.0, 1.0, 0.0, 1), odom, diag_name);
  runWithOKTrajectory(generateTrajectory(1.0, 1.0, 0.0, 2), odom);
  runWithOKTrajectory(generateTrajectory(1.0, 1.0, 0.0, 3), odom);
}

TEST(PlanningValidator, DiagCheckInterval)
{
  const auto diag_name = "planning_validator: trajectory_validation_interval";
  const auto odom = generateDefaultOdometry();

  // Larger interval than threshold -> must be NG
  {
    auto trajectory = generateTrajectory(1.0, 1.0, 0.0, 10);
    auto tp = trajectory.points.back();
    tp.pose.position.x += THRESHOLD_INTERVAL + 0.001;
    trajectory.points.push_back(tp);
    runWithBadTrajectory(trajectory, odom, diag_name);
  }

  // Smaller interval than threshold -> must be OK
  {
    auto trajectory = generateTrajectory(1.0, 1.0, 0.0, 10);
    auto tp = trajectory.points.back();
    tp.pose.position.x += THRESHOLD_INTERVAL - 0.001;
    trajectory.points.push_back(tp);
    runWithOKTrajectory(trajectory, odom, diag_name);
  }
}

TEST(PlanningValidator, DiagCheckRelativeAngle)
{
  const auto diag_name = "planning_validator: trajectory_validation_relative_angle";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto odom = generateDefaultOdometry();

  // Larger Relative Angle than threshold -> must be NG
  {
    auto bad_trajectory = generateTrajectory(interval, 1.0, 0.0, 10);
    auto bad_tp = bad_trajectory.points.back();
    bad_tp.pose.position.x += interval * std::cos(THRESHOLD_RELATIVE_ANGLE + 0.01);
    bad_tp.pose.position.y += interval * std::sin(THRESHOLD_RELATIVE_ANGLE + 0.01);
    bad_trajectory.points.push_back(bad_tp);
    runWithBadTrajectory(bad_trajectory, generateDefaultOdometry(), diag_name);
  }

  // Smaller Relative Angle than threshold -> must be OK
  {
    auto ok_trajectory = generateTrajectory(interval, 1.0, 0.0, 10);
    auto ok_tp = ok_trajectory.points.back();
    ok_tp.pose.position.x += interval * std::cos(THRESHOLD_RELATIVE_ANGLE - 0.01);
    ok_tp.pose.position.y += interval * std::sin(THRESHOLD_RELATIVE_ANGLE - 0.01);
    ok_trajectory.points.push_back(ok_tp);
    runWithOKTrajectory(ok_trajectory, generateDefaultOdometry(), diag_name);
  }
}

TEST(PlanningValidator, DiagCheckCurvature)
{
  const auto diag_name = "planning_validator: trajectory_validation_curvature";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto odom = generateDefaultOdometry();

  // Large y at one point -> must be NG
  {
    auto bad_trajectory = generateTrajectory(interval, 1.0, 0.0, 10);
    bad_trajectory.points.at(5).pose.position.y += 5.0;
    runWithBadTrajectory(bad_trajectory, odom, diag_name);
  }

  // Higher curvature than threshold -> must be NG
  {
    constexpr double curvature = THRESHOLD_CURVATURE * scale_margin;
    auto bad_trajectory =
      generateTrajectoryWithConstantCurvature(interval, 1.0, curvature, 10, WHEELBASE);
    runWithBadTrajectory(bad_trajectory, odom, diag_name);
  }

  // Lower curvature than threshold -> must be OK
  {
    constexpr double curvature = THRESHOLD_CURVATURE / scale_margin;
    auto ok_trajectory =
      generateTrajectoryWithConstantCurvature(interval, 1.0, curvature, 10, WHEELBASE);
    runWithOKTrajectory(ok_trajectory, odom, diag_name);
  }
}

TEST(PlanningValidator, DiagCheckLateralAcceleration)
{
  const auto diag_name = "planning_validator: trajectory_validation_lateral_acceleration";
  constexpr double speed = 10.0;

  // Note: lateral_acceleration is speed^2 * curvature;

  // Higher lateral acc than threshold -> must be NG
  {
    constexpr double curvature = THRESHOLD_LATERAL_ACC / (speed * speed) * scale_margin;
    const auto bad_trajectory =
      generateTrajectoryWithConstantCurvature(1.0, speed, curvature, 10, WHEELBASE);
    runWithBadTrajectory(bad_trajectory, generateDefaultOdometry(), diag_name);
  }

  // Smaller lateral acc than threshold -> must be OK
  {
    constexpr double curvature = THRESHOLD_LATERAL_ACC / (speed * speed) / scale_margin;
    const auto ok_trajectory =
      generateTrajectoryWithConstantCurvature(1.0, speed, curvature, 10, WHEELBASE);
    runWithOKTrajectory(ok_trajectory, generateDefaultOdometry(), diag_name);
  }
}

TEST(PlanningValidator, DiagCheckLongitudinalMaxAcc)
{
  const auto diag_name = "planning_validator: trajectory_validation_acceleration";
  constexpr double speed = 1.0;

  // Larger acceleration than threshold -> must be NG
  {
    constexpr double acceleration = THRESHOLD_LONGITUDINAL_MAX_ACC + epsilon;
    auto bad_trajectory =
      generateTrajectoryWithConstantAcceleration(1.0, speed, 0.0, 20, acceleration);
    runWithBadTrajectory(bad_trajectory, generateDefaultOdometry(), diag_name);
  }

  // Smaller acceleration than threshold -> must be OK
  {
    constexpr double acceleration = THRESHOLD_LONGITUDINAL_MAX_ACC - epsilon;
    auto bad_trajectory =
      generateTrajectoryWithConstantAcceleration(1.0, speed, 0.0, 20, acceleration);
    runWithOKTrajectory(bad_trajectory, generateDefaultOdometry(), diag_name);
  }
}

TEST(PlanningValidator, DiagCheckLongitudinalMinAcc)
{
  const auto diag_name = "planning_validator: trajectory_validation_deceleration";
  constexpr double speed = 20.0;

  const auto test = [&](const auto acceleration, const bool expect_ok) {
    auto trajectory = generateTrajectoryWithConstantAcceleration(1.0, speed, 0.0, 10, acceleration);
    if (expect_ok) {
      runWithOKTrajectory(trajectory, generateDefaultOdometry(), diag_name);
    } else {
      runWithBadTrajectory(trajectory, generateDefaultOdometry(), diag_name);
    }
  };

  // Larger deceleration than threshold -> must be NG
  test(THRESHOLD_LONGITUDINAL_MIN_ACC - epsilon, false);

  // Larger deceleration than threshold -> must be OK
  test(THRESHOLD_LONGITUDINAL_MIN_ACC + epsilon, true);
}

TEST(PlanningValidator, DiagCheckSteering)
{
  const auto diag_name = "planning_validator: trajectory_validation_steering";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto test = [&](const auto steering, const bool expect_ok) {
    auto trajectory =
      generateTrajectoryWithConstantSteering(interval, 1.0, steering, 10, WHEELBASE);
    const auto odom = generateDefaultOdometry();
    expect_ok ? runWithOKTrajectory(trajectory, odom, diag_name)
              : runWithBadTrajectory(trajectory, odom, diag_name);
  };

  // Larger steering than threshold -> must be NG
  test(THRESHOLD_STEERING * scale_margin, false);
  test(-THRESHOLD_STEERING * scale_margin, false);

  // Smaller steering than threshold -> must be OK
  test(THRESHOLD_STEERING / scale_margin, true);
  test(-THRESHOLD_STEERING / scale_margin, true);
}

TEST(PlanningValidator, DiagCheckSteeringRate)
{
  const auto diag_name = "planning_validator: trajectory_validation_steering_rate";

  // TODO(Horibe): interval must be larger than min_interval used in planning_validator.cpp
  constexpr auto interval = 1.1;

  const auto test = [&](const auto steering_rate, const bool expect_ok) {
    auto trajectory =
      generateTrajectoryWithConstantSteeringRate(interval, 1.0, steering_rate, 10, WHEELBASE);
    const auto odom = generateDefaultOdometry();
    expect_ok ? runWithOKTrajectory(trajectory, odom, diag_name)
              : runWithBadTrajectory(trajectory, odom, diag_name);
  };

  // Larger steering rate than threshold -> must be NG
  test(THRESHOLD_STEERING_RATE * scale_margin, false);
  test(-THRESHOLD_STEERING_RATE * scale_margin, false);

  // Smaller steering rate than threshold -> must be OK
  test(THRESHOLD_STEERING_RATE / scale_margin, true);
  test(-THRESHOLD_STEERING_RATE / scale_margin, true);
}

TEST(PlanningValidator, DiagCheckVelocityDeviation)
{
  const auto diag_name = "planning_validator: trajectory_validation_velocity_deviation";
  const auto test = [&](const auto trajectory_speed, const auto ego_speed, const bool expect_ok) {
    const auto trajectory = generateTrajectory(1.0, trajectory_speed, 0.0, 10);
    const auto ego_odom = generateDefaultOdometry(0.0, 0.0, ego_speed);
    expect_ok ? runWithOKTrajectory(trajectory, ego_odom, diag_name)
              : runWithBadTrajectory(trajectory, ego_odom, diag_name);
  };

  // Larger velocity deviation than threshold -> must be NG
  test(1.0 + THRESHOLD_VELOCITY_DEVIATION * scale_margin, 1.0, false);
  test(1.0, 1.0 + THRESHOLD_VELOCITY_DEVIATION * scale_margin, false);

  // Larger velocity deviation than threshold -> must be OK
  test(1.0, 1.0 + THRESHOLD_VELOCITY_DEVIATION / scale_margin, true);
}

TEST(PlanningValidator, DiagCheckDistanceDeviation)
{
  const auto diag_name = "planning_validator: trajectory_validation_distance_deviation";
  const auto test = [&](const auto ego_x, const auto ego_y, const bool expect_ok) {
    const auto trajectory = generateTrajectory(1.0, 3.0, 0.0, 10);
    const auto last_p = trajectory.points.back().pose.position;
    const auto ego_odom = generateDefaultOdometry(last_p.x + ego_x, last_p.y + ego_y, 0.0);
    expect_ok ? runWithOKTrajectory(trajectory, ego_odom, diag_name)
              : runWithBadTrajectory(trajectory, ego_odom, diag_name);
  };

  // Larger distance deviation than threshold -> must be NG
  const auto error_distance = THRESHOLD_DISTANCE_DEVIATION * scale_margin;
  test(error_distance, 0.0, false);
  test(0.0, error_distance, false);
  test(0.0, -error_distance, false);
  test(error_distance, error_distance, false);
  test(error_distance, -error_distance, false);

  // Smaller distance deviation than threshold -> must be OK
  const auto ok_distance = THRESHOLD_DISTANCE_DEVIATION / scale_margin;
  test(ok_distance, 0.0, true);
  test(0.0, ok_distance, true);
  test(0.0, -ok_distance, true);
}

TEST(PlanningValidator, DiagCheckLongitudinalDistanceDeviation)
{
  const auto diag_name =
    "planning_validator: trajectory_validation_longitudinal_distance_deviation";
  const auto trajectory = generateTrajectory(1.0, 3.0, 0.0, 10);
  const auto test = [&](const auto ego_x, const auto ego_y, const bool expect_ok) {
    const auto ego_odom = generateDefaultOdometry(ego_x, ego_y, 0.0);
    expect_ok ? runWithOKTrajectory(trajectory, ego_odom, diag_name)
              : runWithBadTrajectory(trajectory, ego_odom, diag_name);
  };

  const auto invalid_distance = THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION * scale_margin;
  const auto valid_distance = THRESHOLD_LONGITUDINAL_DISTANCE_DEVIATION / scale_margin;

  // behind from idx=0 -> must be NG
  test(-invalid_distance, 0.0, false);

  // ahead from idx=last -> must be NG
  test(trajectory.points.back().pose.position.x + invalid_distance, 0.0, false);

  // In between trajectory points -> must be OK
  const auto mid_point = trajectory.points.at(5).pose.position;
  test(mid_point.x, mid_point.y, true);

  // behind from idx=0, but close to 0 -> must be OK
  test(-valid_distance, 0.0, true);

  // ahead from idx=last, but close to last -> must be OK
  test(trajectory.points.back().pose.position.x + valid_distance, 0.0, true);
}

TEST(PlanningValidator, DiagCheckForwardTrajectoryLength)
{
  const auto diag_name = "planning_validator: trajectory_validation_forward_trajectory_length";
  constexpr auto trajectory_v = 10.0;
  constexpr size_t trajectory_size = 10;
  constexpr auto ego_v = 10.0;
  constexpr auto ego_a = std::abs(PARAMETER_FORWARD_TRAJECTORY_LENGTH_ACCELERATION);
  constexpr auto margin = PARAMETER_FORWARD_TRAJECTORY_LENGTH_MARGIN;

  // Longer trajectory than threshold -> must be OK
  {
    constexpr auto ok_trajectory_length = ego_v * ego_v / (2.0 * ego_a);  // v1^2 - v0^2 = 2as
    std::cerr << "aaa: " << ok_trajectory_length << std::endl;
    constexpr auto trajectory_interval = ok_trajectory_length / trajectory_size;
    const auto ok_trajectory =
      generateTrajectory(trajectory_interval, trajectory_v, 0.0, trajectory_size);
    const auto ego_odom = generateDefaultOdometry(0.0, 0.0, ego_v);
    runWithOKTrajectory(ok_trajectory, ego_odom, diag_name);
  }

  // Smaller trajectory than threshold -> must be NG
  {
    // shorter with 1.0m
    constexpr auto bad_trajectory_length = ego_v * ego_v / (2.0 * ego_a) - margin - 1.0;
    std::cerr << "bbb: " << bad_trajectory_length << std::endl;
    constexpr auto trajectory_interval = bad_trajectory_length / trajectory_size;
    const auto bad_trajectory =
      generateTrajectory(trajectory_interval, trajectory_v, 0.0, trajectory_size);
    const auto ego_odom = generateDefaultOdometry(0.0, 0.0, ego_v);
    runWithBadTrajectory(bad_trajectory, ego_odom, diag_name);
  }

  // Longer trajectory than threshold, but smaller length from ego -> must be NG
  {
    constexpr auto bad_trajectory_length = ego_v * ego_v / (2.0 * ego_a);  // v1^2 - v0^2 = 2as
    std::cerr << "ccc: " << bad_trajectory_length << std::endl;
    constexpr auto trajectory_interval = bad_trajectory_length / trajectory_size;
    const auto bad_trajectory =
      generateTrajectory(trajectory_interval, trajectory_v, 0.0, trajectory_size);
    const auto & p = bad_trajectory.points.at(trajectory_size - 1).pose.position;
    const auto ego_odom = generateDefaultOdometry(p.x, p.y, ego_v);
    runWithBadTrajectory(bad_trajectory, ego_odom, diag_name);
  }
}
