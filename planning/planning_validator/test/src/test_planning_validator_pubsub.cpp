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

#include "planning_validator/planning_validator.hpp"
#include "test_planning_validator_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

/*
 * This test checks the diagnostics message published from the planning_validator node
 */

using autoware_auto_planning_msgs::msg::Trajectory;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using nav_msgs::msg::Odometry;
using planning_validator::PlanningValidator;

class PubSubManager : public rclcpp::Node
{
public:
  PubSubManager() : Node("test_pub_sub")
  {
    traj_pub_ = create_publisher<Trajectory>("/planning_validator/input/trajectory", 1);
    kinematics_pub_ = create_publisher<Odometry>("/planning_validator/input/kinematics", 1);
    diag_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", 1,
      [this](const DiagnosticArray::ConstSharedPtr msg) { received_diags_.push_back(msg); });
  }

  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr kinematics_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diag_sub_;

  std::vector<DiagnosticArray::ConstSharedPtr> received_diags_;
};

void spinSome(rclcpp::Node::SharedPtr node_ptr)
{
  for (int i = 0; i < 50; ++i) {
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

void runWithOKTrajectory(const Trajectory & trajectory)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());
  auto manager = std::make_shared<PubSubManager>();
  EXPECT_GE(manager->traj_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager->traj_pub_->publish(trajectory);
  manager->kinematics_pub_->publish(generateDefaultOdometry());
  spinSome(validator);
  spinSome(manager);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager->received_diags_));
}

void runWithBadTrajectory(const Trajectory & trajectory)
{
  auto validator = std::make_shared<PlanningValidator>(getNodeOptionsWithDefaultParams());
  auto manager = std::make_shared<PubSubManager>();
  EXPECT_GE(manager->traj_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager->traj_pub_->publish(trajectory);
  manager->kinematics_pub_->publish(generateDefaultOdometry());
  spinSome(validator);
  spinSome(manager);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(hasError(manager->received_diags_));
}

// OK cases
TEST(PlanningValidator, DiagCheckForNominalTrajectory)
{
  runWithOKTrajectory(generateTrajectory(NOMINAL_INTERVAL));
}

// Bad cases
TEST(PlanningValidator, DiagCheckForNaNTrajectory)
{
  runWithBadTrajectory(generateNanTrajectory());
}
TEST(PlanningValidator, DiagCheckForInfTrajectory)
{
  runWithBadTrajectory(generateInfTrajectory());
}
TEST(PlanningValidator, DiagCheckForTooLongIntervalTrajectory)
{
  constexpr double ep = 0.001;
  runWithBadTrajectory(generateTrajectory(ERROR_INTERVAL + ep));
}
