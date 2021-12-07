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

#include "planning_error_monitor/planning_error_monitor_node.hpp"
#include "test_planning_error_monitor_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

constexpr double NOMINAL_INTERVAL = 1.0;
constexpr double ERROR_INTERVAL = 1000.0;
constexpr double ERROR_CURVATURE = 2.0;

using autoware_auto_planning_msgs::msg::Trajectory;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using planning_diagnostics::PlanningErrorMonitorNode;

class PubSubManager : public rclcpp::Node
{
public:
  PubSubManager() : Node("test_pub_sub")
  {
    traj_pub_ = create_publisher<Trajectory>("/planning_error_monitor/input/trajectory", 1);
    diag_sub_ = create_subscription<DiagnosticArray>(
      "/diagnostics", 1,
      [this](const DiagnosticArray::ConstSharedPtr msg) { received_diags_.push_back(msg); });
  }

  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
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
  auto error_monitor = std::make_shared<PlanningErrorMonitorNode>(rclcpp::NodeOptions{});
  auto manager = std::make_shared<PubSubManager>();
  EXPECT_GE(manager->traj_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager->traj_pub_->publish(trajectory);
  spinSome(error_monitor);
  spinSome(manager);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(isAllOK(manager->received_diags_));
}

void runWithBadTrajectory(const Trajectory & trajectory)
{
  auto error_monitor = std::make_shared<PlanningErrorMonitorNode>(rclcpp::NodeOptions{});
  auto manager = std::make_shared<PubSubManager>();
  EXPECT_GE(manager->traj_pub_->get_subscription_count(), 1U) << "topic is not connected.";

  manager->traj_pub_->publish(trajectory);
  spinSome(error_monitor);
  spinSome(manager);

  EXPECT_GE(manager->received_diags_.size(), 1U) << "diag has not received!";
  EXPECT_TRUE(hasError(manager->received_diags_));
}

// OK cases
TEST(PlanningErrorMonitor, DiagCheckForNominalTrajectory)
{
  runWithOKTrajectory(generateTrajectory(NOMINAL_INTERVAL));
}

// Bad cases
TEST(PlanningErrorMonitor, DiagCheckForNaNTrajectory)
{
  runWithBadTrajectory(generateNanTrajectory());
}
TEST(PlanningErrorMonitor, DiagCheckForInfTrajectory)
{
  runWithBadTrajectory(generateInfTrajectory());
}
TEST(PlanningErrorMonitor, DiagCheckForTooLongIntervalTrajectory)
{
  runWithBadTrajectory(generateTrajectory(ERROR_INTERVAL));
}
