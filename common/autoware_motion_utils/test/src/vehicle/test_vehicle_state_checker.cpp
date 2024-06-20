// Copyright 2022 TIER IV, Inc.
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

#include "autoware/motion_utils/vehicle/vehicle_state_checker.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"
#include "test_vehicle_state_checker_helper.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>

constexpr double ODOMETRY_HISTORY_500_MS = 0.5;
constexpr double ODOMETRY_HISTORY_1000_MS = 1.0;
constexpr double STOP_DURATION_THRESHOLD_0_MS = 0.0;
constexpr double STOP_DURATION_THRESHOLD_200_MS = 0.2;
constexpr double STOP_DURATION_THRESHOLD_400_MS = 0.4;
constexpr double STOP_DURATION_THRESHOLD_600_MS = 0.6;
constexpr double STOP_DURATION_THRESHOLD_800_MS = 0.8;
constexpr double STOP_DURATION_THRESHOLD_1000_MS = 1.0;

using autoware::motion_utils::VehicleArrivalChecker;
using autoware::motion_utils::VehicleStopChecker;
using autoware::universe_utils::createPoint;
using autoware::universe_utils::createQuaternion;
using autoware::universe_utils::createTranslation;
using nav_msgs::msg::Odometry;

class CheckerNode : public rclcpp::Node
{
public:
  CheckerNode() : Node("test_checker_node")
  {
    vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);
    vehicle_arrival_checker_ = std::make_unique<VehicleArrivalChecker>(this);
  }

  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;
  std::unique_ptr<VehicleArrivalChecker> vehicle_arrival_checker_;
};

class PubManager : public rclcpp::Node
{
public:
  PubManager() : Node("test_pub_node")
  {
    pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state", 1);
    pub_traj_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", 1);
  }

  rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;

  void publishStoppedOdometry(const geometry_msgs::msg::Pose & pose, const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();

      const auto time_diff = now - start_time;
      if (publish_duration < time_diff.seconds()) {
        break;
      }

      Odometry odometry;
      odometry.header.stamp = now;
      odometry.pose.pose = pose;
      odometry.twist.twist.linear = createTranslation(0.0, 0.0, 0.0);
      odometry.twist.twist.angular = createTranslation(0.0, 0.0, 0.0);
      this->pub_odom_->publish(odometry);

      rclcpp::WallRate(10).sleep();
    }
  }

  void publishStoppedOdometry(const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();

      const auto time_diff = now - start_time;
      if (publish_duration < time_diff.seconds()) {
        break;
      }

      Odometry odometry;
      odometry.header.stamp = now;
      odometry.pose.pose.position = createPoint(0.0, 0.0, 0.0);
      odometry.pose.pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);
      odometry.twist.twist.linear = createTranslation(0.0, 0.0, 0.0);
      odometry.twist.twist.angular = createTranslation(0.0, 0.0, 0.0);
      this->pub_odom_->publish(odometry);

      rclcpp::WallRate(10).sleep();
    }
  }

  void publishStoppingOdometry(const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();

      const auto time_diff = now - start_time;
      if (publish_duration < time_diff.seconds()) {
        break;
      }

      Odometry odometry;
      odometry.header.stamp = now;
      odometry.pose.pose.position = createPoint(0.0, 0.0, 0.0);
      odometry.pose.pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);
      odometry.twist.twist.linear = time_diff.seconds() < publish_duration / 2.0
                                      ? createTranslation(1.0, 0.0, 0.0)
                                      : createTranslation(0.0, 0.0, 0.0);
      odometry.twist.twist.angular = createTranslation(0.0, 0.0, 0.0);
      this->pub_odom_->publish(odometry);

      rclcpp::WallRate(10).sleep();
    }
  }

  void publishOldOdometry(const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();

      const auto time_diff = now - start_time;
      if (publish_duration < time_diff.seconds()) {
        break;
      }

      Odometry odometry;
      odometry.header.stamp = now - rclcpp::Duration(15, 0);  // 15 seconds old data
      odometry.pose.pose.position = createPoint(0.0, 0.0, 0.0);
      odometry.pose.pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);
      odometry.twist.twist.linear = createTranslation(0.0, 0.0, 0.0);
      odometry.twist.twist.angular = createTranslation(0.0, 0.0, 0.0);
      this->pub_odom_->publish(odometry);

      rclcpp::WallRate(10).sleep();
    }
  }
};

TEST(vehicle_stop_checker, isVehicleStopped)
{
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishStoppedOdometry(ODOMETRY_HISTORY_500_MS);

    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped());
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishStoppedOdometry(ODOMETRY_HISTORY_1000_MS);

    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped());
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishStoppingOdometry(ODOMETRY_HISTORY_1000_MS);

    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped());
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  // check if the old data will be discarded
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishOldOdometry(ODOMETRY_HISTORY_500_MS);

    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped());
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(checker->vehicle_stop_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }
}

TEST(vehicle_arrival_checker, isVehicleStopped)
{
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishStoppedOdometry(ODOMETRY_HISTORY_500_MS);

    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStopped());
    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishStoppedOdometry(ODOMETRY_HISTORY_1000_MS);

    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStopped());
    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    manager->publishStoppingOdometry(ODOMETRY_HISTORY_1000_MS);

    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStopped());
    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStopped(STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }
}

TEST(vehicle_arrival_checker, isVehicleStoppedAtStopPoint)
{
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position = createPoint(10.0, 0.0, 0.0);
    odom_pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position = createPoint(10.0, 0.0, 0.0);
    goal_pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);

    manager->pub_traj_->publish(generateTrajectoryWithStopPoint(goal_pose));
    manager->publishStoppedOdometry(odom_pose, ODOMETRY_HISTORY_500_MS);

    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint());
    EXPECT_TRUE(
      checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_TRUE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position = createPoint(0.0, 0.0, 0.0);
    odom_pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position = createPoint(10.0, 0.0, 0.0);
    goal_pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);

    manager->pub_traj_->publish(generateTrajectoryWithStopPoint(goal_pose));
    manager->publishStoppedOdometry(odom_pose, ODOMETRY_HISTORY_500_MS);

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint());
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }

  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));

    geometry_msgs::msg::Pose odom_pose;
    odom_pose.position = createPoint(10.0, 0.0, 0.0);
    odom_pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position = createPoint(10.0, 0.0, 0.0);
    goal_pose.orientation = createQuaternion(0.0, 0.0, 0.0, 1.0);

    manager->pub_traj_->publish(generateTrajectoryWithoutStopPoint(goal_pose));
    manager->publishStoppedOdometry(odom_pose, ODOMETRY_HISTORY_500_MS);

    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint());
    EXPECT_FALSE(
      checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(STOP_DURATION_THRESHOLD_0_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_200_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_400_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_600_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_800_MS));
    EXPECT_FALSE(checker->vehicle_arrival_checker_->isVehicleStoppedAtStopPoint(
      STOP_DURATION_THRESHOLD_1000_MS));

    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }
}
