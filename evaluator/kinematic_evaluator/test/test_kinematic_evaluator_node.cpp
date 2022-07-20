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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <kinematic_evaluator/kinematic_evaluator_node.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include "boost/lexical_cast.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

using EvalNode = kinematic_diagnostics::KinematicEvaluatorNode;
using diagnostic_msgs::msg::DiagnosticArray;
using nav_msgs::msg::Odometry;

class EvalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    const auto share_dir = ament_index_cpp::get_package_share_directory("kinematic_evaluator");
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/param/kinematic_evaluator.defaults.yaml"});

    dummy_node = std::make_shared<rclcpp::Node>("kinematic_evaluator_test_node");
    eval_node = std::make_shared<EvalNode>(options);
    // Enable all logging in the node
    auto ret = rcutils_logging_set_logger_level(
      dummy_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      std::cerr << "Failed to set logging severity to DEBUG\n";
    }
    ret = rcutils_logging_set_logger_level(
      eval_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      std::cerr << "Failed to set logging severity to DEBUG\n";
    }

    odom_pub_ =
      rclcpp::create_publisher<Odometry>(dummy_node, "/kinematic_evaluator/input/twist", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(dummy_node);
  }

  ~EvalTest() override
  { /*rclcpp::shutdown();*/
  }

  void setTargetMetric(kinematic_diagnostics::Metric metric)
  {
    const auto metric_str = kinematic_diagnostics::metric_to_str.at(metric);
    const auto is_target_metric = [metric_str](const auto & status) {
      return status.name == metric_str;
    };
    metric_sub_ = rclcpp::create_subscription<DiagnosticArray>(
      dummy_node, "/kinematic_evaluator/metrics", 1,
      [=](const DiagnosticArray::ConstSharedPtr msg) {
        const auto it = std::find_if(msg->status.begin(), msg->status.end(), is_target_metric);
        if (it != msg->status.end()) {
          metric_value_ = boost::lexical_cast<double>(it->values[2].value);
          metric_updated_ = true;
        }
      });
  }

  Odometry makeOdometry(const double speed)
  {
    Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.twist.twist.linear.x = speed;
    return odometry;
  }

  void publishOdometry(const Odometry & odom)
  {
    odom_pub_->publish(odom);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  double publishOdometryAndGetMetric(const Odometry & odom)
  {
    metric_updated_ = false;
    odom_pub_->publish(odom);
    while (!metric_updated_) {
      rclcpp::spin_some(eval_node);
      rclcpp::spin_some(dummy_node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return metric_value_;
  }

  // Latest metric value
  bool metric_updated_ = false;
  double metric_value_;
  // Node
  rclcpp::Node::SharedPtr dummy_node;
  EvalNode::SharedPtr eval_node;
  // Trajectory publishers
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr metric_sub_;
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(EvalTest, TestVelocityStats)
{
  setTargetMetric(kinematic_diagnostics::Metric::velocity_stats);
  Odometry odom = makeOdometry(0.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom), 0.0);
  Odometry odom2 = makeOdometry(1.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom2), 0.5);
  Odometry odom3 = makeOdometry(2.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom3), 1.0);
}
