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

#include <localization_evaluator/localization_evaluator_node.hpp>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include "boost/lexical_cast.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

using EvalNode = localization_diagnostics::LocalizationEvaluatorNode;
using diagnostic_msgs::msg::DiagnosticArray;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class EvalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    const auto share_dir = ament_index_cpp::get_package_share_directory("localization_evaluator");
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/param/localization_evaluator.defaults.yaml"});

    dummy_node = std::make_shared<rclcpp::Node>("localization_evaluator_test_node");
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

    odom_pub_ = rclcpp::create_publisher<Odometry>(
      dummy_node, "/localization_evaluator/input/localization", 1);
    pos_ref_pub_ = rclcpp::create_publisher<PoseWithCovarianceStamped>(
      dummy_node, "/localization_evaluator/input/localization/ref", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(dummy_node);
  }

  ~EvalTest() override { rclcpp::shutdown(); }

  void setTargetMetric(localization_diagnostics::Metric metric)
  {
    const auto metric_str = localization_diagnostics::metric_to_str.at(metric);
    const auto is_target_metric = [metric_str](const auto & status) {
      return status.name == metric_str;
    };
    metric_sub_ = rclcpp::create_subscription<DiagnosticArray>(
      dummy_node, "/localization_evaluator/metrics", 1,
      [=](const DiagnosticArray::ConstSharedPtr msg) {
        const auto it = std::find_if(msg->status.begin(), msg->status.end(), is_target_metric);
        if (it != msg->status.end()) {
          metric_value_ = boost::lexical_cast<double>(it->values[2].value);
          metric_updated_ = true;
        }
      });
  }

  Odometry makeOdometry(const double x, const double y, const double z)
  {
    Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.pose.pose.position.x = x;
    odometry.pose.pose.position.y = y;
    odometry.pose.pose.position.z = z;
    return odometry;
  }

  PoseWithCovarianceStamped makePos(const double x, const double y, const double z)
  {
    PoseWithCovarianceStamped pos;
    pos.header.frame_id = "map";
    pos.pose.pose.position.x = x;
    pos.pose.pose.position.y = y;
    pos.pose.pose.position.z = z;
    return pos;
  }

  double publishOdometryAndGetMetric(
    const Odometry & odom, const PoseWithCovarianceStamped & pos_ref)
  {
    metric_updated_ = false;
    odom_pub_->publish(odom);
    pos_ref_pub_->publish(pos_ref);
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
  // Publishers
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pos_ref_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr metric_sub_;
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(EvalTest, TestLateralErrorStats)
{
  setTargetMetric(localization_diagnostics::Metric::lateral_error);
  Odometry odom = makeOdometry(1.0, 1.0, 0.0);
  PoseWithCovarianceStamped pos_ref = makePos(4.0, 5.0, 0.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom, pos_ref), 3.0);
  Odometry odom2 = makeOdometry(1.0, 1.0, 0.0);
  PoseWithCovarianceStamped pos2_ref = makePos(1.0, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom2, pos2_ref), 1.5);
}

TEST_F(EvalTest, TestAbsoluteErrorStats)
{
  setTargetMetric(localization_diagnostics::Metric::absolute_error);
  Odometry odom = makeOdometry(1.0, 1.0, 0.0);
  PoseWithCovarianceStamped pos_ref = makePos(4.0, 5.0, 0.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom, pos_ref), 5.0);
  Odometry odom2 = makeOdometry(1.0, 1.0, 0.0);
  PoseWithCovarianceStamped pos2_ref = makePos(1.0, 1.0, 0.0);
  EXPECT_DOUBLE_EQ(publishOdometryAndGetMetric(odom2, pos2_ref), 2.5);
}
