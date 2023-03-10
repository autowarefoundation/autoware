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

#include <planning_evaluator/planning_evaluator_node.hpp>

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "boost/lexical_cast.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

using EvalNode = planning_diagnostics::PlanningEvaluatorNode;
using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;
using Objects = autoware_auto_perception_msgs::msg::PredictedObjects;
using diagnostic_msgs::msg::DiagnosticArray;

class EvalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    const auto share_dir = ament_index_cpp::get_package_share_directory("planning_evaluator");
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/param/planning_evaluator.defaults.yaml"});

    dummy_node = std::make_shared<rclcpp::Node>("planning_evaluator_test_node");
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

    traj_pub_ =
      rclcpp::create_publisher<Trajectory>(dummy_node, "/planning_evaluator/input/trajectory", 1);
    ref_traj_pub_ = rclcpp::create_publisher<Trajectory>(
      dummy_node, "/planning_evaluator/input/reference_trajectory", 1);
    objects_pub_ =
      rclcpp::create_publisher<Objects>(dummy_node, "/planning_evaluator/input/objects", 1);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(dummy_node);
    publishEgoPose(0.0, 0.0, 0.0);
  }

  ~EvalTest() override { rclcpp::shutdown(); }

  void setTargetMetric(planning_diagnostics::Metric metric)
  {
    const auto metric_str = planning_diagnostics::metric_to_str.at(metric);
    const auto is_target_metric = [metric_str](const auto & status) {
      return status.name == metric_str;
    };
    metric_sub_ = rclcpp::create_subscription<DiagnosticArray>(
      dummy_node, "/planning_evaluator/metrics", 1, [=](const DiagnosticArray::ConstSharedPtr msg) {
        const auto it = std::find_if(msg->status.begin(), msg->status.end(), is_target_metric);
        if (it != msg->status.end()) {
          metric_value_ = boost::lexical_cast<double>(it->values[2].value);
          metric_updated_ = true;
        }
      });
  }

  Trajectory makeTrajectory(const std::vector<std::pair<double, double>> & traj)
  {
    Trajectory t;
    t.header.frame_id = "map";
    TrajectoryPoint p;
    for (const std::pair<double, double> & point : traj) {
      p.pose.position.x = point.first;
      p.pose.position.y = point.second;
      t.points.push_back(p);
    }
    return t;
  }

  void publishTrajectory(const Trajectory & traj)
  {
    traj_pub_->publish(traj);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  void publishReferenceTrajectory(const Trajectory & traj)
  {
    ref_traj_pub_->publish(traj);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  void publishObjects(const Objects & obj)
  {
    objects_pub_->publish(obj);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  double publishTrajectoryAndGetMetric(const Trajectory & traj)
  {
    metric_updated_ = false;
    traj_pub_->publish(traj);
    while (!metric_updated_) {
      rclcpp::spin_some(eval_node);
      rclcpp::spin_some(dummy_node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return metric_value_;
  }

  void publishEgoPose(const double x, const double y, const double yaw)
  {
    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = dummy_node->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  // Latest metric value
  bool metric_updated_ = false;
  double metric_value_;
  // Node
  rclcpp::Node::SharedPtr dummy_node;
  EvalNode::SharedPtr eval_node;
  // Trajectory publishers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr ref_traj_pub_;
  rclcpp::Publisher<Objects>::SharedPtr objects_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr metric_sub_;
  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

TEST_F(EvalTest, TestCurvature)
{
  setTargetMetric(planning_diagnostics::Metric::curvature);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -1.0);
  t = makeTrajectory({{0.0, 0.0}, {2.0, -2.0}, {4.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.5);
}

TEST_F(EvalTest, TestPointInterval)
{
  setTargetMetric(planning_diagnostics::Metric::point_interval);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
  // double the average interval
  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 6.0;
  t.points.push_back(p);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 2.0);
}

TEST_F(EvalTest, TestRelativeAngle)
{
  setTargetMetric(planning_diagnostics::Metric::relative_angle);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -M_PI_4);
  // add an angle of PI/4 to bring the average to 0
  TrajectoryPoint p;
  p.pose.position.x = 1.0;
  p.pose.position.y = 2.0;
  t.points.push_back(p);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
}

TEST_F(EvalTest, TestLength)
{
  setTargetMetric(planning_diagnostics::Metric::length);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 3.0);
  TrajectoryPoint p;
  p.pose.position.x = 3.0;
  p.pose.position.y = 3.0;
  t.points.push_back(p);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 6.0);
}

TEST_F(EvalTest, TestVelocity)
{
  setTargetMetric(planning_diagnostics::Metric::velocity);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}});
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
}

TEST_F(EvalTest, TestDuration)
{
  setTargetMetric(planning_diagnostics::Metric::duration);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}});
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 3.0);
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 3.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
}

TEST_F(EvalTest, TestAcceleration)
{
  setTargetMetric(planning_diagnostics::Metric::acceleration);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}});
  t.points[0].acceleration_mps2 = 1.0;
  t.points[1].acceleration_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
  t.points[0].acceleration_mps2 = -1.0;
  t.points[1].acceleration_mps2 = -1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -1.0);
  t.points[0].acceleration_mps2 = 0.0;
  t.points[1].acceleration_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.5);
}

TEST_F(EvalTest, TestJerk)
{
  setTargetMetric(planning_diagnostics::Metric::jerk);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}});
  t.points[0].longitudinal_velocity_mps = 1.0;
  t.points[0].acceleration_mps2 = 1.0;
  t.points[1].longitudinal_velocity_mps = 2.0;
  t.points[1].acceleration_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  t.points[0].longitudinal_velocity_mps = 1.0;
  t.points[0].acceleration_mps2 = 1.0;
  t.points[1].longitudinal_velocity_mps = 1.0;
  t.points[1].acceleration_mps2 = 0.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -1.0);
}

TEST_F(EvalTest, TestLateralDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::lateral_deviation);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}});
  publishReferenceTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  Trajectory t2 = makeTrajectory({{0.0, 1.0}, {1.0, 1.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0);
}

TEST_F(EvalTest, TestYawDeviation)
{
  auto setYaw = [](geometry_msgs::msg::Quaternion & msg, const double yaw_rad) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
  };
  setTargetMetric(planning_diagnostics::Metric::yaw_deviation);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}});
  for (auto & p : t.points) {
    setYaw(p.pose.orientation, M_PI);
  }
  publishReferenceTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  Trajectory t2 = t;
  for (auto & p : t2.points) {
    setYaw(p.pose.orientation, 0.0);
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), -M_PI);
  for (auto & p : t2.points) {
    setYaw(p.pose.orientation, -M_PI);
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 0.0);
}

TEST_F(EvalTest, TestVelocityDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::velocity_deviation);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}});
  for (auto & p : t.points) {
    p.longitudinal_velocity_mps = 0.0;
  }
  publishReferenceTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  for (auto & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
}

TEST_F(EvalTest, TestStability)
{
  setTargetMetric(planning_diagnostics::Metric::stability);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  t.points.back().pose.position.x = 0.0;
  t.points.back().pose.position.y = 0.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);

  Trajectory t2 = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t2);
  t2.points.back().pose.position.x = 4.0;
  t2.points.back().pose.position.y = 3.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0 / 4);
}

TEST_F(EvalTest, TestFrechet)
{
  setTargetMetric(planning_diagnostics::Metric::stability_frechet);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);

  // variation in the last point: simple distance from previous last point
  t.points.back().pose.position.x = 0.0;
  t.points.back().pose.position.y = 0.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), std::sqrt(18.0));
  Trajectory t2 = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t2);
  t2.points.back().pose.position.x = 4.0;
  t2.points.back().pose.position.y = 3.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0);

  // variations in the middle points: cannot go back to previous points that minimize the distance
  t2.points[2].pose.position.x = 0.5;
  t2.points[2].pose.position.y = 0.5;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), std::sqrt(2 * (1.5 * 1.5)));
}

TEST_F(EvalTest, TestObstacleDistance)
{
  setTargetMetric(planning_diagnostics::Metric::obstacle_distance);
  Objects objs;
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  objs.objects.push_back(obj);
  publishObjects(objs);

  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.5);
  Trajectory t2 = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0);  // (0.0 + 1.0 + 2.0) / 3
}

TEST_F(EvalTest, TestObstacleTTC)
{
  setTargetMetric(planning_diagnostics::Metric::obstacle_ttc);
  Objects objs;
  autoware_auto_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  objs.objects.push_back(obj);
  publishObjects(objs);

  Trajectory t = makeTrajectory({{3.0, 0.0}, {0.0, 0.0}, {-1.0, 0.0}});
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 3.0);
  // if no exact collision point, last point before collision is used
  t.points[1].pose.position.x = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 2.0);
}
