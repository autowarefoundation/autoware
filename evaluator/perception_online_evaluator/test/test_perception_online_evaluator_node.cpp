// Copyright 2024 TIER IV, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <perception_online_evaluator/perception_online_evaluator_node.hpp>
#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "boost/lexical_cast.hpp"

#include <glog/logging.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

using EvalNode = perception_diagnostics::PerceptionOnlineEvaluatorNode;
using PredictedObjects = autoware_auto_perception_msgs::msg::PredictedObjects;
using PredictedObject = autoware_auto_perception_msgs::msg::PredictedObject;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using ObjectClassification = autoware_auto_perception_msgs::msg::ObjectClassification;

using tier4_autoware_utils::generateUUID;

constexpr double epsilon = 1e-6;

class EvalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("perception_online_evaluator");
    options.arguments(
      {"--ros-args", "--params-file",
       share_dir + "/param/perception_online_evaluator.defaults.yaml"});
    options.append_parameter_override("prediction_time_horizons", std::vector<double>{5.0});
    options.append_parameter_override("smoothing_window_size", 11);

    dummy_node = std::make_shared<rclcpp::Node>("perception_online_evaluator_test", options);
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
    objects_pub_ = rclcpp::create_publisher<PredictedObjects>(
      dummy_node, "/perception_online_evaluator/input/objects", 1);

    marker_sub_ = rclcpp::create_subscription<MarkerArray>(
      eval_node, "perception_online_evaluator/markers", 10,
      [this]([[maybe_unused]] const MarkerArray::ConstSharedPtr msg) {
        has_received_marker_ = true;
      });
    uuid_ = generateUUID();
  }

  ~EvalTest() override
  {
    rclcpp::shutdown();
    google::ShutdownGoogleLogging();
  }

  void setTargetMetric(perception_diagnostics::Metric metric)
  {
    const auto metric_str = perception_diagnostics::metric_to_str.at(metric);
    setTargetMetric(metric_str);
  }

  void setTargetMetric(std::string metric_str)
  {
    const auto is_target_metric = [metric_str](const auto & status) {
      return status.name == metric_str;
    };
    metric_sub_ = rclcpp::create_subscription<DiagnosticArray>(
      eval_node, "/perception_online_evaluator/metrics", 1,
      [=](const DiagnosticArray::ConstSharedPtr msg) {
        const auto it = std::find_if(msg->status.begin(), msg->status.end(), is_target_metric);
        if (it != msg->status.end()) {
          metric_value_ = boost::lexical_cast<double>(it->values[2].value);
          metric_updated_ = true;
        }
      });
  }

  PredictedObject makePredictedObject(
    const std::vector<std::pair<double, double>> & predicted_path,
    const uint8_t label = ObjectClassification::CAR, const double velocity = 2.0)
  {
    PredictedObject object;
    object.object_id = uuid_;
    ObjectClassification classification;
    classification.label = label;
    classification.probability = 1.0;

    object.classification = {classification};

    object.kinematics.initial_pose_with_covariance.pose.position.x = predicted_path.front().first;
    object.kinematics.initial_pose_with_covariance.pose.position.y = predicted_path.front().second;
    object.kinematics.initial_pose_with_covariance.pose.position.z = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.x = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.y = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.z = 0.0;
    object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;

    object.kinematics.initial_twist_with_covariance.twist.linear.x = velocity;
    object.kinematics.initial_twist_with_covariance.twist.linear.y = 0.0;
    object.kinematics.initial_twist_with_covariance.twist.linear.z = 0.0;

    autoware_auto_perception_msgs::msg::PredictedPath path;
    for (size_t i = 0; i < predicted_path.size(); ++i) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = predicted_path[i].first;
      pose.position.y = predicted_path[i].second;
      pose.position.z = 0.0;
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = 0.0;
      pose.orientation.w = 1.0;
      path.path.push_back(pose);
    }

    path.confidence = 1.0;
    path.time_step = rclcpp::Duration::from_seconds(time_step_);
    object.kinematics.predicted_paths.push_back(path);

    return object;
  }

  PredictedObjects makePredictedObjects(
    const std::vector<std::pair<double, double>> & predicted_path,
    const uint8_t label = ObjectClassification::CAR, const double velocity = 2.0)
  {
    PredictedObjects objects;
    objects.objects.push_back(makePredictedObject(predicted_path, label, velocity));
    objects.header.stamp = rclcpp::Time(0);
    return objects;
  }

  PredictedObjects makeStraightPredictedObjects(
    const double time, const uint8_t label = ObjectClassification::CAR, const double velocity = 2.0)
  {
    std::vector<std::pair<double, double>> predicted_path;
    for (size_t i = 0; i <= time_horizon_ / time_step_; i++) {
      predicted_path.push_back({velocity * (time + i * time_step_), 0.0});
    }
    auto objects = makePredictedObjects(predicted_path, label, velocity);
    objects.header.stamp = rclcpp::Time(0) + rclcpp::Duration::from_seconds(time);
    return objects;
  }

  PredictedObjects makeDeviatedStraightPredictedObjects(
    const double time, const double deviation, const uint8_t label = ObjectClassification::CAR,
    const double velocity = 2.0)
  {
    std::vector<std::pair<double, double>> predicted_path;
    for (size_t i = 0; i <= time_horizon_ / time_step_; i++) {
      predicted_path.push_back({velocity * (time + i * time_step_), deviation});
    }
    auto objects = makePredictedObjects(predicted_path, label, velocity);
    objects.header.stamp = rclcpp::Time(0) + rclcpp::Duration::from_seconds(time);
    return objects;
  }

  PredictedObjects rotateObjects(const PredictedObjects objects, const double yaw)
  {
    PredictedObjects rotated_objects = objects;
    for (auto & object : rotated_objects.objects) {
      object.kinematics.initial_pose_with_covariance.pose.orientation.z = sin(yaw / 2);
      object.kinematics.initial_pose_with_covariance.pose.orientation.w = cos(yaw / 2);
    }
    return rotated_objects;
  }

  double publishObjectsAndGetMetric(const PredictedObjects & objects)
  {
    metric_updated_ = false;
    objects_pub_->publish(objects);
    const auto now = rclcpp::Clock().now();
    while (!metric_updated_) {
      rclcpp::spin_some(dummy_node);
      rclcpp::spin_some(eval_node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      // timeout
      if (rclcpp::Clock().now() - now > rclcpp::Duration::from_seconds(5)) {
        throw std::runtime_error("Timeout while waiting for metric update");
      }
    }
    return metric_value_;
  }

  void publishObjects(const PredictedObjects & objects)
  {
    objects_pub_->publish(objects);
    rclcpp::spin_some(eval_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(dummy_node);
  }

  void waitForDummyNode()
  {
    // wait for the marker to be published
    publishObjects(makeStraightPredictedObjects(0));
    while (!has_received_marker_) {
      rclcpp::spin_some(dummy_node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      rclcpp::spin_some(eval_node);
    }
  }

  // Latest metric value
  bool metric_updated_ = false;
  double metric_value_;
  // Node
  rclcpp::Node::SharedPtr dummy_node;
  EvalNode::SharedPtr eval_node;

  // Pub/Sub
  rclcpp::Publisher<PredictedObjects>::SharedPtr objects_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr metric_sub_;
  rclcpp::Subscription<MarkerArray>::SharedPtr marker_sub_;
  bool has_received_marker_{false};

  double time_delay_ = 5.0;
  double time_step_ = 0.5;
  double time_horizon_ = 10.0;

  unique_identifier_msgs::msg::UUID uuid_;
};

// ==========================================================================================
// lateral deviation
TEST_F(EvalTest, testLateralDeviation_deviation0)
{
  waitForDummyNode();
  setTargetMetric("lateral_deviation_CAR");

  const double deviation = 0.0;
  for (double time = 0; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testLateralDeviation_deviation1)
{
  waitForDummyNode();
  setTargetMetric("lateral_deviation_CAR");

  const double deviation = 1.0;
  for (double time = 0; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testLateralDeviation_oscillation)
{
  waitForDummyNode();
  setTargetMetric("lateral_deviation_CAR");

  const double deviation = 1.0;
  double sign = 1.0;
  for (double time = 0; time < time_delay_ * 2; time += time_step_) {
    PredictedObjects objects;
    if (time == time_delay_) {
      objects = makeDeviatedStraightPredictedObjects(time, 0);
    } else {
      objects = makeDeviatedStraightPredictedObjects(time, deviation * sign);
      sign *= -1.0;
    }
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testLateralDeviation_distortion)
{
  waitForDummyNode();
  setTargetMetric("lateral_deviation_CAR");

  const double deviation = 1.0;
  for (double time = 0; time < time_delay_ * 2; time += time_step_) {
    PredictedObjects objects;
    if (time == time_delay_) {
      objects = makeDeviatedStraightPredictedObjects(time, deviation);
    } else if (time == time_delay_ + time_step_) {
      objects = makeDeviatedStraightPredictedObjects(time, -deviation);
    } else {
      objects = makeDeviatedStraightPredictedObjects(time, 0);
    }
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), deviation, epsilon);
}

TEST_F(EvalTest, testLateralDeviation_deviation0_PEDESTRIAN)
{
  waitForDummyNode();
  setTargetMetric("lateral_deviation_PEDESTRIAN");

  const double deviation = 0.0;
  for (double time = 0; time < time_delay_; time += time_step_) {
    const auto objects =
      makeDeviatedStraightPredictedObjects(time, deviation, ObjectClassification::PEDESTRIAN);
    publishObjects(objects);
  }

  const auto last_objects =
    makeDeviatedStraightPredictedObjects(time_delay_, deviation, ObjectClassification::PEDESTRIAN);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}
// ==========================================================================================

// ==========================================================================================
// yaw deviation
TEST_F(EvalTest, testYawDeviation_deviation0)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_CAR");

  const double deviation = 0.0;
  for (double time = 0; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testYawDeviation_deviation1)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_CAR");

  const double deviation = 1.0;
  for (double time = 0; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testYawDeviation_oscillation)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_CAR");

  const double deviation = 1.0;
  double sign = 1.0;
  for (double time = 0; time < time_delay_ * 2; time += time_step_) {
    PredictedObjects objects;
    if (time == time_delay_) {
      objects = makeDeviatedStraightPredictedObjects(time, 0);
    } else {
      objects = makeDeviatedStraightPredictedObjects(time, deviation * sign);
      sign *= -1.0;
    }
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testYawDeviation_distortion)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_CAR");

  const double deviation = 1.0;
  for (double time = 0; time < time_delay_ * 2; time += time_step_) {
    PredictedObjects objects;
    if (time == time_delay_) {
      objects = makeDeviatedStraightPredictedObjects(time, deviation);
    } else if (time == time_delay_ + time_step_) {
      objects = makeDeviatedStraightPredictedObjects(time, -deviation);
    } else {
      objects = makeDeviatedStraightPredictedObjects(time, 0);
    }
    publishObjects(objects);
  }

  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0, epsilon);
}

TEST_F(EvalTest, testYawDeviation_oscillation_rotate)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_CAR");

  const double deviation = 1.0;
  const double yaw = M_PI / 4;
  double sign = 1.0;
  for (double time = 0; time < time_delay_ * 2; time += time_step_) {
    PredictedObjects objects;
    if (time == time_delay_) {
      objects = rotateObjects(makeDeviatedStraightPredictedObjects(time, 0), yaw);
    } else {
      objects = rotateObjects(
        makeDeviatedStraightPredictedObjects(time, deviation * sign), 2 * M_PI * std::rand());
      sign *= -1.0;
    }
    publishObjects(objects);
  }

  const auto last_objects = rotateObjects(
    makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation), 2 * M_PI * std::rand());
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), yaw, epsilon);
}

TEST_F(EvalTest, testYawDeviation_distortion_rotate)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_CAR");

  const double deviation = 1.0;
  const double yaw = M_PI / 4;
  for (double time = 0; time < time_delay_ * 2; time += time_step_) {
    PredictedObjects objects;
    if (time == time_delay_) {
      objects = rotateObjects(makeDeviatedStraightPredictedObjects(time, deviation), yaw);
    } else if (time == time_delay_ + time_step_) {
      objects = rotateObjects(
        makeDeviatedStraightPredictedObjects(time, -deviation), 2 * M_PI * std::rand());
    } else {
      objects =
        rotateObjects(makeDeviatedStraightPredictedObjects(time, 0), 2 * M_PI * std::rand());
    }
    publishObjects(objects);
  }

  const auto last_objects = rotateObjects(
    makeDeviatedStraightPredictedObjects(time_delay_ * 2, deviation), 2 * M_PI * std::rand());
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), yaw, epsilon);
}

TEST_F(EvalTest, testYawDeviation_deviation0_PEDESTRIAN)
{
  waitForDummyNode();
  setTargetMetric("yaw_deviation_PEDESTRIAN");

  const double deviation = 0.0;
  for (double time = 0; time < time_delay_; time += time_step_) {
    const auto objects =
      makeDeviatedStraightPredictedObjects(time, deviation, ObjectClassification::PEDESTRIAN);
    publishObjects(objects);
  }

  const auto last_objects =
    makeDeviatedStraightPredictedObjects(time_delay_, deviation, ObjectClassification::PEDESTRIAN);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

// ==========================================================================================
// predicted path deviation
TEST_F(EvalTest, testPredictedPathDeviation_deviation0)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const double deviation = 0.0;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);

  const double num_points = time_delay_ / time_step_ + 1;
  const double mean_deviation = deviation * (num_points - 1) / num_points;
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), mean_deviation, epsilon);
}

TEST_F(EvalTest, testPredictedPathDeviation_deviation1)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const double deviation = 1.0;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);

  const double num_points = time_delay_ / time_step_ + 1;
  const double mean_deviation = deviation * (num_points - 1) / num_points;
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), mean_deviation, epsilon);
}

TEST_F(EvalTest, testPredictedPathDeviation_deviation2)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const double deviation = 2.0;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);

  const double num_points = time_delay_ / time_step_ + 1;
  const double mean_deviation = deviation * (num_points - 1) / num_points;
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), mean_deviation, epsilon);
}

TEST_F(EvalTest, testPredictedPathDeviation_deviation0_PEDESTRIAN)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_PEDESTRIAN_5.00");

  const auto init_objects = makeStraightPredictedObjects(0, ObjectClassification::PEDESTRIAN);
  publishObjects(init_objects);

  const double deviation = 0.0;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects =
      makeDeviatedStraightPredictedObjects(time, deviation, ObjectClassification::PEDESTRIAN);
    publishObjects(objects);
  }
  const auto last_objects =
    makeDeviatedStraightPredictedObjects(time_delay_, deviation, ObjectClassification::PEDESTRIAN);

  const double num_points = time_delay_ / time_step_ + 1;
  const double mean_deviation = deviation * (num_points - 1) / num_points;
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), mean_deviation, epsilon);
}
// ==========================================================================================

// ==========================================================================================
// predicted path deviation variance
TEST_F(EvalTest, testPredictedPathDeviationVariance_deviation0)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_variance_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const double deviation = 0.0;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);

  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testPredictedPathDeviationVariance_deviation1)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_variance_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const double deviation = 1.0;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
  }
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);

  const double num_points = time_delay_ / time_step_ + 1;
  // deviations
  //   All    - 11 points (num_points)
  //   0.0[m] -  1 points
  //   1.0[m] - 10 points
  const double mean_deviation = deviation * (num_points - 1) / num_points;
  const double variance =
    (pow(0.0 - mean_deviation, 2) + 10 * pow(1.0 - mean_deviation, 2)) / num_points;

  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), variance, epsilon);
}

TEST_F(EvalTest, testPredictedPathDeviationVariance_deviationIncreasing)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_variance_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const double deviation_step = 0.1;
  double deviation = deviation_step;
  for (double time = time_step_; time < time_delay_; time += time_step_) {
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviation);
    publishObjects(objects);
    deviation += deviation_step;
  }
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviation);

  const double num_points = time_delay_ / time_step_ + 1;
  // deviations
  //   All          - 11 points (num_points)
  //   0.0[m]       -  1 points
  //   0.1[m]       -  1 points
  //   0.2[m]       -  1 points
  //      :
  //   0.9[m]       -  1 points
  //   1.0[m]       -  1 points
  const double mean_deviation = std::invoke([&]() {
    double sum = 0.0;
    for (size_t i = 0; i < num_points; ++i) {
      sum += static_cast<double>(i) * deviation_step;
    }
    return sum / num_points;
  });
  double sum_squared_deviations = 0.0;
  for (size_t i = 0; i < num_points; ++i) {
    const double deviation = static_cast<double>(i) * deviation_step;
    sum_squared_deviations += pow(deviation - mean_deviation, 2);
  }
  const double variance = sum_squared_deviations / num_points;

  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), variance, epsilon);
}

TEST_F(EvalTest, testPredictedPathDeviationVariance_deviationOscillating)
{
  waitForDummyNode();

  setTargetMetric("predicted_path_deviation_variance_CAR_5.00");

  const auto init_objects = makeStraightPredictedObjects(0);
  publishObjects(init_objects);

  const std::vector<double> deviations = {-0.1, -0.2, -0.1, 0.0, 0.1, 0.2, 0.1, 0.0, -0.1, -0.2};
  for (size_t i = 0; i < deviations.size() - 1; ++i) {
    const double time = static_cast<double>(i + 1) * time_step_;
    const auto objects = makeDeviatedStraightPredictedObjects(time, deviations[i]);
    publishObjects(objects);
  }

  const double num_points = deviations.size() + 1;
  // deviations
  //   All          - 11 points (num_points)
  //   0.0[m]       -  1 points
  //  -0.1[m]       -  1 points
  //  -0.2[m]       -  1 points
  //  -0.1[m]       -  1 points
  //  -0.0[m]       -  1 points
  //   0.1[m]       -  1 points
  //   0.2[m]       -  1 points
  //   0.1[m]       -  1 points
  //   0.0[m]       -  1 points
  //  -0.1[m]       -  1 points
  //  -0.2[m]       -  1 points
  const double mean_deviation =
    std::accumulate(
      deviations.begin(), deviations.end(), 0.0,
      [](double sum, double deviation) { return sum + std::abs(deviation); }) /
    num_points;

  double sum_squared_deviations = pow(0 - mean_deviation, 2);
  for (const auto deviation : deviations) {
    sum_squared_deviations += pow(std::abs(deviation) - mean_deviation, 2);
  }
  const double variance = sum_squared_deviations / num_points;
  const auto last_objects = makeDeviatedStraightPredictedObjects(time_delay_, deviations.back());
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), variance, epsilon);
}
// ==========================================================================================

// ==========================================================================================
// yaw rate
TEST_F(EvalTest, testYawRate_0)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0);
    publishObjects(objects);
  }

  const auto last_objects =
    makeStraightPredictedObjects(time_delay_ + time_step_, ObjectClassification::CAR, 0.0);
  EXPECT_NEAR(publishObjectsAndGetMetric(last_objects), 0.0, epsilon);
}

TEST_F(EvalTest, testYawRate_01)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  const double yaw_rate = 0.1;

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), yaw_rate * time);
    publishObjects(objects);
  }

  for (double time = time_delay_ + time_step_; time < time_delay_ * 2; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), yaw_rate * time);
    EXPECT_NEAR(publishObjectsAndGetMetric(objects), yaw_rate, epsilon);
  }
}

TEST_F(EvalTest, testYawRate_minus_01)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  const double yaw_rate = 0.1;

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), -yaw_rate * time);
    publishObjects(objects);
  }

  for (double time = time_delay_ + time_step_; time < time_delay_ * 2; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), -yaw_rate * time);
    EXPECT_NEAR(publishObjectsAndGetMetric(objects), yaw_rate, epsilon);
  }
}

TEST_F(EvalTest, testYawRate_1)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  const double yaw_rate = 1.0;

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), yaw_rate * time);
    publishObjects(objects);
  }

  for (double time = time_delay_ + time_step_; time < time_delay_ * 2; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), yaw_rate * time);
    EXPECT_NEAR(publishObjectsAndGetMetric(objects), yaw_rate, epsilon);
  }
}

TEST_F(EvalTest, testYawRate_minus_1)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  const double yaw_rate = 1.0;

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), -yaw_rate * time);
    publishObjects(objects);
  }

  for (double time = time_delay_ + time_step_; time < time_delay_ * 2; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), -yaw_rate * time);
    EXPECT_NEAR(publishObjectsAndGetMetric(objects), yaw_rate, epsilon);
  }
}

TEST_F(EvalTest, testYawRate_5)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  const double yaw_rate = 5.0;

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), yaw_rate * time);
    publishObjects(objects);
  }

  for (double time = time_delay_ + time_step_; time < time_delay_ * 2; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), yaw_rate * time);
    EXPECT_NEAR(publishObjectsAndGetMetric(objects), yaw_rate, epsilon);
  }
}

TEST_F(EvalTest, testYawRate_minus_5)
{
  waitForDummyNode();
  setTargetMetric("yaw_rate_CAR");

  const double yaw_rate = 5.0;

  for (double time = 0; time <= time_delay_; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), -yaw_rate * time);
    publishObjects(objects);
  }

  for (double time = time_delay_ + time_step_; time < time_delay_ * 2; time += time_step_) {
    const auto objects = rotateObjects(
      makeStraightPredictedObjects(time, ObjectClassification::CAR, 0.0), -yaw_rate * time);
    EXPECT_NEAR(publishObjectsAndGetMetric(objects), yaw_rate, epsilon);
  }
}
// TEST_F(EvalTest, testYawRate_rate01)
// ==========================================================================================
