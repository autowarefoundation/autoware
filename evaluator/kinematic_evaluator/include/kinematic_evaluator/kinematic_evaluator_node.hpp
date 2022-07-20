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

#ifndef KINEMATIC_EVALUATOR__KINEMATIC_EVALUATOR_NODE_HPP_
#define KINEMATIC_EVALUATOR__KINEMATIC_EVALUATOR_NODE_HPP_

#include "kinematic_evaluator/metrics_calculator.hpp"
#include "kinematic_evaluator/stat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace kinematic_diagnostics
{
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

/**
 * @brief Node for kinematic evaluation
 */
class KinematicEvaluatorNode : public rclcpp::Node
{
public:
  explicit KinematicEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~KinematicEvaluatorNode();

  /**
   * @brief callback on vehicle twist message
   * @param [in] twist_msg twist message
   */
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief publish the given metric statistic
   */
  DiagnosticStatus generateDiagnosticStatus(
    const Metric & metric, const Stat<double> & metric_stat) const;

private:
  geometry_msgs::msg::Pose getCurrentEgoPose() const;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // Parameters
  std::string output_file_str_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  std::deque<rclcpp::Time> stamps_;
  std::array<std::deque<Stat<double>>, static_cast<size_t>(Metric::SIZE)> metric_stats_;
  std::unordered_map<Metric, Stat<double>> metrics_dict_;
};
}  // namespace kinematic_diagnostics

#endif  // KINEMATIC_EVALUATOR__KINEMATIC_EVALUATOR_NODE_HPP_
