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

#ifndef PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_
#define PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_

#include "planning_evaluator/metrics_calculator.hpp"
#include "planning_evaluator/stat.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_auto_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <array>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace planning_diagnostics
{
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

/**
 * @brief Node for planning evaluation
 */
class PlanningEvaluatorNode : public rclcpp::Node
{
public:
  explicit PlanningEvaluatorNode(const rclcpp::NodeOptions & node_options);
  ~PlanningEvaluatorNode();

  /**
   * @brief callback on receiving a trajectory
   * @param [in] traj_msg received trajectory message
   */
  void onTrajectory(const Trajectory::ConstSharedPtr traj_msg);

  /**
   * @brief callback on receiving a reference trajectory
   * @param [in] traj_msg received trajectory message
   */
  void onReferenceTrajectory(const Trajectory::ConstSharedPtr traj_msg);

  /**
   * @brief callback on receiving a dynamic objects array
   * @param [in] objects_msg received dynamic object array message
   */
  void onObjects(const PredictedObjects::ConstSharedPtr objects_msg);

  /**
   * @brief update the ego pose stored in the MetricsCalculator
   */
  void updateCalculatorEgoPose(const std::string & target_frame);

  /**
   * @brief publish the given metric statistic
   */
  DiagnosticStatus generateDiagnosticStatus(
    const Metric & metric, const Stat<double> & metric_stat) const;

private:
  static bool isFinite(const TrajectoryPoint & p);

  // ROS
  rclcpp::Subscription<Trajectory>::SharedPtr traj_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr ref_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr objects_sub_;
  rclcpp::Publisher<DiagnosticArray>::SharedPtr metrics_pub_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Parameters
  std::string output_file_str_;
  std::string ego_frame_str_;

  // Calculator
  MetricsCalculator metrics_calculator_;
  // Metrics
  std::vector<Metric> metrics_;
  std::deque<rclcpp::Time> stamps_;
  std::array<std::deque<Stat<double>>, static_cast<size_t>(Metric::SIZE)> metric_stats_;
};
}  // namespace planning_diagnostics

#endif  // PLANNING_EVALUATOR__PLANNING_EVALUATOR_NODE_HPP_
