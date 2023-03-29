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

#include "planning_evaluator/planning_evaluator_node.hpp"

#include "boost/lexical_cast.hpp"

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace planning_diagnostics
{
PlanningEvaluatorNode::PlanningEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("planning_evaluator", node_options)
{
  using std::placeholders::_1;

  traj_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&PlanningEvaluatorNode::onTrajectory, this, _1));

  ref_sub_ = create_subscription<Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&PlanningEvaluatorNode::onReferenceTrajectory, this, _1));

  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&PlanningEvaluatorNode::onObjects, this, _1));

  modified_goal_sub_ = create_subscription<PoseWithUuidStamped>(
    "~/input/modified_goal", 1, std::bind(&PlanningEvaluatorNode::onModifiedGoal, this, _1));

  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", 1, std::bind(&PlanningEvaluatorNode::onOdometry, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters
  metrics_calculator_.parameters.trajectory.min_point_dist_m =
    declare_parameter<double>("trajectory.min_point_dist_m");
  metrics_calculator_.parameters.trajectory.lookahead.max_dist_m =
    declare_parameter<double>("trajectory.lookahead.max_dist_m");
  metrics_calculator_.parameters.trajectory.lookahead.max_time_s =
    declare_parameter<double>("trajectory.lookahead.max_time_s");
  metrics_calculator_.parameters.obstacle.dist_thr_m =
    declare_parameter<double>("obstacle.dist_thr_m");

  output_file_str_ = declare_parameter<std::string>("output_file");
  ego_frame_str_ = declare_parameter<std::string>("ego_frame");

  // List of metrics to calculate and publish
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);
  for (const std::string & selected_metric :
       declare_parameter<std::vector<std::string>>("selected_metrics")) {
    Metric metric = str_to_metric.at(selected_metric);
    metrics_.push_back(metric);
  }
}

PlanningEvaluatorNode::~PlanningEvaluatorNode()
{
  if (!output_file_str_.empty()) {
    // column width is the maximum size we might print + 1 for the space between columns
    // Write data using format
    std::ofstream f(output_file_str_);
    f << std::fixed << std::left;
    // header
    f << "#Stamp(ns)";
    for (Metric metric : metrics_) {
      f << " " << metric_descriptions.at(metric);
      f << " . .";  // extra "columns" to align columns headers
    }
    f << std::endl;
    f << "#.";
    for (Metric metric : metrics_) {
      (void)metric;
      f << " min max mean";
    }
    f << std::endl;
    // data
    for (size_t i = 0; i < stamps_.size(); ++i) {
      f << stamps_[i].nanoseconds();
      for (Metric metric : metrics_) {
        const auto & stat = metric_stats_[static_cast<size_t>(metric)][i];
        f << " " << stat;
      }
      f << std::endl;
    }
    f.close();
  }
}

DiagnosticStatus PlanningEvaluatorNode::generateDiagnosticStatus(
  const Metric & metric, const Stat<double> & metric_stat) const
{
  DiagnosticStatus status;
  status.level = status.OK;
  status.name = metric_to_str.at(metric);
  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "min";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.min());
  status.values.push_back(key_value);
  key_value.key = "max";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.max());
  status.values.push_back(key_value);
  key_value.key = "mean";
  key_value.value = boost::lexical_cast<decltype(key_value.value)>(metric_stat.mean());
  status.values.push_back(key_value);
  return status;
}

void PlanningEvaluatorNode::onTrajectory(const Trajectory::ConstSharedPtr traj_msg)
{
  if (!ego_state_ptr_) {
    return;
  }

  auto start = now();
  if (!output_file_str_.empty()) {
    stamps_.push_back(traj_msg->header.stamp);
  }

  DiagnosticArray metrics_msg;
  metrics_msg.header.stamp = now();
  for (Metric metric : metrics_) {
    const auto metric_stat = metrics_calculator_.calculate(Metric(metric), *traj_msg);
    if (!metric_stat) {
      continue;
    }

    if (!output_file_str_.empty()) {
      metric_stats_[static_cast<size_t>(metric)].push_back(*metric_stat);
    }

    if (metric_stat->count() > 0) {
      metrics_msg.status.push_back(generateDiagnosticStatus(metric, *metric_stat));
    }
  }
  if (!metrics_msg.status.empty()) {
    metrics_pub_->publish(metrics_msg);
  }
  metrics_calculator_.setPreviousTrajectory(*traj_msg);
  auto runtime = (now() - start).seconds();
  RCLCPP_DEBUG(get_logger(), "Planning evaluation calculation time: %2.2f ms", runtime * 1e3);
}

void PlanningEvaluatorNode::onModifiedGoal(
  const PoseWithUuidStamped::ConstSharedPtr modified_goal_msg)
{
  modified_goal_ptr_ = modified_goal_msg;
  if (ego_state_ptr_) {
    publishModifiedGoalDeviationMetrics();
  }
}

void PlanningEvaluatorNode::onOdometry(const Odometry::ConstSharedPtr odometry_msg)
{
  ego_state_ptr_ = odometry_msg;
  metrics_calculator_.setEgoPose(odometry_msg->pose.pose);

  if (modified_goal_ptr_) {
    publishModifiedGoalDeviationMetrics();
  }
}

void PlanningEvaluatorNode::publishModifiedGoalDeviationMetrics()
{
  auto start = now();

  DiagnosticArray metrics_msg;
  metrics_msg.header.stamp = now();
  for (Metric metric : metrics_) {
    const auto metric_stat = metrics_calculator_.calculate(
      Metric(metric), modified_goal_ptr_->pose, ego_state_ptr_->pose.pose);
    if (!metric_stat) {
      continue;
    }
    metric_stats_[static_cast<size_t>(metric)].push_back(*metric_stat);
    if (metric_stat->count() > 0) {
      metrics_msg.status.push_back(generateDiagnosticStatus(metric, *metric_stat));
    }
  }
  if (!metrics_msg.status.empty()) {
    metrics_pub_->publish(metrics_msg);
  }
  auto runtime = (now() - start).seconds();
  RCLCPP_DEBUG(
    get_logger(), "Planning evaluation modified goal deviation calculation time: %2.2f ms",
    runtime * 1e3);
}

void PlanningEvaluatorNode::onReferenceTrajectory(const Trajectory::ConstSharedPtr traj_msg)
{
  metrics_calculator_.setReferenceTrajectory(*traj_msg);
}

void PlanningEvaluatorNode::onObjects(const PredictedObjects::ConstSharedPtr objects_msg)
{
  metrics_calculator_.setPredictedObjects(*objects_msg);
}

bool PlanningEvaluatorNode::isFinite(const TrajectoryPoint & point)
{
  const auto & o = point.pose.orientation;
  const auto & p = point.pose.position;
  const auto & v = point.longitudinal_velocity_mps;
  const auto & w = point.lateral_velocity_mps;
  const auto & a = point.acceleration_mps2;
  const auto & z = point.heading_rate_rps;
  const auto & f = point.front_wheel_angle_rad;
  const auto & r = point.rear_wheel_angle_rad;

  return std::isfinite(o.x) && std::isfinite(o.y) && std::isfinite(o.z) && std::isfinite(o.w) &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(v) &&
         std::isfinite(w) && std::isfinite(a) && std::isfinite(z) && std::isfinite(f) &&
         std::isfinite(r);
}
}  // namespace planning_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(planning_diagnostics::PlanningEvaluatorNode)
