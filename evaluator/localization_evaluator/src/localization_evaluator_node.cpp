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

#include "localization_evaluator/localization_evaluator_node.hpp"

#include "boost/lexical_cast.hpp"

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace localization_diagnostics
{
LocalizationEvaluatorNode::LocalizationEvaluatorNode(const rclcpp::NodeOptions & node_options)
: Node("localization_evaluator", node_options),
  odom_sub_(this, "~/input/localization", rclcpp::QoS{1}.get_rmw_qos_profile()),
  pose_gt_sub_(this, "~/input/localization/ref", rclcpp::QoS{1}.get_rmw_qos_profile()),
  sync_(SyncPolicy(100), odom_sub_, pose_gt_sub_)
{
  tf_buffer_ptr_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr_);

  sync_.registerCallback(std::bind(
    &LocalizationEvaluatorNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  output_file_str_ = declare_parameter<std::string>("output_file");
  if (output_file_str_.empty()) {
    RCLCPP_INFO(
      get_logger(),
      "Output file not specified, the results will NOT be saved!"
      "Provide output_file parameter to store the results.");
  }
  // List of metrics to calculate
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);
  for (const std::string & selected_metric :
       declare_parameter<std::vector<std::string>>("selected_metrics")) {
    Metric metric = str_to_metric.at(selected_metric);
    metrics_dict_[metric] = Stat<double>();
    metrics_.push_back(metric);
  }
}

LocalizationEvaluatorNode::~LocalizationEvaluatorNode()
{
  if (!output_file_str_.empty()) {
    std::ofstream f(output_file_str_);
    f << std::left << std::fixed;
    // header
    f << "#Data collected over: " << stamps_.back().seconds() - stamps_[0].seconds() << " seconds."
      << std::endl;
    f << std::setw(24) << "#Stamp [ns]";
    for (Metric metric : metrics_) {
      f << std::setw(30) << metric_descriptions.at(metric);
    }
    f << std::endl;
    f << std::setw(24) << "#";
    for (Metric metric : metrics_) {
      (void)metric;
      f << std::setw(9) << "min" << std::setw(9) << "max" << std::setw(12) << "mean";
    }
    f << std::endl;

    // data
    f << std::setw(24) << stamps_.back().nanoseconds();
    for (Metric metric : metrics_) {
      const auto & stat = metric_stats_[static_cast<size_t>(metric)].back();
      f << stat;
      f << std::setw(4) << "";
    }
    f.close();
  }
}

DiagnosticStatus LocalizationEvaluatorNode::generateDiagnosticStatus(
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

void LocalizationEvaluatorNode::syncCallback(
  const Odometry::ConstSharedPtr & msg, const PoseWithCovarianceStamped::ConstSharedPtr & msg_ref)
{
  RCLCPP_DEBUG(
    get_logger(), "Received two messages at time stamps: %d.%d and %d.%d", msg->header.stamp.sec,
    msg->header.stamp.nanosec, msg_ref->header.stamp.sec, msg_ref->header.stamp.nanosec);

  DiagnosticArray metrics_msg;
  metrics_msg.header.stamp = now();

  geometry_msgs::msg::Point p_lc, p_gt;
  p_lc = msg->pose.pose.position;
  p_gt = msg_ref->pose.pose.position;
  if ((p_lc.x == 0 && p_lc.y == 0 && p_lc.z == 0) || (p_gt.x == 0 && p_gt.y == 0 && p_gt.z == 0)) {
    RCLCPP_INFO(get_logger(), "Received position equals zero, waiting for valid data.");
    return;
  }
  for (Metric metric : metrics_) {
    metrics_dict_[metric] = metrics_calculator_.updateStat(
      metrics_dict_[metric], metric, msg->pose.pose.position, msg_ref->pose.pose.position);
    metric_stats_[static_cast<size_t>(metric)].push_back(metrics_dict_[metric]);
    stamps_.push_back(metrics_msg.header.stamp);
    if (metrics_dict_[metric].count() > 0) {
      metrics_msg.status.push_back(generateDiagnosticStatus(metric, metrics_dict_[metric]));
    }
  }
  if (!metrics_msg.status.empty()) {
    metrics_pub_->publish(metrics_msg);
  }
}
}  // namespace localization_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(localization_diagnostics::LocalizationEvaluatorNode)
