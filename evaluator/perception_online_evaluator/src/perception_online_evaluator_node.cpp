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

#include "perception_online_evaluator/perception_online_evaluator_node.hpp"

#include "perception_online_evaluator/utils/marker_utils.hpp"
#include "tier4_autoware_utils/ros/marker_helper.hpp"
#include "tier4_autoware_utils/ros/parameter.hpp"
#include "tier4_autoware_utils/ros/update_param.hpp"

#include <tier4_autoware_utils/ros/uuid_helper.hpp>

#include "boost/lexical_cast.hpp"

#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace perception_diagnostics
{
PerceptionOnlineEvaluatorNode::PerceptionOnlineEvaluatorNode(
  const rclcpp::NodeOptions & node_options)
: Node("perception_online_evaluator", node_options),
  parameters_(std::make_shared<Parameters>()),
  metrics_calculator_(parameters_)
{
  using std::placeholders::_1;

  google::InitGoogleLogging("perception_online_evaluator_node");
  google::InstallFailureSignalHandler();

  objects_sub_ = create_subscription<PredictedObjects>(
    "~/input/objects", 1, std::bind(&PerceptionOnlineEvaluatorNode::onObjects, this, _1));
  metrics_pub_ = create_publisher<DiagnosticArray>("~/metrics", 1);
  pub_marker_ = create_publisher<MarkerArray>("~/markers", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Parameters
  initParameter();

  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&PerceptionOnlineEvaluatorNode::onParameter, this, std::placeholders::_1));
}

void PerceptionOnlineEvaluatorNode::publishMetrics()
{
  DiagnosticArray metrics_msg;

  // calculate metrics
  for (const Metric & metric : parameters_->metrics) {
    const auto metric_stat_map = metrics_calculator_.calculate(Metric(metric));
    if (!metric_stat_map.has_value()) {
      continue;
    }

    for (const auto & [metric, stat] : metric_stat_map.value()) {
      if (stat.count() > 0) {
        metrics_msg.status.push_back(generateDiagnosticStatus(metric, stat));
      }
    }
  }

  // publish metrics
  if (!metrics_msg.status.empty()) {
    metrics_msg.header.stamp = now();
    metrics_pub_->publish(metrics_msg);
  }
  publishDebugMarker();
}

DiagnosticStatus PerceptionOnlineEvaluatorNode::generateDiagnosticStatus(
  const std::string metric, const Stat<double> & metric_stat) const
{
  DiagnosticStatus status;

  status.level = status.OK;
  status.name = metric;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "min";
  key_value.value = std::to_string(metric_stat.min());
  status.values.push_back(key_value);
  key_value.key = "max";
  key_value.value = std::to_string(metric_stat.max());
  status.values.push_back(key_value);
  key_value.key = "mean";
  key_value.value = std::to_string(metric_stat.mean());
  status.values.push_back(key_value);

  return status;
}

void PerceptionOnlineEvaluatorNode::onObjects(const PredictedObjects::ConstSharedPtr objects_msg)
{
  metrics_calculator_.setPredictedObjects(*objects_msg);
  publishMetrics();
}

void PerceptionOnlineEvaluatorNode::publishDebugMarker()
{
  using marker_utils::createColorFromString;
  using marker_utils::createDeviationLines;
  using marker_utils::createObjectPolygonMarkerArray;
  using marker_utils::createPointsMarkerArray;
  using marker_utils::createPosesMarkerArray;

  MarkerArray marker;

  const auto add = [&marker](MarkerArray added) {
    for (auto & marker : added.markers) {
      marker.lifetime = rclcpp::Duration::from_seconds(1.5);
    }
    tier4_autoware_utils::appendMarkerArray(added, &marker);
  };

  const auto history_path_map = metrics_calculator_.getHistoryPathMap();
  const auto & p = parameters_->debug_marker_parameters;

  for (const auto & [uuid, history_path] : history_path_map) {
    {
      const auto c = createColorFromString(uuid + "_raw");
      if (p.show_history_path) {
        add(createPointsMarkerArray(history_path.first, "history_path_" + uuid, 0, c.r, c.g, c.b));
      }
      if (p.show_history_path_arrows) {
        add(createPosesMarkerArray(
          history_path.first, "history_path_arrows_" + uuid, c.r, c.g, c.b, 0.1, 0.05, 0.05));
      }
    }
    {
      const auto c = createColorFromString(uuid);
      if (p.show_smoothed_history_path) {
        add(createPointsMarkerArray(
          history_path.second, "smoothed_history_path_" + uuid, 0, c.r, c.g, c.b));
      }
      if (p.show_smoothed_history_path_arrows) {
        add(createPosesMarkerArray(
          history_path.second, "smoothed_history_path_arrows_" + uuid, c.r, c.g, c.b, 0.1, 0.05,
          0.05));
      }
    }
  }
  const auto object_data_map = metrics_calculator_.getDebugObjectData();
  for (const auto & [uuid, object_data] : object_data_map) {
    const auto c = createColorFromString(uuid);
    const auto predicted_path = object_data.getPredictedPath();
    const auto history_path = object_data.getHistoryPath();
    if (p.show_predicted_path) {
      add(createPosesMarkerArray(predicted_path, "predicted_path_" + uuid, 0, 0, 1));
    }
    if (p.show_predicted_path_gt) {
      add(createPosesMarkerArray(history_path, "predicted_path_gt_" + uuid, 1, 0, 0));
    }
    if (p.show_deviation_lines) {
      add(createDeviationLines(predicted_path, history_path, "deviation_lines_" + uuid, 1, 1, 1));
    }
    if (p.show_object_polygon) {
      add(createObjectPolygonMarkerArray(
        object_data.object, "object_polygon_" + uuid, 0, c.r, c.g, c.b));
    }
  }

  pub_marker_->publish(marker);
}

rcl_interfaces::msg::SetParametersResult PerceptionOnlineEvaluatorNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  updateParam<size_t>(parameters, "smoothing_window_size", p->smoothing_window_size);

  // update metrics
  {
    std::vector<std::string> metrics_str;
    updateParam<std::vector<std::string>>(parameters, "selected_metrics", metrics_str);
    std::vector<Metric> metrics;
    for (const std::string & selected_metric : metrics_str) {
      const Metric metric = str_to_metric.at(selected_metric);
      metrics.push_back(metric);
    }
    p->metrics = metrics;
  }

  // update parameters for each object class
  {
    const auto get_object_param = [&](std::string && ns) -> std::optional<ObjectParameter> {
      ObjectParameter param{};
      if (updateParam<bool>(parameters, ns + "check_deviation", param.check_deviation)) {
        return param;
      }
      return std::nullopt;
    };

    const std::string ns = "target_object.";
    if (const auto new_param = get_object_param(ns + "car.")) {
      p->object_parameters.at(ObjectClassification::CAR) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "truck.")) {
      p->object_parameters.at(ObjectClassification::TRUCK) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "bus.")) {
      p->object_parameters.at(ObjectClassification::BUS) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "trailer.")) {
      p->object_parameters.at(ObjectClassification::TRAILER) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "bicycle.")) {
      p->object_parameters.at(ObjectClassification::BICYCLE) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "motorcycle.")) {
      p->object_parameters.at(ObjectClassification::MOTORCYCLE) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "pedestrian.")) {
      p->object_parameters.at(ObjectClassification::PEDESTRIAN) = *new_param;
    }
    if (const auto new_param = get_object_param(ns + "unknown.")) {
      p->object_parameters.at(ObjectClassification::UNKNOWN) = *new_param;
    }
  }

  // update debug marker parameters
  {
    const std::string ns = "debug_marker.";
    updateParam<bool>(
      parameters, ns + "history_path", p->debug_marker_parameters.show_history_path);
    updateParam<bool>(
      parameters, ns + "history_path_arrows", p->debug_marker_parameters.show_history_path_arrows);
    updateParam<bool>(
      parameters, ns + "smoothed_history_path",
      p->debug_marker_parameters.show_smoothed_history_path);
    updateParam<bool>(
      parameters, ns + "smoothed_history_path_arrows",
      p->debug_marker_parameters.show_smoothed_history_path_arrows);
    updateParam<bool>(
      parameters, ns + "predicted_path", p->debug_marker_parameters.show_predicted_path);
    updateParam<bool>(
      parameters, ns + "predicted_path_gt", p->debug_marker_parameters.show_predicted_path_gt);
    updateParam<bool>(
      parameters, ns + "deviation_lines", p->debug_marker_parameters.show_deviation_lines);
    updateParam<bool>(
      parameters, ns + "object_polygon", p->debug_marker_parameters.show_object_polygon);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

void PerceptionOnlineEvaluatorNode::initParameter()
{
  using tier4_autoware_utils::getOrDeclareParameter;
  using tier4_autoware_utils::updateParam;

  auto & p = parameters_;

  p->smoothing_window_size = getOrDeclareParameter<int>(*this, "smoothing_window_size");
  p->prediction_time_horizons =
    getOrDeclareParameter<std::vector<double>>(*this, "prediction_time_horizons");

  // set metrics
  const auto selected_metrics =
    getOrDeclareParameter<std::vector<std::string>>(*this, "selected_metrics");
  for (const std::string & selected_metric : selected_metrics) {
    const Metric metric = str_to_metric.at(selected_metric);
    parameters_->metrics.push_back(metric);
  }

  // set parameters for each object class
  {
    const auto get_object_param = [&](std::string && ns) -> ObjectParameter {
      ObjectParameter param{};
      param.check_deviation = getOrDeclareParameter<bool>(*this, ns + "check_deviation");
      return param;
    };

    const std::string ns = "target_object.";
    p->object_parameters.emplace(ObjectClassification::CAR, get_object_param(ns + "car."));
    p->object_parameters.emplace(ObjectClassification::TRUCK, get_object_param(ns + "truck."));
    p->object_parameters.emplace(ObjectClassification::BUS, get_object_param(ns + "bus."));
    p->object_parameters.emplace(ObjectClassification::TRAILER, get_object_param(ns + "trailer."));
    p->object_parameters.emplace(ObjectClassification::BICYCLE, get_object_param(ns + "bicycle."));
    p->object_parameters.emplace(
      ObjectClassification::MOTORCYCLE, get_object_param(ns + "motorcycle."));
    p->object_parameters.emplace(
      ObjectClassification::PEDESTRIAN, get_object_param(ns + "pedestrian."));
    p->object_parameters.emplace(ObjectClassification::UNKNOWN, get_object_param(ns + "unknown."));
  }

  // set debug marker parameters
  {
    const std::string ns = "debug_marker.";
    p->debug_marker_parameters.show_history_path =
      getOrDeclareParameter<bool>(*this, ns + "history_path");
    p->debug_marker_parameters.show_history_path_arrows =
      getOrDeclareParameter<bool>(*this, ns + "history_path_arrows");
    p->debug_marker_parameters.show_smoothed_history_path =
      getOrDeclareParameter<bool>(*this, ns + "smoothed_history_path");
    p->debug_marker_parameters.show_smoothed_history_path_arrows =
      getOrDeclareParameter<bool>(*this, ns + "smoothed_history_path_arrows");
    p->debug_marker_parameters.show_predicted_path =
      getOrDeclareParameter<bool>(*this, ns + "predicted_path");
    p->debug_marker_parameters.show_predicted_path_gt =
      getOrDeclareParameter<bool>(*this, ns + "predicted_path_gt");
    p->debug_marker_parameters.show_deviation_lines =
      getOrDeclareParameter<bool>(*this, ns + "deviation_lines");
    p->debug_marker_parameters.show_object_polygon =
      getOrDeclareParameter<bool>(*this, ns + "object_polygon");
  }
}
}  // namespace perception_diagnostics

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_diagnostics::PerceptionOnlineEvaluatorNode)
