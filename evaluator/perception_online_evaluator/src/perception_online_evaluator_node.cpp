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

#include "autoware/universe_utils/ros/marker_helper.hpp"
#include "autoware/universe_utils/ros/parameter.hpp"
#include "autoware/universe_utils/ros/update_param.hpp"
#include "perception_online_evaluator/utils/marker_utils.hpp"

#include <autoware/universe_utils/ros/uuid_helper.hpp>

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

  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("perception_online_evaluator_node");
    google::InstallFailureSignalHandler();
  }

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
    const auto metric_result = metrics_calculator_.calculate(Metric(metric));
    if (!metric_result.has_value()) {
      continue;
    }

    std::visit(
      [&metrics_msg, this](auto && arg) {
        using T = std::decay_t<decltype(arg)>;
        for (const auto & [metric, value] : arg) {
          if constexpr (std::is_same_v<T, MetricStatMap>) {
            if (value.count() > 0) {
              metrics_msg.status.emplace_back(generateDiagnosticStatus(metric, value));
            }
          } else if constexpr (std::is_same_v<T, MetricValueMap>) {
            metrics_msg.status.emplace_back(generateDiagnosticStatus(metric, value));
          }
        }
      },
      metric_result.value());
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

DiagnosticStatus PerceptionOnlineEvaluatorNode::generateDiagnosticStatus(
  const std::string & metric, const double value) const
{
  DiagnosticStatus status;

  status.level = status.OK;
  status.name = metric;

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "metric_value";
  key_value.value = std::to_string(value);
  status.values.push_back(key_value);

  return status;
}

void PerceptionOnlineEvaluatorNode::onObjects(const PredictedObjects::ConstSharedPtr objects_msg)
{
  metrics_calculator_.setPredictedObjects(*objects_msg, *tf_buffer_);
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
    autoware::universe_utils::appendMarkerArray(added, &marker);
  };

  const auto & p = parameters_->debug_marker_parameters;

  // visualize history path
  {
    const auto history_path_map = metrics_calculator_.getHistoryPathMap();
    int32_t history_path_first_id = 0;
    int32_t smoothed_history_path_first_id = 0;
    size_t i = 0;
    for (const auto & [uuid, history_path] : history_path_map) {
      {
        const auto c = createColorFromString(uuid + "_raw");
        if (p.show_history_path) {
          add(createPointsMarkerArray(history_path.first, "history_path", i, c.r, c.g, c.b));
        }
        if (p.show_history_path_arrows) {
          add(createPosesMarkerArray(
            history_path.first, "history_path_arrows", history_path_first_id, c.r, c.g, c.b, 0.1,
            0.05, 0.05));
          history_path_first_id += history_path.first.size();
        }
      }
      {
        const auto c = createColorFromString(uuid);
        if (p.show_smoothed_history_path) {
          add(createPointsMarkerArray(
            history_path.second, "smoothed_history_path", i, c.r, c.g, c.b));
        }
        if (p.show_smoothed_history_path_arrows) {
          add(createPosesMarkerArray(
            history_path.second, "smoothed_history_path_arrows", smoothed_history_path_first_id,
            c.r, c.g, c.b, 0.1, 0.05, 0.05));
          smoothed_history_path_first_id += history_path.second.size();
        }
      }
      i++;
    }
  }

  // visualize predicted path of past objects
  {
    int32_t predicted_path_first_id = 0;
    int32_t history_path_first_id = 0;
    int32_t deviation_lines_first_id = 0;
    size_t i = 0;
    const auto object_data_map = metrics_calculator_.getDebugObjectData();
    for (const auto & [uuid, object_data] : object_data_map) {
      const auto c = createColorFromString(uuid);
      const auto predicted_path = object_data.getPredictedPath();
      const auto history_path = object_data.getHistoryPath();
      if (p.show_predicted_path) {
        add(createPosesMarkerArray(
          predicted_path, "predicted_path", predicted_path_first_id, 0, 0, 1));
        predicted_path_first_id += predicted_path.size();
      }
      if (p.show_predicted_path_gt) {
        add(createPosesMarkerArray(
          history_path, "predicted_path_gt", history_path_first_id, 1, 0, 0));
        history_path_first_id += history_path.size();
      }
      if (p.show_deviation_lines) {
        add(createDeviationLines(
          predicted_path, history_path, "deviation_lines", deviation_lines_first_id, 1, 1, 1));
        deviation_lines_first_id += predicted_path.size();
      }
      if (p.show_object_polygon) {
        add(createObjectPolygonMarkerArray(object_data.object, "object_polygon", i, c.r, c.g, c.b));
      }
      i++;
    }
  }

  pub_marker_->publish(marker);
}

rcl_interfaces::msg::SetParametersResult PerceptionOnlineEvaluatorNode::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;

  auto & p = parameters_;

  updateParam<size_t>(parameters, "smoothing_window_size", p->smoothing_window_size);
  updateParam<double>(parameters, "stopped_velocity_threshold", p->stopped_velocity_threshold);
  updateParam<double>(
    parameters, "detection_count_purge_seconds", p->detection_count_purge_seconds);
  updateParam<double>(parameters, "objects_count_window_seconds", p->objects_count_window_seconds);

  // update parameters for each object class
  {
    const auto update_object_param = [&p, &parameters](
                                       const auto & semantic, const std::string & ns) {
      auto & config = p->object_parameters.at(semantic);
      updateParam<bool>(parameters, ns + "check_lateral_deviation", config.check_lateral_deviation);
      updateParam<bool>(parameters, ns + "check_yaw_deviation", config.check_yaw_deviation);
      updateParam<bool>(
        parameters, ns + "check_predicted_path_deviation", config.check_predicted_path_deviation);
      updateParam<bool>(parameters, ns + "check_yaw_rate", config.check_yaw_rate);
      updateParam<bool>(
        parameters, ns + "check_total_objects_count", config.check_total_objects_count);
      updateParam<bool>(
        parameters, ns + "check_average_objects_count", config.check_average_objects_count);
      updateParam<bool>(
        parameters, ns + "check_interval_average_objects_count",
        config.check_interval_average_objects_count);
    };
    const std::string ns = "target_object.";
    update_object_param(ObjectClassification::MOTORCYCLE, ns + "motorcycle.");
    update_object_param(ObjectClassification::CAR, ns + "car.");
    update_object_param(ObjectClassification::TRUCK, ns + "truck.");
    update_object_param(ObjectClassification::TRAILER, ns + "trailer.");
    update_object_param(ObjectClassification::BUS, ns + "bus.");
    update_object_param(ObjectClassification::PEDESTRIAN, ns + "pedestrian.");
    update_object_param(ObjectClassification::BICYCLE, ns + "bicycle.");
    update_object_param(ObjectClassification::UNKNOWN, ns + "unknown.");
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
  using autoware::universe_utils::getOrDeclareParameter;
  using autoware::universe_utils::updateParam;

  auto & p = parameters_;

  p->smoothing_window_size = getOrDeclareParameter<int>(*this, "smoothing_window_size");
  p->prediction_time_horizons =
    getOrDeclareParameter<std::vector<double>>(*this, "prediction_time_horizons");
  p->stopped_velocity_threshold =
    getOrDeclareParameter<double>(*this, "stopped_velocity_threshold");
  p->detection_radius_list =
    getOrDeclareParameter<std::vector<double>>(*this, "detection_radius_list");
  p->detection_height_list =
    getOrDeclareParameter<std::vector<double>>(*this, "detection_height_list");
  p->detection_count_purge_seconds =
    getOrDeclareParameter<double>(*this, "detection_count_purge_seconds");
  p->objects_count_window_seconds =
    getOrDeclareParameter<double>(*this, "objects_count_window_seconds");

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
      param.check_lateral_deviation =
        getOrDeclareParameter<bool>(*this, ns + "check_lateral_deviation");
      param.check_yaw_deviation = getOrDeclareParameter<bool>(*this, ns + "check_yaw_deviation");
      param.check_predicted_path_deviation =
        getOrDeclareParameter<bool>(*this, ns + "check_predicted_path_deviation");
      param.check_yaw_rate = getOrDeclareParameter<bool>(*this, ns + "check_yaw_rate");
      param.check_total_objects_count =
        getOrDeclareParameter<bool>(*this, ns + "check_total_objects_count");
      param.check_average_objects_count =
        getOrDeclareParameter<bool>(*this, ns + "check_average_objects_count");
      param.check_interval_average_objects_count =
        getOrDeclareParameter<bool>(*this, ns + "check_interval_average_objects_count");
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
