//  Copyright 2023 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "target_object_type_panel.hpp"

#include <QColor>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

TargetObjectTypePanel::TargetObjectTypePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  node_ = std::make_shared<rclcpp::Node>("matrix_display_node");

  setParameters();

  matrix_widget_ = new QTableWidget(modules_.size(), targets_.size(), this);
  for (size_t i = 0; i < modules_.size(); i++) {
    matrix_widget_->setVerticalHeaderItem(
      i, new QTableWidgetItem(QString::fromStdString(modules_[i])));
  }
  for (size_t j = 0; j < targets_.size(); j++) {
    matrix_widget_->setHorizontalHeaderItem(
      j, new QTableWidgetItem(QString::fromStdString(targets_[j])));
  }

  updateMatrix();

  reload_button_ = new QPushButton("Reload", this);
  connect(
    reload_button_, &QPushButton::clicked, this, &TargetObjectTypePanel::onReloadButtonClicked);

  QVBoxLayout * layout = new QVBoxLayout;
  layout->addWidget(matrix_widget_);
  layout->addWidget(reload_button_);
  setLayout(layout);
}

void TargetObjectTypePanel::onReloadButtonClicked()
{
  RCLCPP_INFO(node_->get_logger(), "Reload button clicked. Update parameter data.");
  updateMatrix();
}

void TargetObjectTypePanel::setParameters()
{
  // Parameter will be investigated for these modules:
  modules_ = {
    "avoidance",
    "avoidance_by_lane_change",
    "lane_change",
    "obstacle_cruise (inside)",
    "obstacle_cruise (outside)",
    "obstacle_stop",
    "obstacle_slowdown"};

  // Parameter will be investigated for targets in each module
  targets_ = {"car", "truck", "bus", "trailer", "unknown", "bicycle", "motorcycle", "pedestrian"};

  // TODO(Horibe): If the param naming strategy is aligned, this should be done automatically based
  // on the modules_ and targets_.

  // default
  ParamNameEnableObject default_param_name;
  default_param_name.name.emplace("car", "car");
  default_param_name.name.emplace("truck", "truck");
  default_param_name.name.emplace("bus", "bus");
  default_param_name.name.emplace("trailer", "trailer");
  default_param_name.name.emplace("unknown", "unknown");
  default_param_name.name.emplace("bicycle", "bicycle");
  default_param_name.name.emplace("motorcycle", "motorcycle");
  default_param_name.name.emplace("pedestrian", "pedestrian");

  // avoidance
  {
    const auto module = "avoidance";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
    param_name.ns = "avoidance.target_object";
    param_name.name.emplace("car", "car.is_target");
    param_name.name.emplace("truck", "truck.is_target");
    param_name.name.emplace("bus", "bus.is_target");
    param_name.name.emplace("trailer", "trailer.is_target");
    param_name.name.emplace("unknown", "unknown.is_target");
    param_name.name.emplace("bicycle", "bicycle.is_target");
    param_name.name.emplace("motorcycle", "motorcycle.is_target");
    param_name.name.emplace("pedestrian", "pedestrian.is_target");
    param_names_.emplace(module, param_name);
  }

  // avoidance_by_lane_change
  {
    const auto module = "avoidance_by_lane_change";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
    param_name.ns = "avoidance_by_lane_change.target_object";
    param_name.name.emplace("car", "car.is_target");
    param_name.name.emplace("truck", "truck.is_target");
    param_name.name.emplace("bus", "bus.is_target");
    param_name.name.emplace("trailer", "trailer.is_target");
    param_name.name.emplace("unknown", "unknown.is_target");
    param_name.name.emplace("bicycle", "bicycle.is_target");
    param_name.name.emplace("motorcycle", "motorcycle.is_target");
    param_name.name.emplace("pedestrian", "pedestrian.is_target");
    param_names_.emplace(module, param_name);
  }

  // lane_change
  {
    const auto module = "lane_change";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner";
    param_name.ns = "lane_change.target_object";
    param_name.name = default_param_name.name;
    param_names_.emplace(module, param_name);
  }

  // obstacle cruise (inside)
  {
    const auto module = "obstacle_cruise (inside)";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
    param_name.ns = "common.cruise_obstacle_type.inside";
    param_name.name = default_param_name.name;
    param_names_.emplace(module, param_name);
  }

  // obstacle cruise (outside)
  {
    const auto module = "obstacle_cruise (outside)";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
    param_name.ns = "common.cruise_obstacle_type.outside";
    param_name.name = default_param_name.name;
    param_names_.emplace(module, param_name);
  }

  // obstacle stop
  {
    const auto module = "obstacle_stop";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
    param_name.ns = "common.stop_obstacle_type";
    param_name.name = default_param_name.name;
    param_names_.emplace(module, param_name);
  }

  // obstacle slowdown
  {
    const auto module = "obstacle_slowdown";
    ParamNameEnableObject param_name;
    param_name.node =
      "/planning/scenario_planning/lane_driving/motion_planning/obstacle_cruise_planner";
    param_name.ns = "common.slow_down_obstacle_type";
    param_name.name = default_param_name.name;
    param_names_.emplace(module, param_name);
  }
}

void TargetObjectTypePanel::updateMatrix()
{
  // blue base
  // const QColor color_in_use("#6eb6cc");
  // const QColor color_no_use("#1d3e48");
  // const QColor color_undefined("#9e9e9e");

  // green base
  const QColor color_in_use("#afff70");
  const QColor color_no_use("#44642b");
  const QColor color_undefined("#9e9e9e");

  const auto set_undefined = [&](const auto i, const auto j) {
    QTableWidgetItem * item = new QTableWidgetItem("N/A");
    item->setForeground(QBrush(Qt::black));  // set the text color to black
    item->setBackground(color_undefined);
    matrix_widget_->setItem(i, j, item);
  };

  for (size_t i = 0; i < modules_.size(); i++) {
    const auto & module = modules_[i];

    // Check if module exists in param_names_
    if (param_names_.find(module) == param_names_.end()) {
      RCLCPP_WARN_STREAM(node_->get_logger(), module << " is not in the param names");
      continue;
    }

    const auto & module_params = param_names_.at(module);
    auto parameters_client =
      std::make_shared<rclcpp::SyncParametersClient>(node_, module_params.node);
    if (!parameters_client->wait_for_service(std::chrono::microseconds(500))) {
      RCLCPP_WARN_STREAM(
        node_->get_logger(), "Failed to find parameter service for node: " << module_params.node);
      for (size_t j = 0; j < targets_.size(); j++) {
        set_undefined(i, j);
      }
      continue;
    }

    for (size_t j = 0; j < targets_.size(); j++) {
      const auto & target = targets_[j];

      // Check if target exists in module's name map
      if (module_params.name.find(target) == module_params.name.end()) {
        RCLCPP_WARN_STREAM(
          node_->get_logger(), target << " parameter is not set in the " << module);
        continue;
      }

      std::string param_name = module_params.ns + "." + module_params.name.at(target);
      auto parameter_result = parameters_client->get_parameters({param_name});

      if (!parameter_result.empty()) {
        bool value = parameter_result[0].as_bool();
        QTableWidgetItem * item = new QTableWidgetItem(value ? "O" : "X");
        item->setForeground(QBrush(value ? Qt::black : Qt::black));  // set the text color to black
        item->setBackground(QBrush(value ? color_in_use : color_no_use));
        matrix_widget_->setItem(i, j, item);
      } else {
        RCLCPP_WARN_STREAM(
          node_->get_logger(),
          "Failed to get parameter " << module_params.node << " " << param_name);

        set_undefined(i, j);
      }
    }
  }
}

PLUGINLIB_EXPORT_CLASS(TargetObjectTypePanel, rviz_common::Panel)
