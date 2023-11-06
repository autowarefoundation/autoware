// Copyright 2023 TIER IV, Inc.
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

#include "yaml-cpp/yaml.h"

#include <QGroupBox>
#include <QLabel>
#include <QScrollArea>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rviz_common/display_context.hpp>
#include <tier4_logging_level_configure_rviz_plugin/logging_level_configure.hpp>

#include <cstdlib>

namespace rviz_plugin
{

LoggingLevelConfigureRvizPlugin::LoggingLevelConfigureRvizPlugin(QWidget * parent)
: rviz_common::Panel(parent)
{
}

void LoggingLevelConfigureRvizPlugin::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  setLoggerNodeMap();

  QVBoxLayout * mainLayout = new QVBoxLayout;

  QStringList levels = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

  constexpr int height = 20;

  // Iterate over the namespaces
  for (const auto & ns_group_info : display_info_vec_) {
    // Create a group box for each namespace
    QGroupBox * groupBox = new QGroupBox(ns_group_info.ns);
    QVBoxLayout * groupLayout = new QVBoxLayout;

    // Iterate over the node/logger pairs within this namespace
    for (const auto & button_info : ns_group_info.button_info_vec) {
      const auto & button_name = button_info.button_name;

      QHBoxLayout * hLayout = new QHBoxLayout;

      // Create a QLabel to display the node name
      QLabel * label = new QLabel(button_name);
      label->setFixedHeight(height);
      label->setFixedWidth(getMaxModuleNameWidth(label));

      hLayout->addWidget(label);

      // Create a button group for each node
      QButtonGroup * buttonGroup = new QButtonGroup(this);

      // Create buttons for each logging level
      for (const QString & level : levels) {
        QPushButton * button = new QPushButton(level);
        button->setFixedHeight(height);
        hLayout->addWidget(button);
        buttonGroup->addButton(button);
        button_map_[button_name][level] = button;
        connect(button, &QPushButton::clicked, this, [this, button, button_name, level]() {
          this->onButtonClick(button, button_name, level);
        });
      }

      // Set the "INFO" button as checked by default and change its color
      updateButtonColors(button_name, button_map_[button_name]["INFO"], "INFO");

      buttonGroups_[button_name] = buttonGroup;
      groupLayout->addLayout(hLayout);  // Add the horizontal layout to the group layout
    }

    groupBox->setLayout(groupLayout);  // Set the group layout to the group box
    mainLayout->addWidget(groupBox);   // Add the group box to the main layout
  }

  // Create a QWidget to hold the main layout
  QWidget * containerWidget = new QWidget;
  containerWidget->setLayout(mainLayout);

  // Create a QScrollArea to make the layout scrollable
  QScrollArea * scrollArea = new QScrollArea;
  scrollArea->setWidget(containerWidget);
  scrollArea->setWidgetResizable(true);

  // Set the QScrollArea as the layout of the main widget
  QVBoxLayout * scrollLayout = new QVBoxLayout;
  scrollLayout->addWidget(scrollArea);
  setLayout(scrollLayout);

  // Setup service clients
  const auto & nodes = getNodeList();
  for (const QString & node : nodes) {
    const auto client = raw_node_->create_client<logging_demo::srv::ConfigLogger>(
      node.toStdString() + "/config_logger");
    client_map_[node] = client;
  }
}

// Calculate the maximum width among all target_module_name.
int LoggingLevelConfigureRvizPlugin::getMaxModuleNameWidth(QLabel * label)
{
  int max_width = 0;
  QFontMetrics metrics(label->font());
  for (const auto & ns_info : display_info_vec_) {
    for (const auto & b : ns_info.button_info_vec) {
      max_width = std::max(metrics.horizontalAdvance(b.button_name), max_width);
    }
  }
  return max_width;
}

// create node list in node_logger_map_ without
QStringList LoggingLevelConfigureRvizPlugin::getNodeList()
{
  QStringList nodes;
  for (const auto & d : display_info_vec_) {
    for (const auto & b : d.button_info_vec) {
      for (const auto & info : b.logger_info_vec) {
        if (!nodes.contains(info.node_name)) {
          nodes.append(info.node_name);
        }
      }
    }
  }
  return nodes;
}

// Modify the signature of the onButtonClick function:
void LoggingLevelConfigureRvizPlugin::onButtonClick(
  QPushButton * button, const QString & target_module_name, const QString & level)
{
  if (button) {
    const auto callback =
      [&](rclcpp::Client<logging_demo::srv::ConfigLogger>::SharedFuture future) {
        std::cerr << "change logging level: "
                  << std::string(future.get()->success ? "success!" : "failed...") << std::endl;
      };

    const auto node_logger_vec = getNodeLoggerNameFromButtonName(target_module_name);
    for (const auto & data : node_logger_vec) {
      const auto req = std::make_shared<logging_demo::srv::ConfigLogger::Request>();

      req->logger_name = data.logger_name.toStdString();
      req->level = level.toStdString();
      std::cerr << "logger level of " << req->logger_name << " is set to " << req->level
                << std::endl;
      client_map_[data.node_name]->async_send_request(req, callback);
    }

    updateButtonColors(
      target_module_name, button, level);  // Modify updateButtonColors to accept QPushButton only.
  }
}

void LoggingLevelConfigureRvizPlugin::updateButtonColors(
  const QString & target_module_name, QPushButton * active_button, const QString & level)
{
  std::unordered_map<QString, QString> colorMap = {
    {"DEBUG", "rgb(181, 255, 20)"}, /* green */
    {"INFO", "rgb(200, 255, 255)"}, /* light blue */
    {"WARN", "rgb(255, 255, 0)"},   /* yellow */
    {"ERROR", "rgb(255, 0, 0)"},    /* red */
    {"FATAL", "rgb(139, 0, 0)"},    /* dark red */
    {"OFF", "rgb(211, 211, 211)"}   /* gray */
  };

  const QString LIGHT_GRAY_TEXT = "rgb(180, 180, 180)";

  const QString color = colorMap.count(level) ? colorMap[level] : colorMap["OFF"];

  for (const auto & button : button_map_[target_module_name]) {
    if (button.second == active_button) {
      button.second->setStyleSheet("background-color: " + color + "; color: black");
    } else {
      button.second->setStyleSheet(
        "background-color: " + colorMap["OFF"] + "; color: " + LIGHT_GRAY_TEXT);
    }
  }
}
void LoggingLevelConfigureRvizPlugin::save(rviz_common::Config config) const
{
  Panel::save(config);
}

void LoggingLevelConfigureRvizPlugin::load(const rviz_common::Config & config)
{
  Panel::load(config);
}

void LoggingLevelConfigureRvizPlugin::setLoggerNodeMap()
{
  const std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("tier4_logging_level_configure_rviz_plugin");
  const std::string default_config_path = package_share_directory + "/config/logger_config.yaml";

  const auto filename =
    raw_node_->declare_parameter<std::string>("config_filename", default_config_path);
  RCLCPP_INFO(raw_node_->get_logger(), "load config file: %s", filename.c_str());

  YAML::Node config = YAML::LoadFile(filename);

  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
    const auto ns = QString::fromStdString(it->first.as<std::string>());
    const YAML::Node ns_config = it->second;

    LoggerNamespaceInfo display_data;
    display_data.ns = ns;

    for (YAML::const_iterator ns_it = ns_config.begin(); ns_it != ns_config.end(); ++ns_it) {
      const auto key = QString::fromStdString(ns_it->first.as<std::string>());
      ButtonInfo button_data;
      button_data.button_name = key;
      const YAML::Node values = ns_it->second;
      for (size_t i = 0; i < values.size(); i++) {
        LoggerInfo data;
        data.node_name = QString::fromStdString(values[i]["node_name"].as<std::string>());
        data.logger_name = QString::fromStdString(values[i]["logger_name"].as<std::string>());
        button_data.logger_info_vec.push_back(data);
      }
      display_data.button_info_vec.push_back(button_data);
    }
    display_info_vec_.push_back(display_data);
  }
}

std::vector<LoggerInfo> LoggingLevelConfigureRvizPlugin::getNodeLoggerNameFromButtonName(
  const QString button_name)
{
  for (const auto & ns_level : display_info_vec_) {
    for (const auto & button : ns_level.button_info_vec) {
      if (button.button_name == button_name) {
        return button.logger_info_vec;
      }
    }
  }
  RCLCPP_ERROR(
    raw_node_->get_logger(), "Failed to find target name: %s", button_name.toStdString().c_str());
  return {};
}

}  // namespace rviz_plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::LoggingLevelConfigureRvizPlugin, rviz_common::Panel)
