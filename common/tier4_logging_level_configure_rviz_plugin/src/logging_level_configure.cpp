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

  QVBoxLayout * layout = new QVBoxLayout;

  QStringList levels = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

  constexpr int height = 20;
  for (const auto & item : node_logger_map_) {
    const auto & target_node_name = item.first;

    QHBoxLayout * hLayout = new QHBoxLayout;

    // Create a QLabel to display the node name.
    QLabel * label = new QLabel(target_node_name);
    label->setFixedHeight(height);  // Set fixed height for the button
    label->setFixedWidth(getMaxModuleNameWidth(label));

    hLayout->addWidget(label);  // Add the QLabel to the hLayout.

    QButtonGroup * group = new QButtonGroup(this);
    for (const QString & level : levels) {
      QPushButton * btn = new QPushButton(level);
      btn->setFixedHeight(height);  // Set fixed height for the button
      hLayout->addWidget(btn);      // Add each QPushButton to the hLayout.
      group->addButton(btn);
      button_map_[target_node_name][level] = btn;
      connect(btn, &QPushButton::clicked, this, [this, btn, target_node_name, level]() {
        this->onButtonClick(btn, target_node_name, level);
      });
    }
    // Set the "INFO" button as checked by default and change its color.
    updateButtonColors(target_node_name, button_map_[target_node_name]["INFO"], "INFO");

    buttonGroups_[target_node_name] = group;
    layout->addLayout(hLayout);
  }

  // Create a QWidget to hold the layout.
  QWidget * containerWidget = new QWidget;
  containerWidget->setLayout(layout);

  // Create a QScrollArea to make the layout scrollable.
  QScrollArea * scrollArea = new QScrollArea;
  scrollArea->setWidget(containerWidget);
  scrollArea->setWidgetResizable(true);

  // Set the QScrollArea as the layout of the main widget.
  QVBoxLayout * mainLayout = new QVBoxLayout;
  mainLayout->addWidget(scrollArea);
  setLayout(mainLayout);

  // set up service clients
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
  for (const auto & item : node_logger_map_) {
    const auto & target_module_name = item.first;
    max_width = std::max(metrics.horizontalAdvance(target_module_name), max_width);
  }
  return max_width;
}

// create node list in node_logger_map_ without
QStringList LoggingLevelConfigureRvizPlugin::getNodeList()
{
  QStringList nodes;
  for (const auto & item : node_logger_map_) {
    const auto & node_logger_vec = item.second;
    for (const auto & node_logger_pair : node_logger_vec) {
      if (!nodes.contains(node_logger_pair.first)) {
        nodes.append(node_logger_pair.first);
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

    for (const auto & node_logger_map : node_logger_map_[target_module_name]) {
      const auto node_name = node_logger_map.first;
      const auto logger_name = node_logger_map.second;
      const auto req = std::make_shared<logging_demo::srv::ConfigLogger::Request>();

      req->logger_name = logger_name.toStdString();
      req->level = level.toStdString();
      std::cerr << "logger level of " << req->logger_name << " is set to " << req->level
                << std::endl;
      client_map_[node_name]->async_send_request(req, callback);
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
    const auto key = QString::fromStdString(it->first.as<std::string>());
    const YAML::Node values = it->second;
    for (size_t i = 0; i < values.size(); i++) {
      const auto node_name = QString::fromStdString(values[i]["node_name"].as<std::string>());
      const auto logger_name = QString::fromStdString(values[i]["logger_name"].as<std::string>());
      node_logger_map_[key].push_back({node_name, logger_name});
    }
  }
}

}  // namespace rviz_plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::LoggingLevelConfigureRvizPlugin, rviz_common::Panel)
