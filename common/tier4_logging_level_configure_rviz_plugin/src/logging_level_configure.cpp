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

#include <QLabel>
#include <rviz_common/display_context.hpp>
#include <tier4_logging_level_configure_rviz_plugin/logging_level_configure.hpp>

#include <cstdlib>

namespace rviz_plugin
{

LoggingLevelConfigureRvizPlugin::LoggingLevelConfigureRvizPlugin(QWidget * parent)
: rviz_common::Panel(parent)
{
}

// Calculate the maximum width among all target_module_name.
int LoggingLevelConfigureRvizPlugin::getMaxModuleNameWidth(QLabel * containerLabel)
{
  int max_width = 0;
  QFontMetrics metrics(containerLabel->font());
  for (const auto & item : logger_node_map_) {
    const auto & target_module_name = item.first;
    int width = metrics.horizontalAdvance(target_module_name);
    if (width > max_width) {
      max_width = width;
    }
  }
  return max_width;
}

// create container list in logger_node_map_ without
QStringList LoggingLevelConfigureRvizPlugin::getContainerList()
{
  QStringList containers;
  for (const auto & item : logger_node_map_) {
    const auto & container_logger_vec = item.second;
    for (const auto & container_logger_pair : container_logger_vec) {
      if (!containers.contains(container_logger_pair.first)) {
        containers.append(container_logger_pair.first);
      }
    }
  }
  return containers;
}

void LoggingLevelConfigureRvizPlugin::onInitialize()
{
  setLoggerNodeMap();

  attachLoggingComponent();

  QVBoxLayout * layout = new QVBoxLayout;

  QStringList levels = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

  QStringList loaded_container;
  constexpr int height = 20;
  for (const auto & item : logger_node_map_) {
    const auto & target_module_name = item.first;

    QHBoxLayout * hLayout = new QHBoxLayout;

    // Create a QLabel to display the container name.
    QLabel * containerLabel = new QLabel(target_module_name);
    containerLabel->setFixedHeight(height);  // Set fixed height for the button
    containerLabel->setFixedWidth(getMaxModuleNameWidth(containerLabel));

    hLayout->addWidget(containerLabel);  // Add the QLabel to the hLayout.

    QButtonGroup * group = new QButtonGroup(this);
    for (const QString & level : levels) {
      QPushButton * btn = new QPushButton(level);
      btn->setFixedHeight(height);  // Set fixed height for the button
      hLayout->addWidget(btn);      // Add each QPushButton to the hLayout.
      group->addButton(btn);
      button_map_[target_module_name][level] = btn;
      connect(btn, &QPushButton::clicked, this, [this, btn, target_module_name, level]() {
        this->onButtonClick(btn, target_module_name, level);
      });
    }
    // Set the "INFO" button as checked by default and change its color.
    updateButtonColors(target_module_name, button_map_[target_module_name]["INFO"]);

    buttonGroups_[target_module_name] = group;
    layout->addLayout(hLayout);
  }

  setLayout(layout);

  // set up service clients
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const auto & containers = getContainerList();
  for (const QString & container : containers) {
    const auto client = raw_node_->create_client<logging_demo::srv::ConfigLogger>(
      container.toStdString() + "/config_logger");
    client_map_[container] = client;
  }
}

void LoggingLevelConfigureRvizPlugin::attachLoggingComponent()
{
  const auto & containers = getContainerList();
  for (const auto & container_name : containers) {
    // Load the component for each container
    QString command = "ros2 component load --node-namespace " + container_name +
                      " --node-name logging_configure " + container_name +
                      " logging_demo logging_demo::LoggerConfig";
    int result = system(qPrintable(command));
    std::cerr << "load logger in " << container_name.toStdString() << ": result = " << result
              << std::endl;
  }
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

    for (const auto & container_logger_map : logger_node_map_[target_module_name]) {
      const auto container_node = container_logger_map.first;
      const auto logger_name = container_logger_map.second;
      const auto req = std::make_shared<logging_demo::srv::ConfigLogger::Request>();

      req->logger_name = logger_name.toStdString();
      req->level = level.toStdString();
      std::cerr << "logger level of " << req->logger_name << " is set to " << req->level
                << std::endl;
      client_map_[container_node]->async_send_request(req, callback);
    }

    updateButtonColors(
      target_module_name, button);  // Modify updateButtonColors to accept QPushButton only.
  }
}

void LoggingLevelConfigureRvizPlugin::updateButtonColors(
  const QString & target_module_name, QPushButton * active_button)
{
  const QString LIGHT_GREEN = "rgb(181, 255, 20)";
  const QString LIGHT_GRAY = "rgb(211, 211, 211)";
  const QString LIGHT_GRAY_TEXT = "rgb(180, 180, 180)";

  for (const auto & button : button_map_[target_module_name]) {
    if (button.second == active_button) {
      button.second->setStyleSheet("background-color: " + LIGHT_GREEN + "; color: black");
    } else {
      button.second->setStyleSheet(
        "background-color: " + LIGHT_GRAY + "; color: " + LIGHT_GRAY_TEXT);
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
  // ===============================================================================================
  // ====================================== Planning ===============================================
  // ===============================================================================================

  QString behavior_planning_container =
    "/planning/scenario_planning/lane_driving/behavior_planning/behavior_planning_container";
  QString motion_planning_container =
    "/planning/scenario_planning/lane_driving/motion_planning/motion_planning_container";

  // behavior_path_planner (all)
  logger_node_map_["behavior_path_planner"] = {
    {behavior_planning_container,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner"},
    {behavior_planning_container, "tier4_autoware_utils"}};

  // behavior_path_planner: avoidance
  logger_node_map_["behavior_path_planner: avoidance"] = {
    {behavior_planning_container,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_path_planner."
     "avoidance"}};

  // behavior_velocity_planner (all)
  logger_node_map_["behavior_velocity_planner"] = {
    {behavior_planning_container,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_velocity_planner"},
    {behavior_planning_container, "tier4_autoware_utils"}};

  // behavior_velocity_planner: intersection
  logger_node_map_["behavior_velocity_planner: intersection"] = {
    {behavior_planning_container,
     "planning.scenario_planning.lane_driving.behavior_planning.behavior_velocity_planner."
     "intersection"}};

  // obstacle_avoidance_planner
  logger_node_map_["motion: obstacle_avoidance"] = {
    {motion_planning_container,
     "planning.scenario_planning.lane_driving.motion_planning.obstacle_avoidance_planner"},
    {motion_planning_container, "tier4_autoware_utils"}};

  // motion_velocity_smoother
  QString container = "/planning/scenario_planning/motion_velocity_smoother_container";
  logger_node_map_["motion: velocity_smoother"] = {
    {container, "planning.scenario_planning.motion_velocity_smoother"},
    {container, "tier4_autoware_utils"}};

  // ===============================================================================================
  // ======================================= Control ===============================================
  // ===============================================================================================

  QString control_container = "/control/control_container";

  // lateral_controller
  logger_node_map_["lateral_controller"] = {
    {control_container, "control.trajectory_follower.controller_node_exe.lateral_controller"},
    {control_container, "tier4_autoware_utils"},
  };

  // longitudinal_controller
  logger_node_map_["longitudinal_controller"] = {
    {control_container, "control.trajectory_follower.controller_node_exe.longitudinal_controller"},
    {control_container, "tier4_autoware_utils"},
  };
}

}  // namespace rviz_plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::LoggingLevelConfigureRvizPlugin, rviz_common::Panel)
