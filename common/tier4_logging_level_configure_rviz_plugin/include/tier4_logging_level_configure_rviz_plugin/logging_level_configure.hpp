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

#ifndef TIER4_LOGGING_LEVEL_CONFIGURE_RVIZ_PLUGIN__LOGGING_LEVEL_CONFIGURE_HPP_
#define TIER4_LOGGING_LEVEL_CONFIGURE_RVIZ_PLUGIN__LOGGING_LEVEL_CONFIGURE_HPP_

#include "logging_demo/srv/config_logger.hpp"

#include <QButtonGroup>
#include <QHBoxLayout>
#include <QLabel>
#include <QMap>
#include <QPushButton>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rviz_plugin
{
struct LoggerInfo
{
  QString node_name;
  QString logger_name;
};
struct ButtonInfo
{
  QString button_name;
  std::vector<LoggerInfo> logger_info_vec;
};
struct LoggerNamespaceInfo
{
  QString ns;  // group namespace
  std::vector<ButtonInfo> button_info_vec;
};
class LoggingLevelConfigureRvizPlugin : public rviz_common::Panel
{
  Q_OBJECT  // This macro is needed for Qt to handle slots and signals

    public : LoggingLevelConfigureRvizPlugin(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  QMap<QString, QButtonGroup *> buttonGroups_;
  rclcpp::Node::SharedPtr raw_node_;

  std::vector<LoggerNamespaceInfo> display_info_vec_;

  // client_map_[node_name] = service_client
  std::unordered_map<QString, rclcpp::Client<logging_demo::srv::ConfigLogger>::SharedPtr>
    client_map_;

  // button_map_[button_name][logging_level] = Q_button_pointer
  std::unordered_map<QString, std::unordered_map<QString, QPushButton *>> button_map_;

  QStringList getNodeList();
  int getMaxModuleNameWidth(QLabel * containerLabel);
  void setLoggerNodeMap();

  ButtonInfo getButtonInfoFromNamespace(const QString & ns);
  std::vector<LoggerInfo> getNodeLoggerNameFromButtonName(const QString button_name);

private Q_SLOTS:
  void onButtonClick(QPushButton * button, const QString & name, const QString & level);
  void updateButtonColors(
    const QString & target_module_name, QPushButton * active_button, const QString & level);
  void changeLogLevel(const QString & container, const QString & level);
};

}  // namespace rviz_plugin

#endif  // TIER4_LOGGING_LEVEL_CONFIGURE_RVIZ_PLUGIN__LOGGING_LEVEL_CONFIGURE_HPP_
