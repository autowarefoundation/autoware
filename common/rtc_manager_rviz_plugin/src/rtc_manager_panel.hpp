//
//  Copyright 2020 Tier IV, Inc. All rights reserved.
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
//

#ifndef RTC_MANAGER_PANEL_HPP_
#define RTC_MANAGER_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QString>
#include <QTableWidget>

#ifndef Q_MOC_RUN
// cpp
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
// ros
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>
// autoware
#include <tier4_rtc_msgs/msg/command.hpp>
#include <tier4_rtc_msgs/msg/cooperate_command.hpp>
#include <tier4_rtc_msgs/msg/cooperate_response.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status.hpp>
#include <tier4_rtc_msgs/msg/cooperate_status_array.hpp>
#include <tier4_rtc_msgs/msg/module.hpp>
#include <tier4_rtc_msgs/srv/auto_mode.hpp>
#include <tier4_rtc_msgs/srv/cooperate_commands.hpp>
#endif

namespace rviz_plugins
{
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::srv::AutoMode;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;

static const QString BG_BLUE = "background-color: #3dffff;";
static const QString BG_YELLOW = "background-color: #ffff3d;";
static const QString BG_PURPLE = "background-color: #9e3dff;";
static const QString BG_ORANGE = "background-color: #ff7f00;";
static const QString BG_GREEN = "background-color: #3dff3d;";
static const QString BG_RED = "background-color: #ff3d3d;";

struct RTCAutoMode : public QObject
{
  Q_OBJECT

public Q_SLOTS:
  void onChangeToAutoMode();
  void onChangeToManualMode();

public:
  std::string module_name;
  QPushButton * auto_module_button_ptr;
  QPushButton * manual_module_button_ptr;
  QLabel * auto_manual_mode_label;
  rclcpp::Client<AutoMode>::SharedPtr enable_auto_mode_cli;
};

class RTCManagerPanel : public rviz_common::Panel
{
  Q_OBJECT
public Q_SLOTS:
  void onClickExecution();
  void onClickWait();
  void onClickVelocityChangeRequest();
  void onClickExecutePathChange();
  void onClickWaitPathChange();
  void onClickExecuteVelChange();
  void onClickWaitVelChange();

public:
  explicit RTCManagerPanel(QWidget * parent = nullptr);
  void onInitialize() override;

protected:
  void onRTCStatus(const CooperateStatusArray::ConstSharedPtr msg);
  void onEnableService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response) const;
  void onClickCommandRequest(const uint8_t command);
  void onClickChangeRequest(const bool is_path_change, const uint8_t command);

private:
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<CooperateStatusArray>::SharedPtr sub_rtc_status_;
  rclcpp::Client<CooperateCommands>::SharedPtr client_rtc_commands_;
  rclcpp::Client<AutoMode>::SharedPtr enable_auto_mode_cli_;
  std::vector<RTCAutoMode *> auto_modes_;

  std::shared_ptr<CooperateStatusArray> cooperate_statuses_ptr_;
  QTableWidget * rtc_table_;
  QTableWidget * auto_mode_table_;
  QPushButton * path_change_button_ptr_ = {nullptr};
  QPushButton * velocity_change_button_ptr_ = {nullptr};
  QPushButton * exec_path_change_button_ptr_ = {nullptr};
  QPushButton * wait_path_change_button_ptr_ = {nullptr};
  QPushButton * exec_vel_change_button_ptr_ = {nullptr};
  QPushButton * wait_vel_change_button_ptr_ = {nullptr};
  QPushButton * exec_button_ptr_ = {nullptr};
  QPushButton * wait_button_ptr_ = {nullptr};
  QLabel * num_rtc_status_ptr_ = {nullptr};

  size_t column_size_ = {7};
  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";
};

}  // namespace rviz_plugins

#endif  // RTC_MANAGER_PANEL_HPP_
