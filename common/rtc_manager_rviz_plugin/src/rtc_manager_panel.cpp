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

#include "rtc_manager_panel.hpp"

#include <QHBoxLayout>
#include <QHeaderView>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

namespace rviz_plugins
{
inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }
inline bool uint2bool(uint8_t var) { return var == static_cast<uint8_t>(0) ? false : true; }
using std::placeholders::_1;
using std::placeholders::_2;

std::string getModuleName(const uint8_t module_type)
{
  switch (module_type) {
    case Module::LANE_CHANGE_LEFT: {
      return "lane_change_left";
    }
    case Module::LANE_CHANGE_RIGHT: {
      return "lane_change_right";
    }
    case Module::AVOIDANCE_LEFT: {
      return "avoidance_left";
    }
    case Module::AVOIDANCE_RIGHT: {
      return "avoidance_right";
    }
    case Module::PULL_OVER: {
      return "pull_over";
    }
    case Module::PULL_OUT: {
      return "pull_out";
    }
    case Module::TRAFFIC_LIGHT: {
      return "traffic_light";
    }
    case Module::INTERSECTION: {
      return "intersection";
    }
    case Module::CROSSWALK: {
      return "crosswalk";
    }
    case Module::BLIND_SPOT: {
      return "blind_spot";
    }
    case Module::DETECTION_AREA: {
      return "detection_area";
    }
    case Module::NO_STOPPING_AREA: {
      return "no_stopping_area";
    }
    case Module::OCCLUSION_SPOT: {
      return "occlusion_spot";
    }
  }
  return "NONE";
}

bool isPathChangeModule(const uint8_t module_type)
{
  if (
    module_type == Module::LANE_CHANGE_LEFT || module_type == Module::LANE_CHANGE_RIGHT ||
    module_type == Module::AVOIDANCE_LEFT || module_type == Module::AVOIDANCE_RIGHT ||
    module_type == Module::PULL_OVER || module_type == Module::PULL_OUT) {
    return true;
  }
  return false;
}

RTCManagerPanel::RTCManagerPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  // TODO(tanaka): replace this magic number to Module::SIZE
  const size_t module_size = 14;
  auto_modes_.reserve(module_size);
  auto * v_layout = new QVBoxLayout;
  auto vertical_header = new QHeaderView(Qt::Vertical);
  vertical_header->hide();
  auto horizontal_header = new QHeaderView(Qt::Horizontal);
  horizontal_header->setSectionResizeMode(QHeaderView::Stretch);
  auto_mode_table_ = new QTableWidget();
  auto_mode_table_->setColumnCount(4);
  auto_mode_table_->setHorizontalHeaderLabels(
    {"Module", "ToAutoMode", "ToManualMode", "ApprovalMode"});
  auto_mode_table_->setVerticalHeader(vertical_header);
  auto_mode_table_->setHorizontalHeader(horizontal_header);
  const size_t num_modules = module_size;
  auto_mode_table_->setRowCount(num_modules);
  for (size_t i = 0; i < num_modules; i++) {
    auto * rtc_auto_mode = new RTCAutoMode();
    rtc_auto_mode->setParent(this);
    // module
    {
      const uint8_t module_type = static_cast<uint8_t>(i);
      rtc_auto_mode->module_name = getModuleName(module_type);
      std::string module_name = rtc_auto_mode->module_name;
      auto label = new QLabel(QString::fromStdString(module_name));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(module_name));
      if (isPathChangeModule(module_type))
        label->setStyleSheet(BG_PURPLE);
      else
        label->setStyleSheet(BG_ORANGE);
      auto_mode_table_->setCellWidget(i, 0, label);
    }
    // mode button
    {
      // auto mode button
      rtc_auto_mode->auto_module_button_ptr = new QPushButton("auto mode");
      rtc_auto_mode->auto_module_button_ptr->setCheckable(true);
      connect(
        rtc_auto_mode->auto_module_button_ptr, &QPushButton::clicked, rtc_auto_mode,
        &RTCAutoMode::onChangeToAutoMode);
      auto_mode_table_->setCellWidget(i, 1, rtc_auto_mode->auto_module_button_ptr);
      // manual mode button
      rtc_auto_mode->manual_module_button_ptr = new QPushButton("manual mode");
      rtc_auto_mode->manual_module_button_ptr->setCheckable(true);
      connect(
        rtc_auto_mode->manual_module_button_ptr, &QPushButton::clicked, rtc_auto_mode,
        &RTCAutoMode::onChangeToManualMode);
      auto_mode_table_->setCellWidget(i, 2, rtc_auto_mode->manual_module_button_ptr);
    }
    // current mode
    {
      QString mode = QString::fromStdString("INIT");
      rtc_auto_mode->auto_manual_mode_label = new QLabel(mode);
      rtc_auto_mode->auto_manual_mode_label->setAlignment(Qt::AlignCenter);
      rtc_auto_mode->auto_manual_mode_label->setText(mode);
      auto_mode_table_->setCellWidget(i, 3, rtc_auto_mode->auto_manual_mode_label);
    }
    auto_modes_.emplace_back(rtc_auto_mode);
  }
  v_layout->addWidget(auto_mode_table_);

  // lateral execution
  auto * exe_path_change_layout = new QHBoxLayout;
  {
    exec_path_change_button_ptr_ = new QPushButton("Execute Path Change");
    exec_path_change_button_ptr_->setCheckable(false);
    exec_path_change_button_ptr_->setStyleSheet(BG_PURPLE);
    connect(
      exec_path_change_button_ptr_, &QPushButton::clicked, this,
      &RTCManagerPanel::onClickExecutePathChange);
    exe_path_change_layout->addWidget(exec_path_change_button_ptr_);
    wait_path_change_button_ptr_ = new QPushButton("Wait Path Change");
    wait_path_change_button_ptr_->setCheckable(false);
    wait_path_change_button_ptr_->setStyleSheet(BG_PURPLE);
    connect(
      wait_path_change_button_ptr_, &QPushButton::clicked, this,
      &RTCManagerPanel::onClickWaitPathChange);
    exe_path_change_layout->addWidget(wait_path_change_button_ptr_);
  }
  v_layout->addLayout(exe_path_change_layout);

  // longitudinal execution
  auto * exe_vel_change_layout = new QHBoxLayout;
  {
    exec_vel_change_button_ptr_ = new QPushButton("Execute Velocity Change");
    exec_vel_change_button_ptr_->setCheckable(false);
    exec_vel_change_button_ptr_->setStyleSheet(BG_ORANGE);
    connect(
      exec_vel_change_button_ptr_, &QPushButton::clicked, this,
      &RTCManagerPanel::onClickExecuteVelChange);
    exe_vel_change_layout->addWidget(exec_vel_change_button_ptr_);
    wait_vel_change_button_ptr_ = new QPushButton("Wait Velocity Change");
    wait_vel_change_button_ptr_->setCheckable(false);
    wait_vel_change_button_ptr_->setStyleSheet(BG_ORANGE);
    connect(
      wait_vel_change_button_ptr_, &QPushButton::clicked, this,
      &RTCManagerPanel::onClickWaitVelChange);
    exe_vel_change_layout->addWidget(wait_vel_change_button_ptr_);
  }
  v_layout->addLayout(exe_vel_change_layout);

  // execution
  auto * rtc_exe_layout = new QHBoxLayout;
  {
    exec_button_ptr_ = new QPushButton("Execute All");
    exec_button_ptr_->setCheckable(false);
    connect(exec_button_ptr_, &QPushButton::clicked, this, &RTCManagerPanel::onClickExecution);
    rtc_exe_layout->addWidget(exec_button_ptr_);
    wait_button_ptr_ = new QPushButton("Wait All");
    wait_button_ptr_->setCheckable(false);
    connect(wait_button_ptr_, &QPushButton::clicked, this, &RTCManagerPanel::onClickWait);
    rtc_exe_layout->addWidget(wait_button_ptr_);
  }
  v_layout->addLayout(rtc_exe_layout);

  // statuses
  auto * rtc_table_layout = new QHBoxLayout;
  {
    auto vertical_header = new QHeaderView(Qt::Vertical);
    vertical_header->hide();
    auto horizontal_header = new QHeaderView(Qt::Horizontal);
    horizontal_header->setSectionResizeMode(QHeaderView::Stretch);
    rtc_table_ = new QTableWidget();
    rtc_table_->setColumnCount(column_size_);
    rtc_table_->setHorizontalHeaderLabels(
      {"ID", "Module", "AW Safe", "Received Cmd", "AutoMode", "StartDistance", "FinishDistance"});
    rtc_table_->setVerticalHeader(vertical_header);
    rtc_table_->setHorizontalHeader(horizontal_header);
    rtc_table_layout->addWidget(rtc_table_);
    v_layout->addLayout(rtc_table_layout);
  }
  setLayout(v_layout);
}

void RTCManagerPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  client_rtc_commands_ = raw_node_->create_client<CooperateCommands>(
    "/api/external/set/rtc_commands", rmw_qos_profile_services_default);

  for (size_t i = 0; i < auto_modes_.size(); i++) {
    auto & a = auto_modes_.at(i);
    // auto mode
    a->enable_auto_mode_cli = raw_node_->create_client<AutoMode>(
      enable_auto_mode_namespace_ + "/" + a->module_name, rmw_qos_profile_services_default);
  }

  sub_rtc_status_ = raw_node_->create_subscription<CooperateStatusArray>(
    "/api/external/get/rtc_status", 1, std::bind(&RTCManagerPanel::onRTCStatus, this, _1));
}

void RTCAutoMode::onChangeToAutoMode()
{
  AutoMode::Request::SharedPtr request = std::make_shared<AutoMode::Request>();
  request->enable = true;
  enable_auto_mode_cli->async_send_request(request);
  auto_manual_mode_label->setText("AutoMode");
  auto_manual_mode_label->setStyleSheet(BG_BLUE);
  auto_module_button_ptr->setChecked(true);
  manual_module_button_ptr->setChecked(false);
}

void RTCAutoMode::onChangeToManualMode()
{
  AutoMode::Request::SharedPtr request = std::make_shared<AutoMode::Request>();
  request->enable = false;
  enable_auto_mode_cli->async_send_request(request);
  auto_manual_mode_label->setText("ManualMode");
  auto_manual_mode_label->setStyleSheet(BG_YELLOW);
  manual_module_button_ptr->setChecked(true);
  auto_module_button_ptr->setChecked(false);
}

CooperateCommand setRTCCommandFromStatus(CooperateStatus & status)
{
  CooperateCommand cooperate_command;
  cooperate_command.uuid = status.uuid;
  cooperate_command.module = status.module;
  cooperate_command.command = status.command_status;
  return cooperate_command;
}

void RTCManagerPanel::onClickChangeRequest(const bool is_path_change, const uint8_t command)
{
  if (!cooperate_statuses_ptr_) return;
  if (cooperate_statuses_ptr_->statuses.empty()) return;
  auto executable_cooperate_commands_request = std::make_shared<CooperateCommands::Request>();
  executable_cooperate_commands_request->stamp = cooperate_statuses_ptr_->stamp;
  // send coop request
  for (auto status : cooperate_statuses_ptr_->statuses) {
    if (is_path_change ^ isPathChangeModule(status.module.type)) continue;
    CooperateCommand cooperate_command = setRTCCommandFromStatus(status);
    cooperate_command.command.type = command;
    executable_cooperate_commands_request->commands.emplace_back(cooperate_command);
    //  To consider needs to change path step by step
    if (is_path_change && !status.auto_mode && status.command_status.type ^ command) {
      break;
    }
  }
  client_rtc_commands_->async_send_request(executable_cooperate_commands_request);
}

void RTCManagerPanel::onClickCommandRequest(const uint8_t command)
{
  if (!cooperate_statuses_ptr_) return;
  if (cooperate_statuses_ptr_->statuses.empty()) return;
  auto executable_cooperate_commands_request = std::make_shared<CooperateCommands::Request>();
  executable_cooperate_commands_request->stamp = cooperate_statuses_ptr_->stamp;
  // send coop request
  for (auto status : cooperate_statuses_ptr_->statuses) {
    CooperateCommand cooperate_command = setRTCCommandFromStatus(status);
    cooperate_command.command.type = command;
    executable_cooperate_commands_request->commands.emplace_back(cooperate_command);
  }
  client_rtc_commands_->async_send_request(executable_cooperate_commands_request);
}

void RTCManagerPanel::onClickExecuteVelChange() { onClickChangeRequest(false, Command::ACTIVATE); }
void RTCManagerPanel::onClickWaitVelChange() { onClickChangeRequest(false, Command::DEACTIVATE); }
void RTCManagerPanel::onClickExecutePathChange() { onClickChangeRequest(true, Command::ACTIVATE); }
void RTCManagerPanel::onClickWaitPathChange() { onClickChangeRequest(true, Command::DEACTIVATE); }
void RTCManagerPanel::onClickExecution() { onClickCommandRequest(Command::ACTIVATE); }
void RTCManagerPanel::onClickWait() { onClickCommandRequest(Command::DEACTIVATE); }

void RTCManagerPanel::onRTCStatus(const CooperateStatusArray::ConstSharedPtr msg)
{
  cooperate_statuses_ptr_ = std::make_shared<CooperateStatusArray>(*msg);
  rtc_table_->clearContents();
  if (msg->statuses.empty()) return;
  // this is to stable rtc display not to occupy too much
  size_t min_display_size{5};
  size_t max_display_size{10};
  // rtc messages are already sorted by distance
  rtc_table_->setRowCount(
    std::max(min_display_size, std::min(msg->statuses.size(), max_display_size)));
  int cnt = 0;
  for (auto status : msg->statuses) {
    if (static_cast<size_t>(cnt) >= max_display_size) return;
    // uuid
    {
      std::stringstream uuid;
      uuid << std::setw(4) << std::setfill('0') << static_cast<int>(status.uuid.uuid.at(0));
      auto label = new QLabel(QString::fromStdString(uuid.str()));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(uuid.str()));
      rtc_table_->setCellWidget(cnt, 0, label);
    }

    // module name
    {
      std::string module_name = getModuleName(status.module.type);
      auto label = new QLabel(QString::fromStdString(module_name));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(module_name));
      rtc_table_->setCellWidget(cnt, 1, label);
    }

    // is aw safe
    bool is_aw_safe = status.safe;
    {
      std::string is_safe = Bool2String(is_aw_safe);
      auto label = new QLabel(QString::fromStdString(is_safe));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(is_safe));
      rtc_table_->setCellWidget(cnt, 2, label);
    }

    // is operator safe
    const bool is_execute = uint2bool(status.command_status.type);
    {
      std::string text = is_execute ? "EXECUTE" : "WAIT";
      if (status.auto_mode) text = "NONE";
      auto label = new QLabel(QString::fromStdString(text));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(text));
      rtc_table_->setCellWidget(cnt, 3, label);
    }

    // is auto mode
    const bool is_rtc_auto_mode = status.auto_mode;
    {
      std::string is_auto_mode = Bool2String(is_rtc_auto_mode);
      auto label = new QLabel(QString::fromStdString(is_auto_mode));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(is_auto_mode));
      rtc_table_->setCellWidget(cnt, 4, label);
    }

    // start distance
    {
      std::string start_distance = std::to_string(status.start_distance);
      auto label = new QLabel(QString::fromStdString(start_distance));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(start_distance));
      rtc_table_->setCellWidget(cnt, 5, label);
    }

    // finish distance
    {
      std::string finish_distance = std::to_string(status.finish_distance);
      auto label = new QLabel(QString::fromStdString(finish_distance));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(finish_distance));
      rtc_table_->setCellWidget(cnt, 6, label);
    }

    // add color for recognition
    if (is_rtc_auto_mode || (is_aw_safe && is_execute)) {
      rtc_table_->cellWidget(cnt, 1)->setStyleSheet(BG_GREEN);
    } else if (is_aw_safe || is_execute) {
      rtc_table_->cellWidget(cnt, 1)->setStyleSheet(BG_YELLOW);
    } else {
      rtc_table_->cellWidget(cnt, 1)->setStyleSheet(BG_RED);
    }
    cnt++;
  }
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::RTCManagerPanel, rviz_common::Panel)
