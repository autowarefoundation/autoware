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
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>

#include <unique_identifier_msgs/msg/uuid.hpp>

namespace rviz_plugins
{
inline std::string Bool2String(const bool var) { return var ? "True" : "False"; }
using std::placeholders::_1;
using std::placeholders::_2;
Module getModuleType(const std::string & module_name)
{
  Module module;
  if (module_name == "blind_spot") {
    module.type = Module::BLIND_SPOT;
  } else if (module_name == "crosswalk") {
    module.type = Module::CROSSWALK;
  } else if (module_name == "detection_area") {
    module.type = Module::DETECTION_AREA;
  } else if (module_name == "intersection") {
    module.type = Module::INTERSECTION;
  } else if (module_name == "no_stopping_area") {
    module.type = Module::NO_STOPPING_AREA;
  } else if (module_name == "occlusion_spot") {
    module.type = Module::OCCLUSION_SPOT;
  } else if (module_name == "stop_line") {
    module.type = Module::NONE;
  } else if (module_name == "traffic_light") {
    module.type = Module::TRAFFIC_LIGHT;
  } else if (module_name == "virtual_traffic_light") {
    module.type = Module::TRAFFIC_LIGHT;
  } else if (module_name == "lane_change_left") {
    module.type = Module::LANE_CHANGE_LEFT;
  } else if (module_name == "lane_change_right") {
    module.type = Module::LANE_CHANGE_RIGHT;
  } else if (module_name == "avoidance_left") {
    module.type = Module::AVOIDANCE_LEFT;
  } else if (module_name == "avoidance_right") {
    module.type = Module::AVOIDANCE_RIGHT;
  } else if (module_name == "pull_over") {
    module.type = Module::PULL_OVER;
  } else if (module_name == "pull_out") {
    module.type = Module::PULL_OUT;
  }
  return module;
}

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
      rtc_auto_mode->module_name = getModuleName(static_cast<uint8_t>(i));
      std::string module_name = rtc_auto_mode->module_name;
      auto label = new QLabel(QString::fromStdString(module_name));
      label->setAlignment(Qt::AlignCenter);
      label->setText(QString::fromStdString(module_name));
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

  sub_rtc_status_ = raw_node_->create_subscription<CooperateStatusArray>(
    "/api/external/get/rtc_status", 1, std::bind(&RTCManagerPanel::onRTCStatus, this, _1));

  client_rtc_commands_ = raw_node_->create_client<CooperateCommands>(
    "/api/external/set/rtc_commands", rmw_qos_profile_services_default);

  for (size_t i = 0; i < auto_modes_.size(); i++) {
    auto & a = auto_modes_.at(i);
    // auto mode
    a->enable_auto_mode_cli = raw_node_->create_client<AutoMode>(
      enable_auto_mode_namespace_ + "/" + a->module_name, rmw_qos_profile_services_default);
  }
}

void RTCAutoMode::onChangeToAutoMode()
{
  AutoMode::Request::SharedPtr request = std::make_shared<AutoMode::Request>();
  request->enable = true;
  enable_auto_mode_cli->async_send_request(request);
  auto_manual_mode_label->setText("AutoMode");
  auto_manual_mode_label->setStyleSheet("background-color: #00FFFF;");
  auto_module_button_ptr->setChecked(true);
  manual_module_button_ptr->setChecked(false);
}

void RTCAutoMode::onChangeToManualMode()
{
  AutoMode::Request::SharedPtr request = std::make_shared<AutoMode::Request>();
  request->enable = false;
  enable_auto_mode_cli->async_send_request(request);
  auto_manual_mode_label->setText("ManualMode");
  auto_manual_mode_label->setStyleSheet("background-color: #FFFF00;");
  manual_module_button_ptr->setChecked(true);
  auto_module_button_ptr->setChecked(false);
}

void RTCManagerPanel::onRTCStatus(const CooperateStatusArray::ConstSharedPtr msg)
{
  rtc_table_->clearContents();
  if (msg->statuses.empty()) return;
  std::vector<CooperateStatus> coop_vec;
  std::copy(msg->statuses.cbegin(), msg->statuses.cend(), std::back_inserter(coop_vec));
  std::sort(
    coop_vec.begin(), coop_vec.end(), [](const CooperateStatus & c1, const CooperateStatus & c2) {
      return c1.start_distance < c2.start_distance;
    });
  // this is to stable rtc display not to occupy too much
  size_t min_display_size{5};
  size_t max_display_size{10};
  rtc_table_->setRowCount(std::max(min_display_size, std::min(coop_vec.size(), max_display_size)));
  int cnt = 0;
  for (auto status : msg->statuses) {
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
    const bool is_execute = status.command_status.type;
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
    for (size_t i = 0; i < column_size_; i++) {
      if (is_rtc_auto_mode || (is_aw_safe && is_execute)) {
        rtc_table_->cellWidget(cnt, i)->setStyleSheet("background-color: #00FF00;");
      } else if (is_aw_safe || is_execute) {
        rtc_table_->cellWidget(cnt, i)->setStyleSheet("background-color: #FFFF00;");
      } else {
        rtc_table_->cellWidget(cnt, i)->setStyleSheet("background-color: #FF0000;");
      }
    }

    cnt++;
  }
}
}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::RTCManagerPanel, rviz_common::Panel)
