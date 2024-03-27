//  Copyright 2023 The Autoware Contributors
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

#ifndef DOOR_PANEL_HPP_
#define DOOR_PANEL_HPP_

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <autoware_ad_api_specs/vehicle.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <vector>

namespace tier4_adapi_rviz_plugins
{

class DoorPanel : public rviz_common::Panel
{
  Q_OBJECT
  using DoorCommand = autoware_ad_api::vehicle::DoorCommand;
  using DoorLayout = autoware_ad_api::vehicle::DoorLayout;
  using DoorStatus = autoware_ad_api::vehicle::DoorStatus;

public:
  explicit DoorPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  component_interface_utils::Client<DoorCommand>::SharedPtr cli_command_;
  component_interface_utils::Client<DoorLayout>::SharedPtr cli_layout_;
  component_interface_utils::Subscription<DoorStatus>::SharedPtr sub_status_;

  struct DoorUI
  {
    QLabel * description;
    QLabel * status;
    QPushButton * open;
    QPushButton * close;
  };
  QGridLayout * layout_;
  QLabel * status_;
  std::vector<DoorUI> doors_;

private slots:
  void on_button(uint32_t index, uint8_t command);
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // DOOR_PANEL_HPP_
