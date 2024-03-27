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

#include "door_panel.hpp"

#include <rviz_common/display_context.hpp>

#include <memory>

namespace tier4_adapi_rviz_plugins
{

DoorPanel::DoorPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  status_ = new QLabel("Trying to get door layout.");
  layout_ = new QGridLayout();
  layout_->addWidget(status_, 0, 0, 1, 4);
  setLayout(layout_);
}

void DoorPanel::onInitialize()
{
  const auto on_layout = [this](const rclcpp::Client<DoorLayout::Service>::SharedFuture future) {
    const auto res = future.get();
    if (!res->status.success) {
      status_->setText(QString::fromStdString("failed to get layout: " + res->status.message));
      return;
    }
    const auto & layouts = res->doors;
    for (size_t index = 0; index < layouts.size(); ++index) {
      const auto & layout = layouts[index];
      DoorUI door;
      door.description = new QLabel(QString::fromStdString(layout.description));
      door.status = new QLabel("unknown");
      door.open = new QPushButton("open");
      door.close = new QPushButton("close");
      doors_.push_back(door);

      layout_->addWidget(door.description, index + 1, 0);
      layout_->addWidget(door.status, index + 1, 1);
      layout_->addWidget(door.open, index + 1, 2);
      layout_->addWidget(door.close, index + 1, 3);

      using Command = autoware_adapi_v1_msgs::msg::DoorCommand;
      const auto on_open = [this, index] { on_button(index, Command::OPEN); };
      const auto on_close = [this, index] { on_button(index, Command::CLOSE); };
      connect(door.open, &QPushButton::clicked, on_open);
      connect(door.close, &QPushButton::clicked, on_close);
    }
    status_->hide();
  };

  const auto on_status = [this](const DoorStatus::Message::ConstSharedPtr msg) {
    using Status = autoware_adapi_v1_msgs::msg::DoorStatus;
    if (doors_.size() != msg->doors.size()) {
      return;
    }
    for (size_t index = 0; index < doors_.size(); ++index) {
      const auto & label = doors_[index].status;
      switch (msg->doors[index].status) {
        case Status::NOT_AVAILABLE:
          label->setText("not available");
          break;
        case Status::OPENED:
          label->setText("opened");
          break;
        case Status::CLOSED:
          label->setText("closed");
          break;
        case Status::OPENING:
          label->setText("opening");
          break;
        case Status::CLOSING:
          label->setText("closing");
          break;
        default:
          label->setText("unknown");
          break;
      }
    }
  };

  auto lock = getDisplayContext()->getRosNodeAbstraction().lock();
  auto node = lock->get_raw_node();

  const auto adaptor = component_interface_utils::NodeAdaptor(node.get());
  adaptor.init_cli(cli_command_);
  adaptor.init_cli(cli_layout_);
  adaptor.init_sub(sub_status_, on_status);

  const auto req = std::make_shared<DoorLayout::Service::Request>();
  cli_layout_->async_send_request(req, on_layout);
}

void DoorPanel::on_button(uint32_t index, uint8_t command)
{
  const auto req = std::make_shared<DoorCommand::Service::Request>();
  req->doors.resize(1);
  req->doors.back().index = index;
  req->doors.back().command = command;
  cli_command_->async_send_request(req);
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::DoorPanel, rviz_common::Panel)
