// Copyright 2023 The Autoware Contributors
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

#include "dummy_doors.hpp"

#include <memory>

namespace vehicle_door_simulator
{

DummyDoors::DummyDoors() : Node("dummy_doors")
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_command_ = create_service<SetDoorCommand>(
    "~/doors/command", std::bind(&DummyDoors::on_command, this, _1, _2));
  srv_layout_ = create_service<GetDoorLayout>(
    "~/doors/layout", std::bind(&DummyDoors::on_layout, this, _1, _2));

  pub_status_ = create_publisher<DoorStatusArray>("~/doors/status", rclcpp::QoS(1));

  const auto period = rclcpp::Rate(5.0).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void DummyDoors::on_command(
  const SetDoorCommand::Request::SharedPtr req, const SetDoorCommand::Response::SharedPtr res)
{
  using autoware_adapi_v1_msgs::msg::DoorCommand;

  for (const auto & door : req->doors) {
    if (statuses_.size() <= door.index) {
      res->status.success = false;
      res->status.message = "invalid door index";
      return;
    }
    if (door.command != DoorCommand::CLOSE && door.command != DoorCommand::OPEN) {
      res->status.success = false;
      res->status.message = "invalid door command";
      return;
    }
  }

  for (const auto & door : req->doors) {
    if (door.command == DoorCommand::CLOSE) {
      statuses_[door.index].status = DoorStatus::CLOSING;
    }
    if (door.command == DoorCommand::OPEN) {
      statuses_[door.index].status = DoorStatus::OPENING;
    }
  }

  res->status.success = true;
}

void DummyDoors::on_layout(
  const GetDoorLayout::Request::SharedPtr, const GetDoorLayout::Response::SharedPtr res)
{
  using autoware_adapi_v1_msgs::msg::DoorLayout;

  res->doors.resize(statuses_.size());
  res->doors[0].description = "dummy door 1";
  res->doors[1].description = "dummy door 2";
  res->doors[2].description = "dummy door 3";
  res->doors[3].description = "dummy door 4";
  res->doors[0].roles = {};
  res->doors[1].roles = {DoorLayout::GET_ON};
  res->doors[2].roles = {DoorLayout::GET_ON, DoorLayout::GET_OFF};
  res->doors[3].roles = {DoorLayout::GET_OFF};
  res->status.success = true;
}

void DummyDoors::on_timer()
{
  using autoware_adapi_v1_msgs::msg::DoorStatus;

  for (auto & status : statuses_) {
    if (status.status == DoorStatus::CLOSING) {
      if (--status.progress <= 0) {
        status.progress = 0;
        status.status = DoorStatus::CLOSED;
      }
    }
    if (status.status == DoorStatus::OPENING) {
      if (max_progress <= ++status.progress) {
        status.progress = max_progress;
        status.status = DoorStatus::OPENED;
      }
    }
  }

  DoorStatusArray message;
  message.stamp = now();
  for (const auto & status : statuses_) {
    DoorStatus door;
    door.status = status.status;
    message.doors.push_back(door);
  }
  pub_status_->publish(message);
}

}  // namespace vehicle_door_simulator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<vehicle_door_simulator::DummyDoors>();
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
}
