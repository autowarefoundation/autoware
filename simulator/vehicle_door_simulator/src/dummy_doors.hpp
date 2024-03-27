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

#ifndef DUMMY_DOORS_HPP_
#define DUMMY_DOORS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/door_status_array.hpp>
#include <autoware_adapi_v1_msgs/srv/get_door_layout.hpp>
#include <autoware_adapi_v1_msgs/srv/set_door_command.hpp>

#include <array>

namespace vehicle_door_simulator
{

class DummyDoors : public rclcpp::Node
{
public:
  DummyDoors();

private:
  using SetDoorCommand = autoware_adapi_v1_msgs::srv::SetDoorCommand;
  using GetDoorLayout = autoware_adapi_v1_msgs::srv::GetDoorLayout;
  using DoorStatus = autoware_adapi_v1_msgs::msg::DoorStatus;
  using DoorStatusArray = autoware_adapi_v1_msgs::msg::DoorStatusArray;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<SetDoorCommand>::SharedPtr srv_command_;
  rclcpp::Service<GetDoorLayout>::SharedPtr srv_layout_;
  rclcpp::Publisher<DoorStatusArray>::SharedPtr pub_status_;

  void on_command(
    const SetDoorCommand::Request::SharedPtr req, const SetDoorCommand::Response::SharedPtr res);
  void on_layout(
    const GetDoorLayout::Request::SharedPtr req, const GetDoorLayout::Response::SharedPtr res);
  void on_timer();

  struct LocalStatus
  {
    using StatusType = DoorStatus::_status_type;
    int progress = 0;
    StatusType status = DoorStatus::CLOSED;
  };
  static constexpr int max_progress = 25;  // 5s * 5Hz
  std::array<LocalStatus, 4> statuses_;
};

}  // namespace vehicle_door_simulator

#endif  // DUMMY_DOORS_HPP_
