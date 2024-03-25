// Copyright 2024 Autoware Foundation
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

#ifndef STUB_TRIGGER_NODE_CLIENT_HPP_
#define STUB_TRIGGER_NODE_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

#include <memory>

class StubTriggerNodeClient : public rclcpp::Node
{
  using SetBool = std_srvs::srv::SetBool;

public:
  StubTriggerNodeClient() : Node("stub_trigger_node_client")
  {
    align_service_client_ = this->create_client<SetBool>("/trigger_node_srv");
  }

  bool send_trigger_node(const bool & trigger)
  {
    // wait for the service to be available
    while (!align_service_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        throw std::runtime_error("Interrupted while waiting for the service.");
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for service...");
    }

    // send the request
    std::shared_ptr<SetBool::Request> request = std::make_shared<SetBool::Request>();
    request->data = trigger;
    auto result = align_service_client_->async_send_request(request);
    if (
      rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      throw std::runtime_error("Service call failed.");
    }
    return result.get()->success;
  }

private:
  rclcpp::Client<SetBool>::SharedPtr align_service_client_;
};

#endif  // STUB_TRIGGER_NODE_CLIENT_HPP_
