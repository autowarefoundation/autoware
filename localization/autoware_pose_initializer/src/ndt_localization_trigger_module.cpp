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

#include "ndt_localization_trigger_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>
#include <string>

namespace autoware::pose_initializer
{
using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;

NdtLocalizationTriggerModule::NdtLocalizationTriggerModule(rclcpp::Node * node) : node_(node)
{
  client_ndt_trigger_ = node_->create_client<SetBool>("ndt_trigger_node");
}

void NdtLocalizationTriggerModule::wait_for_service()
{
  while (!client_ndt_trigger_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(node_->get_logger(), "NDT triggering service is not available, waiting...");
  }
  RCLCPP_INFO(node_->get_logger(), "NDT triggering service is available!");
}

void NdtLocalizationTriggerModule::send_request(bool flag, bool need_spin) const
{
  const auto req = std::make_shared<SetBool::Request>();
  std::string command_name;
  req->data = flag;
  if (flag) {
    command_name = "Activation";
  } else {
    command_name = "Deactivation";
  }

  if (!client_ndt_trigger_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("NDT triggering service is not ready");
  }

  auto future_ndt = client_ndt_trigger_->async_send_request(req);

  if (need_spin) {
    rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_ndt);
  }

  if (future_ndt.get()->success) {
    RCLCPP_INFO(node_->get_logger(), "NDT %s succeeded", command_name.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "NDT %s failed", command_name.c_str());
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "NDT " + command_name + " failed");
  }
}
}  // namespace autoware::pose_initializer
