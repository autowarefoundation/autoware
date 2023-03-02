// Copyright 2022 The Autoware Contributors
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

#include "ekf_localization_trigger_module.hpp"

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp/exceptions.hpp>

#include <memory>
#include <string>

using ServiceException = component_interface_utils::ServiceException;
using Initialize = localization_interface::Initialize;

EkfLocalizationTriggerModule::EkfLocalizationTriggerModule(rclcpp::Node * node)
: logger_(node->get_logger())
{
  client_ekf_trigger_ = node->create_client<SetBool>("ekf_trigger_node");
}

void EkfLocalizationTriggerModule::send_request(bool flag) const
{
  const auto req = std::make_shared<SetBool::Request>();
  std::string command_name;
  req->data = flag;
  if (flag) {
    command_name = "Activation";
  } else {
    command_name = "Deactivation";
  }

  if (!client_ekf_trigger_->service_is_ready()) {
    throw component_interface_utils::ServiceUnready("EKF triggering service is not ready");
  }

  auto future_ekf = client_ekf_trigger_->async_send_request(req);

  if (future_ekf.get()->success) {
    RCLCPP_INFO(logger_, "EKF %s succeeded", command_name.c_str());
  } else {
    RCLCPP_INFO(logger_, "EKF %s failed", command_name.c_str());
    throw ServiceException(
      Initialize::Service::Response::ERROR_ESTIMATION, "EKF " + command_name + " failed");
  }
}
