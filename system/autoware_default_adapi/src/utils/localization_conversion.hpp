// Copyright 2024 TIER IV, Inc.
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

#ifndef UTILS__LOCALIZATION_CONVERSION_HPP_
#define UTILS__LOCALIZATION_CONVERSION_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/srv/initialize_localization.hpp>
#include <tier4_localization_msgs/srv/initialize_localization.hpp>

namespace autoware::default_adapi::localization_conversion
{

using ExternalInitializeRequest =
  autoware_adapi_v1_msgs::srv::InitializeLocalization::Request::SharedPtr;
using InternalInitializeRequest =
  tier4_localization_msgs::srv::InitializeLocalization::Request::SharedPtr;
InternalInitializeRequest convert_request(const ExternalInitializeRequest & external);

using ExternalResponse = autoware_adapi_v1_msgs::msg::ResponseStatus;
using InternalResponse = autoware_common_msgs::msg::ResponseStatus;
ExternalResponse convert_response(const InternalResponse & internal);

template <class ClientT, class RequestT>
ExternalResponse convert_call(ClientT & client, RequestT & req)
{
  return convert_response(client->call(convert_request(req))->status);
}

}  // namespace autoware::default_adapi::localization_conversion

#endif  // UTILS__LOCALIZATION_CONVERSION_HPP_
