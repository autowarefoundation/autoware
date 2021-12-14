// Copyright 2021 Tier IV, Inc.
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

#ifndef TIER4_API_UTILS__TYPES__RESPONSE_HPP_
#define TIER4_API_UTILS__TYPES__RESPONSE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_external_api_msgs/msg/response_status.hpp"

#include <string>

namespace tier4_api_utils
{
using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;

inline bool is_success(const tier4_external_api_msgs::msg::ResponseStatus & status)
{
  return status.code == tier4_external_api_msgs::msg::ResponseStatus::SUCCESS;
}

inline bool is_ignored(const tier4_external_api_msgs::msg::ResponseStatus & status)
{
  return status.code == tier4_external_api_msgs::msg::ResponseStatus::IGNORED;
}

inline bool is_warn(const tier4_external_api_msgs::msg::ResponseStatus & status)
{
  return status.code == tier4_external_api_msgs::msg::ResponseStatus::WARN;
}

inline bool is_error(const tier4_external_api_msgs::msg::ResponseStatus & status)
{
  return status.code == tier4_external_api_msgs::msg::ResponseStatus::ERROR;
}

inline ResponseStatus response_success(const std::string & message = "")
{
  return tier4_external_api_msgs::build<tier4_external_api_msgs::msg::ResponseStatus>()
    .code(tier4_external_api_msgs::msg::ResponseStatus::SUCCESS)
    .message(message);
}

inline ResponseStatus response_ignored(const std::string & message = "")
{
  return tier4_external_api_msgs::build<tier4_external_api_msgs::msg::ResponseStatus>()
    .code(tier4_external_api_msgs::msg::ResponseStatus::IGNORED)
    .message(message);
}

inline ResponseStatus response_warn(const std::string & message = "")
{
  return tier4_external_api_msgs::build<tier4_external_api_msgs::msg::ResponseStatus>()
    .code(tier4_external_api_msgs::msg::ResponseStatus::WARN)
    .message(message);
}

inline ResponseStatus response_error(const std::string & message = "")
{
  return tier4_external_api_msgs::build<tier4_external_api_msgs::msg::ResponseStatus>()
    .code(tier4_external_api_msgs::msg::ResponseStatus::ERROR)
    .message(message);
}

}  // namespace tier4_api_utils

#endif  // TIER4_API_UTILS__TYPES__RESPONSE_HPP_
