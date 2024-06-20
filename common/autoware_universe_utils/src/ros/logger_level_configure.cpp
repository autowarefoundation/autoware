// Copyright 2023 Tier IV, Inc.
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

#include "autoware/universe_utils/ros/logger_level_configure.hpp"

#include <rcutils/logging.h>

namespace autoware::universe_utils
{
LoggerLevelConfigure::LoggerLevelConfigure(rclcpp::Node * node) : ros_logger_(node->get_logger())
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_config_logger_ = node->create_service<ConfigLogger>(
    "~/config_logger", std::bind(&LoggerLevelConfigure::onLoggerConfigService, this, _1, _2));
}

void LoggerLevelConfigure::onLoggerConfigService(
  const ConfigLogger::Request::SharedPtr request, const ConfigLogger::Response::SharedPtr response)
{
  int logging_severity;
  const auto ret_level = rcutils_logging_severity_level_from_string(
    request->level.c_str(), rcl_get_default_allocator(), &logging_severity);

  if (ret_level != RCUTILS_RET_OK) {
    response->success = false;
    RCLCPP_WARN_STREAM(
      ros_logger_, "Failed to change logger level for "
                     << request->logger_name
                     << " due to an invalid logging severity: " << request->level);
    return;
  }

  const auto ret_set =
    rcutils_logging_set_logger_level(request->logger_name.c_str(), logging_severity);

  if (ret_set != RCUTILS_RET_OK) {
    response->success = false;
    RCLCPP_WARN_STREAM(ros_logger_, "Failed to set logger level for " << request->logger_name);
    return;
  }

  response->success = true;
  RCLCPP_INFO_STREAM(
    ros_logger_, "Logger level [" << request->level << "] is set for " << request->logger_name);
  return;
}

}  // namespace autoware::universe_utils
