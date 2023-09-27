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

// =============== Note ===============
// This is a util class implementation of the logger_config_component provided by ROS 2
// https://github.com/ros2/demos/blob/humble/logging_demo/src/logger_config_component.cpp
//
// When ROS 2 officially supports the set_logger_level option in release version, this class can be
// removed.
// https://github.com/ros2/ros2/issues/1355

// =============== How to use ===============
// ___In your_node.hpp___
// #include "tier4_autoware_utils/ros/logger_level_configure.hpp"
// class YourNode : public rclcpp::Node {
//   ...
//
//   // Define logger_configure as a node class member variable
//   std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
// }
//
// ___In your_node.cpp___
// YourNode::YourNode() {
//   ...
//
//   // Set up logger_configure
//   logger_configure_ = std::make_unique<LoggerLevelConfigure>(this);
// }

#ifndef TIER4_AUTOWARE_UTILS__ROS__LOGGER_LEVEL_CONFIGURE_HPP_
#define TIER4_AUTOWARE_UTILS__ROS__LOGGER_LEVEL_CONFIGURE_HPP_

#include "logging_demo/srv/config_logger.hpp"

#include <rclcpp/rclcpp.hpp>

namespace tier4_autoware_utils
{
class LoggerLevelConfigure
{
private:
  using ConfigLogger = logging_demo::srv::ConfigLogger;

public:
  explicit LoggerLevelConfigure(rclcpp::Node * node);

private:
  rclcpp::Logger ros_logger_;
  rclcpp::Service<ConfigLogger>::SharedPtr srv_config_logger_;

  void onLoggerConfigService(
    const ConfigLogger::Request::SharedPtr request,
    const ConfigLogger::Response::SharedPtr response);
};

}  // namespace tier4_autoware_utils
#endif  // TIER4_AUTOWARE_UTILS__ROS__LOGGER_LEVEL_CONFIGURE_HPP_
