// Copyright 2022 TIER IV, Inc.
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

#ifndef COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
#define COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace component_interface_utils
{

struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;
  using ServiceLog = tier4_system_msgs::msg::ServiceLog;

  explicit NodeInterface(rclcpp::Node * node)
  {
    this->node = node;
    this->logger = node->create_publisher<ServiceLog>("/service_log", 10);

    node_name = node->get_namespace();
    if (node_name.empty() || node_name.back() != '/') {
      node_name += "/";
    }
    node_name += node->get_name();
  }

  void log(ServiceLog::_type_type type, const std::string & name, const std::string & yaml = "")
  {
    static const auto type_text = std::unordered_map<ServiceLog::_type_type, std::string>(
      {{ServiceLog::CLIENT_REQUEST, "client call"},
       {ServiceLog::SERVER_REQUEST, "server call"},
       {ServiceLog::SERVER_RESPONSE, "server exit"},
       {ServiceLog::CLIENT_RESPONSE, "client exit"},
       {ServiceLog::ERROR_UNREADY, "client unready"},
       {ServiceLog::ERROR_TIMEOUT, "client timeout"}});
    RCLCPP_INFO_STREAM(node->get_logger(), type_text.at(type) << ": " << name);

    ServiceLog msg;
    msg.stamp = node->now();
    msg.type = type;
    msg.name = name;
    msg.node = node_name;
    msg.yaml = yaml;
    logger->publish(msg);
  }

  rclcpp::Node * node;
  rclcpp::Publisher<ServiceLog>::SharedPtr logger;
  std::string node_name;
};

}  // namespace component_interface_utils

#endif  // COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
