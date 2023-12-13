// Copyright 2023 The Autoware Foundation
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
#ifndef AUTOWARE_AUTO_MSGS_ADAPTER_CORE_HPP_
#define AUTOWARE_AUTO_MSGS_ADAPTER_CORE_HPP_

#include "adapter_control.hpp"
#include "adapter_map.hpp"
#include "adapter_perception.hpp"
#include "adapter_planning.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <string>

namespace autoware_auto_msgs_adapter
{

class AutowareAutoMsgsAdapterNode : public rclcpp::Node
{
public:
  explicit AutowareAutoMsgsAdapterNode(const rclcpp::NodeOptions & node_options);
  using MapStringAdapter = std::map<std::string, std::function<AdapterBaseInterface::SharedPtr()>>;

private:
  AdapterBaseInterface::SharedPtr adapter_;

  MapStringAdapter create_adapter_map(
    const std::string & topic_name_source, const std::string & topic_name_target);

  void print_adapter_options(const MapStringAdapter & map_adapter);

  bool initialize_adapter(
    const MapStringAdapter & map_adapter, const std::string & msg_type_target);
};
}  // namespace autoware_auto_msgs_adapter

#endif  // AUTOWARE_AUTO_MSGS_ADAPTER_CORE_HPP_
