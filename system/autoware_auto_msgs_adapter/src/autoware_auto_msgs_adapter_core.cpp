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

#include "include/autoware_auto_msgs_adapter_core.hpp"

#include "include/adapter_control.hpp"

#include <rclcpp/rclcpp.hpp>

#include <map>

namespace autoware_auto_msgs_adapter
{

using autoware_auto_control_msgs::msg::AckermannControlCommand;
using autoware_control_msgs::msg::Control;

using MapStringAdapter = AutowareAutoMsgsAdapterNode::MapStringAdapter;

AutowareAutoMsgsAdapterNode::AutowareAutoMsgsAdapterNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("autoware_auto_msgs_adapter", node_options)
{
  const std::string msg_type_target = declare_parameter<std::string>("msg_type_target");
  const std::string topic_name_source = declare_parameter<std::string>("topic_name_source");
  const std::string topic_name_target = declare_parameter<std::string>("topic_name_target");

  // Map of available adapters
  auto map_adapter = create_adapter_map(topic_name_source, topic_name_target);

  print_adapter_options(map_adapter);

  // Initialize the adapter with the selected option
  if (!initialize_adapter(map_adapter, msg_type_target)) {
    RCLCPP_ERROR(
      get_logger(), "Unknown msg type: %s. Please refer to previous log for available options.",
      msg_type_target.c_str());
  }
}

MapStringAdapter AutowareAutoMsgsAdapterNode::create_adapter_map(
  const std::string & topic_name_source, const std::string & topic_name_target)
{
  return {
    {"autoware_auto_control_msgs/msg/AckermannControlCommand",
     [&] {
       return std::static_pointer_cast<AdapterBaseInterface>(
         std::make_shared<AdapterControl>(*this, topic_name_source, topic_name_target));
     }},
    {"autoware_auto_mapping_msgs/msg/HADMapBin",
     [&] {
       return std::static_pointer_cast<AdapterBaseInterface>(
         std::make_shared<AdapterMap>(*this, topic_name_source, topic_name_target));
     }},
    {"autoware_auto_perception_msgs/msg/PredictedObjects",
     [&] {
       return std::static_pointer_cast<AdapterBaseInterface>(
         std::make_shared<AdapterPerception>(*this, topic_name_source, topic_name_target));
     }},
    {"autoware_auto_planning_msgs/msg/Trajectory",
     [&] {
       return std::static_pointer_cast<AdapterBaseInterface>(
         std::make_shared<AdapterPlanning>(*this, topic_name_source, topic_name_target));
     }},
  };
}

void AutowareAutoMsgsAdapterNode::print_adapter_options(const MapStringAdapter & map_adapter)
{
  std::string std_options_available;
  for (const auto & entry : map_adapter) {
    std_options_available += entry.first + "\n";
  }
  RCLCPP_INFO(
    get_logger(), "Available msg_type_target options:\n%s", std_options_available.c_str());
}

bool AutowareAutoMsgsAdapterNode::initialize_adapter(
  const MapStringAdapter & map_adapter, const std::string & msg_type_target)
{
  auto it = map_adapter.find(msg_type_target);
  adapter_ = (it != map_adapter.end()) ? it->second() : nullptr;
  return adapter_ != nullptr;
}

}  // namespace autoware_auto_msgs_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_auto_msgs_adapter::AutowareAutoMsgsAdapterNode)
