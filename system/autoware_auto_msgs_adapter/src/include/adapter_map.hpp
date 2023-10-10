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
#ifndef ADAPTER_MAP_HPP_
#define ADAPTER_MAP_HPP_

#include "adapter_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>

#include <string>

namespace autoware_auto_msgs_adapter
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_map_msgs::msg::LaneletMapBin;

class AdapterMap : public autoware_auto_msgs_adapter::AdapterBase<LaneletMapBin, HADMapBin>
{
public:
  AdapterMap(
    rclcpp::Node & node, const std::string & topic_name_source,
    const std::string & topic_name_target, const rclcpp::QoS & qos = rclcpp::QoS{1})
  : AdapterBase(node, topic_name_source, topic_name_target, qos)
  {
    RCLCPP_DEBUG(
      node.get_logger(), "AdapterMap is initialized to convert: %s -> %s",
      topic_name_source.c_str(), topic_name_target.c_str());
  }

protected:
  HADMapBin convert(const LaneletMapBin & msg_source) override
  {
    autoware_auto_mapping_msgs::msg::HADMapBin msg_auto;
    msg_auto.header = msg_source.header;
    msg_auto.format_version = msg_source.version_map_format;
    msg_auto.map_version = msg_source.version_map;
    msg_auto.map_format = 0;
    msg_auto.data = msg_source.data;

    return msg_auto;
  }
};
}  // namespace autoware_auto_msgs_adapter

#endif  // ADAPTER_MAP_HPP_
