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

#ifndef MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_

#include <autoware_lanelet2_extension/version.hpp>
#include <component_interface_specs/map.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_projection/UTM.h>

#include <memory>
#include <string>

class Lanelet2MapLoaderNode : public rclcpp::Node
{
public:
  static constexpr lanelet::autoware::Version version = lanelet::autoware::version;

public:
  explicit Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options);

  static lanelet::LaneletMapPtr load_map(
    const std::string & lanelet2_filename,
    const tier4_map_msgs::msg::MapProjectorInfo & projector_info);
  static autoware_map_msgs::msg::LaneletMapBin create_map_bin_msg(
    const lanelet::LaneletMapPtr map, const std::string & lanelet2_filename,
    const rclcpp::Time & now);

private:
  using MapProjectorInfo = map_interface::MapProjectorInfo;

  void on_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg);

  component_interface_utils::Subscription<MapProjectorInfo>::SharedPtr sub_map_projector_info_;
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr pub_map_bin_;
};

#endif  // MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
