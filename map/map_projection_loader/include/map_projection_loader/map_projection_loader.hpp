// Copyright 2023 TIER IV, Inc.
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

#ifndef MAP_PROJECTION_LOADER__MAP_PROJECTION_LOADER_HPP_
#define MAP_PROJECTION_LOADER__MAP_PROJECTION_LOADER_HPP_

#include "rclcpp/rclcpp.hpp"

#include <component_interface_specs/map.hpp>
#include <component_interface_utils/rclcpp.hpp>

#include <string>

tier4_map_msgs::msg::MapProjectorInfo load_info_from_yaml(const std::string & filename);

class MapProjectionLoader : public rclcpp::Node
{
public:
  MapProjectionLoader();

private:
  using MapProjectorInfo = map_interface::MapProjectorInfo;
  component_interface_utils::Publisher<MapProjectorInfo>::SharedPtr publisher_;
};

#endif  // MAP_PROJECTION_LOADER__MAP_PROJECTION_LOADER_HPP_
