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

#ifndef MAP_PROJECTION_LOADER__LOAD_INFO_FROM_LANELET2_MAP_HPP_
#define MAP_PROJECTION_LOADER__LOAD_INFO_FROM_LANELET2_MAP_HPP_

#include <lanelet2_extension/io/autoware_osm_parser.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

#include "tier4_map_msgs/msg/map_projector_info.hpp"

#include <string>

tier4_map_msgs::msg::MapProjectorInfo load_info_from_lanelet2_map(const std::string & filename);

#endif  // MAP_PROJECTION_LOADER__LOAD_INFO_FROM_LANELET2_MAP_HPP_
