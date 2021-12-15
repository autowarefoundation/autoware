// Copyright 2020 Tier IV, Inc
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

#ifndef HAD_MAP_UTILS__HAD_MAP_CONVERSION_HPP_
#define HAD_MAP_UTILS__HAD_MAP_CONVERSION_HPP_

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <memory>
#include "had_map_utils/visibility_control.hpp"

namespace autoware
{
namespace common
{
namespace had_map_utils
{

void HAD_MAP_UTILS_PUBLIC toBinaryMsg(
  const std::shared_ptr<lanelet::LaneletMap> & map,
  autoware_auto_mapping_msgs::msg::HADMapBin & msg);

void HAD_MAP_UTILS_PUBLIC fromBinaryMsg(
  const autoware_auto_mapping_msgs::msg::HADMapBin & msg,
  std::shared_ptr<lanelet::LaneletMap> & map);

}  // namespace had_map_utils
}  // namespace common
}  // namespace autoware

#endif  // HAD_MAP_UTILS__HAD_MAP_CONVERSION_HPP_
