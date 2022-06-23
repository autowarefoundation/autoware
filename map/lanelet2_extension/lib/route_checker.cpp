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

#include "lanelet2_extension/utility/route_checker.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>

namespace lanelet
{
namespace utils
{
bool route::isRouteValid(const HADMapRoute route_msg, const lanelet::LaneletMapPtr lanelet_map_ptr_)
{
  for (const auto & route_section : route_msg.segments) {
    for (const auto & primitive : route_section.primitives) {
      const auto id = primitive.id;
      try {
        lanelet_map_ptr_->laneletLayer.get(id);
      } catch (const std::exception & e) {
        std::cerr
          << e.what()
          << ". Maybe the loaded route was created on a different Map from the current one. "
             "Try to load the other Route again."
          << std::endl;
        return false;
      }
    }
  }
  return true;
}

}  // namespace utils
}  // namespace lanelet
