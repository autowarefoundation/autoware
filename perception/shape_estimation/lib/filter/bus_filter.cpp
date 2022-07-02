// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "shape_estimation/filter/bus_filter.hpp"

bool BusFilter::filter(
  const autoware_auto_perception_msgs::msg::Shape & shape,
  [[maybe_unused]] const geometry_msgs::msg::Pose & pose)
{
  constexpr float min_width = 2.0;
  constexpr float max_width = 3.2;
  constexpr float max_length = 17.0;
  return utils::filterVehicleBoundingBox(shape, min_width, max_width, max_length);
}
