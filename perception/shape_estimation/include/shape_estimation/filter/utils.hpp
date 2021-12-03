// Copyright 2021 TierIV. All rights reserved.
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

#ifndef SHAPE_ESTIMATION__FILTER__UTILS_HPP_
#define SHAPE_ESTIMATION__FILTER__UTILS_HPP_

#include <autoware_auto_perception_msgs/msg/shape.hpp>

namespace utils
{
bool filterVehicleBoundingBox(
  const autoware_auto_perception_msgs::msg::Shape & shape, const float min_width,
  const float max_width, const float max_length);
}  // namespace utils

#endif  // SHAPE_ESTIMATION__FILTER__UTILS_HPP_
