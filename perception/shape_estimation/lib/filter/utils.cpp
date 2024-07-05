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

#include "autoware/shape_estimation/filter/utils.hpp"
namespace autoware::shape_estimation
{

namespace utils
{
bool filterVehicleBoundingBox(
  const autoware_perception_msgs::msg::Shape & shape, const float min_width, const float max_width,
  const float max_length)
{
  const float x = shape.dimensions.x;
  const float y = shape.dimensions.y;
  const float s = x * y;

  if (shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    return true;
  }
  if (x < min_width && y < min_width) {
    return false;
  }
  if (max_width < x && max_width < y) {
    return false;
  }

  if (max_length < x || max_length < y) {
    return false;
  }

  if (max_length * max_width < s) {
    return false;
  }
  return true;
}

}  // namespace utils
}  // namespace autoware::shape_estimation
