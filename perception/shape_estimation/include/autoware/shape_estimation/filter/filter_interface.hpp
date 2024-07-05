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

#ifndef AUTOWARE__SHAPE_ESTIMATION__FILTER__FILTER_INTERFACE_HPP_
#define AUTOWARE__SHAPE_ESTIMATION__FILTER__FILTER_INTERFACE_HPP_

#include <autoware_perception_msgs/msg/shape.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <string>

namespace autoware::shape_estimation
{
namespace filter
{

class ShapeEstimationFilterInterface
{
public:
  ShapeEstimationFilterInterface() {}

  virtual ~ShapeEstimationFilterInterface() {}

  virtual bool filter(
    const autoware_perception_msgs::msg::Shape & shape, const geometry_msgs::msg::Pose & pose) = 0;
};

}  // namespace filter
}  // namespace autoware::shape_estimation

#endif  // AUTOWARE__SHAPE_ESTIMATION__FILTER__FILTER_INTERFACE_HPP_
