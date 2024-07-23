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

#ifndef AUTOWARE__SHAPE_ESTIMATION__FILTER__TRUCK_FILTER_HPP_
#define AUTOWARE__SHAPE_ESTIMATION__FILTER__TRUCK_FILTER_HPP_

#include "autoware/shape_estimation/filter/filter_interface.hpp"
#include "utils.hpp"
namespace autoware::shape_estimation
{
namespace filter
{
class TruckFilter : public ShapeEstimationFilterInterface
{
public:
  TruckFilter() = default;

  ~TruckFilter() = default;

  bool filter(
    const autoware_perception_msgs::msg::Shape & shape,
    const geometry_msgs::msg::Pose & pose) override;
};
}  // namespace filter
}  // namespace autoware::shape_estimation

#endif  // AUTOWARE__SHAPE_ESTIMATION__FILTER__TRUCK_FILTER_HPP_
