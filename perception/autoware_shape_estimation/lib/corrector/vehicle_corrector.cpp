// Copyright 2024 TIER IV, Inc. All rights reserved.
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

#include "autoware/shape_estimation/corrector/vehicle_corrector.hpp"

namespace autoware::shape_estimation
{
namespace corrector

{
bool VehicleCorrector::correct(
  autoware_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose)
{
  // Guard
  if (!params_) return false;

  if (use_reference_yaw_)
    return corrector_utils::correctWithReferenceYaw(params_.get(), shape, pose);
  else
    return corrector_utils::correctWithDefaultValue(params_.get(), shape, pose);
}

}  // namespace corrector
}  // namespace autoware::shape_estimation
