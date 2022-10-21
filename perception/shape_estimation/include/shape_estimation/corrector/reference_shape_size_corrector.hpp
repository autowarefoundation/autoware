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

#ifndef SHAPE_ESTIMATION__CORRECTOR__REFERENCE_SHAPE_SIZE_CORRECTOR_HPP_
#define SHAPE_ESTIMATION__CORRECTOR__REFERENCE_SHAPE_SIZE_CORRECTOR_HPP_

#include "shape_estimation/corrector/corrector_interface.hpp"
#include "shape_estimation/shape_estimator.hpp"
#include "utils.hpp"

class ReferenceShapeBasedVehicleCorrector : public ShapeEstimationCorrectorInterface
{
  ReferenceShapeSizeInfo ref_shape_size_info_;

public:
  explicit ReferenceShapeBasedVehicleCorrector(const ReferenceShapeSizeInfo & ref_shape_size_info)
  : ref_shape_size_info_(ref_shape_size_info)
  {
  }

  virtual ~ReferenceShapeBasedVehicleCorrector() = default;

  bool correct(
    autoware_auto_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose) override
  {
    return corrector_utils::correctWithReferenceYawAndShapeSize(ref_shape_size_info_, shape, pose);
  }
};

#endif  // SHAPE_ESTIMATION__CORRECTOR__REFERENCE_SHAPE_SIZE_CORRECTOR_HPP_
