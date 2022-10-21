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

#ifndef SHAPE_ESTIMATION__CORRECTOR__VEHICLE_CORRECTOR_HPP_
#define SHAPE_ESTIMATION__CORRECTOR__VEHICLE_CORRECTOR_HPP_

#include "shape_estimation/corrector/corrector_interface.hpp"
#include "shape_estimation/shape_estimator.hpp"
#include "utils.hpp"

class VehicleCorrector : public ShapeEstimationCorrectorInterface
{
protected:
  boost::optional<corrector_utils::CorrectionBBParameters> params_;

private:
  bool use_reference_yaw_;

public:
  explicit VehicleCorrector(const bool use_reference_yaw) : use_reference_yaw_(use_reference_yaw) {}

  virtual ~VehicleCorrector() = default;

  bool correct(
    autoware_auto_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose) override
  {
    // Guard
    if (!params_) return false;

    if (use_reference_yaw_)
      return corrector_utils::correctWithReferenceYaw(params_.get(), shape, pose);
    else
      return correctWithDefaultValue(params_.get(), shape, pose);
  }

  void setParams(const corrector_utils::CorrectionBBParameters & params) { params_ = params; }
};

#endif  // SHAPE_ESTIMATION__CORRECTOR__VEHICLE_CORRECTOR_HPP_
