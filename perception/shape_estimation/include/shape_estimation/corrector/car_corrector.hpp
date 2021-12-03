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

#ifndef SHAPE_ESTIMATION__CORRECTOR__CAR_CORRECTOR_HPP_
#define SHAPE_ESTIMATION__CORRECTOR__CAR_CORRECTOR_HPP_

#include "shape_estimation/corrector/corrector_interface.hpp"
#include "utils.hpp"

class CarCorrector : public ShapeEstimationCorrectorInterface
{
private:
  utils::CorrectionParameters params_;
  bool use_reference_yaw_;

public:
  explicit CarCorrector(bool use_reference_yaw = false) : use_reference_yaw_(use_reference_yaw)
  {
    params_.min_width = 1.2;
    params_.max_width = 2.2;
    params_.avg_width = (params_.min_width + params_.max_width) * 0.5;
    params_.min_length = 3.0;
    params_.max_length = 5.0;
    params_.avg_length = (params_.min_length + params_.max_length) * 0.5;
  }

  ~CarCorrector() = default;

  bool correct(
    autoware_auto_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output) override;
};

#endif  // SHAPE_ESTIMATION__CORRECTOR__CAR_CORRECTOR_HPP_
