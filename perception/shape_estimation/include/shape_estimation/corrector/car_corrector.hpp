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

#include "shape_estimation/corrector/vehicle_corrector.hpp"
#include "utils.hpp"

class CarCorrector : public VehicleCorrector
{
public:
  explicit CarCorrector(const bool use_reference_yaw = false) : VehicleCorrector(use_reference_yaw)
  {
    corrector_utils::CorrectionBBParameters params;
    params.min_width = 1.2;
    params.max_width = 2.5;
    params.default_width = (params.min_width + params.max_width) * 0.5;
    params.min_length = 3.0;
    params.max_length = 5.8;
    params.default_length = (params.min_length + params.max_length) * 0.5;
    setParams(params);
  }

  ~CarCorrector() = default;
};

#endif  // SHAPE_ESTIMATION__CORRECTOR__CAR_CORRECTOR_HPP_
