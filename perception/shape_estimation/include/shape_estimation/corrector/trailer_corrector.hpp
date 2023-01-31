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

#ifndef SHAPE_ESTIMATION__CORRECTOR__TRAILER_CORRECTOR_HPP_
#define SHAPE_ESTIMATION__CORRECTOR__TRAILER_CORRECTOR_HPP_

#include "shape_estimation/corrector/vehicle_corrector.hpp"
#include "utils.hpp"

// Generally speaking, trailer would be much larger than bus and truck.
// But currently we do not make large differences among bus/truck/trailer
// because current our vehicle classification is not reliable enough.
class TrailerCorrector : public VehicleCorrector
{
public:
  explicit TrailerCorrector(const bool use_reference_yaw = false)
  : VehicleCorrector(use_reference_yaw)
  {
    corrector_utils::CorrectionBBParameters params;
    params.min_width = 2.0;
    params.max_width = 3.2;
    params.default_width = (params.min_width + params.max_width) * 0.5;
    params.min_length = 5.0;
    params.max_length = 24.0;
    params.default_length = 8.0;  // Ideally, it should be 12m from average Japanese trailer size.
    setParams(params);
  }

  ~TrailerCorrector() = default;
};

#endif  // SHAPE_ESTIMATION__CORRECTOR__TRAILER_CORRECTOR_HPP_
