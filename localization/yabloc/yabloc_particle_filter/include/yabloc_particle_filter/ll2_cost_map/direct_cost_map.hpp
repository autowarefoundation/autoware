// Copyright 2023 TIER IV, Inc.
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

#ifndef YABLOC_PARTICLE_FILTER__LL2_COST_MAP__DIRECT_COST_MAP_HPP_
#define YABLOC_PARTICLE_FILTER__LL2_COST_MAP__DIRECT_COST_MAP_HPP_

#include <opencv4/opencv2/imgproc.hpp>

namespace yabloc
{
cv::Mat direct_cost_map(const cv::Mat & cost_map, const cv::Mat & intensity);
}  // namespace yabloc

#endif  // YABLOC_PARTICLE_FILTER__LL2_COST_MAP__DIRECT_COST_MAP_HPP_
