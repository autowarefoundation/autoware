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

#ifndef YABLOC_PARTICLE_FILTER__CAMERA_CORRECTOR__LOGIT_HPP_
#define YABLOC_PARTICLE_FILTER__CAMERA_CORRECTOR__LOGIT_HPP_

namespace yabloc
{
float logit_to_prob(float logit, float gain = 1.0f);

/**
 * Convert probability to logit
 * This function is much faster than logit_to_prob() because it refers to pre-computed table
 *
 * @param[in] prob
 * @return logit
 */
float prob_to_logit(float prob);
}  // namespace yabloc

#endif  // YABLOC_PARTICLE_FILTER__CAMERA_CORRECTOR__LOGIT_HPP_
