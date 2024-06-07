// Copyright 2023 Tier IV, Inc.
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

#ifndef AUTOWARE_SAMPLER_COMMON__CONSTRAINTS__FOOTPRINT_HPP_
#define AUTOWARE_SAMPLER_COMMON__CONSTRAINTS__FOOTPRINT_HPP_

#include "autoware_sampler_common/structures.hpp"

namespace autoware::sampler_common::constraints
{
/// @brief Calculate the footprint of the given path
/// @param path sequence of pose used to build the footprint
/// @param constraints input constraint object containing vehicle footprint offsets
/// @return the polygon footprint of the path
MultiPoint2d buildFootprintPoints(const Path & path, const Constraints & constraints);
}  // namespace autoware::sampler_common::constraints

#endif  // AUTOWARE_SAMPLER_COMMON__CONSTRAINTS__FOOTPRINT_HPP_
