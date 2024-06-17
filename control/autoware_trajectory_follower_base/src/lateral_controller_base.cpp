// Copyright 2022 The Autoware Foundation
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

#include "autoware/trajectory_follower_base/lateral_controller_base.hpp"

namespace autoware::motion::control::trajectory_follower
{
void LateralControllerBase::sync(LongitudinalSyncData const & longitudinal_sync_data)
{
  longitudinal_sync_data_ = longitudinal_sync_data;
}

}  // namespace autoware::motion::control::trajectory_follower
