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

#ifndef TRAJECTORY_FOLLOWER__SYNC_DATA_HPP_
#define TRAJECTORY_FOLLOWER__SYNC_DATA_HPP_

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
struct LateralSyncData
{
  bool is_steer_converged{false};
};

struct LongitudinalSyncData
{
  bool is_velocity_converged{false};
};

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__SYNC_DATA_HPP_
