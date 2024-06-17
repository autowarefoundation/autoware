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

#ifndef AUTOWARE__TRAJECTORY_FOLLOWER_BASE__LATERAL_CONTROLLER_BASE_HPP_
#define AUTOWARE__TRAJECTORY_FOLLOWER_BASE__LATERAL_CONTROLLER_BASE_HPP_

#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/sync_data.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_control_msgs/msg/lateral.hpp"

#include <boost/optional.hpp>
namespace autoware::motion::control::trajectory_follower
{
struct LateralOutput
{
  autoware_control_msgs::msg::Lateral control_cmd;
  LateralSyncData sync_data;
};

class LateralControllerBase
{
public:
  virtual bool isReady(const InputData & input_data) = 0;
  virtual LateralOutput run(InputData const & input_data) = 0;
  void sync(LongitudinalSyncData const & longitudinal_sync_data);

protected:
  LongitudinalSyncData longitudinal_sync_data_;
};

}  // namespace autoware::motion::control::trajectory_follower

#endif  // AUTOWARE__TRAJECTORY_FOLLOWER_BASE__LATERAL_CONTROLLER_BASE_HPP_
