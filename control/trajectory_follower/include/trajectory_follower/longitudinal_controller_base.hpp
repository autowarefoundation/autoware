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

#ifndef TRAJECTORY_FOLLOWER__LONGITUDINAL_CONTROLLER_BASE_HPP_
#define TRAJECTORY_FOLLOWER__LONGITUDINAL_CONTROLLER_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "trajectory_follower/input_data.hpp"
#include "trajectory_follower/sync_data.hpp"

#include "autoware_auto_control_msgs/msg/longitudinal_command.hpp"

#include <boost/optional.hpp>

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower
{
struct LongitudinalOutput
{
  autoware_auto_control_msgs::msg::LongitudinalCommand control_cmd;
  LongitudinalSyncData sync_data;
};
class LongitudinalControllerBase
{
public:
  virtual boost::optional<LongitudinalOutput> run() = 0;
  virtual void setInputData(InputData const & input_data) = 0;
  void sync(LateralSyncData const & lateral_sync_data) { lateral_sync_data_ = lateral_sync_data; }

protected:
  LateralSyncData lateral_sync_data_;
};

}  // namespace trajectory_follower
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER__LONGITUDINAL_CONTROLLER_BASE_HPP_
