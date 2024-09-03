// Copyright 2022 The Autoware Contributors
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

#ifndef EKF_LOCALIZATION_TRIGGER_MODULE_HPP_
#define EKF_LOCALIZATION_TRIGGER_MODULE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace autoware::pose_initializer
{
class EkfLocalizationTriggerModule
{
private:
  using SetBool = std_srvs::srv::SetBool;

public:
  explicit EkfLocalizationTriggerModule(rclcpp::Node * node);
  void wait_for_service();
  void send_request(bool flag, bool need_spin = false) const;

private:
  rclcpp::Node * node_;
  rclcpp::Client<SetBool>::SharedPtr client_ekf_trigger_;
};
}  // namespace autoware::pose_initializer

#endif  // EKF_LOCALIZATION_TRIGGER_MODULE_HPP_
