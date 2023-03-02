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

#ifndef POSE_INITIALIZER__POSE_INITIALIZER_CORE_HPP_
#define POSE_INITIALIZER__POSE_INITIALIZER_CORE_HPP_

#include <component_interface_specs/localization.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <memory>

class StopCheckModule;
class NdtModule;
class GnssModule;
class EkfLocalizationTriggerModule;
class NdtLocalizationTriggerModule;

class PoseInitializer : public rclcpp::Node
{
public:
  PoseInitializer();
  ~PoseInitializer();

private:
  using ServiceException = component_interface_utils::ServiceException;
  using Initialize = localization_interface::Initialize;
  using State = localization_interface::InitializationState;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  rclcpp::CallbackGroup::SharedPtr group_srv_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_reset_;
  component_interface_utils::Publisher<State>::SharedPtr pub_state_;
  component_interface_utils::Service<Initialize>::SharedPtr srv_initialize_;
  State::Message state_;
  std::array<double, 36> output_pose_covariance_;
  std::array<double, 36> gnss_particle_covariance_;
  std::unique_ptr<GnssModule> gnss_;
  std::unique_ptr<NdtModule> ndt_;
  std::unique_ptr<StopCheckModule> stop_check_;
  std::unique_ptr<EkfLocalizationTriggerModule> ekf_localization_trigger_;
  std::unique_ptr<NdtLocalizationTriggerModule> ndt_localization_trigger_;
  double stop_check_duration_;
  void change_state(State::Message::_state_type state);
  void on_initialize(
    const Initialize::Service::Request::SharedPtr req,
    const Initialize::Service::Response::SharedPtr res);
  PoseWithCovarianceStamped get_gnss_pose();
};

#endif  // POSE_INITIALIZER__POSE_INITIALIZER_CORE_HPP_
