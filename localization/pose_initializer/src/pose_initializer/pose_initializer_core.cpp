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

#include "pose_initializer_core.hpp"

#include "copy_vector_to_array.hpp"
#include "ekf_localization_trigger_module.hpp"
#include "gnss_module.hpp"
#include "ndt_localization_trigger_module.hpp"
#include "ndt_module.hpp"
#include "stop_check_module.hpp"
#include "yabloc_module.hpp"

#include <memory>
#include <vector>

PoseInitializer::PoseInitializer() : Node("pose_initializer")
{
  const auto node = component_interface_utils::NodeAdaptor(this);
  group_srv_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  node.init_pub(pub_state_);
  node.init_srv(srv_initialize_, this, &PoseInitializer::on_initialize, group_srv_);
  pub_reset_ = create_publisher<PoseWithCovarianceStamped>("pose_reset", 1);

  output_pose_covariance_ = get_covariance_parameter(this, "output_pose_covariance");
  gnss_particle_covariance_ = get_covariance_parameter(this, "gnss_particle_covariance");

  if (declare_parameter<bool>("ekf_enabled")) {
    ekf_localization_trigger_ = std::make_unique<EkfLocalizationTriggerModule>(this);
  }
  if (declare_parameter<bool>("gnss_enabled")) {
    gnss_ = std::make_unique<GnssModule>(this);
  }
  if (declare_parameter<bool>("yabloc_enabled")) {
    yabloc_ = std::make_unique<YabLocModule>(this);
  }
  if (declare_parameter<bool>("ndt_enabled")) {
    ndt_ = std::make_unique<NdtModule>(this);
    ndt_localization_trigger_ = std::make_unique<NdtLocalizationTriggerModule>(this);
  }
  if (declare_parameter<bool>("stop_check_enabled")) {
    // Add 1.0 sec margin for twist buffer.
    stop_check_duration_ = declare_parameter<double>("stop_check_duration");
    stop_check_ = std::make_unique<StopCheckModule>(this, stop_check_duration_ + 1.0);
  }
  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);

  change_state(State::Message::UNINITIALIZED);

  if (declare_parameter<bool>("user_defined_initial_pose.enable")) {
    const auto initial_pose_array =
      declare_parameter<std::vector<double>>("user_defined_initial_pose.pose");
    if (initial_pose_array.size() != 7) {
      throw std::invalid_argument(
        "Could not set user defined initial pose. The size of initial_pose is " +
        std::to_string(initial_pose_array.size()) + ". It must be 7.");
    } else if (
      std::abs(initial_pose_array[3]) < 1e-6 && std::abs(initial_pose_array[4]) < 1e-6 &&
      std::abs(initial_pose_array[5]) < 1e-6 && std::abs(initial_pose_array[6]) < 1e-6) {
      throw std::invalid_argument("Input quaternion is invalid. All elements are close to zero.");
    } else {
      geometry_msgs::msg::Pose initial_pose;
      initial_pose.position.x = initial_pose_array[0];
      initial_pose.position.y = initial_pose_array[1];
      initial_pose.position.z = initial_pose_array[2];
      initial_pose.orientation.x = initial_pose_array[3];
      initial_pose.orientation.y = initial_pose_array[4];
      initial_pose.orientation.z = initial_pose_array[5];
      initial_pose.orientation.w = initial_pose_array[6];

      set_user_defined_initial_pose(initial_pose);
    }
  }
}

PoseInitializer::~PoseInitializer()
{
  // to delete gnss module
}

void PoseInitializer::change_state(State::Message::_state_type state)
{
  state_.stamp = now();
  state_.state = state;
  pub_state_->publish(state_);
}

// To execute in the constructor, you need to call ros spin.
// Conversely, ros spin should not be called elsewhere
void PoseInitializer::change_node_trigger(bool flag, bool need_spin)
{
  try {
    if (ekf_localization_trigger_) {
      ekf_localization_trigger_->wait_for_service();
      ekf_localization_trigger_->send_request(flag, need_spin);
    }
    if (ndt_localization_trigger_) {
      ndt_localization_trigger_->wait_for_service();
      ndt_localization_trigger_->send_request(flag, need_spin);
    }
  } catch (const ServiceException & error) {
    throw;
  }
}

void PoseInitializer::set_user_defined_initial_pose(const geometry_msgs::msg::Pose initial_pose)
{
  try {
    change_state(State::Message::INITIALIZING);
    change_node_trigger(false, true);

    PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.pose = initial_pose;
    pose.pose.covariance = output_pose_covariance_;
    pub_reset_->publish(pose);

    change_node_trigger(true, true);
    change_state(State::Message::INITIALIZED);

    RCLCPP_INFO(get_logger(), "Set user defined initial pose");
  } catch (const ServiceException & error) {
    change_state(State::Message::UNINITIALIZED);
    RCLCPP_WARN(get_logger(), "Could not set user defined initial pose");
  }
}

void PoseInitializer::on_initialize(
  const Initialize::Service::Request::SharedPtr req,
  const Initialize::Service::Response::SharedPtr res)
{
  // NOTE: This function is not executed during initialization because mutually exclusive.
  if (stop_check_ && !stop_check_->isVehicleStopped(stop_check_duration_)) {
    throw ServiceException(
      Initialize::Service::Response::ERROR_UNSAFE, "The vehicle is not stopped.");
  }
  try {
    change_state(State::Message::INITIALIZING);
    change_node_trigger(false, false);

    auto pose = req->pose.empty() ? get_gnss_pose() : req->pose.front();
    if (ndt_) {
      pose = ndt_->align_pose(pose);
    } else if (yabloc_) {
      // If both the NDT and YabLoc initializer are enabled, prioritize NDT as it offers more
      // accuracy pose.
      pose = yabloc_->align_pose(pose);
    }
    pose.pose.covariance = output_pose_covariance_;
    pub_reset_->publish(pose);

    change_node_trigger(true, false);
    res->status.success = true;
    change_state(State::Message::INITIALIZED);
  } catch (const ServiceException & error) {
    res->status = error.status();
    change_state(State::Message::UNINITIALIZED);
  }
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseInitializer::get_gnss_pose()
{
  if (gnss_) {
    PoseWithCovarianceStamped pose = gnss_->get_pose();
    pose.pose.covariance = gnss_particle_covariance_;
    return pose;
  }
  throw ServiceException(
    Initialize::Service::Response::ERROR_GNSS_SUPPORT, "GNSS is not supported.");
}
