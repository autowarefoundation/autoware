// Copyright 2022 TIER IV, Inc.
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

#include "motion.hpp"

#include <memory>
#include <unordered_map>

namespace default_ad_api
{

MotionNode::MotionNode(const rclcpp::NodeOptions & options)
: Node("motion", options), vehicle_stop_checker_(this)
{
  stop_check_duration_ = declare_parameter("stop_check_duration", 1.0);
  require_accept_start_ = declare_parameter("require_accept_start", false);
  is_calling_set_pause_ = false;

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_srv(srv_accept_, this, &MotionNode::on_accept);
  adaptor.init_pub(pub_state_);
  adaptor.init_cli(cli_set_pause_, group_cli_);
  adaptor.init_sub(sub_is_paused_, this, &MotionNode::on_is_paused);
  adaptor.init_sub(sub_is_start_requested_, this, &MotionNode::on_is_start_requested);

  rclcpp::Rate rate(10);
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
  state_ = State::Unknown;
}

void MotionNode::update_state()
{
  if (!is_paused_ || !is_start_requested_) {
    return;
  }

  const auto get_next_state = [this]() {
    if (is_paused_.value()) {
      if (!is_start_requested_.value()) {
        return State::Paused;
      } else {
        return require_accept_start_ ? State::Starting : State::Resuming;
      }
    } else {
      if (!vehicle_stop_checker_.isVehicleStopped(stop_check_duration_)) {
        return State::Moving;
      } else {
        return is_start_requested_.value() ? State::Resumed : State::Pausing;
      }
    }
  };
  const auto next_state = get_next_state();

  // Once the state becomes pausing, it must become a state where is_paused is true
  if (state_ == State::Pausing) {
    switch (next_state) {
      case State::Paused:
      case State::Starting:
      case State::Resuming:
        break;
      case State::Moving:
      case State::Pausing:
      case State::Resumed:
      case State::Unknown:
        return;
    }
  }

  // Prevents transition from starting to resuming
  if (state_ == State::Resuming && next_state == State::Starting) {
    return;
  }

  change_state(next_state);
}

void MotionNode::change_state(const State state)
{
  using MotionState = autoware_ad_api::motion::State::Message;
  static const auto mapping = std::unordered_map<State, MotionState::_state_type>(
    {{State::Unknown, MotionState::UNKNOWN},
     {State::Pausing, MotionState::STOPPED},
     {State::Paused, MotionState::STOPPED},
     {State::Starting, MotionState::STARTING},
     {State::Resuming, MotionState::MOVING},
     {State::Resumed, MotionState::MOVING},
     {State::Moving, MotionState::MOVING}});

  if (mapping.at(state_) != mapping.at(state)) {
    MotionState msg;
    msg.stamp = now();
    msg.state = mapping.at(state);
    pub_state_->publish(msg);
  }
  state_ = state;
}

void MotionNode::change_pause(bool pause)
{
  if (!is_calling_set_pause_ && cli_set_pause_->service_is_ready()) {
    const auto req = std::make_shared<control_interface::SetPause::Service::Request>();
    req->pause = pause;
    is_calling_set_pause_ = true;
    cli_set_pause_->async_send_request(req, [this](auto) { is_calling_set_pause_ = false; });
  }
}

void MotionNode::on_timer()
{
  update_state();

  if (state_ == State::Pausing) {
    return change_pause(true);
  }
  if (state_ == State::Resuming) {
    return change_pause(false);
  }
}

void MotionNode::on_is_paused(const control_interface::IsPaused::Message::ConstSharedPtr msg)
{
  is_paused_ = msg->data;
}

void MotionNode::on_is_start_requested(
  const control_interface::IsStartRequested::Message::ConstSharedPtr msg)
{
  is_start_requested_ = msg->data;
}

void MotionNode::on_accept(
  const autoware_ad_api::motion::AcceptStart::Service::Request::SharedPtr,
  const autoware_ad_api::motion::AcceptStart::Service::Response::SharedPtr res)
{
  if (state_ != State::Starting) {
    using AcceptStartResponse = autoware_ad_api::motion::AcceptStart::Service::Response;
    throw component_interface_utils::ServiceException(
      AcceptStartResponse::ERROR_NOT_STARTING, "The motion state is not starting");
  }
  change_state(State::Resuming);
  res->status.success = true;
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::MotionNode)
