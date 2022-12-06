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

#include "autoware_state.hpp"

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace transition
{

using State = autoware_auto_system_msgs::msg::AutowareState;
using Value = State::_state_type;
struct PairHash
{
  size_t operator()(const std::pair<Value, Value> & pair) const noexcept
  {
    // This hash only works for uint8 pairs.
    const auto prev = static_cast<size_t>(pair.first);
    const auto curr = static_cast<size_t>(pair.second);
    return (prev << 8) | (curr << 0);
  }
};

double get_wait_time(const Value state)
{
  if (state == State::INITIALIZING) {
    return 1.0;
  }
  if (state == State::PLANNING) {
    return 3.0;
  }
  if (state == State::ARRIVED_GOAL) {
    return 2.0;
  }
  return 0.0;
}

const double arrived_goal_timeout = get_wait_time(State::ARRIVED_GOAL);

// clang-format off
std::unordered_map<std::pair<Value, Value>, Value, PairHash> matrix =
{
  {{State::INITIALIZING      , State::INITIALIZING      }, State::INITIALIZING      },
  {{State::INITIALIZING      , State::WAITING_FOR_ROUTE }, State::WAITING_FOR_ROUTE },
  {{State::INITIALIZING      , State::PLANNING          }, State::WAITING_FOR_ROUTE },
  {{State::INITIALIZING      , State::WAITING_FOR_ENGAGE}, State::WAITING_FOR_ROUTE },
  {{State::INITIALIZING      , State::DRIVING           }, State::WAITING_FOR_ROUTE },
  {{State::INITIALIZING      , State::ARRIVED_GOAL      }, State::WAITING_FOR_ROUTE },
  {{State::WAITING_FOR_ROUTE , State::INITIALIZING      }, State::WAITING_FOR_ROUTE },
  {{State::WAITING_FOR_ROUTE , State::WAITING_FOR_ROUTE }, State::WAITING_FOR_ROUTE },
  {{State::WAITING_FOR_ROUTE , State::PLANNING          }, State::PLANNING          },
  {{State::WAITING_FOR_ROUTE , State::WAITING_FOR_ENGAGE}, State::PLANNING          },
  {{State::WAITING_FOR_ROUTE , State::DRIVING           }, State::DRIVING           },
  {{State::WAITING_FOR_ROUTE , State::ARRIVED_GOAL      }, State::DRIVING           },
  {{State::PLANNING          , State::INITIALIZING      }, State::PLANNING          },
  {{State::PLANNING          , State::WAITING_FOR_ROUTE }, State::WAITING_FOR_ROUTE },
  {{State::PLANNING          , State::PLANNING          }, State::PLANNING          },
  {{State::PLANNING          , State::WAITING_FOR_ENGAGE}, State::WAITING_FOR_ENGAGE},
  {{State::PLANNING          , State::DRIVING           }, State::WAITING_FOR_ENGAGE},
  {{State::PLANNING          , State::ARRIVED_GOAL      }, State::WAITING_FOR_ENGAGE},
  {{State::WAITING_FOR_ENGAGE, State::INITIALIZING      }, State::WAITING_FOR_ENGAGE},
  {{State::WAITING_FOR_ENGAGE, State::WAITING_FOR_ROUTE }, State::WAITING_FOR_ROUTE },
  {{State::WAITING_FOR_ENGAGE, State::PLANNING          }, State::PLANNING          },
  {{State::WAITING_FOR_ENGAGE, State::WAITING_FOR_ENGAGE}, State::WAITING_FOR_ENGAGE},
  {{State::WAITING_FOR_ENGAGE, State::DRIVING           }, State::DRIVING           },
  {{State::WAITING_FOR_ENGAGE, State::ARRIVED_GOAL      }, State::ARRIVED_GOAL      },
  {{State::DRIVING           , State::INITIALIZING      }, State::DRIVING           },
  {{State::DRIVING           , State::WAITING_FOR_ROUTE }, State::WAITING_FOR_ROUTE },
  {{State::DRIVING           , State::PLANNING          }, State::PLANNING          },
  {{State::DRIVING           , State::WAITING_FOR_ENGAGE}, State::WAITING_FOR_ENGAGE},
  {{State::DRIVING           , State::DRIVING           }, State::DRIVING           },
  {{State::DRIVING           , State::ARRIVED_GOAL      }, State::ARRIVED_GOAL      },
  {{State::ARRIVED_GOAL      , State::INITIALIZING      }, State::ARRIVED_GOAL      },
  {{State::ARRIVED_GOAL      , State::WAITING_FOR_ROUTE }, State::WAITING_FOR_ROUTE },
  {{State::ARRIVED_GOAL      , State::PLANNING          }, State::WAITING_FOR_ROUTE },
  {{State::ARRIVED_GOAL      , State::WAITING_FOR_ENGAGE}, State::WAITING_FOR_ROUTE },
  {{State::ARRIVED_GOAL      , State::DRIVING           }, State::WAITING_FOR_ROUTE },
  {{State::ARRIVED_GOAL      , State::ARRIVED_GOAL      }, State::ARRIVED_GOAL      }
};
// clang-format on

}  // namespace transition

namespace default_ad_api
{

AutowareStateNode::AutowareStateNode(const rclcpp::NodeOptions & options)
: Node("autoware_state", options)
{
  const std::vector<std::string> module_names = {
    "sensing", "perception", "map", "localization", "planning", "control", "vehicle", "system",
  };

  for (size_t i = 0; i < module_names.size(); ++i) {
    const auto name = "/system/component_state_monitor/component/launch/" + module_names[i];
    const auto qos = rclcpp::QoS(1).transient_local();
    const auto callback = [this, i](const ModeChangeAvailable::ConstSharedPtr msg) {
      component_states_[i] = msg->available;
    };
    sub_component_states_.push_back(create_subscription<ModeChangeAvailable>(name, qos, callback));
  }

  pub_autoware_state_ = create_publisher<AutowareState>("/autoware/state", 1);
  srv_autoware_shutdown_ = create_service<std_srvs::srv::Trigger>(
    "/autoware/shutdown",
    std::bind(&AutowareStateNode::on_shutdown, this, std::placeholders::_1, std::placeholders::_2));

  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(sub_localization_, this, &AutowareStateNode::on_localization);
  adaptor.init_sub(sub_routing_, this, &AutowareStateNode::on_routing);
  adaptor.init_sub(sub_operation_mode_, this, &AutowareStateNode::on_operation_mode);

  // TODO(Takagi, Isamu): remove default value
  const auto rate = rclcpp::Rate(declare_parameter("update_rate", 10.0));
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });

  component_states_.resize(module_names.size());
  launch_state_ = LaunchState::Initializing;
  localization_state_.state = LocalizationState::UNKNOWN;
  routing_state_.state = RoutingState::UNKNOWN;
  operation_mode_state_.mode = OperationModeState::UNKNOWN;

  previous_stamp_ = now();
  previous_state_ = std::nullopt;
}

void AutowareStateNode::on_localization(const LocalizationState::ConstSharedPtr msg)
{
  localization_state_ = *msg;
}
void AutowareStateNode::on_routing(const RoutingState::ConstSharedPtr msg)
{
  routing_state_ = *msg;
}
void AutowareStateNode::on_operation_mode(const OperationModeState::ConstSharedPtr msg)
{
  operation_mode_state_ = *msg;
}

void AutowareStateNode::on_shutdown(
  const Trigger::Request::SharedPtr, const Trigger::Response::SharedPtr res)
{
  launch_state_ = LaunchState::Finalizing;
  res->success = true;
  res->message = "Shutdown Autoware.";
}

void AutowareStateNode::on_timer()
{
  const auto convert_state = [this]() {
    if (launch_state_ == LaunchState::Finalizing) {
      return AutowareState::FINALIZING;
    }
    if (launch_state_ == LaunchState::Initializing) {
      return AutowareState::INITIALIZING;
    }
    if (localization_state_.state == LocalizationState::UNKNOWN) {
      return AutowareState::INITIALIZING;
    }
    if (routing_state_.state == RoutingState::UNKNOWN) {
      return AutowareState::INITIALIZING;
    }
    if (operation_mode_state_.mode == OperationModeState::UNKNOWN) {
      return AutowareState::INITIALIZING;
    }
    if (localization_state_.state != LocalizationState::INITIALIZED) {
      return AutowareState::INITIALIZING;
    }
    if (routing_state_.state == RoutingState::UNSET) {
      return AutowareState::WAITING_FOR_ROUTE;
    }
    if (routing_state_.state == RoutingState::ARRIVED) {
      return AutowareState::ARRIVED_GOAL;
    }
    if (operation_mode_state_.mode != OperationModeState::STOP) {
      if (operation_mode_state_.is_autoware_control_enabled) {
        return AutowareState::DRIVING;
      }
    }
    if (operation_mode_state_.is_autonomous_mode_available) {
      return AutowareState::WAITING_FOR_ENGAGE;
    }
    return AutowareState::PLANNING;
  };

  // Update launch state.
  if (launch_state_ == LaunchState::Initializing) {
    bool is_initialized = true;
    for (const auto & state : component_states_) {
      is_initialized &= state;
    }
    if (is_initialized) {
      launch_state_ = LaunchState::Running;
    }
  }

  // Update routing state to reproduce old logic.
  if (routing_state_.state == RoutingState::ARRIVED) {
    const auto duration = (now() - rclcpp::Time(routing_state_.stamp)).seconds();
    if (transition::arrived_goal_timeout < duration) {
      routing_state_.state = RoutingState::UNSET;
    }
  }

  // Change next state to reproduce old logic.
  auto current_state = convert_state();
  if (previous_state_) {
    const auto pair = std::make_pair(previous_state_.value(), current_state);
    if (transition::matrix.count(pair)) {
      current_state = transition::matrix.at(pair);
    }
  }

  // Wait state change for sync to reproduce old logic.
  const auto current_stamp = now();
  if (previous_state_) {
    const auto duration = (current_stamp - previous_stamp_).seconds();
    const auto previous = previous_state_.value();
    if (duration < transition::get_wait_time(previous)) {
      current_state = previous;
    }
  }
  if (current_state != previous_state_) {
    previous_stamp_ = current_stamp;
    previous_state_ = current_state;
  }

  AutowareState msg;
  msg.stamp = current_stamp;
  msg.state = current_state;
  pub_autoware_state_->publish(msg);
}

}  // namespace default_ad_api

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(default_ad_api::AutowareStateNode)
