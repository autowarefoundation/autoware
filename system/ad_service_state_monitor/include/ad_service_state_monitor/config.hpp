// Copyright 2020 Tier IV, Inc.
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

#ifndef AD_SERVICE_STATE_MONITOR__CONFIG_HPP_
#define AD_SERVICE_STATE_MONITOR__CONFIG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <utility>
#include <vector>

struct TopicConfig
{
  explicit TopicConfig(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
    const std::string & namespace_prefix, const std::string & name)
  : module(
      interface->declare_parameter(namespace_prefix + ".module",
      rclcpp::PARAMETER_STRING).get<std::string>()),
    name(name),
    type(
      interface->declare_parameter(namespace_prefix + ".type",
      rclcpp::PARAMETER_STRING).get<std::string>()),
    transient_local(
      interface->declare_parameter(namespace_prefix + ".transient_local",
      static_cast<rclcpp::ParameterValue>(false)).get<bool>()),
    best_effort(
      interface->declare_parameter(namespace_prefix + ".best_effort",
      static_cast<rclcpp::ParameterValue>(false)).get<bool>()),
    timeout(
      interface->declare_parameter(namespace_prefix + ".timeout",
      rclcpp::PARAMETER_DOUBLE).get<double>()),
    warn_rate(
      interface->declare_parameter(namespace_prefix + ".warn_rate",
      rclcpp::PARAMETER_DOUBLE).get<double>())
  {
  }

  std::string module;
  std::string name;
  std::string type;
  bool transient_local;
  bool best_effort;
  double timeout;
  double warn_rate;
};

struct ParamConfig
{
  explicit ParamConfig(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
    const std::string & namespace_prefix, const std::string & name)
  : module(interface->declare_parameter(namespace_prefix + ".module", rclcpp::PARAMETER_STRING)
             .get<std::string>()),
    name(name)
  {
  }

  std::string module;
  std::string name;
};

struct TfConfig
{
  explicit TfConfig(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr interface,
    const std::string & namespace_prefix, [[maybe_unused]] const std::string & name)
  : module(interface->declare_parameter(namespace_prefix + ".module", rclcpp::PARAMETER_STRING)
             .get<std::string>()),
    from(interface->declare_parameter(namespace_prefix + ".from", rclcpp::PARAMETER_STRING)
           .get<std::string>()),
    to(interface->declare_parameter(namespace_prefix + ".to", rclcpp::PARAMETER_STRING)
         .get<std::string>()),
    timeout(interface->declare_parameter(namespace_prefix + ".timeout", rclcpp::PARAMETER_DOUBLE)
              .get<double>())
  {
  }

  std::string module;
  std::string from;
  std::string to;
  double timeout;
};

struct TopicStats
{
  rclcpp::Time checked_time;
  std::vector<TopicConfig> ok_list;
  std::vector<TopicConfig> non_received_list;
  std::vector<std::pair<TopicConfig, rclcpp::Time>> timeout_list;  // pair<TfConfig, last_received>
  std::vector<std::pair<TopicConfig, double>> slow_rate_list;      // pair<TfConfig, rate>
};

struct ParamStats
{
  rclcpp::Time checked_time;
  std::vector<ParamConfig> ok_list;
  std::vector<ParamConfig> non_set_list;
};

struct TfStats
{
  rclcpp::Time checked_time;
  std::vector<TfConfig> ok_list;
  std::vector<TfConfig> non_received_list;
  std::vector<std::pair<TfConfig, rclcpp::Time>> timeout_list;  // pair<TfConfig, last_received>
};

#endif  // AD_SERVICE_STATE_MONITOR__CONFIG_HPP_
