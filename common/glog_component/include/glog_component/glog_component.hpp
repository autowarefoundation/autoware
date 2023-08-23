// Copyright 2023 TIER IV, Inc.
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

#ifndef GLOG_COMPONENT__GLOG_COMPONENT_HPP_
#define GLOG_COMPONENT__GLOG_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <glog/logging.h>

class GlogComponent : public rclcpp::Node
{
public:
  explicit GlogComponent(const rclcpp::NodeOptions & node_options);
};

#endif  // GLOG_COMPONENT__GLOG_COMPONENT_HPP_
