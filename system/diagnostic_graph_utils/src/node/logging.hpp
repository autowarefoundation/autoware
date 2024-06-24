// Copyright 2024 The Autoware Contributors
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

#ifndef NODE__LOGGING_HPP_
#define NODE__LOGGING_HPP_

#include "diagnostic_graph_utils/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>

namespace diagnostic_graph_utils
{

class LoggingNode : public rclcpp::Node
{
public:
  explicit LoggingNode(const rclcpp::NodeOptions & options);

private:
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_timer();
  void dump_unit(DiagUnit * unit, int depth, const std::string & indent);
  DiagGraphSubscription sub_graph_;
  rclcpp::TimerBase::SharedPtr timer_;

  DiagUnit * root_unit_;
  int max_depth_;
  std::string root_path_;
  std::ostringstream dump_text_;
};

}  // namespace diagnostic_graph_utils

#endif  // NODE__LOGGING_HPP_
