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

#include "logging.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace diagnostic_graph_utils
{

LoggingNode::LoggingNode(const rclcpp::NodeOptions & options) : Node("logging", options)
{
  root_path_ = declare_parameter<std::string>("root_path");
  max_depth_ = declare_parameter<int>("max_depth");

  using std::placeholders::_1;
  sub_graph_.register_create_callback(std::bind(&LoggingNode::on_create, this, _1));
  sub_graph_.subscribe(*this, 1);

  const auto period = rclcpp::Rate(declare_parameter<double>("show_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void LoggingNode::on_create(DiagGraph::ConstSharedPtr graph)
{
  // Search root node.
  root_unit_ = nullptr;
  for (const auto & unit : graph->units()) {
    if (unit->path() == root_path_) {
      root_unit_ = unit;
      return;
    }
  }
  RCLCPP_ERROR_STREAM(get_logger(), "root unit is not found: " << root_path_);
}

void LoggingNode::on_timer()
{
  static const auto message = "The target mode is not available for the following reasons:";
  if (root_unit_ && root_unit_->level() != DiagUnit::DiagnosticStatus::OK) {
    dump_text_.str("");
    dump_text_.clear(std::stringstream::goodbit);
    dump_unit(root_unit_, 0, "  ");
    RCLCPP_WARN_STREAM(get_logger(), message << std::endl << dump_text_.str());
  }
}

void LoggingNode::dump_unit(DiagUnit * unit, int depth, const std::string & indent)
{
  const auto text_level = [](DiagUnit::DiagnosticLevel level) {
    if (level == DiagUnit::DiagnosticStatus::OK) return "OK   ";
    if (level == DiagUnit::DiagnosticStatus::WARN) return "WARN ";
    if (level == DiagUnit::DiagnosticStatus::ERROR) return "ERROR";
    if (level == DiagUnit::DiagnosticStatus::STALE) return "STALE";
    return "-----";
  };

  if (max_depth_ < depth) {
    return;
  }
  if (unit->level() == DiagUnit::DiagnosticStatus::OK) {
    return;
  }

  std::string path = unit->path();
  if (path.empty()) {
    path = "[anonymous group]";
  }

  dump_text_ << indent << "- " + path << " " << text_level(unit->level()) << std::endl;
  for (const auto & child : unit->children()) {
    dump_unit(child.unit, depth + 1, indent + "  ");
  }
}

}  // namespace diagnostic_graph_utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(diagnostic_graph_utils::LoggingNode)
