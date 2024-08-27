// Copyright 2024 TIER IV, Inc.
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

#include "autoware/universe_utils/system/time_keeper.hpp"

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <fmt/format.h>

#include <stdexcept>

namespace autoware::universe_utils
{

ProcessingTimeNode::ProcessingTimeNode(const std::string & name) : name_(name)
{
}

std::shared_ptr<ProcessingTimeNode> ProcessingTimeNode::add_child(const std::string & name)
{
  auto new_child_node = std::make_shared<ProcessingTimeNode>(name);
  new_child_node->parent_node_ = weak_from_this();
  child_nodes_.push_back(new_child_node);
  return new_child_node;
}

std::string ProcessingTimeNode::to_string() const
{
  std::function<void(
    const ProcessingTimeNode &, std::ostringstream &, const std::string &, bool, bool)>
    construct_string = [&](
                         const ProcessingTimeNode & node, std::ostringstream & oss,
                         const std::string & prefix, bool is_last, bool is_root) {
      if (!is_root) {
        oss << prefix << (is_last ? "└── " : "├── ");
      }
      if (!node.comment_.empty()) {
        oss << node.name_ << " (" << node.processing_time_ << "ms) : " << node.comment_ << "\n";
      } else {
        oss << node.name_ << " (" << node.processing_time_ << "ms)\n";
      }
      for (size_t i = 0; i < node.child_nodes_.size(); ++i) {
        const auto & child = node.child_nodes_[i];
        construct_string(
          *child, oss, prefix + (is_last ? "    " : "│   "), i == node.child_nodes_.size() - 1,
          false);
      }
    };

  std::ostringstream oss;
  construct_string(*this, oss, "", true, true);
  return oss.str();
}

tier4_debug_msgs::msg::ProcessingTimeTree ProcessingTimeNode::to_msg() const
{
  tier4_debug_msgs::msg::ProcessingTimeTree time_tree_msg;

  std::function<void(const ProcessingTimeNode &, tier4_debug_msgs::msg::ProcessingTimeTree &, int)>
    construct_msg = [&](
                      const ProcessingTimeNode & node,
                      tier4_debug_msgs::msg::ProcessingTimeTree & tree_msg, int parent_id) {
      tier4_debug_msgs::msg::ProcessingTimeNode time_node_msg;
      time_node_msg.name = node.name_;
      time_node_msg.processing_time = node.processing_time_;
      time_node_msg.id = static_cast<int>(tree_msg.nodes.size() + 1);
      time_node_msg.parent_id = parent_id;
      time_node_msg.comment = node.comment_;
      tree_msg.nodes.emplace_back(time_node_msg);

      for (const auto & child : node.child_nodes_) {
        construct_msg(*child, tree_msg, time_node_msg.id);
      }
    };
  construct_msg(*this, time_tree_msg, 0);

  return time_tree_msg;
}

std::weak_ptr<ProcessingTimeNode> ProcessingTimeNode::get_parent_node() const
{
  return parent_node_;
}
std::vector<std::shared_ptr<ProcessingTimeNode>> ProcessingTimeNode::get_child_nodes() const
{
  return child_nodes_;
}
void ProcessingTimeNode::set_time(const double processing_time)
{
  processing_time_ = processing_time;
}

void ProcessingTimeNode::set_comment(const std::string & comment)
{
  comment_ = comment;
}

std::string ProcessingTimeNode::get_name() const
{
  return name_;
}

void TimeKeeper::add_reporter(std::ostream * os)
{
  reporters_.emplace_back([os](const std::shared_ptr<ProcessingTimeNode> & node) {
    *os << "==========================" << std::endl;
    *os << node->to_string() << std::endl;
  });
}

void TimeKeeper::add_reporter(rclcpp::Publisher<ProcessingTimeDetail>::SharedPtr publisher)
{
  reporters_.emplace_back([publisher](const std::shared_ptr<ProcessingTimeNode> & node) {
    publisher->publish(node->to_msg());
  });
}

void TimeKeeper::start_track(const std::string & func_name)
{
  if (current_time_node_ == nullptr) {
    current_time_node_ = std::make_shared<ProcessingTimeNode>(func_name);
    root_node_ = current_time_node_;
    root_node_thread_id_ = std::this_thread::get_id();
  } else {
    if (root_node_thread_id_ != std::this_thread::get_id()) {
      RCLCPP_WARN(
        rclcpp::get_logger("TimeKeeper"),
        "TimeKeeper::start_track() is called from a different thread. Ignoring the call.");
      return;
    }
    current_time_node_ = current_time_node_->add_child(func_name);
  }
  stop_watch_.tic(func_name);
}

void TimeKeeper::comment(const std::string & comment)
{
  if (current_time_node_ == nullptr) {
    throw std::runtime_error("You must call start_track() first, but comment() is called");
  }
  current_time_node_->set_comment(comment);
}

void TimeKeeper::end_track(const std::string & func_name)
{
  if (root_node_thread_id_ != std::this_thread::get_id()) {
    return;
  }
  if (current_time_node_->get_name() != func_name) {
    throw std::runtime_error(fmt::format(
      "You must call end_track({}) first, but end_track({}) is called",
      current_time_node_->get_name(), func_name));
  }
  const double processing_time = stop_watch_.toc(func_name);
  current_time_node_->set_time(processing_time);
  current_time_node_ = current_time_node_->get_parent_node().lock();

  if (current_time_node_ == nullptr) {
    report();
  }
}

void TimeKeeper::report()
{
  if (current_time_node_ != nullptr) {
    throw std::runtime_error(fmt::format(
      "You must call end_track({}) first, but report() is called", current_time_node_->get_name()));
  }
  for (const auto & reporter : reporters_) {
    reporter(root_node_);
  }
  current_time_node_.reset();
  root_node_.reset();
}

ScopedTimeTrack::ScopedTimeTrack(const std::string & func_name, TimeKeeper & time_keeper)
: func_name_(func_name), time_keeper_(time_keeper)
{
  time_keeper_.start_track(func_name_);
}

ScopedTimeTrack::~ScopedTimeTrack()  // NOLINT
{
  time_keeper_.end_track(func_name_);
}

}  // namespace autoware::universe_utils
