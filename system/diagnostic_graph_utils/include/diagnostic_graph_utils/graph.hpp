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

#ifndef DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_
#define DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_

#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace diagnostic_graph_utils
{

class DiagLink;
class DiagUnit
{
public:
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticLevel = DiagnosticStatus::_level_type;

  struct DiagChild
  {
    DiagLink * link;
    DiagUnit * unit;
  };

  virtual DiagnosticStatus create_diagnostic_status() const = 0;
  virtual DiagnosticLevel level() const = 0;
  virtual std::string type() const = 0;
  virtual std::string path() const = 0;
  virtual std::vector<DiagChild> children() const = 0;
};

class DiagLink
{
public:
  using DiagLinkStruct = tier4_system_msgs::msg::DiagLinkStruct;
  using DiagLinkStatus = tier4_system_msgs::msg::DiagLinkStatus;
  DiagLink(const DiagLinkStruct & msg, DiagUnit * parent, DiagUnit * child) : struct_(msg)
  {
    parent_ = parent;
    child_ = child;
  }
  void update(const DiagLinkStatus & msg) { status_ = msg; }
  DiagUnit * parent() const { return parent_; }
  DiagUnit * child() const { return child_; }

private:
  DiagLinkStruct struct_;
  DiagLinkStatus status_;
  DiagUnit * parent_;
  DiagUnit * child_;
};

class DiagNode : public DiagUnit
{
public:
  using DiagNodeStruct = tier4_system_msgs::msg::DiagNodeStruct;
  using DiagNodeStatus = tier4_system_msgs::msg::DiagNodeStatus;
  explicit DiagNode(const DiagNodeStruct & msg) : struct_(msg) {}
  void update(const DiagNodeStatus & msg) { status_ = msg; }

  DiagnosticStatus create_diagnostic_status() const override;
  DiagnosticLevel level() const override { return status_.level; }
  std::string type() const override { return struct_.type; }
  std::string path() const override { return struct_.path; }
  std::vector<DiagChild> children() const override { return children_; }
  void add_child(const DiagChild & child) { children_.push_back(child); }

private:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
  std::vector<DiagChild> children_;
};

class DiagLeaf : public DiagUnit
{
public:
  using DiagLeafStruct = tier4_system_msgs::msg::DiagLeafStruct;
  using DiagLeafStatus = tier4_system_msgs::msg::DiagLeafStatus;
  explicit DiagLeaf(const DiagLeafStruct & msg) : struct_(msg) {}
  void update(const DiagLeafStatus & msg) { status_ = msg; }

  DiagnosticStatus create_diagnostic_status() const override;
  DiagnosticLevel level() const override { return status_.level; }
  std::string type() const override { return struct_.type; }
  std::string path() const override { return struct_.path; }
  std::vector<DiagChild> children() const override { return {}; }

private:
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
};

class DiagGraph
{
public:
  using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
  using SharedPtr = std::shared_ptr<DiagGraph>;
  using ConstSharedPtr = std::shared_ptr<const DiagGraph>;
  void create(const DiagGraphStruct & msg);
  bool update(const DiagGraphStatus & msg);
  rclcpp::Time created_stamp() const { return created_stamp_; }
  rclcpp::Time updated_stamp() const { return updated_stamp_; }
  std::string id() const { return id_; }
  std::vector<DiagUnit *> units() const;
  std::vector<DiagNode *> nodes() const;
  std::vector<DiagLeaf *> diags() const;
  std::vector<DiagLink *> links() const;

private:
  rclcpp::Time created_stamp_;
  rclcpp::Time updated_stamp_;
  std::string id_;
  std::vector<std::unique_ptr<DiagNode>> nodes_;
  std::vector<std::unique_ptr<DiagLeaf>> diags_;
  std::vector<std::unique_ptr<DiagLink>> links_;
};

}  // namespace diagnostic_graph_utils

#endif  // DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_
