// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__GRAPH__UNITS_HPP_
#define COMMON__GRAPH__UNITS_HPP_

#include "names.hpp"
#include "types.hpp"

#include <rclcpp/time.hpp>

#include <optional>
#include <string>
#include <vector>

namespace diagnostic_graph_aggregator
{

class UnitLink
{
public:
  UnitLink() : parent_(nullptr), child_(nullptr), struct_(), status_() {}

  void initialize_object(BaseUnit * parent, BaseUnit * child);
  void initialize_struct();
  void initialize_status();  // cppcheck-suppress functionStatic
  DiagLinkStruct create_struct() const { return struct_; }
  DiagLinkStatus create_status() const { return status_; }
  BaseUnit * parent() const { return parent_; }
  BaseUnit * child() const { return child_; }

private:
  BaseUnit * parent_;
  BaseUnit * child_;
  DiagLinkStruct struct_;
  DiagLinkStatus status_;
};

class BaseUnit
{
public:
  explicit BaseUnit(const UnitLoader & unit);
  virtual ~BaseUnit() = default;
  virtual DiagnosticLevel level() const = 0;
  virtual std::string path() const = 0;
  virtual std::string type() const = 0;
  virtual std::vector<UnitLink *> child_links() const = 0;
  virtual bool is_leaf() const = 0;
  size_t index() const { return index_; }
  size_t parent_size() const { return parents_.size(); }

protected:
  bool update();

private:
  virtual void update_status() = 0;
  size_t index_;
  std::vector<UnitLink *> parents_;
  std::optional<DiagnosticLevel> prev_level_;
};

class NodeUnit : public BaseUnit
{
public:
  explicit NodeUnit(const UnitLoader & unit);
  void initialize_struct();
  void initialize_status();
  bool is_leaf() const override { return false; }
  DiagNodeStruct create_struct() const { return struct_; }
  DiagNodeStatus create_status() const { return status_; }
  DiagnosticLevel level() const override { return status_.level; }
  std::string path() const override { return struct_.path; }

protected:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
};

class LeafUnit : public BaseUnit
{
public:
  explicit LeafUnit(const UnitLoader & unit);
  void initialize_struct();
  void initialize_status();
  bool is_leaf() const override { return true; }
  DiagLeafStruct create_struct() const { return struct_; }
  DiagLeafStatus create_status() const { return status_; }
  DiagnosticLevel level() const override { return status_.level; }
  std::string name() const { return struct_.name; }
  std::string path() const override { return struct_.path; }

protected:
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
};

class DiagUnit : public LeafUnit
{
public:
  explicit DiagUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::diag; }
  std::vector<UnitLink *> child_links() const override { return {}; }
  bool on_time(const rclcpp::Time & stamp);
  bool on_diag(const rclcpp::Time & stamp, const DiagnosticStatus & status);

private:
  void update_status() override;
  double timeout_;
  std::optional<rclcpp::Time> last_updated_time_;
};

class MaxUnit : public NodeUnit
{
public:
  explicit MaxUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::max; }
  std::vector<UnitLink *> child_links() const override { return links_; }

protected:
  std::vector<UnitLink *> links_;

private:
  void update_status() override;
};

class ShortCircuitMaxUnit : public MaxUnit
{
public:
  using MaxUnit::MaxUnit;
  std::string type() const override { return unit_name::short_circuit_max; }

private:
  void update_status() override;  // cppcheck-suppress uselessOverride
};

class MinUnit : public NodeUnit
{
public:
  explicit MinUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::min; }
  std::vector<UnitLink *> child_links() const override { return links_; }

protected:
  std::vector<UnitLink *> links_;

private:
  void update_status() override;
};

class RemapUnit : public NodeUnit
{
public:
  explicit RemapUnit(const UnitLoader & unit);
  std::vector<UnitLink *> child_links() const override { return {link_}; }

protected:
  UnitLink * link_;
  DiagnosticLevel level_from_;
  DiagnosticLevel level_to_;

private:
  void update_status() override;
};

class WarnToOkUnit : public RemapUnit
{
public:
  explicit WarnToOkUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::warn_to_ok; }
};

class WarnToErrorUnit : public RemapUnit
{
public:
  explicit WarnToErrorUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::warn_to_error; }
};

class ConstUnit : public NodeUnit
{
public:
  using NodeUnit::NodeUnit;
  std::vector<UnitLink *> child_links() const override { return {}; }

private:
  void update_status() override;
};

class OkUnit : public ConstUnit
{
public:
  explicit OkUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::ok; }
};

class WarnUnit : public ConstUnit
{
public:
  explicit WarnUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::warn; }
};

class ErrorUnit : public ConstUnit
{
public:
  explicit ErrorUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::error; }
};

class StaleUnit : public ConstUnit
{
public:
  explicit StaleUnit(const UnitLoader & unit);
  std::string type() const override { return unit_name::stale; }
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__UNITS_HPP_
