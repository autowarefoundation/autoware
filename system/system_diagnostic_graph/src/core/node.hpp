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

#ifndef CORE__NODE_HPP_
#define CORE__NODE_HPP_

#include "config.hpp"
#include "debug.hpp"
#include "types.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class BaseNode
{
public:
  virtual ~BaseNode() = default;
  virtual void update() = 0;
  virtual DiagnosticNode report() const = 0;
  virtual DiagDebugData debug() const = 0;
  virtual std::vector<BaseNode *> links() const = 0;

  DiagnosticLevel level() const { return node_.status.level; }
  std::string name() const { return node_.status.name; }

  size_t index() const { return index_; }
  void set_index(const size_t index) { index_ = index; }

protected:
  size_t index_ = 0;
  DiagnosticNode node_;
};

class UnitNode : public BaseNode
{
public:
  explicit UnitNode(const std::string & name);
  ~UnitNode() override;

  DiagnosticNode report() const override;
  DiagDebugData debug() const override;
  void update() override;
  void create(Graph & graph, const NodeConfig & config);

  std::vector<BaseNode *> links() const override;

private:
  std::unique_ptr<BaseExpr> expr_;
};

class DiagNode : public BaseNode
{
public:
  explicit DiagNode(const std::string & name, const std::string & hardware);

  DiagnosticNode report() const override;
  DiagDebugData debug() const override;
  void update() override;
  void callback(const DiagnosticStatus & status);

  std::vector<BaseNode *> links() const override { return {}; }

private:
};

}  // namespace system_diagnostic_graph

#endif  // CORE__NODE_HPP_
