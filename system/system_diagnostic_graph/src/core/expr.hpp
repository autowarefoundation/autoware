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

#ifndef CORE__EXPR_HPP_
#define CORE__EXPR_HPP_

#include "types.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

struct ExprStatus
{
  DiagnosticLevel level;
  std::vector<std::pair<BaseNode *, bool>> links;
};

class BaseExpr
{
public:
  static std::unique_ptr<BaseExpr> create(Graph & graph, YAML::Node yaml);
  virtual ~BaseExpr() = default;
  virtual ExprStatus eval() const = 0;
  virtual std::vector<BaseNode *> get_dependency() const = 0;
};

class ConstExpr : public BaseExpr
{
public:
  explicit ConstExpr(const DiagnosticLevel level);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  DiagnosticLevel level_;
};

class UnitExpr : public BaseExpr
{
public:
  UnitExpr(Graph & graph, YAML::Node yaml);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  UnitNode * node_;
};

class DiagExpr : public BaseExpr
{
public:
  DiagExpr(Graph & graph, YAML::Node yaml);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  DiagNode * node_;
};

class AndExpr : public BaseExpr
{
public:
  AndExpr(Graph & graph, YAML::Node yaml);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  std::vector<std::unique_ptr<BaseExpr>> list_;
};

class OrExpr : public BaseExpr
{
public:
  OrExpr(Graph & graph, YAML::Node yaml);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  std::vector<std::unique_ptr<BaseExpr>> list_;
};

class IfExpr : public BaseExpr
{
public:
  IfExpr(Graph & graph, YAML::Node yaml);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  std::unique_ptr<BaseExpr> cond_;
  std::unique_ptr<BaseExpr> then_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__EXPR_HPP_
