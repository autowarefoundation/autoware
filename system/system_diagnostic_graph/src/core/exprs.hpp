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

#ifndef CORE__EXPRS_HPP_
#define CORE__EXPRS_HPP_

#include "config.hpp"
#include "types.hpp"

#include <memory>
#include <string>
#include <unordered_map>
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

class LinkExpr : public BaseExpr
{
public:
  LinkExpr(ExprInit & exprs, ConfigObject & config);
  void init(ConfigObject & config, std::unordered_map<std::string, BaseNode *> nodes);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  BaseNode * node_;
};

class AndExpr : public BaseExpr
{
public:
  AndExpr(ExprInit & exprs, ConfigObject & config, bool short_circuit);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  bool short_circuit_;
  std::vector<std::unique_ptr<BaseExpr>> list_;
};

class OrExpr : public BaseExpr
{
public:
  OrExpr(ExprInit & exprs, ConfigObject & config);
  ExprStatus eval() const override;
  std::vector<BaseNode *> get_dependency() const override;

private:
  std::vector<std::unique_ptr<BaseExpr>> list_;
};

class ExprInit
{
public:
  explicit ExprInit(const std::string & mode);
  std::unique_ptr<BaseExpr> create(ExprConfig config);
  auto get() const { return links_; }

private:
  std::string mode_;
  std::vector<std::pair<LinkExpr *, ConfigObject>> links_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__EXPRS_HPP_
