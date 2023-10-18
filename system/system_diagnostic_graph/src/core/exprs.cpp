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

#include "exprs.hpp"

#include "nodes.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// DEBUG
#include <iostream>

namespace system_diagnostic_graph
{

using LinkStatus = std::vector<std::pair<BaseNode *, bool>>;

void extend(LinkStatus & a, const LinkStatus & b)
{
  a.insert(a.end(), b.begin(), b.end());
}

void extend_false(LinkStatus & a, const LinkStatus & b)
{
  for (const auto & p : b) {
    a.push_back(std::make_pair(p.first, false));
  }
}

auto create_expr_list(ExprInit & exprs, ConfigObject & config)
{
  std::vector<std::unique_ptr<BaseExpr>> result;
  const auto list = config.take_list("list");
  for (size_t i = 0; i < list.size(); ++i) {
    auto dict = parse_expr_config(config.mark().index(i), list[i]);
    auto expr = exprs.create(dict);
    if (expr) {
      result.push_back(std::move(expr));
    }
  }
  return result;
}

ConstExpr::ConstExpr(const DiagnosticLevel level)
{
  level_ = level;
}

ExprStatus ConstExpr::eval() const
{
  ExprStatus status;
  status.level = level_;
  return status;
}

std::vector<BaseNode *> ConstExpr::get_dependency() const
{
  return {};
}

LinkExpr::LinkExpr(ExprInit & exprs, ConfigObject & config)
{
  // TODO(Takagi, Isamu): remove
  (void)config;
  (void)exprs;
}

void LinkExpr::init(ConfigObject & config, std::unordered_map<std::string, BaseNode *> nodes)
{
  const auto path = config.take_text("path");
  if (!nodes.count(path)) {
    throw ConfigError("node path '" + path + "' does not exist");
  }
  node_ = nodes.at(path);
}

ExprStatus LinkExpr::eval() const
{
  ExprStatus status;
  status.level = node_->level();
  status.links.push_back(std::make_pair(node_, true));
  return status;
}

std::vector<BaseNode *> LinkExpr::get_dependency() const
{
  return {node_};
}

AndExpr::AndExpr(ExprInit & exprs, ConfigObject & config, bool short_circuit)
{
  list_ = create_expr_list(exprs, config);
  short_circuit_ = short_circuit;
}

ExprStatus AndExpr::eval() const
{
  if (list_.empty()) {
    ExprStatus status;
    status.level = DiagnosticStatus::OK;
    return status;
  }

  ExprStatus status;
  status.level = DiagnosticStatus::OK;
  for (const auto & expr : list_) {
    const auto result = expr->eval();
    status.level = std::max(status.level, result.level);
    extend(status.links, result.links);
    if (short_circuit_ && status.level != DiagnosticStatus::OK) {
      break;
    }
  }
  status.level = std::min(status.level, DiagnosticStatus::ERROR);
  return status;
}

std::vector<BaseNode *> AndExpr::get_dependency() const
{
  std::vector<BaseNode *> depends;
  for (const auto & expr : list_) {
    const auto nodes = expr->get_dependency();
    depends.insert(depends.end(), nodes.begin(), nodes.end());
  }
  return depends;
}

OrExpr::OrExpr(ExprInit & exprs, ConfigObject & config)
{
  list_ = create_expr_list(exprs, config);
}

ExprStatus OrExpr::eval() const
{
  if (list_.empty()) {
    ExprStatus status;
    status.level = DiagnosticStatus::OK;
    return status;
  }

  ExprStatus status;
  status.level = DiagnosticStatus::ERROR;
  for (const auto & expr : list_) {
    const auto result = expr->eval();
    status.level = std::min(status.level, result.level);
    extend(status.links, result.links);
  }
  status.level = std::min(status.level, DiagnosticStatus::ERROR);
  return status;
}

std::vector<BaseNode *> OrExpr::get_dependency() const
{
  std::vector<BaseNode *> depends;
  for (const auto & expr : list_) {
    const auto nodes = expr->get_dependency();
    depends.insert(depends.end(), nodes.begin(), nodes.end());
  }
  return depends;
}

ExprInit::ExprInit(const std::string & mode)
{
  mode_ = mode;
}

std::unique_ptr<BaseExpr> ExprInit::create(ExprConfig config)
{
  if (!config.mode.check(mode_)) {
    return nullptr;
  }
  if (config.type == "link") {
    auto expr = std::make_unique<LinkExpr>(*this, config.dict);
    links_.push_back(std::make_pair(expr.get(), config.dict));
    return expr;
  }
  if (config.type == "and") {
    return std::make_unique<AndExpr>(*this, config.dict, false);
  }
  if (config.type == "short-circuit-and") {
    return std::make_unique<AndExpr>(*this, config.dict, true);
  }
  if (config.type == "or") {
    return std::make_unique<OrExpr>(*this, config.dict);
  }
  if (config.type == "debug-ok") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::OK);
  }
  if (config.type == "debug-warn") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::WARN);
  }
  if (config.type == "debug-error") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::ERROR);
  }
  if (config.type == "debug-stale") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::STALE);
  }
  throw ConfigError("unknown expr type: " + config.type + " " + config.dict.mark().str());
}

}  // namespace system_diagnostic_graph
