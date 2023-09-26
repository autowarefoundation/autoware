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

#include "expr.hpp"

#include "config.hpp"
#include "graph.hpp"
#include "node.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

//
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

std::unique_ptr<BaseExpr> BaseExpr::create(Graph & graph, YAML::Node yaml)
{
  if (!yaml.IsMap()) {
    throw ConfigError("expr object is not a dict");
  }
  if (!yaml["type"]) {
    throw ConfigError("expr object has no 'type' field");
  }

  const auto type = take<std::string>(yaml, "type");

  if (type == "unit") {
    return std::make_unique<UnitExpr>(graph, yaml);
  }
  if (type == "diag") {
    return std::make_unique<DiagExpr>(graph, yaml);
  }
  if (type == "and") {
    return std::make_unique<AndExpr>(graph, yaml);
  }
  if (type == "or") {
    return std::make_unique<OrExpr>(graph, yaml);
  }
  if (type == "if") {
    return std::make_unique<IfExpr>(graph, yaml);
  }
  if (type == "debug-ok") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::OK);
  }
  if (type == "debug-warn") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::WARN);
  }
  if (type == "debug-error") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::ERROR);
  }
  if (type == "debug-stale") {
    return std::make_unique<ConstExpr>(DiagnosticStatus::STALE);
  }
  throw ConfigError("unknown expr type: " + type);
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

UnitExpr::UnitExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["name"]) {
    throw ConfigError("unit object has no 'name' field");
  }
  const auto name = take<std::string>(yaml, "name");
  node_ = graph.find_unit(name);
  if (!node_) {
    throw ConfigError("unit node '" + name + "' does not exist");
  }
}

ExprStatus UnitExpr::eval() const
{
  ExprStatus status;
  status.level = node_->level();
  status.links.push_back(std::make_pair(node_, true));
  return status;
}

std::vector<BaseNode *> UnitExpr::get_dependency() const
{
  return {node_};
}

DiagExpr::DiagExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["name"]) {
    throw ConfigError("diag object has no 'name' field");
  }
  const auto name = yaml["name"].as<std::string>();
  const auto hardware = yaml["hardware"].as<std::string>("");
  node_ = graph.find_diag(name, hardware);
  if (!node_) {
    node_ = graph.make_diag(name, hardware);
  }
}

ExprStatus DiagExpr::eval() const
{
  ExprStatus status;
  status.level = node_->level();
  status.links.push_back(std::make_pair(node_, true));
  return status;
}

std::vector<BaseNode *> DiagExpr::get_dependency() const
{
  return {node_};
}

AndExpr::AndExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["list"]) {
    throw ConfigError("expr object has no 'list' field");
  }
  if (!yaml["list"].IsSequence()) {
    throw ConfigError("list field is not a list");
  }

  for (const auto & node : yaml["list"]) {
    list_.push_back(BaseExpr::create(graph, node));
  }
}

ExprStatus AndExpr::eval() const
{
  std::vector<ExprStatus> results;
  for (const auto & expr : list_) {
    results.push_back(expr->eval());
  }
  std::vector<DiagnosticLevel> levels;
  for (const auto & result : results) {
    levels.push_back(result.level);
  }
  ExprStatus status;
  for (const auto & result : results) {
    extend(status.links, result.links);
  }
  const auto level = *std::max_element(levels.begin(), levels.end());
  status.level = std::min(level, DiagnosticStatus::ERROR);
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

OrExpr::OrExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["list"]) {
    throw ConfigError("expr object has no 'list' field");
  }
  if (!yaml["list"].IsSequence()) {
    throw ConfigError("list field is not a list");
  }

  for (const auto & node : yaml["list"]) {
    list_.push_back(BaseExpr::create(graph, node));
  }
}

ExprStatus OrExpr::eval() const
{
  std::vector<ExprStatus> results;
  for (const auto & expr : list_) {
    results.push_back(expr->eval());
  }
  std::vector<DiagnosticLevel> levels;
  for (const auto & result : results) {
    levels.push_back(result.level);
  }
  ExprStatus status;
  for (const auto & result : results) {
    extend(status.links, result.links);
  }
  const auto level = *std::min_element(levels.begin(), levels.end());
  status.level = std::min(level, DiagnosticStatus::ERROR);
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

IfExpr::IfExpr(Graph & graph, YAML::Node yaml)
{
  if (!yaml["cond"]) {
    throw ConfigError("expr object has no 'cond' field");
  }
  if (!yaml["then"]) {
    throw ConfigError("expr object has no 'then' field");
  }
  cond_ = BaseExpr::create(graph, yaml["cond"]);
  then_ = BaseExpr::create(graph, yaml["then"]);
}

ExprStatus IfExpr::eval() const
{
  const auto cond = cond_->eval();
  const auto then = then_->eval();
  ExprStatus status;
  if (cond.level == DiagnosticStatus::OK) {
    status.level = std::min(then.level, DiagnosticStatus::ERROR);
    extend(status.links, cond.links);
    extend(status.links, then.links);
  } else {
    status.level = std::min(cond.level, DiagnosticStatus::ERROR);
    extend(status.links, cond.links);
    extend_false(status.links, then.links);
  }
  return status;
}

std::vector<BaseNode *> IfExpr::get_dependency() const
{
  std::vector<BaseNode *> depends;
  {
    const auto nodes = cond_->get_dependency();
    depends.insert(depends.end(), nodes.begin(), nodes.end());
  }
  {
    const auto nodes = then_->get_dependency();
    depends.insert(depends.end(), nodes.begin(), nodes.end());
  }
  return depends;
}

}  // namespace system_diagnostic_graph
