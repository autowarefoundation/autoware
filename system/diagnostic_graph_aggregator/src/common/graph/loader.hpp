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

#ifndef COMMON__GRAPH__LOADER_HPP_
#define COMMON__GRAPH__LOADER_HPP_

#include "types.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace diagnostic_graph_aggregator
{

class UnitLoader
{
public:
  struct GraphLinks;
  UnitLoader(UnitConfig * config, GraphLinks & links) : config_(config), links_(links) {}
  const std::string & path() const;
  const std::string & type() const;
  size_t index() const;
  TreeData & data() const;
  UnitLink * child() const;
  std::vector<UnitLink *> children() const;
  std::vector<UnitLink *> parents() const;

private:
  UnitConfig * config_;
  GraphLinks & links_;
};

class GraphLoader
{
public:
  explicit GraphLoader(const std::string & file);
  std::vector<std::unique_ptr<NodeUnit>> release_nodes();
  std::vector<std::unique_ptr<DiagUnit>> release_diags();
  std::vector<std::unique_ptr<UnitLink>> release_links();

private:
  std::unique_ptr<UnitLink> create_link();
  std::unique_ptr<DiagUnit> create_diag(const UnitLoader & unit);
  std::unique_ptr<NodeUnit> create_node(const UnitLoader & unit);

  // Note: keep order correspondence between links and unit children for viewer.
  std::vector<std::unique_ptr<NodeUnit>> nodes_;
  std::vector<std::unique_ptr<DiagUnit>> diags_;
  std::vector<std::unique_ptr<UnitLink>> links_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LOADER_HPP_
