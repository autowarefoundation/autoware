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

#ifndef CORE__GRAPH_HPP_
#define CORE__GRAPH_HPP_

#include "types.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace system_diagnostic_graph
{

class Graph
{
public:
  UnitNode * make_unit(const std::string & name);
  UnitNode * find_unit(const std::string & name);
  DiagNode * make_diag(const std::string & name, const std::string & hardware);
  DiagNode * find_diag(const std::string & name, const std::string & hardware);
  void topological_sort();
  const std::vector<std::unique_ptr<BaseNode>> & nodes() { return nodes_; }

private:
  std::vector<std::unique_ptr<BaseNode>> nodes_;
  std::map<std::string, UnitNode *> units_;
  std::map<std::pair<std::string, std::string>, DiagNode *> diags_;
};

}  // namespace system_diagnostic_graph

#endif  // CORE__GRAPH_HPP_
