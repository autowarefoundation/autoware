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

#include "loader.hpp"

#include <unordered_map>
#include <utility>

namespace diagnostic_graph_aggregator
{

GraphRoot load_graph_nodes(const std::string & path)
{
  GraphRoot result;
  {
    std::unordered_map<BaseUnit *, GraphNode::UniquePtr> mapping;
    Graph graph;
    graph.init(path);

    for (const auto & node : graph.nodes()) {
      auto data = std::make_unique<GraphNode>();
      data->path = node->path();
      data->type = node->type();
      mapping[node] = std::move(data);
    }

    for (const auto & [node, data] : mapping) {
      for (const auto & link : node->children()) {
        const auto parent = data.get();
        const auto child = mapping.at(link).get();
        child->parents.push_back(parent);
        parent->children.push_back(child);
      }
    }

    for (auto & [node, data] : mapping) {
      result.owner.push_back(std::move(data));
    }
    for (const auto & node : result.owner) {
      result.nodes.push_back(node.get());
    }
  }
  return result;
}

}  // namespace diagnostic_graph_aggregator
