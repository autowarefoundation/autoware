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

#include "graph/graph.hpp"
#include "graph/types.hpp"
#include "graph/units.hpp"

#include <iostream>
#include <string>
#include <unordered_map>

namespace diagnostic_graph_aggregator
{

struct GraphNode
{
  using UniquePtr = std::unique_ptr<GraphNode>;
  std::string type;
  std::string path;
  std::vector<GraphNode *> children;
  std::vector<GraphNode *> parents;
};

struct GraphRoot
{
  std::vector<GraphNode::UniquePtr> owner;
  std::vector<GraphNode *> nodes;
};

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

void dump_plantuml_path(const std::string & path)
{
  const auto graph = load_graph_nodes(path);
  const auto color = "#FFFFFF";

  for (const auto & node : graph.nodes) {
    std::cout << "card " << node << " " << color << " [" << std::endl;
    std::cout << node->path << std::endl;
    std::cout << "]" << std::endl;
  }

  for (const auto & node : graph.nodes) {
    for (const auto & child : node->children) {
      std::cout << node << " --> " << child << std::endl;
    }
  }
}

void dump_tree_node(const GraphNode * node, const std::string & indent = "", bool root = true)
{
  const auto path = node->path.empty() ? "" : node->path + " ";
  const auto type = "(" + node->type + ")";
  std::cout << indent << "- " << path << type << std::endl;

  if (root || node->parents.size() == 1) {
    for (const auto child : node->children) {
      dump_tree_node(child, indent + "    ", false);
    }
  }
}

void dump_tree_path(const std::string & path)
{
  const auto graph = load_graph_nodes(path);

  std::cout << "===== root nodes =================================" << std::endl;
  for (const auto & node : graph.nodes) {
    if (node->parents.size() == 0 && node->children.size() != 0) {
      dump_tree_node(node);
    }
  }
  std::cout << "===== intermediate nodes =========================" << std::endl;
  for (const auto & node : graph.nodes) {
    if (node->parents.size() >= 2) {
      dump_tree_node(node);
    }
  }

  std::cout << "===== isolated nodes =============================" << std::endl;
  for (const auto & node : graph.nodes) {
    if (node->parents.size() == 0 && node->children.size() == 0) {
      dump_tree_node(node);
    }
  }
}

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: " << argv[0] << " <path>" << std::endl;
    return 1;
  }
  diagnostic_graph_aggregator::dump_tree_path(argv[1]);
}
