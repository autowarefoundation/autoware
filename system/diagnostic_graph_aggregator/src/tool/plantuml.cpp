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

#include "utils/loader.hpp"

#include <iostream>

namespace diagnostic_graph_aggregator
{

void dump_root(const std::string & path)
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

}  // namespace diagnostic_graph_aggregator

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: plantuml <path>" << std::endl;
    return 1;
  }
  diagnostic_graph_aggregator::dump_root(argv[1]);
}
