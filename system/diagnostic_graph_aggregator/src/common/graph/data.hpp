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

#ifndef COMMON__GRAPH__DATA_HPP_
#define COMMON__GRAPH__DATA_HPP_

#include "error.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <vector>

namespace diagnostic_graph_aggregator
{

class TreeData
{
public:
  using Item = TreeData;
  using List = std::vector<TreeData::Item>;

  static TreeData Load(const std::string & path);
  static TreeData None();
  TreeData(const YAML::Node & yaml, const TreePath & path);

  const auto & path() const { return path_; }
  Item required(const std::string & name);
  Item optional(const std::string & name);
  bool is_valid() const;

  Item child(const std::string & path);
  List children(const std::string & path = "");
  std::string text(const std::string & fail = "");
  double real(double fail);

private:
  YAML::Node yaml_;
  TreePath path_;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__DATA_HPP_
