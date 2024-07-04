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

#include "data.hpp"

namespace diagnostic_graph_aggregator
{

TreeData TreeData::Load(const std::string & path)
{
  return TreeData(YAML::LoadFile(path), TreePath(path));
}

TreeData TreeData::None()
{
  return TreeData(YAML::Node(), TreePath(""));
}

TreeData::TreeData(const YAML::Node & yaml, const TreePath & path) : path_(path)
{
  yaml_ = yaml;
}

TreeData::Item TreeData::required(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  const auto tree_path = path_.field(name);
  if (!yaml_[name]) {
    throw FieldNotFound(tree_path);
  }
  const auto data = yaml_[name];
  yaml_.remove(name);
  return TreeData(data, tree_path);
}

TreeData::Item TreeData::optional(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  const auto tree_path = path_.field(name);
  if (!yaml_[name]) {
    return TreeData(YAML::Node(YAML::NodeType::Undefined), tree_path);
  }
  const auto data = yaml_[name];
  yaml_.remove(name);
  return TreeData(data, tree_path);
}

bool TreeData::is_valid() const
{
  return yaml_.Type() != YAML::NodeType::Undefined;
}

TreeData::Item TreeData::child(const std::string & path)
{
  return TreeData(yaml_, path_.child(path));
}

TreeData::List TreeData::children(const std::string & path)
{
  if (yaml_.Type() == YAML::NodeType::Undefined) {
    return {};
  }
  if (yaml_.Type() != YAML::NodeType::Sequence) {
    throw InvalidType(path_);
  }

  TreeData::List result;
  for (size_t i = 0; i < yaml_.size(); ++i) {
    result.emplace_back(yaml_[i], path_.child(path, i));
  }
  return result;
}

std::string TreeData::text(const std::string & fail)
{
  // TODO(Takagi, Isamu): check conversion failure
  return yaml_.as<std::string>(fail);
}

double TreeData::real(double fail)
{
  // TODO(Takagi, Isamu): check conversion failure
  return yaml_.as<double>(fail);
}

}  // namespace diagnostic_graph_aggregator
