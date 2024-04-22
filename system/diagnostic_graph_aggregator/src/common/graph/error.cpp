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

#include "error.hpp"

namespace diagnostic_graph_aggregator
{

TreePath::TreePath(const std::string & file)
{
  file_ = file;
}

TreePath TreePath::field(const std::string & name)
{
  TreePath result(file_);
  result.node_ = node_;
  result.name_ = name;
  return result;
}

TreePath TreePath::child(const std::string & path)
{
  const auto sep = node_.empty() ? "" : "-";
  TreePath result(file_);
  result.node_ = node_ + sep + path;
  return result;
}

TreePath TreePath::child(const std::string & path, const size_t index)
{
  const auto sep = path.empty() ? "" : "-";
  return child(path + sep + std::to_string(index));
}

TreePath TreePath::child(const size_t index)
{
  return child(std::to_string(index));
}

std::string TreePath::text() const
{
  const auto sep = node_.empty() ? "" : ":";
  return " (" + file_ + sep + node_ + ")";
}

}  // namespace diagnostic_graph_aggregator
