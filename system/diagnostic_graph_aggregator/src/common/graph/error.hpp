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

#ifndef COMMON__GRAPH__ERROR_HPP_
#define COMMON__GRAPH__ERROR_HPP_

#include <stdexcept>
#include <string>

namespace diagnostic_graph_aggregator
{

struct TreePath
{
  explicit TreePath(const std::string & file);
  TreePath field(const std::string & name);
  TreePath child(const std::string & path);
  TreePath child(const std::string & path, const size_t index);
  TreePath child(const size_t index);
  std::string text() const;
  std::string file() const { return file_; }
  std::string node() const { return node_; }
  std::string name() const { return name_; }

private:
  std::string file_;
  std::string node_;
  std::string name_;
};

struct Exception : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct FileNotFound : public Exception
{
  explicit FileNotFound(const TreePath & path, const std::string & file)
  : Exception(format(path, file))
  {
  }
  static std::string format(const TreePath & path, const std::string & file)
  {
    return "file is not found: " + file + path.text();
  }
};

struct FieldNotFound : public Exception
{
  explicit FieldNotFound(const TreePath & path) : Exception(format(path)) {}
  static std::string format(const TreePath & path)
  {
    return "required field is not found: " + path.name() + path.text();
  }
};

struct InvalidType : public Exception
{
  explicit InvalidType(const TreePath & path) : Exception(format(path)) {}
  static std::string format(const TreePath & path)
  {
    return "field is not a list type: " + path.name() + path.text();
  }
};

struct ModeNotFound : public Exception
{
  explicit ModeNotFound(const std::string & path) : Exception(format(path)) {}
  static std::string format(const std::string & path) { return "mode path is not found: " + path; }
};

struct PathConflict : public Exception
{
  explicit PathConflict(const std::string & path) : Exception(format(path)) {}
  static std::string format(const std::string & path) { return "node path is not unique: " + path; }
};

struct PathNotFound : public Exception
{
  explicit PathNotFound(const TreePath & path, const std::string & link)
  : Exception(format(path, link))
  {
  }
  static std::string format(const TreePath & path, const std::string & link)
  {
    return "path is not found: " + link + path.text();
  }
};

struct UnknownSubstitution : public Exception
{
  explicit UnknownSubstitution(const TreePath & path, const std::string & substitution)
  : Exception(format(path, substitution))
  {
  }
  static std::string format(const TreePath & path, const std::string & substitution)
  {
    return "unknown substitution: " + substitution + path.text();
  }
};

struct UnknownUnitType : public Exception
{
  explicit UnknownUnitType(const TreePath & path, const std::string & type)
  : Exception(format(path, type))
  {
  }
  static std::string format(const TreePath & path, const std::string & type)
  {
    return "unknown unit type: " + type + path.text();
  }
};

struct GraphStructure : public Exception
{
  using Exception::Exception;
};

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__ERROR_HPP_
