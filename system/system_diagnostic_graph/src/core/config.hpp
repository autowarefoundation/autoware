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

#ifndef CORE__CONFIG_HPP_
#define CORE__CONFIG_HPP_

#include <yaml-cpp/yaml.h>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace system_diagnostic_graph
{

struct ConfigError : public std::runtime_error
{
  using runtime_error::runtime_error;
};

class ErrorMarker
{
public:
  explicit ErrorMarker(const std::string & file = "");
  std::string str() const;
  ErrorMarker type(const std::string & type) const;
  ErrorMarker index(size_t index) const;

private:
  std::string file_;
  std::string type_;
  std::vector<size_t> indices_;
};

class ConfigObject
{
public:
  ConfigObject(const ErrorMarker & mark, YAML::Node yaml, const std::string & type);
  ErrorMarker mark() const;
  std::optional<YAML::Node> take_yaml(const std::string & name);
  std::string take_text(const std::string & name);
  std::string take_text(const std::string & name, const std::string & fail);
  std::vector<YAML::Node> take_list(const std::string & name);

private:
  ErrorMarker mark_;
  std::string type_;
  std::unordered_map<std::string, YAML::Node> dict_;
};

struct ConfigFilter
{
  bool check(const std::string & mode) const;
  std::unordered_set<std::string> excludes;
  std::unordered_set<std::string> includes;
};

struct ExprConfig
{
  std::string type;
  ConfigFilter mode;
  ConfigObject dict;
};

struct NodeConfig
{
  std::string path;
  ConfigFilter mode;
  ConfigObject dict;
};

struct FileConfig
{
  ErrorMarker mark;
  std::string path;
};

struct ConfigFile
{
  explicit ConfigFile(const ErrorMarker & mark) : mark(mark) {}
  ErrorMarker mark;
  std::vector<FileConfig> files;
  std::vector<NodeConfig> units;
  std::vector<NodeConfig> diags;
};

using ConfigDict = std::unordered_map<std::string, YAML::Node>;

ConfigError create_error(const ErrorMarker & mark, const std::string & message);
ConfigFile load_config_root(const std::string & path);

ExprConfig parse_expr_config(const ErrorMarker & mark, YAML::Node yaml);
ExprConfig parse_expr_config(ConfigObject & dict);

}  // namespace system_diagnostic_graph

#endif  // CORE__CONFIG_HPP_
