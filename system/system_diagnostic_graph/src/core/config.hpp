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
#include <stdexcept>
#include <string>
#include <vector>

namespace system_diagnostic_graph
{

struct ConfigError : public std::runtime_error
{
  using runtime_error::runtime_error;
};

struct NodeConfig_
{
  std::string path;
  std::string name;
  YAML::Node yaml;
};

struct FileConfig_
{
  std::string path;
  std::vector<std::shared_ptr<FileConfig_>> files;
  std::vector<std::shared_ptr<NodeConfig_>> nodes;
};

template <class T>
T take(YAML::Node yaml, const std::string & field)
{
  const auto result = yaml[field].as<T>();
  yaml.remove(field);
  return result;
}

using NodeConfig = std::shared_ptr<NodeConfig_>;
using FileConfig = std::shared_ptr<FileConfig_>;
ConfigError create_error(const FileConfig & config, const std::string & message);
ConfigError create_error(const NodeConfig & config, const std::string & message);
std::vector<NodeConfig> load_config_file(const std::string & path);

NodeConfig parse_config_node(YAML::Node yaml, const FileConfig & scope);
FileConfig parse_config_path(YAML::Node yaml, const FileConfig & scope);
FileConfig parse_config_path(const std::string & path, const FileConfig & scope);
FileConfig parse_config_file(const std::string & path);

}  // namespace system_diagnostic_graph

#endif  // CORE__CONFIG_HPP_
