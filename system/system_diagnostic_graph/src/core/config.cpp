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

#include "config.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// DEBUG
#include <iostream>

namespace system_diagnostic_graph
{

ErrorMarker::ErrorMarker(const std::string & file)
{
  file_ = file;
}

std::string ErrorMarker::str() const
{
  if (type_.empty()) {
    return file_;
  }

  std::string result = type_;
  for (const auto & index : indices_) {
    result += "-" + std::to_string(index);
  }
  return result + " in " + file_;
}

ErrorMarker ErrorMarker::type(const std::string & type) const
{
  ErrorMarker mark = *this;
  mark.type_ = type;
  return mark;
}

ErrorMarker ErrorMarker::index(size_t index) const
{
  ErrorMarker mark = *this;
  mark.indices_.push_back(index);
  return mark;
}

ConfigObject::ConfigObject(const ErrorMarker & mark, YAML::Node yaml, const std::string & type)
{
  if (!yaml.IsMap()) {
    throw create_error(mark, type + " is not a dict type");
  }
  for (const auto & kv : yaml) {
    dict_[kv.first.as<std::string>()] = kv.second;
  }
  mark_ = mark;
  type_ = type;
}

ErrorMarker ConfigObject::mark() const
{
  return mark_;
}

std::optional<YAML::Node> ConfigObject::take_yaml(const std::string & name)
{
  if (!dict_.count(name)) {
    return std::nullopt;
  }
  const auto yaml = dict_.at(name);
  dict_.erase(name);
  return yaml;
}

std::string ConfigObject::take_text(const std::string & name)
{
  if (!dict_.count(name)) {
    throw create_error(mark_, "object has no '" + name + "' field");
  }

  const auto yaml = dict_.at(name);
  dict_.erase(name);
  return yaml.as<std::string>();
}

std::string ConfigObject::take_text(const std::string & name, const std::string & fail)
{
  if (!dict_.count(name)) {
    return fail;
  }

  const auto yaml = dict_.at(name);
  dict_.erase(name);
  return yaml.as<std::string>();
}

std::vector<YAML::Node> ConfigObject::take_list(const std::string & name)
{
  if (!dict_.count(name)) {
    return std::vector<YAML::Node>();
  }

  const auto yaml = dict_.at(name);
  dict_.erase(name);

  if (!yaml.IsSequence()) {
    throw ConfigError("the '" + name + "' field is not a list type");
  }
  return std::vector<YAML::Node>(yaml.begin(), yaml.end());
}

bool ConfigFilter::check(const std::string & mode) const
{
  if (!excludes.empty() && excludes.count(mode) != 0) return false;
  if (!includes.empty() && includes.count(mode) == 0) return false;
  return true;
}

ConfigError create_error(const ErrorMarker & mark, const std::string & message)
{
  (void)mark;
  return ConfigError(message);
}

ConfigFilter parse_mode_filter(const ErrorMarker & mark, std::optional<YAML::Node> yaml)
{
  std::unordered_set<std::string> excludes;
  std::unordered_set<std::string> includes;
  if (yaml) {
    ConfigObject dict(mark, yaml.value(), "mode filter");

    for (const auto & mode : dict.take_list("except")) {
      excludes.emplace(mode.as<std::string>());
    }
    for (const auto & mode : dict.take_list("only")) {
      includes.emplace(mode.as<std::string>());
    }
  }
  return ConfigFilter{excludes, includes};
}

FileConfig parse_file_config(const ErrorMarker & mark, YAML::Node yaml)
{
  ConfigObject dict(mark, yaml, "file object");
  const auto relative_path = dict.take_text("path");
  const auto package_name = dict.take_text("package");
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  return FileConfig{mark, package_path + "/" + relative_path};
}

NodeConfig parse_node_config(const ErrorMarker & mark, YAML::Node yaml)
{
  ConfigObject dict(mark, yaml, "node object");
  const auto path = dict.take_text("path");
  const auto mode = parse_mode_filter(dict.mark(), dict.take_yaml("mode"));
  return NodeConfig{path, mode, dict};
}

ExprConfig parse_expr_config(const ErrorMarker & mark, YAML::Node yaml)
{
  ConfigObject dict(mark, yaml, "expr object");
  return parse_expr_config(dict);
}

ExprConfig parse_expr_config(ConfigObject & dict)
{
  const auto type = dict.take_text("type");
  const auto mode = parse_mode_filter(dict.mark(), dict.take_yaml("mode"));
  return ExprConfig{type, mode, dict};
}

void dump(const ConfigFile & config)
{
  std::cout << "=================================================================" << std::endl;
  std::cout << config.mark.str() << std::endl;
  for (const auto & file : config.files) {
    std::cout << " - f: " << file.path << " (" << file.mark.str() << ")" << std::endl;
  }
  for (const auto & unit : config.units) {
    std::cout << " - u: " << unit.path << " (" << unit.dict.mark().str() << ")" << std::endl;
  }
  for (const auto & diag : config.diags) {
    std::cout << " - d: " << diag.path << " (" << diag.dict.mark().str() << ")" << std::endl;
  }
}

template <class T, class F>
auto apply(const ErrorMarker & mark, F & func, const std::vector<YAML::Node> & list)
{
  std::vector<T> result;
  for (size_t i = 0; i < list.size(); ++i) {
    result.push_back(func(mark.index(i), list[i]));
  }
  return result;
}

ConfigFile load_config_file(const FileConfig & file)
{
  if (!std::filesystem::exists(file.path)) {
    throw create_error(file.mark, "config file '" + file.path + "' does not exist");
  }

  const auto yaml = YAML::LoadFile(file.path);
  const auto mark = ErrorMarker(file.path);
  auto dict = ConfigObject(mark, yaml, "config file");

  std::vector<YAML::Node> units;
  std::vector<YAML::Node> diags;
  for (const auto & node : dict.take_list("nodes")) {
    const auto type = node["type"].as<std::string>();
    if (type == "diag") {
      diags.push_back(node);
    } else {
      units.push_back(node);
    }
  }

  ConfigFile config(mark);
  config.files = apply<FileConfig>(mark.type("file"), parse_file_config, dict.take_list("files"));
  config.units = apply<NodeConfig>(mark.type("unit"), parse_node_config, units);
  config.diags = apply<NodeConfig>(mark.type("diag"), parse_node_config, diags);
  return config;
}

ConfigFile load_config_root(const std::string & path)
{
  const auto mark = ErrorMarker("root file");
  std::vector<ConfigFile> configs;
  configs.push_back(load_config_file(FileConfig{mark, path}));

  // Use an index because updating the vector invalidates the iterator.
  for (size_t i = 0; i < configs.size(); ++i) {
    for (const auto & file : configs[i].files) {
      configs.push_back(load_config_file(file));
    }
  }

  ConfigFile result(mark);
  for (const auto & config : configs) {
    result.files.insert(result.files.end(), config.files.begin(), config.files.end());
    result.units.insert(result.units.end(), config.units.begin(), config.units.end());
    result.diags.insert(result.diags.end(), config.diags.begin(), config.diags.end());
  }
  return result;
}

}  // namespace system_diagnostic_graph
