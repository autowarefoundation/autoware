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

#include "error.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// DEBUG
#include <iostream>

namespace diagnostic_graph_aggregator
{

template <class T>
void extend(std::vector<T> & u, const std::vector<T> & v)
{
  u.insert(u.end(), v.begin(), v.end());
}

template <class T>
auto enumerate(const std::vector<T> & v)
{
  std::vector<std::pair<size_t, T>> result;
  for (size_t i = 0; i < v.size(); ++i) {
    result.push_back(std::make_pair(i, v[i]));
  }
  return result;
}

ConfigData::ConfigData(const std::string & path)
{
  file = path;
}

ConfigData ConfigData::load(YAML::Node yaml)
{
  if (!yaml.IsMap()) {
    throw error<InvalidType>("object is not a dict type", *this);
  }
  for (const auto & kv : yaml) {
    object[kv.first.as<std::string>()] = kv.second;
  }
  return *this;
}

ConfigData ConfigData::type(const std::string & name) const
{
  ConfigData data(file);
  data.mark = mark.empty() ? name : mark + "-" + name;
  return data;
}

ConfigData ConfigData::node(const size_t index) const
{
  return type(std::to_string(index));
}

std::optional<YAML::Node> ConfigData::take_yaml(const std::string & name)
{
  if (!object.count(name)) {
    return std::nullopt;
  }

  const auto yaml = object.at(name);
  object.erase(name);
  return yaml;
}

std::string ConfigData::take_text(const std::string & name)
{
  if (!object.count(name)) {
    throw error<FieldNotFound>("required field is not found", name, *this);
  }

  const auto yaml = object.at(name);
  object.erase(name);
  return yaml.as<std::string>();
}

std::string ConfigData::take_text(const std::string & name, const std::string & fail)
{
  if (!object.count(name)) {
    return fail;
  }

  const auto yaml = object.at(name);
  object.erase(name);
  return yaml.as<std::string>();
}

std::vector<YAML::Node> ConfigData::take_list(const std::string & name)
{
  if (!object.count(name)) {
    return std::vector<YAML::Node>();
  }

  const auto yaml = object.at(name);
  object.erase(name);

  if (!yaml.IsSequence()) {
    throw error<InvalidType>("field is not a list type", name, *this);
  }
  return std::vector<YAML::Node>(yaml.begin(), yaml.end());
}

void resolve_link_nodes(RootConfig & root)
{
  std::unordered_map<std::string, UnitConfig::SharedPtr> paths;
  for (const auto & node : root.nodes) {
    if (node->path.empty()) {
      continue;
    }
    if (paths.count(node->path)) {
      throw error<PathConflict>("object path is not unique", node->path);
    }
    paths[node->path] = node;
  }

  std::vector<UnitConfig::SharedPtr> nodes;
  std::vector<UnitConfig::SharedPtr> links;
  for (const auto & node : root.nodes) {
    if (node->type == "link") {
      links.push_back(node);
    } else {
      nodes.push_back(node);
    }
  }

  std::unordered_map<UnitConfig::SharedPtr, UnitConfig::SharedPtr> targets;
  for (const auto & node : nodes) {
    targets[node] = node;
  }
  for (const auto & node : links) {
    const auto path = node->data.take_text("link");
    if (!paths.count(path)) {
      throw error<PathNotFound>("link path is not found", path, node->data);
    }
    const auto link = paths.at(path);
    if (link->type == "link") {
      throw error<GraphStructure>("link target is link type", path, node->data);
    }
    targets[node] = link;
  }
  for (const auto & node : nodes) {
    for (auto & child : node->children) {
      child = targets.at(child);
    }
  }
  root.nodes = nodes;
}

void resolve_remove_edits(RootConfig & root)
{
  std::unordered_map<std::string, UnitConfig::SharedPtr> paths;
  for (const auto & node : root.nodes) {
    paths[node->path] = node;
  }

  std::unordered_set<UnitConfig::SharedPtr> removes;
  for (const auto & edit : root.edits) {
    if (edit->type == "remove") {
      if (!paths.count(edit->path)) {
        throw error<PathNotFound>("remove path is not found", edit->path, edit->data);
      }
      removes.insert(paths.at(edit->path));
    }
  }

  const auto filter = [removes](const std::vector<UnitConfig::SharedPtr> & nodes) {
    std::vector<UnitConfig::SharedPtr> result;
    for (const auto & node : nodes) {
      if (!removes.count(node)) {
        result.push_back(node);
      }
    }
    return result;
  };
  for (const auto & node : root.nodes) {
    node->children = filter(node->children);
  }
  root.nodes = filter(root.nodes);
}

std::string complement_node_type(ConfigData & data)
{
  if (data.object.count("diag")) {
    return "diag";
  }
  return data.take_text("type");
}

std::string resolve_substitution(const std::string & substitution, const ConfigData & data)
{
  std::stringstream ss(substitution);
  std::string word;
  std::vector<std::string> words;
  while (getline(ss, word, ' ')) {
    words.push_back(word);
  }

  if (words.size() == 2 && words[0] == "find-pkg-share") {
    return ament_index_cpp::get_package_share_directory(words[1]);
  }
  if (words.size() == 1 && words[0] == "dirname") {
    return std::filesystem::path(data.file).parent_path();
  }
  throw error<UnknownType>("unknown substitution", substitution, data);
}

std::string resolve_file_path(const std::string & path, const ConfigData & data)
{
  static const std::regex pattern(R"(\$\(([^()]*)\))");
  std::smatch m;
  std::string result = path;
  while (std::regex_search(result, m, pattern)) {
    const std::string prefix = m.prefix();
    const std::string suffix = m.suffix();
    result = prefix + resolve_substitution(m.str(1), data) + suffix;
  }
  return result;
}

PathConfig::SharedPtr parse_path_config(const ConfigData & data)
{
  const auto path = std::make_shared<PathConfig>(data);
  path->original = path->data.take_text("path");
  path->resolved = resolve_file_path(path->original, path->data);
  return path;
}

UnitConfig::SharedPtr parse_node_config(const ConfigData & data)
{
  const auto node = std::make_shared<UnitConfig>(data);
  node->path = node->data.take_text("path", "");
  node->type = node->data.take_text("type", "");

  if (node->type.empty()) {
    node->type = complement_node_type(node->data);
  }

  for (const auto & [index, yaml] : enumerate(node->data.take_list("list"))) {
    const auto child = data.node(index).load(yaml);
    node->children.push_back(parse_node_config(child));
  }
  return node;
}

EditConfig::SharedPtr parse_edit_config(const ConfigData & data)
{
  const auto edit = std::make_shared<EditConfig>(data);
  edit->path = edit->data.take_text("path", "");
  edit->type = edit->data.take_text("type", "");
  return edit;
}

FileConfig::SharedPtr parse_file_config(const ConfigData & data)
{
  const auto file = std::make_shared<FileConfig>(data);
  const auto path_data = data.type("file");
  const auto node_data = data.type("node");
  const auto edit_data = data.type("edit");
  const auto paths = file->data.take_list("files");
  const auto nodes = file->data.take_list("nodes");
  const auto edits = file->data.take_list("edits");

  for (const auto & [index, yaml] : enumerate(paths)) {
    const auto path = path_data.node(index).load(yaml);
    file->paths.push_back(parse_path_config(path));
  }
  for (const auto & [index, yaml] : enumerate(nodes)) {
    const auto node = node_data.node(index).load(yaml);
    file->nodes.push_back(parse_node_config(node));
  }
  for (const auto & [index, yaml] : enumerate(edits)) {
    const auto edit = edit_data.node(index).load(yaml);
    file->edits.push_back(parse_edit_config(edit));
  }
  return file;
}

FileConfig::SharedPtr load_file_config(PathConfig & config)
{
  const auto path = std::filesystem::path(config.resolved);
  if (!std::filesystem::exists(path)) {
    throw error<FileNotFound>("file is not found", path, config.data);
  }
  const auto file = ConfigData(path).load(YAML::LoadFile(path));
  return parse_file_config(file);
}

RootConfig load_root_config(const PathConfig::SharedPtr root)
{
  std::vector<PathConfig::SharedPtr> paths;
  paths.push_back(root);

  std::vector<FileConfig::SharedPtr> files;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto path = paths[i];
    const auto file = load_file_config(*path);
    files.push_back(file);
    extend(paths, file->paths);
  }

  std::vector<UnitConfig::SharedPtr> nodes;
  std::vector<EditConfig::SharedPtr> edits;
  for (const auto & file : files) {
    extend(nodes, file->nodes);
    extend(edits, file->edits);
  }
  for (size_t i = 0; i < nodes.size(); ++i) {
    const auto node = nodes[i];
    extend(nodes, node->children);
  }

  RootConfig config;
  config.files = files;
  config.nodes = nodes;
  config.edits = edits;
  resolve_link_nodes(config);
  resolve_remove_edits(config);
  return config;
}

RootConfig load_root_config(const std::string & path)
{
  const auto root = std::make_shared<PathConfig>(ConfigData("root-file"));
  root->original = path;
  root->resolved = path;
  return load_root_config(root);
}

}  // namespace diagnostic_graph_aggregator
