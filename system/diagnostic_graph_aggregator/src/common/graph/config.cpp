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

#include "names.hpp"
#include "types.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <deque>
#include <filesystem>
#include <queue>
#include <regex>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace diagnostic_graph_aggregator
{

std::string resolve_substitution(const std::string & substitution, const TreeData & data)
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
    return std::filesystem::path(data.path().file()).parent_path();
  }
  throw UnknownSubstitution(data.path(), substitution);
}

std::string resolve_file_path(const std::string & path, const TreeData & data)
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

FileLoader::FileLoader(const PathConfig * path)
{
  if (!std::filesystem::exists(path->resolved)) {
    throw FileNotFound(path->data.path(), path->resolved);
  }

  TreeData tree = TreeData::Load(path->resolved);
  const auto files = tree.optional("files").children("files");
  const auto edits = tree.optional("edits").children("edits");
  const auto units = tree.optional("units").children("units");
  for (const auto & data : files) create_path_config(data);
  for (const auto & data : edits) create_edit_config(data);
  for (const auto & data : units) create_unit_config(data);
}

PathConfig * FileLoader::create_path_config(const TreeData & data)
{
  const auto path = paths_.emplace_back(std::make_unique<PathConfig>(data)).get();
  path->original = path->data.required("path").text();
  path->resolved = resolve_file_path(path->original, data);
  return path;
}

EditConfig * FileLoader::create_edit_config(const TreeData & data)
{
  const auto edit = edits_.emplace_back(std::make_unique<EditConfig>(data)).get();
  edit->type = edit->data.required("type").text();
  return edit;
}

UnitConfig * FileLoader::create_unit_config(const TreeData & data)
{
  const auto unit = units_.emplace_back(std::make_unique<UnitConfig>(data)).get();
  unit->type = unit->data.required("type").text();
  unit->path = unit->data.optional("path").text();

  const auto item = unit->data.optional("item").child("c");
  if (item.is_valid()) {
    unit->item = create_link_config(item, unit);
  }
  const auto list = unit->data.optional("list").children();
  for (const auto & tree_data : list) {
    unit->list.push_back(create_link_config(tree_data, unit));
  }
  return unit;
}

LinkConfig * FileLoader::create_link_config(const TreeData & data, UnitConfig * unit)
{
  const auto link = links_.emplace_back(std::make_unique<LinkConfig>()).get();
  link->parent = unit;
  link->child = create_unit_config(data);
  return link;
}

void FileLoader::release(FileConfig & config)
{
  for (auto & path : paths_) config.paths.push_back(std::move(path));
  for (auto & edit : edits_) config.edits.push_back(std::move(edit));
  for (auto & unit : units_) config.units.push_back(std::move(unit));
  for (auto & link : links_) config.links.push_back(std::move(link));
}

TreeLoader TreeLoader::Load(const std::string & path)
{
  PathConfig root(TreeData::None());
  root.original = path;
  root.resolved = resolve_file_path(path, root.data);
  return TreeLoader(&root);
}

TreeLoader::TreeLoader(const PathConfig * root)
{
  std::queue<const PathConfig *> paths;
  paths.push(root);

  // TODO(Takagi, Isamu): check include loop.
  while (!paths.empty()) {
    files_.emplace_back(paths.front());
    paths.pop();
    for (const auto & path : files_.back().paths()) {
      paths.push(path.get());
    }
  }
}

auto create_path_mapping(const std::vector<std::unique_ptr<UnitConfig>> & units)
{
  std::unordered_map<std::string, UnitConfig *> path_to_unit;
  for (const auto & unit : units) {
    if (unit->path.empty()) {
      continue;
    }
    if (path_to_unit.count(unit->path)) {
      throw PathConflict(unit->path);
    }
    path_to_unit[unit->path] = unit.get();
  }
  return path_to_unit;
}

void apply_links(FileConfig & config)
{
  // Separate units into link types and others.
  std::vector<std::unique_ptr<UnitConfig>> link_units;
  std::vector<std::unique_ptr<UnitConfig>> node_units;
  for (auto & unit : config.units) {
    if (unit->type == unit_name::link) {
      link_units.push_back(std::move(unit));
    } else {
      node_units.push_back(std::move(unit));
    }
  }

  // Create a mapping from path to unit.
  const auto path_to_unit = create_path_mapping(node_units);

  // Create a mapping from unit to unit.
  std::unordered_map<UnitConfig *, UnitConfig *> unit_to_unit;
  for (const auto & unit : link_units) {
    const auto path = unit->data.required("link").text();
    if (path_to_unit.count(path) == 0) {
      throw PathNotFound(unit->data.path(), path);
    }
    unit_to_unit[unit.get()] = path_to_unit.at(path);
  }

  // Update links.
  for (const auto & link : config.links) {
    if (unit_to_unit.count(link->child) != 0) {
      link->child = unit_to_unit.at(link->child);
    }
  }

  // Remove link type units from the graph.
  config.units = std::move(node_units);
}

void apply_edits(FileConfig & config)
{
  // Create a mapping from path to unit.
  const auto path_to_unit = create_path_mapping(config.units);

  // List units to remove and links from/to them.
  std::unordered_set<UnitConfig *> remove_units;
  std::unordered_set<LinkConfig *> remove_links;
  for (const auto & edit : config.edits) {
    if (edit->type == edit_name::remove) {
      const auto path = edit->data.required("path").text();
      if (path_to_unit.count(path) == 0) {
        throw PathNotFound(edit->data.path(), path);
      }
      remove_units.insert(path_to_unit.at(path));
    }
  }
  for (const auto & link : config.links) {
    if (remove_units.count(link->parent) || remove_units.count(link->child)) {
      remove_links.insert(link.get());
    }
  }

  // Filter references to the removed links.
  for (const auto & unit : config.units) {
    if (remove_links.count(unit->item) != 0) {
      unit->item = nullptr;
    }
    std::vector<LinkConfig *> filtered_list;
    for (const auto & link : unit->list) {
      if (remove_links.count(link) == 0) {
        filtered_list.push_back(link);
      }
    }
    unit->list = filtered_list;
  }

  // Remove units and links.
  std::vector<std::unique_ptr<UnitConfig>> filtered_units;
  std::vector<std::unique_ptr<LinkConfig>> filtered_links;
  for (auto & unit : config.units) {
    if (remove_units.count(unit.get()) == 0) {
      filtered_units.push_back(std::move(unit));
    }
  }
  for (auto & link : config.links) {
    if (remove_links.count(link.get()) == 0) {
      filtered_links.push_back(std::move(link));
    }
  }
  config.units = std::move(filtered_units);
  config.links = std::move(filtered_links);
}

void topological_sort(const FileConfig & config)
{
  std::unordered_map<UnitConfig *, int> degrees;
  std::deque<UnitConfig *> units;
  std::deque<UnitConfig *> result;
  std::deque<UnitConfig *> buffer;

  // Create a list of raw pointer units.
  for (const auto & unit : config.units) units.push_back(unit.get());

  // Count degrees of each unit.
  for (const auto * const unit : units) {
    if (const auto * const link = unit->item) ++degrees[link->child];
    for (const auto * const link : unit->list) ++degrees[link->child];
  }

  // Find initial units that are zero degrees.
  for (const auto & unit : units) {
    if (degrees[unit] == 0) buffer.push_back(unit);
  }

  // Sort by topological order.
  while (!buffer.empty()) {
    const auto unit = buffer.front();
    buffer.pop_front();
    if (const auto & link = unit->item) {
      if (--degrees[link->child] == 0) {
        buffer.push_back(link->child);
      }
    }
    for (const auto & link : unit->list) {
      if (--degrees[link->child] == 0) {
        buffer.push_back(link->child);
      }
    }
    result.push_back(unit);
  }

  // Detect circulation because the result does not include the units on the loop.
  if (result.size() != units.size()) {
    throw GraphStructure("detect graph circulation");
  }
}

FileConfig TreeLoader::construct()
{
  FileConfig config;
  for (auto & file : files_) file.release(config);
  apply_links(config);
  apply_edits(config);
  topological_sort(config);  // Check graph structure.
  return config;
}

}  // namespace diagnostic_graph_aggregator
