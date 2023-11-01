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

#include "core/error.hpp"
#include "core/graph.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>

using namespace system_diagnostic_graph;  // NOLINT(build/namespaces)

std::filesystem::path resource(const std::string & path)
{
  return std::filesystem::path(TEST_RESOURCE_PATH) / path;
}

TEST(ConfigFile, RootNotFound)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("fake-file-name.yaml")), FileNotFound);
}

TEST(ConfigFile, FileNotFound)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("file-not-found.yaml")), FileNotFound);
}

TEST(ConfigFile, UnknownSubstitution)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("unknown-substitution.yaml")), UnknownType);
}

TEST(ConfigFile, UnknownNodeType)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("unknown-node-type.yaml")), UnknownType);
}

TEST(ConfigFile, InvalidDictType)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("invalid-dict-type.yaml")), InvalidType);
}

TEST(ConfigFile, InvalidListType)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("invalid-list-type.yaml")), InvalidType);
}

TEST(ConfigFile, FieldNotFound)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("field-not-found.yaml")), FieldNotFound);
}

TEST(ConfigFile, PathConflict)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("path-conflict.yaml")), PathConflict);
}

TEST(ConfigFile, PathNotFound)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("path-not-found.yaml")), PathNotFound);
}

TEST(ConfigFile, GraphCirculation)
{
  Graph graph;
  EXPECT_THROW(graph.init(resource("graph-circulation.yaml")), GraphStructure);
}
