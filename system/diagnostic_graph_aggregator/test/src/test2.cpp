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

#include "graph/error.hpp"
#include "graph/graph.hpp"
#include "utils.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diagnostic_graph.hpp>

#include <gtest/gtest.h>

using namespace diagnostic_graph_aggregator;  // NOLINT(build/namespaces)

using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using tier4_system_msgs::msg::DiagnosticGraph;

constexpr auto OK = DiagnosticStatus::OK;
constexpr auto WARN = DiagnosticStatus::WARN;
constexpr auto ERROR = DiagnosticStatus::ERROR;
constexpr auto STALE = DiagnosticStatus::STALE;

struct GraphTestParam
{
  std::string config;
  std::vector<uint8_t> inputs;
  uint8_t result;
};

class GraphTest : public testing::TestWithParam<GraphTestParam>
{
};

DiagnosticArray create_input(const std::vector<uint8_t> & levels)
{
  DiagnosticArray array;
  for (size_t i = 0; i < levels.size(); ++i) {
    DiagnosticStatus status;
    status.level = levels[i];
    status.name = "test: input-" + std::to_string(i);
    array.status.push_back(status);
  }
  return array;
};

uint8_t get_output(const Graph & graph, const rclcpp::Time & stamp)
{
  const auto struct_nodes = graph.create_struct(stamp).nodes;
  const auto status_nodes = graph.create_status(stamp).nodes;

  for (size_t i = 0; i < struct_nodes.size(); ++i) {
    if (struct_nodes[i].path == "output") {
      return status_nodes[i].level;
    }
  }
  throw std::runtime_error("output node is not found");
}

TEST_P(GraphTest, Aggregation)
{
  const auto stamp = rclcpp::Clock().now();
  const auto param = GetParam();
  Graph graph;
  graph.create(resource(param.config));

  const auto array = create_input(param.inputs);
  for (const auto & status : array.status) {
    graph.update(stamp, status);
  }

  const auto output = get_output(graph, stamp);
  EXPECT_EQ(output, param.result);
}

// clang-format off

INSTANTIATE_TEST_SUITE_P(And, GraphTest,
  testing::Values(
    GraphTestParam{"test2/and.yaml", {OK,    OK   }, OK   },
    GraphTestParam{"test2/and.yaml", {OK,    WARN }, WARN },
    GraphTestParam{"test2/and.yaml", {OK,    ERROR}, ERROR},
    GraphTestParam{"test2/and.yaml", {OK,    STALE}, ERROR},
    GraphTestParam{"test2/and.yaml", {WARN,  OK   }, WARN },
    GraphTestParam{"test2/and.yaml", {WARN,  WARN }, WARN },
    GraphTestParam{"test2/and.yaml", {WARN,  ERROR}, ERROR},
    GraphTestParam{"test2/and.yaml", {WARN,  STALE}, ERROR},
    GraphTestParam{"test2/and.yaml", {ERROR, OK   }, ERROR},
    GraphTestParam{"test2/and.yaml", {ERROR, WARN }, ERROR},
    GraphTestParam{"test2/and.yaml", {ERROR, ERROR}, ERROR},
    GraphTestParam{"test2/and.yaml", {ERROR, STALE}, ERROR},
    GraphTestParam{"test2/and.yaml", {STALE, OK   }, ERROR},
    GraphTestParam{"test2/and.yaml", {STALE, WARN }, ERROR},
    GraphTestParam{"test2/and.yaml", {STALE, ERROR}, ERROR},
    GraphTestParam{"test2/and.yaml", {STALE, STALE}, ERROR}
  )
);

INSTANTIATE_TEST_SUITE_P(Or, GraphTest,
  testing::Values(
    GraphTestParam{"test2/or.yaml", {OK,    OK   }, OK   },
    GraphTestParam{"test2/or.yaml", {OK,    WARN }, OK   },
    GraphTestParam{"test2/or.yaml", {OK,    ERROR}, OK   },
    GraphTestParam{"test2/or.yaml", {OK,    STALE}, OK   },
    GraphTestParam{"test2/or.yaml", {WARN,  OK   }, OK   },
    GraphTestParam{"test2/or.yaml", {WARN,  WARN }, WARN },
    GraphTestParam{"test2/or.yaml", {WARN,  ERROR}, WARN },
    GraphTestParam{"test2/or.yaml", {WARN,  STALE}, WARN },
    GraphTestParam{"test2/or.yaml", {ERROR, OK   }, OK   },
    GraphTestParam{"test2/or.yaml", {ERROR, WARN }, WARN },
    GraphTestParam{"test2/or.yaml", {ERROR, ERROR}, ERROR},
    GraphTestParam{"test2/or.yaml", {ERROR, STALE}, ERROR},
    GraphTestParam{"test2/or.yaml", {STALE, OK   }, OK   },
    GraphTestParam{"test2/or.yaml", {STALE, WARN }, WARN },
    GraphTestParam{"test2/or.yaml", {STALE, ERROR}, ERROR},
    GraphTestParam{"test2/or.yaml", {STALE, STALE}, ERROR}
  )
);

INSTANTIATE_TEST_SUITE_P(WarnToOk, GraphTest,
  testing::Values(
    GraphTestParam{"test2/warn-to-ok.yaml", {OK   }, OK   },
    GraphTestParam{"test2/warn-to-ok.yaml", {WARN }, OK},
    GraphTestParam{"test2/warn-to-ok.yaml", {ERROR}, ERROR},
    GraphTestParam{"test2/warn-to-ok.yaml", {STALE}, STALE}
  )
);

INSTANTIATE_TEST_SUITE_P(WarnToError, GraphTest,
  testing::Values(
    GraphTestParam{"test2/warn-to-error.yaml", {OK   }, OK   },
    GraphTestParam{"test2/warn-to-error.yaml", {WARN }, ERROR},
    GraphTestParam{"test2/warn-to-error.yaml", {ERROR}, ERROR},
    GraphTestParam{"test2/warn-to-error.yaml", {STALE}, STALE}
  )
);

// clang-format on
