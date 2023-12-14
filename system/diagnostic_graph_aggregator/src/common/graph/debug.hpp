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

#ifndef COMMON__GRAPH__DEBUG_HPP_
#define COMMON__GRAPH__DEBUG_HPP_

#include <array>
#include <string>

namespace diagnostic_graph_aggregator
{

constexpr size_t diag_debug_size = 4;
using DiagDebugData = std::array<std::string, diag_debug_size>;

}  // namespace diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__DEBUG_HPP_
