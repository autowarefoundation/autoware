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

#ifndef COMMON__GRAPH__NAMES_HPP_
#define COMMON__GRAPH__NAMES_HPP_

namespace diagnostic_graph_aggregator::unit_name
{

constexpr char const * link = "link";
constexpr char const * diag = "diag";
constexpr char const * min = "or";
constexpr char const * max = "and";
constexpr char const * short_circuit_max = "short-circuit-and";
constexpr char const * warn_to_ok = "warn-to-ok";
constexpr char const * warn_to_error = "warn-to-error";
constexpr char const * ok = "ok";
constexpr char const * warn = "warn";
constexpr char const * error = "error";
constexpr char const * stale = "stale";

}  // namespace diagnostic_graph_aggregator::unit_name

namespace diagnostic_graph_aggregator::edit_name
{

constexpr char const * remove = "remove";

}  // namespace diagnostic_graph_aggregator::edit_name

#endif  // COMMON__GRAPH__NAMES_HPP_
