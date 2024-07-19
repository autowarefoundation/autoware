// Copyright 2023 TIER IV, Inc.
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

#include "autoware/tracking_object_merger/association/solver/mu_ssp.hpp"

#include <mussp/mussp.h>

#include <array>
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::tracking_object_merger::gnn_solver
{
void MuSSP::maximizeLinearAssignment(
  const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
  std::unordered_map<int, int> * reverse_assignment)
{
  // Terminate if the graph is empty
  if (cost.size() == 0 || cost.at(0).size() == 0) {
    return;
  }

  // Solve DA by muSSP
  solve_muSSP(cost, direct_assignment, reverse_assignment);
}
}  // namespace autoware::tracking_object_merger::gnn_solver
