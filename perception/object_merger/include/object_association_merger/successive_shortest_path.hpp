// Copyright 2020 Tier IV, Inc.
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

#ifndef OBJECT_ASSOCIATION_MERGER__SUCCESSIVE_SHORTEST_PATH_HPP_
#define OBJECT_ASSOCIATION_MERGER__SUCCESSIVE_SHORTEST_PATH_HPP_

#include <unordered_map>
#include <vector>

namespace assignment_problem
{
// See IMPORTANT NOTE at the top of the file.
void MaximizeLinearAssignment(
  const std::vector<std::vector<double>> & cost, std::unordered_map<int, int> * direct_assignment,
  std::unordered_map<int, int> * reverse_assignment);
}  // namespace assignment_problem

#endif  // OBJECT_ASSOCIATION_MERGER__SUCCESSIVE_SHORTEST_PATH_HPP_
