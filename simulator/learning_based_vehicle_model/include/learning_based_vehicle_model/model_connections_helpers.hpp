// Copyright 2024 The Autoware Foundation.
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

#ifndef LEARNING_BASED_VEHICLE_MODEL__MODEL_CONNECTIONS_HELPERS_HPP_
#define LEARNING_BASED_VEHICLE_MODEL__MODEL_CONNECTIONS_HELPERS_HPP_

#include <cstring>
#include <vector>

std::vector<double> fillVectorUsingMap(
  std::vector<double> vector1, std::vector<double> vector2, const std::vector<int> & map,
  bool inverse);

std::vector<int> createConnectionsMap(
  const std::vector<char *> & connection_names_1, const std::vector<char *> & connection_names_2);

#endif  // LEARNING_BASED_VEHICLE_MODEL__MODEL_CONNECTIONS_HELPERS_HPP_
