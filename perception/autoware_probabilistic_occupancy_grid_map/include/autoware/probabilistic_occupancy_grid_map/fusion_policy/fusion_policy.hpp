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

#ifndef AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__FUSION_POLICY__FUSION_POLICY_HPP_
#define AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__FUSION_POLICY__FUSION_POLICY_HPP_

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

/*min and max prob threshold to prevent log-odds to diverge*/
#define EPSILON_PROB 0.03

namespace autoware::occupancy_grid_map
{
namespace fusion_policy
{
enum class FusionMethod { OVERWRITE, LOG_ODDS, DEMPSTER_SHAFER };

unsigned char convertProbabilityToChar(const double & probability);
double convertCharToProbability(const unsigned char & occupancy);
std::vector<unsigned char> convertProbabilityToChar(const std::vector<double> & probabilities);
std::vector<double> convertCharToProbability(const std::vector<unsigned char> & occupancies);

namespace overwrite_fusion
{
enum State : unsigned char { UNKNOWN = 0U, FREE = 1U, OCCUPIED = 2U };
State getApproximateState(const unsigned char & occupancy);
unsigned char overwriteFusion(const std::vector<unsigned char> & occupancies);
}  // namespace overwrite_fusion

namespace log_odds_fusion
{
double logOddsFusion(const std::vector<double> & probabilities);
double logOddsFusion(
  const std::vector<double> & probabilities, const std::vector<double> & weights);
}  // namespace log_odds_fusion

namespace dempster_shafer_fusion
{
struct dempsterShaferOccupancy;
double dempsterShaferFusion(const std::vector<double> & probability);
double dempsterShaferFusion(
  const std::vector<double> & probability, const std::vector<double> & reliability);
}  // namespace dempster_shafer_fusion

unsigned char singleFrameOccupancyFusion(
  const std::vector<unsigned char> & occupancy, FusionMethod method);
unsigned char singleFrameOccupancyFusion(
  const std::vector<unsigned char> & occupancy, FusionMethod method,
  const std::vector<double> & reliability);

}  // namespace fusion_policy
}  // namespace autoware::occupancy_grid_map

#endif  // AUTOWARE__PROBABILISTIC_OCCUPANCY_GRID_MAP__FUSION_POLICY__FUSION_POLICY_HPP_
