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

#include "autoware/probabilistic_occupancy_grid_map/updater/log_odds_bayes_filter_updater.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"

#include <algorithm>

// cspell: ignore LOBF

namespace autoware::occupancy_grid_map
{
namespace costmap_2d
{

void OccupancyGridMapLOBFUpdater::initRosParam(rclcpp::Node & /*node*/)
{
  // nothing to load
}

inline unsigned char OccupancyGridMapLOBFUpdater::applyLOBF(
  const unsigned char & z, const unsigned char & o)
{
  using fusion_policy::convertCharToProbability;
  using fusion_policy::convertProbabilityToChar;
  using fusion_policy::log_odds_fusion::logOddsFusion;

  constexpr unsigned char unknown = cost_value::NO_INFORMATION;
  constexpr unsigned char unknown_margin = 1;
  /* Tau and ST decides how fast the observation decay to the unknown status*/
  constexpr double tau = 0.75;
  constexpr double sample_time = 0.1;

  // if the observation is unknown, decay the estimation
  if (z >= unknown - unknown_margin && z <= unknown + unknown_margin) {
    char diff = static_cast<char>(o) - static_cast<char>(unknown);
    const double decay = std::exp(-sample_time / tau);
    const double fused = static_cast<double>(unknown) + static_cast<double>(diff) * decay;
    return static_cast<unsigned char>(fused);
  } else {
    // else, do the log-odds fusion
    const std::vector<double> probability = {
      convertCharToProbability(z), convertCharToProbability(o)};
    const unsigned char fused = convertProbabilityToChar(logOddsFusion(probability));
    return fused;
  }
}

bool OccupancyGridMapLOBFUpdater::update(const Costmap2D & single_frame_occupancy_grid_map)
{
  updateOrigin(
    single_frame_occupancy_grid_map.getOriginX(), single_frame_occupancy_grid_map.getOriginY());
  for (unsigned int x = 0; x < getSizeInCellsX(); x++) {
    for (unsigned int y = 0; y < getSizeInCellsY(); y++) {
      unsigned int index = getIndex(x, y);
      costmap_[index] = applyLOBF(single_frame_occupancy_grid_map.getCost(x, y), costmap_[index]);
    }
  }
  return true;
}

}  // namespace costmap_2d
}  // namespace autoware::occupancy_grid_map
