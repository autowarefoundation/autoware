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

#include "autoware/probabilistic_occupancy_grid_map/fusion_policy/fusion_policy.hpp"

namespace autoware::occupancy_grid_map
{
namespace fusion_policy
{

//// utils ////

// convert [0, 1] to [0, 255]
unsigned char convertProbabilityToChar(const double & probability)
{
  return static_cast<unsigned char>(probability * 255);
}
// convert [0, 255] to [0, 1]
double convertCharToProbability(const unsigned char & occupancy)
{
  return static_cast<double>(occupancy) / 255.0;
}

// convert [0, 1] to [0, 255] vectors
std::vector<unsigned char> convertProbabilityToChar(const std::vector<double> & probabilities)
{
  std::vector<unsigned char> occupancies;
  for (const auto & probability : probabilities) {
    occupancies.push_back(convertProbabilityToChar(probability));
  }
  return occupancies;
}
// convert [0, 255] to [0, 1] vectors
std::vector<double> convertCharToProbability(const std::vector<unsigned char> & occupancies)
{
  std::vector<double> probabilities;
  for (const auto & occupancy : occupancies) {
    probabilities.push_back(convertCharToProbability(occupancy));
  }
  return probabilities;
}

/// @brief fusion with overwrite policy
namespace overwrite_fusion
{

/**
 * @brief convert char value to occupancy state
 *
 * @param occupancy [0, 255]
 * @return State
 */
State getApproximateState(const unsigned char & occupancy)
{
  if (occupancy >= cost_value::OCCUPIED_THRESHOLD) {
    return State::OCCUPIED;
  } else if (occupancy <= cost_value::FREE_THRESHOLD) {
    return State::FREE;
  } else {
    return State::UNKNOWN;
  }
}

/**
 * @brief override fusion
 *
 * @param occupancies : occupancies to be fused
 * @return unsigned char
 */
unsigned char overwriteFusion(const std::vector<unsigned char> & occupancies)
{
  if (occupancies.size() == 0) {
    throw std::runtime_error("occupancies size is 0");
  } else if (occupancies.size() == 1) {
    return occupancies[0];
  }

  auto fusion_occupancy = 128;  // unknown
  auto fusion_state = getApproximateState(fusion_occupancy);
  for (const auto & cell : occupancies) {
    auto state = getApproximateState(cell);
    if (state > fusion_state) {
      fusion_state = state;
      fusion_occupancy = cell;
    } else if (state < fusion_state) {
      continue;
    } else {
      fusion_occupancy = (fusion_occupancy / 2 + cell / 2);
    }
  }
  return fusion_occupancy;
}
}  // namespace overwrite_fusion

/// @brief fusion with log-odds policy
namespace log_odds_fusion
{
/**
 * @brief log-odds fusion
 *
 * @param probabilities : probabilities of occupancy [0, 1]
 * @return double
 */
double logOddsFusion(const std::vector<double> & probabilities)
{
  double log_odds = 0.0;
  for (const auto & probability : probabilities) {
    const double p = std::max(EPSILON_PROB, std::min(1.0 - EPSILON_PROB, probability));
    log_odds += std::log(p / (1.0 - p));
  }
  return 1.0 / (1.0 + std::exp(-log_odds));
}

/**
 * @brief weighted log-odds fusion
 *
 * @param probabilities : probabilities of occupancy [0, 1]
 * @param weights : weights of probabilities
 * @return double
 */
double logOddsFusion(const std::vector<double> & probabilities, const std::vector<double> & weights)
{
  // check if the size of probabilities and weights are the same
  if (probabilities.size() != weights.size()) {
    // warning and return normal log-odds fusion
    std::cout
      << "The size of probabilities and weights are not the same. Return normal log-odds fusion."
      << std::endl;
    return logOddsFusion(probabilities);
  }

  double log_odds = 0.0;
  for (size_t i = 0; i < probabilities.size(); i++) {
    const double p = std::max(EPSILON_PROB, std::min(1.0 - EPSILON_PROB, probabilities[i]));
    log_odds += weights[i] * std::log(p / (1.0 - p));
  }
  return 1.0 / (1.0 + std::exp(-log_odds));
}
}  // namespace log_odds_fusion

/// @brief fusion with Dempster-Shafer Theory
namespace dempster_shafer_fusion
{
/**
 * @brief conflict modified Dempster-Shafer Theory Data structure
 * see https://www.diva-portal.org/smash/get/diva2:852457/FULLTEXT01.pdf
 *
 */
struct dempsterShaferOccupancy
{
  double occupied;
  double empty;
  double unknown;
  double conflict_threshold = 0.9;

  // initialize without args
  dempsterShaferOccupancy()
  {
    occupied = 0.0;
    empty = 0.0;
    unknown = 1.0;
  }

  // initialize with probability
  explicit dempsterShaferOccupancy(double occupied_probability)
  {
    // confine to [0, 1]
    double p = std::max(0.0, std::min(1.0, occupied_probability));
    occupied = p;
    empty = 1.0 - p;
    unknown = 0.0;
  }

  // initialize with probability and reliability
  dempsterShaferOccupancy(double occupied_probability, double reliability)
  {
    // confine to [0, 1]
    double p = std::max(0.0, std::min(1.0, occupied_probability));
    occupied = p * reliability;
    empty = (1.0 - p) * reliability;
    unknown = 1.0 - occupied - empty;
  }

  // normalize
  void normalize()
  {
    double sum = occupied + empty + unknown;
    occupied /= sum;
    empty /= sum;
    unknown /= sum;
  }

  // calc conflict factor K
  double calcK(const dempsterShaferOccupancy & other) const
  {
    return (occupied * other.empty + empty * other.occupied);
  }
  // calc sum of occupied probability mass
  double calcOccupied(const dempsterShaferOccupancy & other) const
  {
    return occupied * other.occupied + occupied * other.unknown + unknown * other.occupied;
  }
  // calc sum of empty probability mass
  double calcEmpty(const dempsterShaferOccupancy & other) const
  {
    return empty * other.empty + empty * other.unknown + unknown * other.empty;
  }

  // Dempster-Shafer fusion
  dempsterShaferOccupancy operator+(const dempsterShaferOccupancy & other) const
  {
    dempsterShaferOccupancy result;
    double K = calcK(other);
    double O = calcOccupied(other);
    double E = calcEmpty(other);

    if (K > conflict_threshold) {
      // highly conflict
      result.occupied = O;
      result.empty = E;
      result.unknown = 1 - O - E;
    } else {
      // low conflict
      result.occupied = O / (1.0 - K);
      result.empty = E / (1.0 - K);
      result.unknown = 1 - result.occupied - result.empty;
    }
    return result;
  }

  // get occupancy probability via Pignistic Probability
  double getPignisticProbability() const { return occupied + unknown / 2.0; }
};

/**
 * @brief Dempster-Shafer fusion
 *
 * @param probability
 * @return double
 */
double dempsterShaferFusion(const std::vector<double> & probability)
{
  dempsterShaferOccupancy result;  // init with unknown
  for (const auto & p : probability) {
    result = result + dempsterShaferOccupancy(p);
  }
  return result.getPignisticProbability();
}

/**
 * @brief Dempster-Shafer fusion with reliability
 *
 * @param probability
 * @param reliability
 * @return double
 */
double dempsterShaferFusion(
  const std::vector<double> & probability, const std::vector<double> & reliability)
{
  // check if the size of probabilities and weights are the same
  if (probability.size() != reliability.size()) {
    // warning and return normal dempster-shafer fusion probability
    std::cout << "The size of probabilities and reliability are not the same. Return normal "
                 "dempster-shafer fusion."
              << std::endl;
    return dempsterShaferFusion(probability);
  }

  dempsterShaferOccupancy result;  // init with unknown
  for (size_t i = 0; i < probability.size(); i++) {
    result = result + dempsterShaferOccupancy(probability[i], reliability[i]);
  }
  return result.getPignisticProbability();
}
}  // namespace dempster_shafer_fusion

unsigned char singleFrameOccupancyFusion(
  const std::vector<unsigned char> & occupancy, FusionMethod method)
{
  if (method == FusionMethod::OVERWRITE) {
    return overwrite_fusion::overwriteFusion(occupancy);
  } else if (method == FusionMethod::LOG_ODDS) {
    auto probability = convertCharToProbability(occupancy);
    return convertProbabilityToChar(log_odds_fusion::logOddsFusion(probability));
  } else if (method == FusionMethod::DEMPSTER_SHAFER) {
    auto probability = convertCharToProbability(occupancy);
    return convertProbabilityToChar(dempster_shafer_fusion::dempsterShaferFusion(probability));
  } else {
    std::cout << "Unknown fusion method: " << std::endl;
    return 128;
  }
}

unsigned char singleFrameOccupancyFusion(
  const std::vector<unsigned char> & occupancy, FusionMethod method,
  const std::vector<double> & reliability)
{
  if (method == FusionMethod::OVERWRITE) {
    return overwrite_fusion::overwriteFusion(occupancy);
  } else if (method == FusionMethod::LOG_ODDS) {
    auto probability = convertCharToProbability(occupancy);
    return convertProbabilityToChar(log_odds_fusion::logOddsFusion(probability, reliability));
  } else if (method == FusionMethod::DEMPSTER_SHAFER) {
    auto probability = convertCharToProbability(occupancy);
    return convertProbabilityToChar(
      dempster_shafer_fusion::dempsterShaferFusion(probability, reliability));
  } else {
    std::cout << "Unknown fusion method: " << std::endl;
    return 128;
  }
}

}  // namespace fusion_policy
}  // namespace autoware::occupancy_grid_map
