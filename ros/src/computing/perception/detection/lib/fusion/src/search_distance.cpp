#include <algorithm>
#include <iterator>
#include <map>
#include "fusion/search_distance.h"


float getShortest(const std::vector<float>& candidates) {
  if (candidates.empty())
    return 0.0f;

  return *std::min_element(candidates.begin(), candidates.end());
}


float getMedian(const std::vector<float>& candidates) {
  size_t num_candidates = candidates.size();

  if (num_candidates == 0) {
    return 0.0f;
  } else if (num_candidates == 1) {
    return candidates.at(0);
  }

  /* Sort candidate by its distances */
  std::vector<float> sorted_candidates(candidates);
  std::sort(sorted_candidates.begin(), sorted_candidates.end());

  float median;

  if (num_candidates % 2 == 0) {
    median = (sorted_candidates.at(num_candidates/2 - 1) + sorted_candidates.at(num_candidates/2)) / 2;
  } else {
    median = sorted_candidates.at(num_candidates/2);
  }

  return median;
}


float getMode(const std::vector<float>& candidates) {
  if (candidates.empty())
    return 0.0f;

  /*
    Express histogram by std::map.
    key (first) of map = bin (category) of histgram (bin means integer distance of candidates here)
    value (second) of map = candidate's real value which belong to that category
  */
  std::map<int, std::vector<float>> histogram;

  /* Categorize each candidates into histogram */
  for (const auto& sample : candidates) {
    int bin = static_cast<float>(sample);
    histogram[bin].push_back(sample);
  }

  /* Search the most common bin */
  size_t max_num_candidates = 0;
  int max_bin = 0;
  for (const auto& bin : histogram) {
    if (max_num_candidates < bin.second.size()) {
      max_bin = bin.first;
      max_num_candidates = bin.second.size();
    }
  }

  /* Calculate median value of candidates which belong to most common bin */
  return getMedian(histogram[max_bin]);
}
