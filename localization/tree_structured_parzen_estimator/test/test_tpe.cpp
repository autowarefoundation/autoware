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

#include "tree_structured_parzen_estimator/tree_structured_parzen_estimator.hpp"

#include <gtest/gtest.h>

TEST(TreeStructuredParzenEstimatorTest, TPE_is_better_than_random_search_on_sphere_function)
{
  auto sphere_function = [](const TreeStructuredParzenEstimator::Input & input) {
    double value = 0.0;
    const auto n = static_cast<int64_t>(input.size());
    for (int64_t i = 0; i < n; i++) {
      const double v = input[i] * 10;
      value += v * v;
    }
    return value;
  };

  constexpr int64_t k_outer_trials_num = 20;
  constexpr int64_t k_inner_trials_num = 200;
  std::cout << std::fixed;
  std::vector<double> mean_scores;
  std::vector<double> sample_mean(5, 0.0);
  std::vector<double> sample_stddev{1.0, 1.0, 0.1, 0.1, 0.1};

  for (const int64_t n_startup_trials : {k_inner_trials_num, k_inner_trials_num / 2}) {
    const std::string method = ((n_startup_trials == k_inner_trials_num) ? "Random" : "TPE");

    std::vector<double> scores;
    for (int64_t i = 0; i < k_outer_trials_num; i++) {
      double best_score = std::numeric_limits<double>::lowest();
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, n_startup_trials, sample_mean,
        sample_stddev);
      for (int64_t trial = 0; trial < k_inner_trials_num; trial++) {
        const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
        const double score = -sphere_function(input);
        estimator.add_trial({input, score});
        best_score = std::max(best_score, score);
      }
      scores.push_back(best_score);
    }

    const double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    const double mean = sum / static_cast<double>(scores.size());
    mean_scores.push_back(mean);
    double sq_sum = std::accumulate(
      scores.begin(), scores.end(), 0.0,
      [mean](double total, double score) { return total + (score - mean) * (score - mean); });
    const double stddev = std::sqrt(sq_sum / static_cast<double>(scores.size()));

    std::cout << method << ", mean = " << mean << ", stddev = " << stddev << std::endl;
  }
  ASSERT_LT(mean_scores[0], mean_scores[1]);
}
