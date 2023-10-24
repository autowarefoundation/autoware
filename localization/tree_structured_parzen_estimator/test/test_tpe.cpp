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
    const int64_t n = input.size();
    for (int64_t i = 0; i < n; i++) {
      const double v = input[i] * 10;
      value += v * v;
    }
    return value;
  };

  constexpr int64_t kOuterTrialsNum = 10;
  constexpr int64_t kInnerTrialsNum = 100;
  std::cout << std::fixed;
  std::vector<double> mean_scores;
  for (const int64_t n_startup_trials : {kInnerTrialsNum, kInnerTrialsNum / 10}) {
    const std::string method = ((n_startup_trials == kInnerTrialsNum) ? "Random" : "TPE");

    std::vector<double> scores;
    for (int64_t i = 0; i < kOuterTrialsNum; i++) {
      double best_score = std::numeric_limits<double>::lowest();
      const std::vector<bool> is_loop_variable(6, false);
      TreeStructuredParzenEstimator estimator(
        TreeStructuredParzenEstimator::Direction::MAXIMIZE, n_startup_trials, is_loop_variable);
      for (int64_t trial = 0; trial < kInnerTrialsNum; trial++) {
        const TreeStructuredParzenEstimator::Input input = estimator.get_next_input();
        const double score = -sphere_function(input);
        estimator.add_trial({input, score});
        best_score = std::max(best_score, score);
      }
      scores.push_back(best_score);
    }

    const double sum = std::accumulate(scores.begin(), scores.end(), 0.0);
    const double mean = sum / scores.size();
    mean_scores.push_back(mean);
    double sq_sum = 0.0;
    for (const double score : scores) {
      sq_sum += (score - mean) * (score - mean);
    }
    const double stddev = std::sqrt(sq_sum / scores.size());

    std::cout << method << ", mean = " << mean << ", stddev = " << stddev << std::endl;
  }
  ASSERT_LT(mean_scores[0], mean_scores[1]);
}
