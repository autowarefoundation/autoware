// Copyright 2023 Autoware Foundation
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

#ifndef LOCALIZATION_UTIL__TREE_STRUCTURED_PARZEN_ESTIMATOR_HPP_
#define LOCALIZATION_UTIL__TREE_STRUCTURED_PARZEN_ESTIMATOR_HPP_

/*
A implementation of tree-structured parzen estimator (TPE)
See below pdf for the TPE algorithm detail.
https://papers.nips.cc/paper_files/paper/2011/file/86e8f7ab32cfd12577bc2619bc635690-Paper.pdf

Optuna is also used as a reference for implementation.
https://github.com/optuna/optuna
*/

#include <cstdint>
#include <random>
#include <vector>

class TreeStructuredParzenEstimator
{
public:
  using Input = std::vector<double>;
  using Score = double;
  struct Trial
  {
    Input input;
    Score score;
  };

  enum Direction {
    MINIMIZE = 0,
    MAXIMIZE = 1,
  };

  enum Index {
    TRANS_X = 0,
    TRANS_Y = 1,
    TRANS_Z = 2,
    ANGLE_X = 3,
    ANGLE_Y = 4,
    ANGLE_Z = 5,
    INDEX_NUM = 6,
  };

  TreeStructuredParzenEstimator() = delete;
  TreeStructuredParzenEstimator(
    const Direction direction, const int64_t n_startup_trials, std::vector<double> sample_mean,
    std::vector<double> sample_stddev);
  void add_trial(const Trial & trial);
  [[nodiscard]] Input get_next_input() const;

private:
  static constexpr double max_good_rate = 0.10;
  static constexpr int64_t n_ei_candidates = 100;

  static std::mt19937_64 engine;

  [[nodiscard]] double compute_log_likelihood_ratio(const Input & input) const;
  [[nodiscard]] static double log_gaussian_pdf(
    const Input & input, const Input & mu, const Input & sigma);

  std::vector<Trial> trials_;
  int64_t above_num_;
  const Direction direction_;
  const int64_t n_startup_trials_;
  const int64_t input_dimension_;
  const std::vector<double> sample_mean_;
  const std::vector<double> sample_stddev_;
  Input base_stddev_;
};

#endif  // LOCALIZATION_UTIL__TREE_STRUCTURED_PARZEN_ESTIMATOR_HPP_
