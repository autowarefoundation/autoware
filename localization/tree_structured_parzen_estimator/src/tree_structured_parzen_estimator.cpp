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

#include "tree_structured_parzen_estimator/tree_structured_parzen_estimator.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>

// random number generator
std::mt19937_64 TreeStructuredParzenEstimator::engine(std::random_device{}());
std::uniform_real_distribution<double> TreeStructuredParzenEstimator::dist_uniform(
  TreeStructuredParzenEstimator::MIN_VALUE, TreeStructuredParzenEstimator::MAX_VALUE);
std::normal_distribution<double> TreeStructuredParzenEstimator::dist_normal(0.0, 1.0);

TreeStructuredParzenEstimator::TreeStructuredParzenEstimator(
  const Direction direction, const int64_t n_startup_trials, std::vector<bool> is_loop_variable)
: above_num_(0),
  direction_(direction),
  n_startup_trials_(n_startup_trials),
  input_dimension_(is_loop_variable.size()),
  is_loop_variable_(is_loop_variable),
  base_stddev_(input_dimension_, VALUE_WIDTH)
{
}

void TreeStructuredParzenEstimator::add_trial(const Trial & trial)
{
  trials_.push_back(trial);
  std::sort(trials_.begin(), trials_.end(), [this](const Trial & lhs, const Trial & rhs) {
    return (direction_ == Direction::MAXIMIZE ? lhs.score > rhs.score : lhs.score < rhs.score);
  });
  above_num_ =
    std::min(static_cast<int64_t>(25), static_cast<int64_t>(trials_.size() * MAX_GOOD_RATE));
}

TreeStructuredParzenEstimator::Input TreeStructuredParzenEstimator::get_next_input() const
{
  if (static_cast<int64_t>(trials_.size()) < n_startup_trials_ || above_num_ == 0) {
    // Random sampling based on prior until the number of trials reaches `n_startup_trials_`.
    Input input(input_dimension_);
    for (int64_t j = 0; j < input_dimension_; j++) {
      input[j] = dist_uniform(engine);
    }
    return input;
  }

  Input best_input;
  double best_log_likelihood_ratio = std::numeric_limits<double>::lowest();
  const double coeff = BASE_STDDEV_COEFF * std::pow(above_num_, -1.0 / (4 + input_dimension_));
  std::vector<double> weights = get_weights(above_num_);
  weights.push_back(PRIOR_WEIGHT);
  std::discrete_distribution<int64_t> dist(weights.begin(), weights.end());
  for (int64_t i = 0; i < N_EI_CANDIDATES; i++) {
    Input mu, sigma;
    const int64_t index = dist(engine);
    if (index == above_num_) {
      mu = Input(input_dimension_, 0.0);
      sigma = base_stddev_;
    } else {
      mu = trials_[index].input;
      sigma = base_stddev_;
      for (int64_t j = 0; j < input_dimension_; j++) {
        sigma[j] *= coeff;
      }
    }
    // sample from the normal distribution
    Input input(input_dimension_);
    for (int64_t j = 0; j < input_dimension_; j++) {
      input[j] = mu[j] + dist_normal(engine) * sigma[j];
      input[j] =
        (is_loop_variable_[j] ? normalize_loop_variable(input[j])
                              : std::clamp(input[j], MIN_VALUE, MAX_VALUE));
    }
    const double log_likelihood_ratio = compute_log_likelihood_ratio(input);
    if (log_likelihood_ratio > best_log_likelihood_ratio) {
      best_log_likelihood_ratio = log_likelihood_ratio;
      best_input = input;
    }
  }
  return best_input;
}

double TreeStructuredParzenEstimator::compute_log_likelihood_ratio(const Input & input) const
{
  const int64_t n = trials_.size();

  // The above KDE and the below KDE are calculated respectively, and the ratio is the criteria to
  // select best sample.
  std::vector<double> above_logs;
  std::vector<double> below_logs;

  // Scott's rule
  const double coeff_above =
    BASE_STDDEV_COEFF * std::pow(above_num_, -1.0 / (4 + input_dimension_));
  const double coeff_below =
    BASE_STDDEV_COEFF * std::pow(n - above_num_, -1.0 / (4 + input_dimension_));
  Input sigma_above = base_stddev_;
  Input sigma_below = base_stddev_;
  for (int64_t j = 0; j < input_dimension_; j++) {
    sigma_above[j] *= coeff_above;
    sigma_below[j] *= coeff_below;
  }

  std::vector<double> above_weights = get_weights(above_num_);
  std::vector<double> below_weights = get_weights(n - above_num_);
  std::reverse(below_weights.begin(), below_weights.end());  // below_weights is ascending order

  // calculate the sum of weights to normalize
  double above_sum = std::accumulate(above_weights.begin(), above_weights.end(), 0.0);
  double below_sum = std::accumulate(below_weights.begin(), below_weights.end(), 0.0);

  // above includes prior
  above_sum += PRIOR_WEIGHT;

  for (int64_t i = 0; i < n; i++) {
    if (i < above_num_) {
      const double log_p = log_gaussian_pdf(input, trials_[i].input, sigma_above);
      const double w = above_weights[i] / above_sum;
      const double log_w = std::log(w);
      above_logs.push_back(log_p + log_w);
    } else {
      const double log_p = log_gaussian_pdf(input, trials_[i].input, sigma_below);
      const double w = below_weights[i - above_num_] / below_sum;
      const double log_w = std::log(w);
      below_logs.push_back(log_p + log_w);
    }
  }

  // prior
  if (PRIOR_WEIGHT > 0.0) {
    const double log_p = log_gaussian_pdf(input, Input(input_dimension_, 0.0), base_stddev_);
    const double log_w = std::log(PRIOR_WEIGHT / above_sum);
    above_logs.push_back(log_p + log_w);
  }

  auto log_sum_exp = [](const std::vector<double> & log_vec) {
    const double max = *std::max_element(log_vec.begin(), log_vec.end());
    double sum = 0.0;
    for (const double log_v : log_vec) {
      sum += std::exp(log_v - max);
    }
    return max + std::log(sum);
  };

  const double above = log_sum_exp(above_logs);
  const double below = log_sum_exp(below_logs);
  const double r = above - below;
  return r;
}

double TreeStructuredParzenEstimator::log_gaussian_pdf(
  const Input & input, const Input & mu, const Input & sigma) const
{
  const double log_2pi = std::log(2.0 * M_PI);
  auto log_gaussian_pdf_1d = [&](const double diff, const double sigma) {
    return -0.5 * log_2pi - std::log(sigma) - (diff * diff) / (2.0 * sigma * sigma);
  };

  const int64_t n = input.size();

  double result = 0.0;
  for (int64_t i = 0; i < n; i++) {
    double diff = input[i] - mu[i];
    if (is_loop_variable_[i]) {
      diff = normalize_loop_variable(diff);
    }
    result += log_gaussian_pdf_1d(diff, sigma[i]);
  }
  return result;
}

std::vector<double> TreeStructuredParzenEstimator::get_weights(const int64_t n)
{
  // See optuna
  // https://github.com/optuna/optuna/blob/4bfab78e98bf786f6a2ce6e593a26e3f8403e08d/optuna/samplers/_tpe/sampler.py#L50-L58
  std::vector<double> weights;
  constexpr int64_t WEIGHT_ALPHA = 25;
  if (n == 0) {
    return weights;
  } else if (n < WEIGHT_ALPHA) {
    weights.resize(n, 1.0);
  } else {
    weights.resize(n);
    const double unit = (1.0 - 1.0 / n) / (n - WEIGHT_ALPHA);
    for (int64_t i = 0; i < n; i++) {
      weights[i] = (i < WEIGHT_ALPHA ? 1.0 : 1.0 - unit * (i - WEIGHT_ALPHA));
    }
  }

  return weights;
}

double TreeStructuredParzenEstimator::normalize_loop_variable(const double value)
{
  // Normalize the loop variable to [-1, 1)
  double result = value;
  while (result >= MAX_VALUE) {
    result -= VALUE_WIDTH;
  }
  while (result < MIN_VALUE) {
    result += VALUE_WIDTH;
  }
  return result;
}
