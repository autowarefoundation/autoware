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

#include "localization_util/tree_structured_parzen_estimator.hpp"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>

// random number generator
std::mt19937_64 TreeStructuredParzenEstimator::engine(0);

TreeStructuredParzenEstimator::TreeStructuredParzenEstimator(
  const Direction direction, const int64_t n_startup_trials, std::vector<double> sample_mean,
  std::vector<double> sample_stddev)
: above_num_(0),
  direction_(direction),
  n_startup_trials_(n_startup_trials),
  input_dimension_(INDEX_NUM),
  sample_mean_(std::move(sample_mean)),
  sample_stddev_(std::move(sample_stddev))
{
  if (sample_mean_.size() != ANGLE_Z) {
    std::cerr << "sample_mean size is invalid" << std::endl;
    throw std::runtime_error("sample_mean size is invalid");
  }
  if (sample_stddev_.size() != ANGLE_Z) {
    std::cerr << "sample_stddev size is invalid" << std::endl;
    throw std::runtime_error("sample_stddev size is invalid");
  }
  // base_stddev_ is defined as the stable convergence range of ndt_scan_matcher.
  base_stddev_.resize(input_dimension_);
  base_stddev_[TRANS_X] = 0.25;                // [m]
  base_stddev_[TRANS_Y] = 0.25;                // [m]
  base_stddev_[TRANS_Z] = 0.25;                // [m]
  base_stddev_[ANGLE_X] = 1.0 / 180.0 * M_PI;  // [rad]
  base_stddev_[ANGLE_Y] = 1.0 / 180.0 * M_PI;  // [rad]
  base_stddev_[ANGLE_Z] = 2.5 / 180.0 * M_PI;  // [rad]
}

void TreeStructuredParzenEstimator::add_trial(const Trial & trial)
{
  trials_.push_back(trial);
  std::sort(trials_.begin(), trials_.end(), [this](const Trial & lhs, const Trial & rhs) {
    return (direction_ == Direction::MAXIMIZE ? lhs.score > rhs.score : lhs.score < rhs.score);
  });
  above_num_ = std::min(
    {static_cast<int64_t>(10),
     static_cast<int64_t>(static_cast<double>(trials_.size()) * max_good_rate)});
}

TreeStructuredParzenEstimator::Input TreeStructuredParzenEstimator::get_next_input() const
{
  std::normal_distribution<double> dist_normal_trans_x(
    sample_mean_[TRANS_X], sample_stddev_[TRANS_X]);
  std::normal_distribution<double> dist_normal_trans_y(
    sample_mean_[TRANS_Y], sample_stddev_[TRANS_Y]);
  std::normal_distribution<double> dist_normal_trans_z(
    sample_mean_[TRANS_Z], sample_stddev_[TRANS_Z]);
  std::normal_distribution<double> dist_normal_angle_x(
    sample_mean_[ANGLE_X], sample_stddev_[ANGLE_X]);
  std::normal_distribution<double> dist_normal_angle_y(
    sample_mean_[ANGLE_Y], sample_stddev_[ANGLE_Y]);
  std::uniform_real_distribution<double> dist_uniform_angle_z(-M_PI, M_PI);

  if (static_cast<int64_t>(trials_.size()) < n_startup_trials_ || above_num_ == 0) {
    // Random sampling based on prior until the number of trials reaches `n_startup_trials_`.
    Input input(input_dimension_);
    input[TRANS_X] = dist_normal_trans_x(engine);
    input[TRANS_Y] = dist_normal_trans_y(engine);
    input[TRANS_Z] = dist_normal_trans_z(engine);
    input[ANGLE_X] = dist_normal_angle_x(engine);
    input[ANGLE_Y] = dist_normal_angle_y(engine);
    input[ANGLE_Z] = dist_uniform_angle_z(engine);
    return input;
  }

  Input best_input;
  double best_log_likelihood_ratio = std::numeric_limits<double>::lowest();
  for (int64_t i = 0; i < n_ei_candidates; i++) {
    Input input(input_dimension_);
    input[TRANS_X] = dist_normal_trans_x(engine);
    input[TRANS_Y] = dist_normal_trans_y(engine);
    input[TRANS_Z] = dist_normal_trans_z(engine);
    input[ANGLE_X] = dist_normal_angle_x(engine);
    input[ANGLE_Y] = dist_normal_angle_y(engine);
    input[ANGLE_Z] = dist_uniform_angle_z(engine);
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
  const auto n = static_cast<int64_t>(trials_.size());

  // The above KDE and the below KDE are calculated respectively, and the ratio is the criteria to
  // select best sample.
  std::vector<double> above_logs;
  std::vector<double> below_logs;

  for (int64_t i = 0; i < n; i++) {
    const double log_p = log_gaussian_pdf(input, trials_[i].input, base_stddev_);
    if (i < above_num_) {
      const double w = 1.0 / static_cast<double>(above_num_);
      const double log_w = std::log(w);
      above_logs.push_back(log_p + log_w);
    } else {
      const double w = 1.0 / static_cast<double>(n - above_num_);
      const double log_w = std::log(w);
      below_logs.push_back(log_p + log_w);
    }
  }

  auto log_sum_exp = [](const std::vector<double> & log_vec) {
    const double max = *std::max_element(log_vec.begin(), log_vec.end());
    double sum = std::accumulate(
      log_vec.begin(), log_vec.end(), 0.0,
      [max](double total, double log_v) { return total + std::exp(log_v - max); });
    return max + std::log(sum);
  };

  const double above = log_sum_exp(above_logs);
  const double below = log_sum_exp(below_logs);

  // Multiply by a constant so that the score near the "below sample" becomes lower.
  // cspell:disable-line TODO(Shintaro Sakoda): It's theoretically incorrect, consider it again
  // later.
  const double r = above - below * 5.0;
  return r;
}

double TreeStructuredParzenEstimator::log_gaussian_pdf(
  const Input & input, const Input & mu, const Input & sigma)
{
  const double log_2pi = std::log(2.0 * M_PI);
  auto log_gaussian_pdf_1d = [&](const double diff, const double sigma) {
    return -0.5 * log_2pi - std::log(sigma) - (diff * diff) / (2.0 * sigma * sigma);
  };

  const auto n = static_cast<int64_t>(input.size());

  double result = 0.0;
  for (int64_t i = 0; i < n; i++) {
    double diff = input[i] - mu[i];
    if (i == ANGLE_Z) {
      // Normalize the loop variable to [-pi, pi)
      while (diff >= M_PI) {
        diff -= 2 * M_PI;
      }
      while (diff < -M_PI) {
        diff += 2 * M_PI;
      }
    }
    // Experimentally, it is better to consider only trans_xy and yaw, so ignore trans_z, angle_x,
    // angle_y.
    if (i == TRANS_Z || i == ANGLE_X || i == ANGLE_Y) {
      continue;
    }
    result += log_gaussian_pdf_1d(diff, sigma[i]);
  }
  return result;
}
