// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <vector>

#include "common/types.hpp"
#include "gtest/gtest.h"
#include "trajectory_follower/lowpass_filter.hpp"

using autoware::common::types::float64_t;

TEST(TestLowpassFilter, LowpassFilter1d)
{
  using autoware::motion::control::trajectory_follower::LowpassFilter1d;

  const float64_t epsilon = 1e-6;
  LowpassFilter1d lowpass_filter_1d(0.0, 0.1);

  // initial state
  EXPECT_NEAR(lowpass_filter_1d.getValue(), 0.0, epsilon);

  // random filter
  EXPECT_NEAR(lowpass_filter_1d.filter(0.0), 0.0, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.filter(1.0), 0.9, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.filter(2.0), 1.89, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.getValue(), 1.89, epsilon);

  // reset
  lowpass_filter_1d.reset(-1.1);
  EXPECT_NEAR(lowpass_filter_1d.getValue(), -1.1, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.filter(0.0), -0.11, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.getValue(), -0.11, epsilon);
}
TEST(TestLowpassFilter, MoveAverageFilter) {
  namespace MoveAverageFilter = autoware::motion::control::trajectory_follower::MoveAverageFilter;

  {  // Fail case: window size higher than the vector size
    const int64_t window_size = 5;
    std::vector<float64_t> vec = {1.0, 2.0, 3.0, 4.0};
    EXPECT_FALSE(MoveAverageFilter::filt_vector(window_size, vec));
  }
  {
    const int64_t window_size = 0;
    const std::vector<float64_t> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<float64_t> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    for (size_t i = 0; i < filtered_vec.size(); ++i) {
      EXPECT_EQ(filtered_vec[i], original_vec[i]);
    }
  }
  {
    const int64_t window_size = 1;
    const std::vector<float64_t> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<float64_t> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 13.0 / 3);
    EXPECT_EQ(filtered_vec[3], original_vec[3]);
  }
  {
    const int64_t window_size = 2;
    const std::vector<float64_t> original_vec = {1.0, 3.0, 4.0, 6.0, 7.0, 10.0};
    std::vector<float64_t> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 21.0 / 5);
    EXPECT_EQ(filtered_vec[3], 30.0 / 5);
    EXPECT_EQ(filtered_vec[4], 23.0 / 3);
    EXPECT_EQ(filtered_vec[5], original_vec[5]);
  }
}
TEST(TestLowpassFilter, Butterworth2dFilter) {
  using autoware::motion::control::trajectory_follower::Butterworth2dFilter;
  const float64_t dt = 1.0;
  const float64_t cutoff_hz = 1.0;
  Butterworth2dFilter filter(dt, cutoff_hz);
  for (float64_t i = 1.0; i < 10.0; ++i) {
    EXPECT_LT(filter.filter(i), i);
  }

  const std::vector<float64_t> original_vec = {1.0, 2.0, 3.0, 4.0};
  std::vector<float64_t> filtered_vec;
  filter.filt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size());
  EXPECT_EQ(filtered_vec[0], original_vec[0]);
  for (size_t i = 1; i < filtered_vec.size(); ++i) {
    EXPECT_LT(filtered_vec[i], original_vec[i]);
  }

  filtered_vec.clear();
  filter.filtfilt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size());

  std::vector<float64_t> coefficients;
  filter.getCoefficients(coefficients);
  EXPECT_EQ(coefficients.size(), size_t(6));
}
