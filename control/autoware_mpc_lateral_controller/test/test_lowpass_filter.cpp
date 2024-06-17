// Copyright 2021 The Autoware Foundation
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

#include "autoware/mpc_lateral_controller/lowpass_filter.hpp"
#include "gtest/gtest.h"

#include <vector>

TEST(TestLowpassFilter, MoveAverageFilter)
{
  namespace MoveAverageFilter =
    autoware::motion::control::mpc_lateral_controller::MoveAverageFilter;

  {  // Fail case: window size higher than the vector size
    const int window_size = 5;
    std::vector<double> vec = {1.0, 2.0, 3.0, 4.0};
    EXPECT_FALSE(MoveAverageFilter::filt_vector(window_size, vec));
  }  // namespace autoware::motion::control::mpc_lateral_controller::MoveAverageFilter;
  {
    const int window_size = 0;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    for (size_t i = 0; i < filtered_vec.size(); ++i) {
      EXPECT_EQ(filtered_vec[i], original_vec[i]);
    }
  }
  {
    const int window_size = 1;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 13.0 / 3);
    EXPECT_EQ(filtered_vec[3], original_vec[3]);
  }
  {
    const int window_size = 2;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0, 7.0, 10.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 8.0 / 3);
    EXPECT_EQ(filtered_vec[2], 21.0 / 5);
    EXPECT_EQ(filtered_vec[3], 30.0 / 5);
    EXPECT_EQ(filtered_vec[4], 23.0 / 3);
    EXPECT_EQ(filtered_vec[5], original_vec[5]);
  }
  {
    const int window_size = 3;
    const std::vector<double> original_vec = {1.0, 1.0, 1.0, 1.0};
    std::vector<double> filtered_vec = original_vec;
    EXPECT_TRUE(MoveAverageFilter::filt_vector(window_size, filtered_vec));
    ASSERT_EQ(filtered_vec.size(), original_vec.size());
    EXPECT_EQ(filtered_vec[0], original_vec[0]);
    EXPECT_EQ(filtered_vec[1], 1.0);
    EXPECT_EQ(filtered_vec[2], 1.0);
    EXPECT_EQ(filtered_vec[3], original_vec[3]);
  }
  {
    const int window_size = 4;
    const std::vector<double> original_vec = {1.0, 3.0, 4.0, 6.0, 7.0, 10.0};
    std::vector<double> filtered_vec = original_vec;
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
TEST(TestLowpassFilter, Butterworth2dFilter)
{
  using autoware::motion::control::mpc_lateral_controller::Butterworth2dFilter;
  const double dt = 1.0;
  const double cutoff_hz = 1.0;
  Butterworth2dFilter filter(dt, cutoff_hz);
  for (double i = 1.0; i < 10.0; ++i) {
    EXPECT_LT(filter.filter(i), i);
  }

  const std::vector<double> original_vec = {1.0, 2.0, 3.0, 4.0};
  std::vector<double> filtered_vec;
  filter.filt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size());
  EXPECT_NEAR(filtered_vec[0], original_vec[0], 1.0e-10);
  for (size_t i = 1; i < filtered_vec.size(); ++i) {
    EXPECT_LT(filtered_vec[i], original_vec[i]);
  }

  filtered_vec.clear();
  filter.filtfilt_vector(original_vec, filtered_vec);
  ASSERT_EQ(filtered_vec.size(), original_vec.size());

  std::vector<double> coefficients;
  filter.getCoefficients(coefficients);
  EXPECT_EQ(coefficients.size(), size_t(5));
}

// Comparison of the coefficients
TEST(TestLowpassFilter, Butterworth2dFilterCoeffs)
{
  using autoware::motion::control::mpc_lateral_controller::Butterworth2dFilter;

  // Case 1:
  // cutoff_frequency = 1.0 [Hz], sampling_time = 0.033
  //
  //   0.0093487 +0.0186974z +0.0093487z²
  //   ----------------------------------
  //       0.7458606 -1.7084658z +z²
  {
    const double sampling_time = 0.033;
    const double f_cutoff_hz = 1.0;
    Butterworth2dFilter butt_filter(sampling_time, f_cutoff_hz);
    std::vector<double> coeff;
    butt_filter.getCoefficients(coeff);
    constexpr double ep = 1.0e-6;
    EXPECT_NEAR(coeff.at(0), 1.7084658, ep);   // a1
    EXPECT_NEAR(coeff.at(1), -0.7458606, ep);  // a2
    EXPECT_NEAR(coeff.at(2), 0.0093487, ep);   // b0
    EXPECT_NEAR(coeff.at(3), 0.0186974, ep);   // b1
    EXPECT_NEAR(coeff.at(4), 0.0093487, ep);   // b2
  }

  // Case 1:
  // cutoff_frequency = 2.0 [Hz], sampling_time = 0.05
  //
  //    0.0674553 +0.1349105z +0.0674553z²
  //    ----------------------------------
  //        0.4128016 -1.1429805z +z²
  {
    const double sampling_time = 0.05;
    const double f_cutoff_hz = 2.0;
    Butterworth2dFilter butt_filter(sampling_time, f_cutoff_hz);
    std::vector<double> coeff;
    butt_filter.getCoefficients(coeff);
    constexpr double ep = 1.0e-6;
    EXPECT_NEAR(coeff.at(0), 1.1429805, ep);   // a1
    EXPECT_NEAR(coeff.at(1), -0.4128016, ep);  // a2
    EXPECT_NEAR(coeff.at(2), 0.0674553, ep);   // b0
    EXPECT_NEAR(coeff.at(3), 0.1349105, ep);   // b1
    EXPECT_NEAR(coeff.at(4), 0.0674553, ep);   // b2
  }
}
