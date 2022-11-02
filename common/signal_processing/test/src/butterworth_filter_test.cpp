// Copyright 2022 Tier IV, Inc.
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

#include "butterworth_filter_test.hpp"

TEST_F(ButterWorthTestFixture, butterworthOrderTest)
{
  double tol = 1e-4;

  // 1st Method
  const double & Wp{2.};   // pass-band frequency [rad/sec]
  const double & Ws{3.};   // stop-band frequency [rad/sec]
  const double & Ap{6.};   // pass-band ripple mag or loss [dB]
  const double & As{20.};  // stop band ripple attenuation [dB]

  ButterworthFilter bf;
  bf.Buttord(Wp, Ws, Ap, As);

  const auto & NWc = bf.getOrderCutOff();
  // print("The computed order and frequency for the give specification : ");
  // print("Minimum order N = ", NWc.N, ", and The cut-off frequency Wc = ", NWc.Wc, "rad/sec \n");
  bf.printFilterSpecs();

  /**
   * Approximate the continuous and discrete time transfer functions.
   * */
  bf.computeContinuousTimeTF();

  // Print continuous time roots.
  bf.printFilterContinuousTimeRoots();
  bf.printContinuousTimeTF();

  // Compute the discrete time transfer function.
  bf.computeDiscreteTimeTF();
  bf.printDiscreteTimeTF();

  ASSERT_EQ(5, NWc.N);
  ASSERT_NEAR(1.89478, NWc.Wc_rad_sec, tol);

  // test transfer functions
  bf.computeContinuousTimeTF();
  bf.computeDiscreteTimeTF();

  const std::vector<double> & An = bf.getAn();
  const std::vector<double> & Bn = bf.getBn();

  /**
   * Bd = [0.1913    0.9564    1.9128    1.9128    0.9564    0.1913]
   * Ad = [1.0000    1.8849    1.8881    1.0137    0.2976    0.0365]
   */

  ASSERT_NEAR(1.8849, An[1], tol);
  ASSERT_NEAR(1.8881, An[2], tol);
  ASSERT_NEAR(1.0137, An[3], tol);
  ASSERT_NEAR(0.29762, An[4], tol);
  ASSERT_NEAR(0.0365, An[5], tol);

  ASSERT_NEAR(0.9564, Bn[1], tol);
  ASSERT_NEAR(1.9128, Bn[2], tol);
  ASSERT_NEAR(1.9128, Bn[3], tol);
  ASSERT_NEAR(0.9564, Bn[4], tol);
  ASSERT_NEAR(0.1913, Bn[5], tol);
}

TEST_F(ButterWorthTestFixture, butterDefinedSamplingOrder1)
{
  ButterworthFilter bf;
  const double tol{1e-12};

  // Test with a defined sampling frequency
  const int order{1};
  const double cut_off_frq_hz{5.};
  const double sampling_frq_hz{40.};
  const bool use_sampling_frequency = true;

  // Prepare the filter
  bf.setOrder(order);
  bf.setCutOffFrequency(cut_off_frq_hz, sampling_frq_hz);
  bf.computeContinuousTimeTF(use_sampling_frequency);
  bf.computeDiscreteTimeTF(use_sampling_frequency);

  const auto & An = bf.getAn();
  const auto & Bn = bf.getBn();

  std::vector<double> const & An_ground_truth{1., -0.414213562373095};
  std::vector<double> const & Bn_ground_truth{0.292893218813452, 0.292893218813452};

  for (size_t k = 0; k < An.size(); ++k) {
    ASSERT_NEAR(An[k], An_ground_truth[k], tol);
    ASSERT_NEAR(Bn[k], Bn_ground_truth[k], tol);
  }
}

TEST_F(ButterWorthTestFixture, butterDefinedSamplingOrder2)
{
  ButterworthFilter bf;
  const double tol{1e-12};

  // Test with defined sampling frequency
  const int order{2};
  const double cut_off_frq_hz{10.};
  const double sampling_frq_hz{100};
  const bool use_sampling_frequency = true;

  // Prepare the filter
  bf.setOrder(order);
  bf.setCutOffFrequency(cut_off_frq_hz, sampling_frq_hz);
  bf.computeContinuousTimeTF(use_sampling_frequency);
  bf.computeDiscreteTimeTF(use_sampling_frequency);

  const auto & An = bf.getAn();
  const auto & Bn = bf.getBn();

  const std::vector<double> & An_ground_truth{1., -1.142980502539901, 0.412801598096189};
  const std::vector<double> & Bn_ground_truth{
    0.067455273889072, 0.134910547778144, 0.067455273889072};

  for (size_t k = 0; k < An.size(); ++k) {
    ASSERT_NEAR(An[k], An_ground_truth[k], tol);
    ASSERT_NEAR(Bn[k], Bn_ground_truth[k], tol);
  }
}

TEST_F(ButterWorthTestFixture, butterDefinedSamplingOrder3)
{
  ButterworthFilter bf;
  const double tol{1e-12};

  // Test with a defined sampling frequency
  const int order{3};
  const double cut_off_frq_hz{10.};
  const double sampling_frq_hz{100};
  const bool use_sampling_frequency = true;

  // Prepare the filter
  bf.setOrder(order);
  bf.setCutOffFrequency(cut_off_frq_hz, sampling_frq_hz);
  bf.computeContinuousTimeTF(use_sampling_frequency);
  bf.computeDiscreteTimeTF(use_sampling_frequency);

  const auto & An = bf.getAn();
  const auto & Bn = bf.getBn();

  const std::vector<double> & An_ground_truth{
    1., -1.760041880343169, 1.182893262037831, -0.278059917634546};

  const std::vector<double> & Bn_ground_truth{
    0.018098933007514, 0.054296799022543, 0.054296799022543, 0.018098933007514};

  for (size_t k = 0; k < An.size(); ++k) {
    ASSERT_NEAR(An[k], An_ground_truth[k], tol);
    ASSERT_NEAR(Bn[k], Bn_ground_truth[k], tol);
  }
}
