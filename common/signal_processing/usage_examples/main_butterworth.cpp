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

#include "signal_processing/butterworth.hpp"

#include <iostream>
#include <vector>

/** The filter tool can be used in the following ways.
 *
 * 1- Defining specs Wp, Ws, Ap, As and obtaining order and cut-off frequency (rad/sec)
 *      ButterworthFilter bf;
 *      bf.Buttord(Wp, Ws, Ap, As); Wp and Ws in [rad/sec]
 *      bf.computeContinuousTimeTF();
 *      bf.computeDiscreteTimeTF();
 *
 *   2- Defining the order and cut-off frequency (rad/sec) directly and computing the filter TFs
 *      bf.setOrder(N); N is integer
 *      bf.setCutOffFrequency(Wc); Wc in [rad/sec]
 *      bf.computeContinuousTimeTF();
 *      bf.computeDiscreteTimeTF();
 *
 *   3- Defining the order N, cut-off and sampling frequencies (Hz)
 *      bf.setOrder(N); N is integer
 *      bf.setCutOffFrequency_Hz(fc, fs); cut-off fc and sampling fs are in [Hz]
 *      bf.computeContinuousTimeTF();
 *      bf.computeDiscreteTimeTF();
 * */

int main()
{
  // 1st Method
  double Wp{2.};   // pass-band frequency [rad/sec]
  double Ws{3.};   // stop-band frequency [rad/sec]
  double Ap{6.};   // pass-band ripple mag or loss [dB]
  double As{20.};  // stop band ripple attenuation [dB]

  ButterworthFilter bf1;
  bf1.Buttord(Wp, Ws, Ap, As);

  auto NWc = bf1.getOrderCutOff();
  print("The computed order and frequency for the give specification : ");
  print(
    "Minimum order N = ", NWc.N, ", and The cut-off frequency Wc = ", NWc.Wc_rad_sec, "rad/sec \n");
  bf1.printFilterSpecs();

  /**
   * Approximate the continuous and discrete time transfer functions.
   * */
  bf1.computeContinuousTimeTF();

  // Print continuous time roots.
  bf1.printFilterContinuousTimeRoots();
  bf1.printContinuousTimeTF();

  // Compute the discrete time transfer function.
  bf1.computeDiscreteTimeTF();
  bf1.printDiscreteTimeTF();

  // 2nd METHOD
  // Setting filter order N and cut-off frequency explicitly
  print("SECOND TYPE of FILTER INITIALIZATION ");

  ButterworthFilter bf2;
  bf2.setOrder(2);
  bf2.setCutOffFrequency(2.0);
  bf2.printFilterSpecs();

  // Get the computed order and Cut-off frequency
  NWc = bf2.getOrderCutOff();

  // Print continuous time roots.
  bf2.computeContinuousTimeTF();
  bf2.printFilterContinuousTimeRoots();
  bf2.printContinuousTimeTF();

  // Compute the discrete time transfer function.
  bf2.computeDiscreteTimeTF();
  bf2.printDiscreteTimeTF();

  // 3rd METHOD
  // defining a sampling frequency together with the cut-off fc, fs
  print("THIRD TYPE of FILTER INITIALIZATION ");

  ButterworthFilter bf3;
  bf3.setOrder(3);

  bf3.setCutOffFrequency(10, 100);
  bf3.printFilterSpecs();

  bool use_sampling_frequency{true};

  bf3.computeContinuousTimeTF(use_sampling_frequency);
  bf3.printFilterContinuousTimeRoots();
  bf3.printContinuousTimeTF();

  // Compute Discrete Time TF
  bf3.computeDiscreteTimeTF(use_sampling_frequency);
  bf3.printDiscreteTimeTF();

  auto AnBn = bf3.getAnBn();
  auto An = bf3.getAn();
  auto Bn = bf3.getBn();

  print("An : ");

  for (double it : An) {
    std::cout << std::setprecision(4) << it << ", ";
  }

  print("\nBn : \n");

  for (double it : Bn) {
    std::cout << std::setprecision(4) << it << ", ";
  }

  return 0;
}
