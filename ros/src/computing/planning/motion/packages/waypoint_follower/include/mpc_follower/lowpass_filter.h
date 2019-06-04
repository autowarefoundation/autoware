/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file lowpass_filter.h
 * @brief vehicle model interface class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <stdio.h>

/** 
 * @class 2nd-order Butterworth Filter
 * @brief filtering values
 */

/*
 * reference : S. Butterworth, "On the Theory of Filter Amplifier", Experimental wireless, 1930.
 */

class Butterworth2dFilter
{
private:
  double y1_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double y2_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double u1_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double u2_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double a0_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double a1_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double a2_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double b0_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double b1_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time
  double b2_; //!< @brief filter coefficient calculated with cutoff frequency and sampling time

public:
  /**
   * @brief constructor with initialization
   * @param [in] dt sampling time
   * @param [in] f_cutoff_hz cutoff frequency [Hz]
   */
  Butterworth2dFilter(double dt = 0.01, double f_cutoff_hz = 5.0);

  /**
   * @brief destructor
   */
  ~Butterworth2dFilter();

  /**
   * @brief constructor
   * @param [in] dt sampling time
   * @param [in] f_cutoff_hz cutoff frequency [Hz]
   */
  void initialize(const double &dt, const double &f_cutoff_hz);

  /**
   * @brief filtering (call this function at each sampling time with input)
   * @param [in] u scalar input for filter
   * @return filtered scalar value
   */
  double filter(const double &u);

  /**
   * @brief filtering for time-series data
   * @param [in] t time-series data for input vector
   * @param [out] u object vector
   */
  void filt_vector(const std::vector<double> &t, std::vector<double> &u);

  /**
   * @brief filtering for time-series data from both forward-backward direction for zero phase delay
   * @param [in] t time-series data for input vector
   * @param [out] u object vector
   */
  void filtfilt_vector(const std::vector<double> &t, std::vector<double> &u); // filtering forward and backward direction

  /**
   * @brief get filter coefficients
   * @param [out] coeffs coefficients of filter [a0, a1, a2, b0, b1, b2].
   */
  void getCoefficients(std::vector<double> &coeffs);
};

/** 
 * @class Move Average Filter
 * @brief filtering values
 */
class MoveAverageFilter
{
public:
  /**
   * @brief constructor
   */
  MoveAverageFilter();

  /**
   * @brief destructor
   */
  ~MoveAverageFilter();

  /**
   * @brief filtering vector
   * @param [in] num index distance for moving average filter
   * @param [out] u object vector
   */
  static bool filt_vector(const int num, std::vector<double> &u);
};