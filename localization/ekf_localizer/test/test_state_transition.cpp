// Copyright 2018-2019 Autoware Foundation
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

#define _USE_MATH_DEFINES
#include "ekf_localizer/state_index.hpp"
#include "ekf_localizer/state_transition.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(StateTransition, normalize_yaw)
{
  const double tolerance = 1e-6;
  EXPECT_NEAR(normalize_yaw(M_PI * 4 / 3), -M_PI * 2 / 3, tolerance);
  EXPECT_NEAR(normalize_yaw(-M_PI * 4 / 3), M_PI * 2 / 3, tolerance);
  EXPECT_NEAR(normalize_yaw(M_PI * 9 / 2), M_PI * 1 / 2, tolerance);
  EXPECT_NEAR(normalize_yaw(M_PI * 4), M_PI * 0, tolerance);
}

TEST(predict_next_state, predict_next_state)
{
  // This function is the definition of state transition so we just check
  // if the calculation is done according to the formula
  Vector6d x_curr;
  x_curr(0) = 2.;
  x_curr(1) = 3.;
  x_curr(2) = M_PI / 2.;
  x_curr(3) = M_PI / 4.;
  x_curr(4) = 10.;
  x_curr(5) = 2. * M_PI / 3.;

  const double dt = 0.5;

  const Vector6d x_next = predict_next_state(x_curr, dt);

  const double tolerance = 1e-10;
  EXPECT_NEAR(x_next(0), 2. + 10. * std::cos(M_PI / 2. + M_PI / 4.) * 0.5, tolerance);
  EXPECT_NEAR(x_next(1), 3. + 10. * std::sin(M_PI / 2. + M_PI / 4.) * 0.5, tolerance);
  EXPECT_NEAR(x_next(2), normalize_yaw(M_PI / 2. + M_PI / 3.), tolerance);
  EXPECT_NEAR(x_next(3), x_curr(3), tolerance);
  EXPECT_NEAR(x_next(4), x_curr(4), tolerance);
  EXPECT_NEAR(x_next(5), x_curr(5), tolerance);
}

TEST(create_state_transition_matrix, NumericalApproximation)
{
  // The transition matrix A = df / dx
  // We check if df = A * dx approximates f(x + dx) - f(x)

  {
    // check around x = 0
    const double dt = 0.1;
    const Vector6d dx = 0.1 * Vector6d::Ones();
    const Vector6d x = Vector6d::Zero();

    const Matrix6d a = create_state_transition_matrix(x, dt);
    const Vector6d df = predict_next_state(x + dx, dt) - predict_next_state(x, dt);

    EXPECT_LT((df - a * dx).norm(), 2e-3);
  }

  {
    // check around random x
    const double dt = 0.1;
    const Vector6d dx = 0.1 * Vector6d::Ones();
    const Vector6d x = (Vector6d() << 0.1, 0.2, 0.1, 0.4, 0.1, 0.3).finished();

    const Matrix6d a = create_state_transition_matrix(x, dt);
    const Vector6d df = predict_next_state(x + dx, dt) - predict_next_state(x, dt);

    EXPECT_LT((df - a * dx).norm(), 5e-3);
  }
}

TEST(process_noise_covariance, process_noise_covariance)
{
  const Matrix6d q = process_noise_covariance(1., 2., 3.);
  EXPECT_EQ(q(2, 2), 1.);  // for yaw
  EXPECT_EQ(q(4, 4), 2.);  // for vx
  EXPECT_EQ(q(5, 5), 3.);  // for wz

  // Make sure other elements are zero
  EXPECT_EQ(process_noise_covariance(0, 0, 0).norm(), 0.);
}
