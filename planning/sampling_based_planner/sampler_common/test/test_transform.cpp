// Copyright 2023 Tier IV, Inc.
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

#include <sampler_common/transform/spline_transform.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <random>
#include <ratio>

constexpr auto TOL = 1E-6;  // 1µm tolerance

TEST(splineTransform, makeSpline2D)
{
  sampler_common::transform::Spline2D spline1({0.0, 1.0, 2.0}, {0.0, 1.0, 2.0});
  sampler_common::transform::Spline2D spline2({-10.0, 5.0, -2.0}, {99.0, 3.0, -20.0});
}

TEST(splineTransform, toFrenet)
{
  sampler_common::transform::Spline2D spline({0.0, 1.0, 2.0}, {0.0, 0.0, 0.0});
  auto frenet = spline.frenet({0.0, 0.0});
  EXPECT_NEAR(frenet.s, 0.0, TOL);
  EXPECT_NEAR(frenet.d, 0.0, TOL);
  frenet = spline.frenet({1.0, 0.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, 0.0, TOL);
  frenet = spline.frenet({2.0, 0.0});
  EXPECT_NEAR(frenet.s, 2.0, TOL);
  EXPECT_NEAR(frenet.d, 0.0, TOL);
  frenet = spline.frenet({1.0, 1.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, 1.0, TOL);
  frenet = spline.frenet({1.5, -2.0});
  EXPECT_NEAR(frenet.s, 1.5, TOL);
  EXPECT_NEAR(frenet.d, -2.0, TOL);
  GTEST_SKIP() << "Skipping edge cases";
  // EDGE CASE before spline
  frenet = spline.frenet({-1.0, 1.0});
  EXPECT_NEAR(frenet.s, -1.0, TOL);
  EXPECT_NEAR(frenet.d, 1.0, TOL);
  // EDGE CASE after spline
  frenet = spline.frenet({3.0, -1.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, -1.0, TOL);
  // EDGE CASE 90 deg angle
  sampler_common::transform::Spline2D spline2({0.0, 1.0, 2.0}, {0.0, 1.0, 0.0});
  frenet = spline2.frenet({1.0, 2.0});
  EXPECT_NEAR(frenet.s, 1.0, TOL);
  EXPECT_NEAR(frenet.d, 1.0, TOL);
}

TEST(splineTransform, toCartesian)
{
  sampler_common::transform::Spline2D spline({0.0, 1.0, 2.0}, {0.0, 0.0, 0.0});
  auto cart = spline.cartesian({1.0, 0.0});
  EXPECT_NEAR(cart.x(), 1.0, TOL);
  EXPECT_NEAR(cart.y(), 0.0, TOL);
}

TEST(splineTransform, benchFrenet)
{
  GTEST_SKIP() << "Skipping benchmark test";
  std::random_device rd;
  std::mt19937 gen(rd());
  constexpr auto precision = 1e-2;
  for (auto size = 100; size <= 1000; size += 100) {
    std::chrono::nanoseconds naive{0};
    std::chrono::nanoseconds lut{0};
    std::vector<double> xs(size);
    std::vector<double> ys(size);
    auto x = 0.0;
    auto y = 0.0;
    std::generate(xs.begin(), xs.end(), [&]() { return ++x; });
    std::generate(ys.begin(), ys.end(), [&]() { return ++y; });
    sampler_common::transform::Spline2D spline(xs, ys);
    auto points_distribution = std::uniform_real_distribution(0.0, static_cast<double>(size));
    constexpr auto nb_iter = 1e3;
    for (auto iter = 0; iter < nb_iter; ++iter) {
      double x = points_distribution(gen);
      double y = points_distribution(gen);
      auto naive_start = std::chrono::steady_clock::now();
      auto frenet_naive = spline.frenet_naive({x, y}, precision);
      auto naive_end = std::chrono::steady_clock::now();
      naive += naive_end - naive_start;
      auto lut_start = std::chrono::steady_clock::now();
      auto frenet_lut = spline.frenet({x, y}, precision);
      auto lut_end = std::chrono::steady_clock::now();
      lut += lut_end - lut_start;
      EXPECT_NEAR(frenet_naive.s, frenet_lut.s, precision);
      EXPECT_NEAR(frenet_naive.d, frenet_lut.d, precision);
    }
    std::cout << "size = " << size << std::endl;
    std::cout << "\tnaive: " << std::chrono::duration_cast<std::chrono::milliseconds>(naive).count()
              << "ms\n";
    std::cout << "\tlut  : " << std::chrono::duration_cast<std::chrono::milliseconds>(lut).count()
              << "ms\n";
  }
}
