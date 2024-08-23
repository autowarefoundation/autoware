// Copyright 2024 Tier IV, Inc.
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

#include "autoware/motion_utils/trajectory_container/interpolator/linear.hpp"

#include <autoware/motion_utils/trajectory_container/interpolator.hpp>

#include <matplotlibcpp17/pyplot.h>

#include <random>
#include <vector>

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  // create random values
  std::vector<double> axis = {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
  std::vector<double> values;
  std::random_device seed_gen;
  std::mt19937 engine(seed_gen());
  std::uniform_real_distribution<> dist(-1.0, 1.0);
  for (size_t i = 0; i < axis.size(); ++i) {
    values.push_back(dist(engine));
  }
  // Scatter Data
  plt.scatter(Args(axis, values));

  using autoware::motion_utils::trajectory_container::interpolator::Interpolator;
  using autoware::motion_utils::trajectory_container::interpolator::InterpolatorCreator;
  // Linear Interpolator
  {
    using autoware::motion_utils::trajectory_container::interpolator::Linear;
    auto interpolator = *InterpolatorCreator<Linear>().set_axis(axis).set_values(values).create();
    std::vector<double> x;
    std::vector<double> y;
    for (double i = axis.front(); i < axis.back(); i += 0.01) {
      x.push_back(i);
      y.push_back(interpolator.compute(i));
    }
    plt.plot(Args(x, y), Kwargs("label"_a = "Linear"));
  }

  // AkimaSpline Interpolator
  {
    using autoware::motion_utils::trajectory_container::interpolator::AkimaSpline;
    auto interpolator =
      *InterpolatorCreator<AkimaSpline>().set_axis(axis).set_values(values).create();
    std::vector<double> x;
    std::vector<double> y;
    for (double i = axis.front(); i < axis.back(); i += 0.01) {
      x.push_back(i);
      y.push_back(interpolator.compute(i));
    }
    plt.plot(Args(x, y), Kwargs("label"_a = "AkimaSpline"));
  }

  // CubicSpline Interpolator
  {
    using autoware::motion_utils::trajectory_container::interpolator::CubicSpline;
    auto interpolator =
      *InterpolatorCreator<CubicSpline>().set_axis(axis).set_values(values).create();
    std::vector<double> x;
    std::vector<double> y;
    for (double i = axis.front(); i < axis.back(); i += 0.01) {
      x.push_back(i);
      y.push_back(interpolator.compute(i));
    }
    plt.plot(Args(x, y), Kwargs("label"_a = "CubicSpline"));
  }

  // NearestNeighbor Interpolator
  {
    using autoware::motion_utils::trajectory_container::interpolator::NearestNeighbor;
    auto interpolator =
      *InterpolatorCreator<NearestNeighbor<double>>().set_axis(axis).set_values(values).create();
    std::vector<double> x;
    std::vector<double> y;
    for (double i = axis.front(); i < axis.back(); i += 0.01) {
      x.push_back(i);
      y.push_back(interpolator.compute(i));
    }
    plt.plot(Args(x, y), Kwargs("label"_a = "NearestNeighbor"));
  }

  // Stairstep Interpolator
  {
    using autoware::motion_utils::trajectory_container::interpolator::Stairstep;
    auto interpolator =
      *InterpolatorCreator<Stairstep<double>>().set_axis(axis).set_values(values).create();
    std::vector<double> x;
    std::vector<double> y;
    for (double i = axis.front(); i < axis.back(); i += 0.01) {
      x.push_back(i);
      y.push_back(interpolator.compute(i));
    }
    plt.plot(Args(x, y), Kwargs("label"_a = "Stairstep"));
  }

  plt.legend();
  plt.show();
  return 0;
}
