// Copyright 2021 Tier IV, Inc.
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

#include "kinematic_evaluator/metrics_calculator.hpp"

#include "kinematic_evaluator/metrics/kinematic_metrics.hpp"

namespace kinematic_diagnostics
{
Stat<double> MetricsCalculator::updateStat(
  const Metric metric, const Odometry & odom, const Stat<double> stat_prev) const
{
  switch (metric) {
    case Metric::velocity_stats:
      return metrics::updateVelocityStats(odom.twist.twist.linear.x, stat_prev);
    default:
      throw std::runtime_error(
        "[MetricsCalculator][calculate()] unknown Metric " +
        std::to_string(static_cast<int>(metric)));
  }
}

}  // namespace kinematic_diagnostics
