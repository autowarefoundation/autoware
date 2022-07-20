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

#include "localization_evaluator/metrics_calculator.hpp"

#include "localization_evaluator/metrics/localization_metrics.hpp"

namespace localization_diagnostics
{
Stat<double> MetricsCalculator::updateStat(
  const Stat<double> stat_prev, const Metric metric, const geometry_msgs::msg::Point & pos,
  const geometry_msgs::msg::Point & pos_ref) const
{
  if (
    (pos_ref.x == 0 && pos_ref.y == 0 && pos_ref.z == 0) ||
    (pos.x == 0 && pos.y == 0 && pos.z == 0)) {
    return stat_prev;
  }
  switch (metric) {
    case Metric::lateral_error:
      return metrics::updateLateralStats(stat_prev, pos.x, pos_ref.x);
    case Metric::absolute_error:
      return metrics::updateAbsoluteStats(stat_prev, pos, pos_ref);
    default:
      throw std::runtime_error(
        "[MetricsCalculator][calculate()] unknown Metric " +
        std::to_string(static_cast<int>(metric)));
  }
}

}  // namespace localization_diagnostics
