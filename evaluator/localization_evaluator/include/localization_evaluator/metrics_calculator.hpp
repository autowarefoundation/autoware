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

#ifndef LOCALIZATION_EVALUATOR__METRICS_CALCULATOR_HPP_
#define LOCALIZATION_EVALUATOR__METRICS_CALCULATOR_HPP_

#include "localization_evaluator/metrics/metric.hpp"
#include "localization_evaluator/parameters.hpp"
#include "localization_evaluator/stat.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>

namespace localization_diagnostics
{
class MetricsCalculator
{
public:
  Parameters parameters;

  MetricsCalculator() = default;
  /**
   * @brief update Metrics
   * @param [in] stat_prev Previous statistics
   * @param [in] metric metric enum value
   * @param [in] pos current position
   * @param [in] pos_ref reference position
   * @return string describing the requested metric
   */
  Stat<double> updateStat(
    const Stat<double> stat_prev, const Metric metric, const geometry_msgs::msg::Point & pos,
    const geometry_msgs::msg::Point & pos_ref) const;
};  // class MetricsCalculator

}  // namespace localization_diagnostics

#endif  // LOCALIZATION_EVALUATOR__METRICS_CALCULATOR_HPP_
