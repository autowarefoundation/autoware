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

#ifndef PLANNING_EVALUATOR__METRICS__METRIC_HPP_
#define PLANNING_EVALUATOR__METRICS__METRIC_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace planning_diagnostics
{
/**
 * @brief Enumeration of trajectory metrics
 */
enum class Metric {
  curvature,
  point_interval,
  relative_angle,
  length,
  duration,
  velocity,
  acceleration,
  jerk,
  lateral_deviation,
  yaw_deviation,
  velocity_deviation,
  stability,
  stability_frechet,
  obstacle_distance,
  obstacle_ttc,
  modified_goal_longitudinal_deviation,
  modified_goal_lateral_deviation,
  modified_goal_yaw_deviation,
  SIZE,
};

/** TODO(Maxime CLEMENT):
 * make the addition of metrics simpler, e.g. with some macro ADD_METRIC(metric, metric_description)
 */
static const std::unordered_map<std::string, Metric> str_to_metric = {
  {"curvature", Metric::curvature},
  {"point_interval", Metric::point_interval},
  {"relative_angle", Metric::relative_angle},
  {"length", Metric::length},
  {"duration", Metric::duration},
  {"velocity", Metric::velocity},
  {"acceleration", Metric::acceleration},
  {"jerk", Metric::jerk},
  {"lateral_deviation", Metric::lateral_deviation},
  {"yaw_deviation", Metric::yaw_deviation},
  {"velocity_deviation", Metric::velocity_deviation},
  {"stability", Metric::stability},
  {"stability_frechet", Metric::stability_frechet},
  {"obstacle_distance", Metric::obstacle_distance},
  {"obstacle_ttc", Metric::obstacle_ttc},
  {"modified_goal_longitudinal_deviation", Metric::modified_goal_longitudinal_deviation},
  {"modified_goal_lateral_deviation", Metric::modified_goal_lateral_deviation},
  {"modified_goal_yaw_deviation", Metric::modified_goal_yaw_deviation}};

static const std::unordered_map<Metric, std::string> metric_to_str = {
  {Metric::curvature, "curvature"},
  {Metric::point_interval, "point_interval"},
  {Metric::relative_angle, "relative_angle"},
  {Metric::length, "length"},
  {Metric::duration, "duration"},
  {Metric::velocity, "velocity"},
  {Metric::acceleration, "acceleration"},
  {Metric::jerk, "jerk"},
  {Metric::lateral_deviation, "lateral_deviation"},
  {Metric::yaw_deviation, "yaw_deviation"},
  {Metric::velocity_deviation, "velocity_deviation"},
  {Metric::stability, "stability"},
  {Metric::stability_frechet, "stability_frechet"},
  {Metric::obstacle_distance, "obstacle_distance"},
  {Metric::obstacle_ttc, "obstacle_ttc"},
  {Metric::modified_goal_longitudinal_deviation, "modified_goal_longitudinal_deviation"},
  {Metric::modified_goal_lateral_deviation, "modified_goal_lateral_deviation"},
  {Metric::modified_goal_yaw_deviation, "modified_goal_yaw_deviation"}};

// Metrics descriptions
static const std::unordered_map<Metric, std::string> metric_descriptions = {
  {Metric::curvature, "Curvature[1/rad]"},
  {Metric::point_interval, "Interval_between_points[m]"},
  {Metric::relative_angle, "Relative_angle[rad]"},
  {Metric::length, "Trajectory_length[m]"},
  {Metric::duration, "Trajectory_duration[s]"},
  {Metric::velocity, "Trajectory_velocity[m/s]"},
  {Metric::acceleration, "Trajectory_acceleration[m/s²]"},
  {Metric::jerk, "Trajectory_jerk[m/s³]"},
  {Metric::lateral_deviation, "Lateral_deviation[m]"},
  {Metric::yaw_deviation, "Yaw_deviation[rad]"},
  {Metric::velocity_deviation, "Velocity_deviation[m/s]"},
  {Metric::stability, "Stability[m]"},
  {Metric::stability_frechet, "StabilityFrechet[m]"},
  {Metric::obstacle_distance, "Obstacle_distance[m]"},
  {Metric::obstacle_ttc, "Obstacle_time_to_collision[s]"},
  {Metric::modified_goal_longitudinal_deviation, "Modified_goal_longitudinal_deviation[m]"},
  {Metric::modified_goal_lateral_deviation, "Modified_goal_lateral_deviation[m]"},
  {Metric::modified_goal_yaw_deviation, "Modified_goal_yaw_deviation[rad]"}};

namespace details
{
static struct CheckCorrectMaps
{
  CheckCorrectMaps()
  {
    if (
      str_to_metric.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_to_str.size() != static_cast<size_t>(Metric::SIZE) ||
      metric_descriptions.size() != static_cast<size_t>(Metric::SIZE)) {
      std::cerr << "[metrics/metrics.hpp] Maps are not defined for all metrics: ";
      std::cerr << str_to_metric.size() << " " << metric_to_str.size() << " "
                << metric_descriptions.size() << std::endl;
    }
  }
} check;

}  // namespace details
}  // namespace planning_diagnostics

#endif  // PLANNING_EVALUATOR__METRICS__METRIC_HPP_
