// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_sampling_planner_module/manager.hpp"

#include "autoware/universe_utils/ros/update_param.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

void SamplingPlannerModuleManager::init(rclcpp::Node * node)
{
  // init manager interface
  initInterface(node, {""});

  SamplingPlannerParameters p{};
  {
    std::string ns{"constraints.hard"};
    p.max_curvature = node->declare_parameter<double>(ns + ".max_curvature");
    p.min_curvature = node->declare_parameter<double>(ns + ".min_curvature");
    ns = std::string{"constraints.soft"};
    p.lateral_deviation_weight =
      node->declare_parameter<double>(ns + ".lateral_deviation_weight");       // [[unused]] Delete?
    p.length_weight = node->declare_parameter<double>(ns + ".length_weight");  // [[unused]] Delete?
    p.curvature_weight =
      node->declare_parameter<double>(ns + ".curvature_weight");  // [[unused]] Delete?
    p.weights = node->declare_parameter<std::vector<double>>(ns + ".weights");
  }
  {
    std::string ns{"sampling"};
    p.enable_frenet = node->declare_parameter<bool>(ns + ".enable_frenet");
    p.enable_bezier = node->declare_parameter<bool>(
      ns + ".enable_bezier");  // [[unused]] will be used in the future
    p.resolution = node->declare_parameter<double>(ns + ".resolution");
    p.previous_path_reuse_points_nb =
      node->declare_parameter<int>(ns + ".previous_path_reuse_points_nb");
    p.nb_target_lateral_positions =
      node->declare_parameter<int>(ns + ".nb_target_lateral_positions");
    p.target_lengths = node->declare_parameter<std::vector<double>>(ns + ".target_lengths");
    p.target_lateral_positions =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_positions");
    ns += ".frenet";
    p.target_lateral_velocities =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_velocities");
    p.target_lateral_accelerations =
      node->declare_parameter<std::vector<double>>(ns + ".target_lateral_accelerations");
  }
  {
    std::string ns{"preprocessing"};
    p.force_zero_deviation = node->declare_parameter<bool>(
      ns + ".force_zero_initial_deviation");  // [[unused]] will be used in the future
    p.force_zero_heading = node->declare_parameter<bool>(
      ns + ".force_zero_initial_heading");  // [[unused]] will be used in the future
    p.smooth_reference = node->declare_parameter<bool>(
      ns + ".smooth_reference_trajectory");  // [[unused]] will be used in the future
  }
  parameters_ = std::make_shared<SamplingPlannerParameters>(p);
}

void SamplingPlannerModuleManager::updateModuleParams(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware::universe_utils::updateParam;

  auto & p = parameters_;

  {
    std::string ns{"constraints.hard"};
    updateParam<double>(parameters, ns + ".max_curvature", p->max_curvature);
    updateParam<double>(parameters, ns + ".min_curvature", p->min_curvature);
    ns = std::string{"constraints.soft"};
    updateParam<double>(parameters, ns + ".lateral_deviation_weight", p->lateral_deviation_weight);
    updateParam<double>(parameters, ns + ".length_weight", p->length_weight);
    updateParam<double>(parameters, ns + ".curvature_weight", p->curvature_weight);
    updateParam<std::vector<double>>(parameters, ns + ".weights", p->weights);
  }
  {
    std::string ns{"sampling"};
    updateParam<bool>(parameters, ns + ".enable_frenet", p->enable_frenet);
    updateParam<bool>(parameters, ns + ".enable_bezier", p->enable_bezier);
    updateParam<double>(parameters, ns + ".resolution", p->resolution);

    updateParam<int>(
      parameters, ns + ".previous_path_reuse_points_nb", p->previous_path_reuse_points_nb);

    updateParam<int>(
      parameters, ns + ".nb_target_lateral_positions", p->nb_target_lateral_positions);
    updateParam<std::vector<double>>(parameters, ns + ".target_lengths", p->target_lengths);

    updateParam<std::vector<double>>(
      parameters, ns + ".target_lateral_positions", p->target_lateral_positions);

    ns += ".frenet";
    updateParam<std::vector<double>>(
      parameters, ns + ".target_lateral_velocities", p->target_lateral_velocities);

    updateParam<std::vector<double>>(
      parameters, ns + ".target_lateral_accelerations", p->target_lateral_accelerations);
  }
  {
    std::string ns{"preprocessing"};
    updateParam<bool>(parameters, ns + ".force_zero_initial_deviation", p->force_zero_deviation);
    updateParam<bool>(parameters, ns + ".force_zero_initial_heading", p->force_zero_heading);
    updateParam<bool>(parameters, ns + ".smooth_reference_trajectory", p->smooth_reference);
  }

  std::for_each(observers_.begin(), observers_.end(), [&](const auto & observer) {
    if (!observer.expired()) {
      const auto sampling_planner_ptr =
        std::dynamic_pointer_cast<SamplingPlannerModule>(observer.lock());
      if (sampling_planner_ptr) {
        sampling_planner_ptr->updateModuleParams(p);
      }
    }
  });
}

}  // namespace autoware::behavior_path_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_path_planner::SamplingPlannerModuleManager,
  autoware::behavior_path_planner::SceneModuleManagerInterface)
