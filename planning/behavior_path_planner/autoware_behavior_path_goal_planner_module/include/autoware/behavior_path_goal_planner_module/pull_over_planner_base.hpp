// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__PULL_OVER_PLANNER_BASE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__PULL_OVER_PLANNER_BASE_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_planner_parameters.hpp"
#include "autoware/behavior_path_planner_common/data_manager.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>

#include <memory>
#include <utility>
#include <vector>

using autoware::universe_utils::LinearRing2d;
using geometry_msgs::msg::Pose;
using tier4_planning_msgs::msg::PathWithLaneId;

namespace autoware::behavior_path_planner
{
enum class PullOverPlannerType {
  NONE = 0,
  SHIFT,
  ARC_FORWARD,
  ARC_BACKWARD,
  FREESPACE,
};

struct PullOverPath
{
  PullOverPlannerType type{PullOverPlannerType::NONE};
  std::vector<PathWithLaneId> partial_paths{};
  size_t path_idx{0};
  // accelerate with constant acceleration to the target velocity
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  Pose start_pose{};
  Pose end_pose{};
  std::vector<Pose> debug_poses{};
  size_t goal_id{};
  size_t id{};
  bool decided_velocity{false};

  /**
   * @brief Set paths and start/end poses
   * By setting partial_paths, full_path, parking_path and curvature are also set at the same time
   * @param input_partial_paths partial paths
   * @param input_start_pose start pose
   * @param input_end_pose end pose
   */
  void setPaths(
    const std::vector<PathWithLaneId> input_partial_paths, const Pose & input_start_pose,
    const Pose & input_end_pose)
  {
    partial_paths = input_partial_paths;
    start_pose = input_start_pose;
    end_pose = input_end_pose;

    updatePathData();
  }

  // Note: return copy value (not const&) because the value is used in multi threads
  PathWithLaneId getFullPath() const { return full_path; }

  PathWithLaneId getParkingPath() const { return parking_path; }

  PathWithLaneId getCurrentPath() const
  {
    if (partial_paths.empty()) {
      return PathWithLaneId{};
    } else if (partial_paths.size() <= path_idx) {
      return partial_paths.back();
    }
    return partial_paths.at(path_idx);
  }

  bool incrementPathIndex()
  {
    if (partial_paths.size() - 1 <= path_idx) {
      return false;
    }
    path_idx += 1;
    return true;
  }

  bool isValidPath() const { return type != PullOverPlannerType::NONE; }

  std::vector<double> getFullPathCurvatures() const { return full_path_curvatures; }
  std::vector<double> getParkingPathCurvatures() const { return parking_path_curvatures; }
  double getFullPathMaxCurvature() const { return full_path_max_curvature; }
  double getParkingPathMaxCurvature() const { return parking_path_max_curvature; }

private:
  void updatePathData()
  {
    updateFullPath();
    updateParkingPath();
    updateCurvatures();
  }

  void updateFullPath()
  {
    PathWithLaneId path{};
    for (size_t i = 0; i < partial_paths.size(); ++i) {
      if (i == 0) {
        path.points.insert(
          path.points.end(), partial_paths.at(i).points.begin(), partial_paths.at(i).points.end());
      } else {
        // skip overlapping point
        path.points.insert(
          path.points.end(), next(partial_paths.at(i).points.begin()),
          partial_paths.at(i).points.end());
      }
    }
    full_path.points = autoware::motion_utils::removeOverlapPoints(path.points);
  }

  void updateParkingPath()
  {
    if (full_path.points.empty()) {
      updateFullPath();
    }
    const size_t start_idx =
      autoware::motion_utils::findNearestIndex(full_path.points, start_pose.position);

    PathWithLaneId path{};
    std::copy(
      full_path.points.begin() + start_idx, full_path.points.end(),
      std::back_inserter(path.points));
    parking_path = path;
  }

  void updateCurvatures()
  {
    const auto calculateCurvaturesAndMax =
      [](const auto & path) -> std::pair<std::vector<double>, double> {
      std::vector<double> curvatures = autoware::motion_utils::calcCurvature(path.points);
      double max_curvature = 0.0;
      if (!curvatures.empty()) {
        max_curvature = std::abs(*std::max_element(
          curvatures.begin(), curvatures.end(),
          [](const double & a, const double & b) { return std::abs(a) < std::abs(b); }));
      }
      return std::make_pair(curvatures, max_curvature);
    };
    std::tie(full_path_curvatures, full_path_max_curvature) =
      calculateCurvaturesAndMax(getFullPath());
    std::tie(parking_path_curvatures, parking_path_max_curvature) =
      calculateCurvaturesAndMax(getParkingPath());
  }

  // curvatures
  std::vector<double> full_path_curvatures{};
  std::vector<double> parking_path_curvatures{};
  std::vector<double> current_path_curvatures{};
  double parking_path_max_curvature{0.0};
  double full_path_max_curvature{0.0};
  double current_path_max_curvature{0.0};

  // path
  PathWithLaneId full_path{};
  PathWithLaneId parking_path{};
};

class PullOverPlannerBase
{
public:
  PullOverPlannerBase(rclcpp::Node & node, const GoalPlannerParameters & parameters)
  {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo();
    vehicle_footprint_ = vehicle_info_.createFootprint();
    parameters_ = parameters;
  }
  virtual ~PullOverPlannerBase() = default;

  void setPreviousModuleOutput(const BehaviorModuleOutput & previous_module_output)
  {
    previous_module_output_ = previous_module_output;
  }

  void setPlannerData(const std::shared_ptr<const PlannerData> planner_data)
  {
    planner_data_ = planner_data;
  }

  virtual PullOverPlannerType getPlannerType() const = 0;
  virtual std::optional<PullOverPath> plan(const Pose & goal_pose) = 0;

protected:
  std::shared_ptr<const PlannerData> planner_data_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  LinearRing2d vehicle_footprint_;
  GoalPlannerParameters parameters_;

  BehaviorModuleOutput previous_module_output_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__PULL_OVER_PLANNER_BASE_HPP_
