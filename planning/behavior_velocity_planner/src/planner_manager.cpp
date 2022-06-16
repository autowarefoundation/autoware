// Copyright 2020 Tier IV, Inc.
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

#include "behavior_velocity_planner/planner_manager.hpp"

#include <boost/format.hpp>

#include <memory>
#include <string>

namespace behavior_velocity_planner
{
namespace
{
std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose)
{
  const std::string json_dumps_pose =
    (boost::format(
       R"({"position":{"x":%lf,"y":%lf,"z":%lf},"orientation":{"w":%lf,"x":%lf,"y":%lf,"z":%lf}})") %
     pose.position.x % pose.position.y % pose.position.z % pose.orientation.w % pose.orientation.x %
     pose.orientation.y % pose.orientation.z)
      .str();
  return json_dumps_pose;
}

diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
  const std::string stop_reason, const geometry_msgs::msg::Pose & stop_pose)
{
  diagnostic_msgs::msg::DiagnosticStatus stop_reason_diag;
  diagnostic_msgs::msg::KeyValue stop_reason_diag_kv;
  stop_reason_diag.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  stop_reason_diag.name = "stop_reason";
  stop_reason_diag.message = stop_reason;
  stop_reason_diag_kv.key = "stop_pose";
  stop_reason_diag_kv.value = jsonDumpsPose(stop_pose);
  stop_reason_diag.values.push_back(stop_reason_diag_kv);
  return stop_reason_diag;
}
}  // namespace

void BehaviorVelocityPlannerManager::launchSceneModule(
  const std::shared_ptr<SceneModuleManagerInterface> & scene_module_manager_ptr)
{
  scene_manager_ptrs_.push_back(scene_module_manager_ptr);
}

autoware_auto_planning_msgs::msg::PathWithLaneId BehaviorVelocityPlannerManager::planPathVelocity(
  const std::shared_ptr<const PlannerData> & planner_data,
  const autoware_auto_planning_msgs::msg::PathWithLaneId & input_path_msg)
{
  autoware_auto_planning_msgs::msg::PathWithLaneId output_path_msg = input_path_msg;

  int first_stop_path_point_index = static_cast<int>(output_path_msg.points.size() - 1);
  std::string stop_reason_msg("path_end");

  for (const auto & scene_manager_ptr : scene_manager_ptrs_) {
    scene_manager_ptr->updateSceneModuleInstances(planner_data, input_path_msg);
    scene_manager_ptr->plan(&output_path_msg);
    boost::optional<int> firstStopPathPointIndex = scene_manager_ptr->getFirstStopPathPointIndex();

    if (firstStopPathPointIndex) {
      if (firstStopPathPointIndex.get() < first_stop_path_point_index) {
        first_stop_path_point_index = firstStopPathPointIndex.get();
        stop_reason_msg = scene_manager_ptr->getModuleName();
      }
    }
  }

  stop_reason_diag_ = makeStopReasonDiag(
    stop_reason_msg, output_path_msg.points[first_stop_path_point_index].point.pose);

  return output_path_msg;
}

diagnostic_msgs::msg::DiagnosticStatus BehaviorVelocityPlannerManager::getStopReasonDiag() const
{
  return stop_reason_diag_;
}
}  // namespace behavior_velocity_planner
