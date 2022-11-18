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

#include "behavior_path_planner/scene_module/pull_out/geometric_pull_out.hpp"

#include "behavior_path_planner/scene_module/pull_out/util.hpp"

using lanelet::utils::getArcCoordinates;
using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::calcOffsetPose;
namespace behavior_path_planner
{
using pull_out_utils::combineReferencePath;
using pull_out_utils::getPullOutLanes;

GeometricPullOut::GeometricPullOut(
  rclcpp::Node & node, const PullOutParameters & parameters,
  const ParallelParkingParameters & parallel_parking_parameters)
: PullOutPlannerBase{node, parameters}, parallel_parking_parameters_{parallel_parking_parameters}
{
}

boost::optional<PullOutPath> GeometricPullOut::plan(Pose start_pose, Pose goal_pose)
{
  PullOutPath output;

  // combine road lane and shoulder lane
  const auto road_lanes = util::getExtendedCurrentLanes(planner_data_);
  const auto shoulder_lanes = getPullOutLanes(planner_data_);
  auto lanes = road_lanes;
  lanes.insert(lanes.end(), shoulder_lanes.begin(), shoulder_lanes.end());

  // todo: set params only once?
  planner_.setData(planner_data_, parallel_parking_parameters_);
  const bool found_valid_path =
    planner_.planPullOut(start_pose, goal_pose, road_lanes, shoulder_lanes);
  if (!found_valid_path) {
    return {};
  }

  // collision check with objects in shoulder lanes
  const auto arc_path = planner_.getArcPath();
  const auto shoulder_lane_objects =
    util::filterObjectsByLanelets(*(planner_data_->dynamic_object), shoulder_lanes);
  if (util::checkCollisionBetweenPathFootprintsAndObjects(
        vehicle_footprint_, arc_path, shoulder_lane_objects, parameters_.collision_check_margin)) {
    return {};
  }

  output.partial_paths = planner_.getPaths();
  output.start_pose = planner_.getArcPaths().at(0).points.back().point.pose;
  output.end_pose = planner_.getArcPaths().at(1).points.back().point.pose;

  return output;
}
}  // namespace behavior_path_planner
