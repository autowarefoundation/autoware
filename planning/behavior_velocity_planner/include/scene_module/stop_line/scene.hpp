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

#ifndef SCENE_MODULE__STOP_LINE__SCENE_HPP_
#define SCENE_MODULE__STOP_LINE__SCENE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace behavior_velocity_planner
{
class StopLineModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, STOPPED, START };

  struct SegmentIndexWithPose
  {
    size_t index;
    geometry_msgs::msg::Pose pose;
  };

  struct SegmentIndexWithPoint2d
  {
    size_t index;
    Point2d point;
  };

  struct SegmentIndexWithOffset
  {
    size_t index;
    double offset;
  };

  struct DebugData
  {
    double base_link2front;
    boost::optional<geometry_msgs::msg::Pose> stop_pose;
  };

  struct PlannerParam
  {
    double stop_margin;
    double stop_check_dist;
    double stop_duration_sec;
  };

public:
  StopLineModule(
    const int64_t module_id, const lanelet::ConstLineString3d & stop_line,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  int64_t module_id_;

  geometry_msgs::msg::Point getCenterOfStopLine(const lanelet::ConstLineString3d & stop_line);

  boost::optional<StopLineModule::SegmentIndexWithPoint2d> findCollision(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path, const LineString2d & stop_line);

  boost::optional<StopLineModule::SegmentIndexWithOffset> findOffsetSegment(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const StopLineModule::SegmentIndexWithPoint2d & collision);

  boost::optional<StopLineModule::SegmentIndexWithPose> calcStopPose(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const boost::optional<StopLineModule::SegmentIndexWithOffset> & offset_segment);

  autoware_auto_planning_msgs::msg::PathWithLaneId insertStopPose(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & path,
    const StopLineModule::SegmentIndexWithPose & insert_index_with_pose,
    autoware_planning_msgs::msg::StopReason * stop_reason);

  lanelet::ConstLineString3d stop_line_;
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__STOP_LINE__SCENE_HPP_
