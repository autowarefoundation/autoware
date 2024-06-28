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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace autoware::behavior_velocity_planner
{
class StopLineModule : public SceneModuleInterface
{
  using StopLineWithLaneId = std::pair<lanelet::ConstLineString3d, int64_t>;

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
    std::vector<LineString2d> search_segments;
    LineString2d search_stopline;
  };

  struct PlannerParam
  {
    double stop_margin;
    double stop_duration_sec;
    double hold_stop_margin_distance;
    bool use_initialization_stop_line_state;
    bool show_stop_line_collision_check;
  };

public:
  StopLineModule(
    const int64_t module_id, const size_t lane_id, const lanelet::ConstLineString3d & stop_line,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  std::shared_ptr<const rclcpp::Time> stopped_time_;

  geometry_msgs::msg::Point getCenterOfStopLine(const lanelet::ConstLineString3d & stop_line);

  int64_t lane_id_;

  lanelet::ConstLineString3d stop_line_;

  // State machine
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // SCENE_HPP_
