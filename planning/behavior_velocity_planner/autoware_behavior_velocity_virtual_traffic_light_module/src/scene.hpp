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

#ifndef SCENE_HPP_
#define SCENE_HPP_

#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
class VirtualTrafficLightModule : public SceneModuleInterface
{
public:
  enum class State : uint8_t {
    NONE = 0,
    REQUESTING = 1,
    PASSING = 2,
    FINALIZING = 3,
    FINALIZED = 4,
  };

  struct MapData
  {
    std::string instrument_type{};
    std::string instrument_id{};
    std::vector<tier4_v2x_msgs::msg::KeyValue> custom_tags{};
    autoware::universe_utils::Point3d instrument_center{};
    std::optional<autoware::universe_utils::LineString3d> stop_line{};
    autoware::universe_utils::LineString3d start_line{};
    std::vector<autoware::universe_utils::LineString3d> end_lines{};
  };

  struct ModuleData
  {
    geometry_msgs::msg::Pose head_pose{};
    tier4_planning_msgs::msg::PathWithLaneId path{};
    std::optional<geometry_msgs::msg::Pose> stop_head_pose_at_stop_line;
    std::optional<geometry_msgs::msg::Pose> stop_head_pose_at_end_line;
  };

  struct PlannerParam
  {
    double max_delay_sec;
    double near_line_distance;
    double dead_line_margin;
    double hold_stop_margin_distance;
    double max_yaw_deviation_rad;
    bool check_timeout_after_stop_line;
  };

public:
  VirtualTrafficLightModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::autoware::VirtualTrafficLight & reg_elem, lanelet::ConstLanelet lane,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

private:
  const int64_t lane_id_;
  const lanelet::autoware::VirtualTrafficLight & reg_elem_;
  const lanelet::ConstLanelet lane_;
  const PlannerParam planner_param_;
  State state_{State::NONE};
  tier4_v2x_msgs::msg::InfrastructureCommand command_;
  MapData map_data_;
  ModuleData module_data_;

  void updateInfrastructureCommand();

  void setStopReason(const Pose & stop_pose, StopReason * stop_reason);

  void setVelocityFactor(
    const geometry_msgs::msg::Pose & stop_pose,
    autoware_adapi_v1_msgs::msg::VelocityFactor * velocity_factor);

  std::optional<size_t> getPathIndexOfFirstEndLine();

  bool isBeforeStartLine(const size_t end_line_idx);

  bool isBeforeStopLine(const size_t end_line_idx);

  bool isAfterAnyEndLine(const size_t end_line_idx);

  bool isNearAnyEndLine(const size_t end_line_idx);

  std::optional<tier4_v2x_msgs::msg::VirtualTrafficLightState> findCorrespondingState();

  bool isStateTimeout(const tier4_v2x_msgs::msg::VirtualTrafficLightState & state);

  bool hasRightOfWay(const tier4_v2x_msgs::msg::VirtualTrafficLightState & state);

  void insertStopVelocityAtStopLine(
    tier4_planning_msgs::msg::PathWithLaneId * path,
    tier4_planning_msgs::msg::StopReason * stop_reason, const size_t end_line_idx);

  void insertStopVelocityAtEndLine(
    tier4_planning_msgs::msg::PathWithLaneId * path,
    tier4_planning_msgs::msg::StopReason * stop_reason, const size_t end_line_idx);
};
}  // namespace autoware::behavior_velocity_planner
#endif  // SCENE_HPP_
