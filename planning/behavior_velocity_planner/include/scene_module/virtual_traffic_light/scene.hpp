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

#ifndef SCENE_MODULE__VIRTUAL_TRAFFIC_LIGHT__SCENE_HPP_
#define SCENE_MODULE__VIRTUAL_TRAFFIC_LIGHT__SCENE_HPP_

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <lanelet2_extension/regulatory_elements/virtual_traffic_light.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <vehicle_info_util/vehicle_info.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <memory>
#include <string>
#include <vector>

namespace behavior_velocity_planner
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
    std::vector<autoware_v2x_msgs::msg::KeyValue> custom_tags{};
    autoware_utils::Point3d instrument_center{};
    boost::optional<autoware_utils::LineString3d> stop_line{};
    autoware_utils::LineString3d start_line{};
    std::vector<autoware_utils::LineString3d> end_lines{};
  };

  struct ModuleData
  {
    geometry_msgs::msg::Pose head_pose{};
    autoware_auto_planning_msgs::msg::PathWithLaneId path{};
    boost::optional<geometry_msgs::msg::Pose> stop_head_pose_at_stop_line;
    boost::optional<geometry_msgs::msg::Pose> stop_head_pose_at_end_line;
  };

  struct PlannerParam
  {
    double max_delay_sec;
  };

public:
  VirtualTrafficLightModule(
    const int64_t module_id, const lanelet::autoware::VirtualTrafficLight & reg_elem,
    lanelet::ConstLanelet lane, const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;

private:
  const int64_t module_id_;
  const lanelet::autoware::VirtualTrafficLight & reg_elem_;
  const lanelet::ConstLanelet lane_;
  const PlannerParam planner_param_;
  State state_{State::NONE};
  autoware_v2x_msgs::msg::InfrastructureCommand command_;
  MapData map_data_;
  ModuleData module_data_;

  void updateInfrastructureCommand();

  void setStopReason(
    const geometry_msgs::msg::Pose & stop_pose,
    autoware_planning_msgs::msg::StopReason * stop_reason);

  bool isBeforeStartLine();

  bool isBeforeStopLine();

  bool isAfterAnyEndLine();

  bool isNearAnyEndLine();

  boost::optional<autoware_v2x_msgs::msg::VirtualTrafficLightState> findCorrespondingState();

  bool isStateTimeout(const autoware_v2x_msgs::msg::VirtualTrafficLightState & state);

  bool hasRightOfWay(const autoware_v2x_msgs::msg::VirtualTrafficLightState & state);

  void insertStopVelocityAtStopLine(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason);

  void insertStopVelocityAtEndLine(
    autoware_auto_planning_msgs::msg::PathWithLaneId * path,
    autoware_planning_msgs::msg::StopReason * stop_reason);
};
}  // namespace behavior_velocity_planner
#endif  // SCENE_MODULE__VIRTUAL_TRAFFIC_LIGHT__SCENE_HPP_
