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

#ifndef SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_
#define SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <utilization/boost_geometry_helper.hpp>

#include <autoware_auto_perception_msgs/msg/looking_traffic_signal.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace behavior_velocity_planner
{
class TrafficLightModule : public SceneModuleInterface
{
public:
  enum class State { APPROACH, GO_OUT };
  enum class Input { PERCEPTION, EXTERNAL, NONE };  // EXTERNAL: FOA, V2X, etc.

  struct DebugData
  {
    double base_link2front;
    std::vector<std::tuple<
      std::shared_ptr<const lanelet::TrafficLight>,
      autoware_auto_perception_msgs::msg::TrafficSignal>>
      tl_state;
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    geometry_msgs::msg::Pose first_stop_pose;
    std::vector<geometry_msgs::msg::Pose> dead_line_poses;
    std::vector<geometry_msgs::msg::Point> traffic_light_points;
    std::optional<geometry_msgs::msg::Point> highest_confidence_traffic_light_point = {
      std::nullopt};
  };

  struct PlannerParam
  {
    double stop_margin;
    double tl_state_timeout;
    double external_tl_state_timeout;
    double yellow_lamp_period;
    bool enable_pass_judge;
  };

public:
  TrafficLightModule(
    const int64_t module_id, const int64_t lane_id,
    const lanelet::TrafficLight & traffic_light_reg_elem, lanelet::ConstLanelet lane,
    const PlannerParam & planner_param, const rclcpp::Logger logger,
    const rclcpp::Clock::SharedPtr clock);

  bool modifyPathVelocity(PathWithLaneId * path, StopReason * stop_reason) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  visualization_msgs::msg::MarkerArray createVirtualWallMarkerArray() override;

  inline autoware_auto_perception_msgs::msg::LookingTrafficSignal getTrafficSignal() const
  {
    return looking_tl_state_;
  }

  inline State getTrafficLightModuleState() const { return state_; }

  inline boost::optional<int> getFirstRefStopPathPointIndex() const
  {
    return first_ref_stop_path_point_index_;
  }

private:
  bool isStopSignal(const lanelet::ConstLineStringsOrPolygons3d & traffic_lights);

  bool isTrafficSignalStop(
    const autoware_auto_perception_msgs::msg::TrafficSignal & tl_state) const;

  autoware_auto_planning_msgs::msg::PathWithLaneId insertStopPose(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
    const size_t & insert_target_point_idx, const Eigen::Vector2d & target_point,
    tier4_planning_msgs::msg::StopReason * stop_reason);

  bool isPassthrough(const double & signed_arc_length) const;

  bool hasTrafficLightCircleColor(
    const autoware_auto_perception_msgs::msg::TrafficSignal & tl_state,
    const uint8_t & lamp_color) const;

  bool hasTrafficLightShape(
    const autoware_auto_perception_msgs::msg::TrafficSignal & tl_state,
    const uint8_t & lamp_shape) const;

  bool getHighestConfidenceTrafficSignal(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_auto_perception_msgs::msg::TrafficSignalStamped & highest_confidence_tl_state);

  bool getExternalTrafficSignal(
    const lanelet::ConstLineStringsOrPolygons3d & traffic_lights,
    autoware_auto_perception_msgs::msg::TrafficSignalStamped & external_tl_state);

  bool updateTrafficSignal(const lanelet::ConstLineStringsOrPolygons3d & traffic_lights);

  autoware_auto_perception_msgs::msg::TrafficSignalWithJudge generateTlStateWithJudgeFromTlState(
    const autoware_auto_perception_msgs::msg::TrafficSignal tl_state) const;

  // Lane id
  const int64_t lane_id_;

  // Key Feature
  const lanelet::TrafficLight & traffic_light_reg_elem_;
  lanelet::ConstLanelet lane_;

  // State
  State state_;

  // Input
  Input input_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // prevent paththrough chattering
  bool is_prev_state_stop_;

  boost::optional<int> first_ref_stop_path_point_index_;

  // Traffic Light State
  autoware_auto_perception_msgs::msg::LookingTrafficSignal looking_tl_state_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_MODULE__TRAFFIC_LIGHT__SCENE_HPP_
