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
#include <optional>
#include <tuple>
#include <vector>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <behavior_velocity_planner_common/scene_module_interface.hpp>
#include <behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace behavior_velocity_planner
{
class TrafficLightModule : public SceneModuleInterface
{
public:
  using TrafficSignal = autoware_perception_msgs::msg::TrafficSignal;
  using TrafficSignalElement = autoware_perception_msgs::msg::TrafficSignalElement;
  using Time = rclcpp::Time;
  enum class State { APPROACH, GO_OUT };

  struct DebugData
  {
    double base_link2front;
    std::vector<std::tuple<
      std::shared_ptr<const lanelet::TrafficLight>, autoware_perception_msgs::msg::TrafficSignal>>
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
    double yellow_lamp_period;
    double stop_time_hysteresis;
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
  motion_utils::VirtualWalls createVirtualWalls() override;

  inline TrafficSignal getTrafficSignal() const { return looking_tl_state_; }

  inline State getTrafficLightModuleState() const { return state_; }

  inline std::optional<int> getFirstRefStopPathPointIndex() const
  {
    return first_ref_stop_path_point_index_;
  }

private:
  bool isStopSignal();

  bool isTrafficSignalStop(const TrafficSignal & tl_state) const;

  autoware_auto_planning_msgs::msg::PathWithLaneId insertStopPose(
    const autoware_auto_planning_msgs::msg::PathWithLaneId & input,
    const size_t & insert_target_point_idx, const Eigen::Vector2d & target_point,
    tier4_planning_msgs::msg::StopReason * stop_reason);

  bool isPassthrough(const double & signed_arc_length) const;

  bool hasTrafficLightCircleColor(const TrafficSignal & tl_state, const uint8_t & lamp_color) const;

  bool hasTrafficLightShape(const TrafficSignal & tl_state, const uint8_t & lamp_shape) const;

  bool findValidTrafficSignal(TrafficSignalStamped & valid_traffic_signal) const;

  bool isTrafficSignalTimedOut() const;

  void updateTrafficSignal();

  // Lane id
  const int64_t lane_id_;

  // Key Feature
  const lanelet::TrafficLight & traffic_light_reg_elem_;
  lanelet::ConstLanelet lane_;

  // State
  State state_;

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  // prevent pass through chattering
  bool is_prev_state_stop_;

  // prevent stop chattering
  std::unique_ptr<Time> stop_signal_received_time_ptr_{};

  std::optional<int> first_ref_stop_path_point_index_;

  std::optional<Time> traffic_signal_stamp_;

  // Traffic Light State
  TrafficSignal looking_tl_state_;
};
}  // namespace behavior_velocity_planner

#endif  // SCENE_HPP_
