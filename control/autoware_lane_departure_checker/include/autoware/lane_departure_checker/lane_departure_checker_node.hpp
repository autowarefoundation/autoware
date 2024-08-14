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

#ifndef AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_
#define AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_

#include "autoware/lane_departure_checker/lane_departure_checker.hpp"
#include "autoware/universe_utils/ros/polling_subscriber.hpp"

#include <autoware/universe_utils/ros/debug_publisher.hpp>
#include <autoware/universe_utils/ros/processing_time_publisher.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_debug_msgs/msg/float64_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::lane_departure_checker
{
using autoware_map_msgs::msg::LaneletMapBin;

struct NodeParam
{
  bool will_out_of_lane_checker;
  bool out_of_lane_checker;
  bool boundary_departure_checker;

  double update_rate;
  bool visualize_lanelet;
  bool include_right_lanes;
  bool include_left_lanes;
  bool include_opposite_lanes;
  bool include_conflicting_lanes;
  std::vector<std::string> boundary_types_to_detect;
};

class LaneDepartureCheckerNode : public rclcpp::Node
{
public:
  explicit LaneDepartureCheckerNode(const rclcpp::NodeOptions & options);

private:
  // Subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry> sub_odom_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware::universe_utils::polling_policy::Newest>
    sub_lanelet_map_bin_{this, "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<LaneletRoute> sub_route_{
    this, "~/input/route"};
  autoware::universe_utils::InterProcessPollingSubscriber<Trajectory> sub_reference_trajectory_{
    this, "~/input/reference_trajectory"};
  autoware::universe_utils::InterProcessPollingSubscriber<Trajectory> sub_predicted_trajectory_{
    this, "~/input/predicted_trajectory"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_adapi_v1_msgs::msg::OperationModeState>
    sub_operation_mode_{this, "/api/operation_mode/state"};
  autoware::universe_utils::InterProcessPollingSubscriber<
    autoware_vehicle_msgs::msg::ControlModeReport>
    sub_control_mode_{this, "/vehicle/status/control_mode"};

  // Data Buffer
  nav_msgs::msg::Odometry::ConstSharedPtr current_odom_;
  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::ConstLanelets shoulder_lanelets_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_;
  lanelet::routing::RoutingGraphPtr routing_graph_;
  LaneletRoute::ConstSharedPtr route_;
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr cov_;
  LaneletRoute::ConstSharedPtr last_route_;
  lanelet::ConstLanelets route_lanelets_;
  Trajectory::ConstSharedPtr reference_trajectory_;
  Trajectory::ConstSharedPtr predicted_trajectory_;
  autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr operation_mode_;
  autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr control_mode_;

  // Callback
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onLaneletMapBin(const LaneletMapBin::ConstSharedPtr msg);
  void onRoute(const LaneletRoute::ConstSharedPtr msg);
  void onReferenceTrajectory(const Trajectory::ConstSharedPtr msg);
  void onPredictedTrajectory(const Trajectory::ConstSharedPtr msg);
  void onOperationMode(const autoware_adapi_v1_msgs::msg::OperationModeState::ConstSharedPtr msg);
  void onControlMode(const autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg);

  // Publisher
  autoware::universe_utils::DebugPublisher debug_publisher_{this, "~/debug"};
  autoware::universe_utils::ProcessingTimePublisher processing_diag_publisher_{
    this, "~/debug/processing_time_ms_diag"};
  rclcpp::Publisher<tier4_debug_msgs::msg::Float64Stamped>::SharedPtr processing_time_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  bool isDataReady();
  bool isDataTimeout();
  bool isDataValid();
  void onTimer();

  // Parameter
  NodeParam node_param_;
  Param param_;
  double vehicle_length_m_;

  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Core
  Input input_{};
  Output output_{};
  std::unique_ptr<LaneDepartureChecker> lane_departure_checker_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_{this};

  void checkLaneDeparture(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void checkTrajectoryDeviation(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // Visualization
  visualization_msgs::msg::MarkerArray createMarkerArray() const;

  // Lanelet Neighbor Search
  lanelet::ConstLanelets getAllSharedLineStringLanelets(
    const lanelet::ConstLanelet & current_lane, const bool is_right, const bool is_left,
    const bool is_opposite, const bool is_conflicting, const bool & invert_opposite);

  lanelet::ConstLanelets getAllRightSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false);

  lanelet::ConstLanelets getAllLeftSharedLinestringLanelets(
    const lanelet::ConstLanelet & lane, const bool & include_opposite,
    const bool & invert_opposite = false);

  boost::optional<lanelet::ConstLanelet> getLeftLanelet(const lanelet::ConstLanelet & lanelet);

  lanelet::Lanelets getLeftOppositeLanelets(const lanelet::ConstLanelet & lanelet);
  boost::optional<lanelet::ConstLanelet> getRightLanelet(
    const lanelet::ConstLanelet & lanelet) const;

  lanelet::Lanelets getRightOppositeLanelets(const lanelet::ConstLanelet & lanelet);
};
}  // namespace autoware::lane_departure_checker

#endif  // AUTOWARE__LANE_DEPARTURE_CHECKER__LANE_DEPARTURE_CHECKER_NODE_HPP_
