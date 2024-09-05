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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/interface/steering_factor_interface.hpp"
#include "autoware/universe_utils/ros/logger_level_configure.hpp"
#include "planner_manager.hpp"

#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware/universe_utils/ros/published_time_publisher.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/approval.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_change_module.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::SetParametersResult;
using steering_factor_interface::SteeringFactorInterface;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::LateralOffset;
using tier4_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::RerouteAvailability;
using tier4_planning_msgs::msg::Scenario;
using tier4_planning_msgs::msg::StopReasonArray;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class BehaviorPathPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options);

  // Getter method for waiting approval modules
  std::vector<std::string> getWaitingApprovalModules();

  // Getter method for running modules
  std::vector<std::string> getRunningModules();

private:
  // subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletRoute, universe_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<
    LaneletMapBin, universe_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<Odometry> velocity_subscriber_{
    this, "~/input/odometry"};
  autoware::universe_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "~/input/accel"};
  autoware::universe_utils::InterProcessPollingSubscriber<Scenario> scenario_subscriber_{
    this, "~/input/scenario"};
  autoware::universe_utils::InterProcessPollingSubscriber<PredictedObjects> perception_subscriber_{
    this, "~/input/perception"};
  autoware::universe_utils::InterProcessPollingSubscriber<OccupancyGrid> occupancy_grid_subscriber_{
    this, "~/input/occupancy_grid_map"};
  autoware::universe_utils::InterProcessPollingSubscriber<OccupancyGrid> costmap_subscriber_{
    this, "~/input/costmap"};
  autoware::universe_utils::InterProcessPollingSubscriber<TrafficLightGroupArray>
    traffic_signals_subscriber_{this, "~/input/traffic_signals"};
  autoware::universe_utils::InterProcessPollingSubscriber<LateralOffset> lateral_offset_subscriber_{
    this, "~/input/lateral_offset"};
  autoware::universe_utils::InterProcessPollingSubscriber<OperationModeState>
    operation_mode_subscriber_{
      this, "/system/operation_mode/state", rclcpp::QoS{1}.transient_local()};
  autoware::universe_utils::InterProcessPollingSubscriber<tier4_planning_msgs::msg::VelocityLimit>
    external_limit_max_velocity_subscriber_{this, "/planning/scenario_planning/max_velocity"};

  // publisher
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_signal_publisher_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_signal_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr bound_publisher_;
  rclcpp::Publisher<PoseWithUuidStamped>::SharedPtr modified_goal_publisher_;
  rclcpp::Publisher<StopReasonArray>::SharedPtr stop_reason_publisher_;
  rclcpp::Publisher<RerouteAvailability>::SharedPtr reroute_availability_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, rclcpp::Publisher<Path>::SharedPtr> path_candidate_publishers_;
  std::map<std::string, rclcpp::Publisher<Path>::SharedPtr> path_reference_publishers_;

  std::shared_ptr<PlannerData> planner_data_;
  Scenario::ConstSharedPtr current_scenario_{nullptr};
  LaneletMapBin::ConstSharedPtr map_ptr_{nullptr};
  LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  bool has_received_map_{false};
  bool has_received_route_{false};

  std::shared_ptr<PlannerManager> planner_manager_;

  std::unique_ptr<SteeringFactorInterface> steering_factor_interface_ptr_;

  std::mutex mutex_pd_;       // mutex for planner_data_
  std::mutex mutex_manager_;  // mutex for bt_manager_ or planner_manager_

  // setup
  void takeData();
  bool isDataReady();

  // callback
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);
  void onPerception(const PredictedObjects::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onCostMap(const OccupancyGrid::ConstSharedPtr msg);
  void onTrafficSignals(const TrafficLightGroupArray::ConstSharedPtr msg);
  void onMap(const LaneletMapBin::ConstSharedPtr map_msg);
  void onRoute(const LaneletRoute::ConstSharedPtr route_msg);
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void onLateralOffset(const LateralOffset::ConstSharedPtr msg);
  void on_external_velocity_limiter(
    const tier4_planning_msgs::msg::VelocityLimit::ConstSharedPtr msg);

  SetParametersResult onSetParam(const std::vector<rclcpp::Parameter> & parameters);

  OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run();

  /**
   * @brief extract path from behavior tree output
   */
  PathWithLaneId::SharedPtr getPath(
    const BehaviorModuleOutput & output, const std::shared_ptr<PlannerData> & planner_data,
    const std::shared_ptr<PlannerManager> & planner_manager);

  bool keepInputPoints(const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const;

  /**
   * @brief skip smooth goal connection
   */
  void computeTurnSignal(
    const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
    const BehaviorModuleOutput & output);

  // debug
  rclcpp::Publisher<AvoidanceDebugMsgArray>::SharedPtr debug_avoidance_msg_array_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_turn_signal_info_publisher_;

  /**
   * @brief publish reroute availability
   */
  void publish_reroute_availability() const;

  /**
   * @brief publish steering factor from intersection
   */
  void publish_steering_factor(
    const std::shared_ptr<PlannerData> & planner_data, const TurnIndicatorsCommand & turn_signal);

  /**
   * @brief publish turn signal debug info
   */
  void publish_turn_signal_debug_data(const TurnSignalDebugData & debug_data);

  /**
   * @brief publish left and right bound
   */
  void publish_bounds(const PathWithLaneId & path);

  /**
   * @brief publish debug messages
   */
  void publishSceneModuleDebugMsg(
    const std::shared_ptr<SceneModuleVisitor> & debug_messages_data_ptr);

  /**
   * @brief publish path candidate
   */
  void publishPathCandidate(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  void publishPathReference(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  /**
   * @brief convert path with lane id to path for publish path candidate
   */
  Path convertToPath(
    const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready,
    const std::shared_ptr<PlannerData> & planner_data);

  std::unique_ptr<autoware::universe_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<autoware::universe_utils::PublishedTimePublisher> published_time_publisher_;
};
}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
