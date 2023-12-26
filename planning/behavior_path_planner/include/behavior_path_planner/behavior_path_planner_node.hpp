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

#ifndef BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
#define BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_

#include "behavior_path_planner/planner_manager.hpp"
#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/interface/steering_factor_interface.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_planning_msgs/msg/approval.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_change_module.hpp>
#include <tier4_planning_msgs/msg/reroute_availability.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
#include <tier4_planning_msgs/msg/stop_reason_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace behavior_path_planner
{
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_perception_msgs::msg::TrafficSignalArray;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::SetParametersResult;
using steering_factor_interface::SteeringFactorInterface;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::LateralOffset;
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
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Subscription<Odometry>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr acceleration_subscriber_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_subscriber_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr perception_subscriber_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Subscription<TrafficSignalArray>::SharedPtr traffic_signals_subscriber_;
  rclcpp::Subscription<LateralOffset>::SharedPtr lateral_offset_subscriber_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_subscriber_;
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

  std::shared_ptr<PlannerManager> planner_manager_;

  std::unique_ptr<SteeringFactorInterface> steering_factor_interface_ptr_;
  Scenario::SharedPtr current_scenario_{nullptr};

  HADMapBin::ConstSharedPtr map_ptr_{nullptr};
  LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  bool has_received_map_{false};
  bool has_received_route_{false};

  std::mutex mutex_pd_;       // mutex for planner_data_
  std::mutex mutex_manager_;  // mutex for bt_manager_ or planner_manager_
  std::mutex mutex_map_;      // mutex for has_received_map_ and map_ptr_
  std::mutex mutex_route_;    // mutex for has_received_route_ and route_ptr_

  // setup
  bool isDataReady();

  // parameters
  BehaviorPathPlannerParameters getCommonParam();

  // callback
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);
  void onPerception(const PredictedObjects::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onCostMap(const OccupancyGrid::ConstSharedPtr msg);
  void onTrafficSignals(const TrafficSignalArray::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr map_msg);
  void onRoute(const LaneletRoute::ConstSharedPtr route_msg);
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void onLateralOffset(const LateralOffset::ConstSharedPtr msg);
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

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
