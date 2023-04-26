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

#include "behavior_path_planner/data_manager.hpp"
#include "behavior_path_planner/scene_module/scene_module_interface.hpp"

#ifdef USE_OLD_ARCHITECTURE
#include "behavior_path_planner/behavior_tree_manager.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/goal_planner/goal_planner_module.hpp"
#include "behavior_path_planner/scene_module/lane_change/lane_change_module.hpp"
#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"
#include "behavior_path_planner/scene_module/pull_out/pull_out_module.hpp"
#include "behavior_path_planner/scene_module/side_shift/side_shift_module.hpp"
#else
#include "behavior_path_planner/planner_manager.hpp"
#include "behavior_path_planner/scene_module/avoidance/manager.hpp"
#include "behavior_path_planner/scene_module/avoidance_by_lc/manager.hpp"
#include "behavior_path_planner/scene_module/goal_planner/manager.hpp"
#include "behavior_path_planner/scene_module/lane_change/manager.hpp"
#include "behavior_path_planner/scene_module/pull_out/manager.hpp"
#include "behavior_path_planner/scene_module/side_shift/manager.hpp"
#endif

#include "behavior_path_planner/steering_factor_interface.hpp"
#include "behavior_path_planner/turn_signal_decider.hpp"
#include "behavior_path_planner/utils/avoidance/avoidance_module_data.hpp"
#include "behavior_path_planner/utils/avoidance_by_lc/module_data.hpp"
#include "behavior_path_planner/utils/goal_planner/goal_planner_parameters.hpp"
#include "behavior_path_planner/utils/lane_change/lane_change_module_data.hpp"
#include "behavior_path_planner/utils/lane_following/module_data.hpp"
#include "behavior_path_planner/utils/pull_out/pull_out_parameters.hpp"
#include "behavior_path_planner/utils/side_shift/side_shift_parameters.hpp"

#include "tier4_planning_msgs/msg/detail/lane_change_debug_msg_array__struct.hpp"
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
#include <tier4_planning_msgs/msg/lane_change_debug_msg_array.hpp>
#include <tier4_planning_msgs/msg/path_change_module.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>
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
using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using geometry_msgs::msg::TwistStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;
using rcl_interfaces::msg::SetParametersResult;
using steering_factor_interface::SteeringFactorInterface;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::LaneChangeDebugMsgArray;
using tier4_planning_msgs::msg::LateralOffset;
using tier4_planning_msgs::msg::Scenario;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class BehaviorPathPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorPathPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Subscription<LaneletRoute>::SharedPtr route_subscriber_;
  rclcpp::Subscription<HADMapBin>::SharedPtr vector_map_subscriber_;
  rclcpp::Subscription<Odometry>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<AccelWithCovarianceStamped>::SharedPtr acceleration_subscriber_;
  rclcpp::Subscription<Scenario>::SharedPtr scenario_subscriber_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr perception_subscriber_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr costmap_subscriber_;
  rclcpp::Subscription<LateralOffset>::SharedPtr lateral_offset_subscriber_;
  rclcpp::Subscription<OperationModeState>::SharedPtr operation_mode_subscriber_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr path_publisher_;
  rclcpp::Publisher<TurnIndicatorsCommand>::SharedPtr turn_signal_publisher_;
  rclcpp::Publisher<HazardLightsCommand>::SharedPtr hazard_signal_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr bound_publisher_;
  rclcpp::Publisher<PoseWithUuidStamped>::SharedPtr modified_goal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::map<std::string, rclcpp::Publisher<Path>::SharedPtr> path_candidate_publishers_;
  std::map<std::string, rclcpp::Publisher<Path>::SharedPtr> path_reference_publishers_;

  std::shared_ptr<PlannerData> planner_data_;

#ifdef USE_OLD_ARCHITECTURE
  std::shared_ptr<BehaviorTreeManager> bt_manager_;
#else
  std::shared_ptr<PlannerManager> planner_manager_;
#endif

  std::unique_ptr<SteeringFactorInterface> steering_factor_interface_ptr_;
  Scenario::SharedPtr current_scenario_{nullptr};

  HADMapBin::ConstSharedPtr map_ptr_{nullptr};
  LaneletRoute::ConstSharedPtr route_ptr_{nullptr};
  bool has_received_map_{false};
  bool has_received_route_{false};

  TurnSignalDecider turn_signal_decider_;

  std::mutex mutex_pd_;       // mutex for planner_data_
  std::mutex mutex_manager_;  // mutex for bt_manager_ or planner_manager_
  std::mutex mutex_map_;      // mutex for has_received_map_ and map_ptr_
  std::mutex mutex_route_;    // mutex for has_received_route_ and route_ptr_

  // setup
  bool isDataReady();

  // parameters
  std::shared_ptr<AvoidanceParameters> avoidance_param_ptr_;
  std::shared_ptr<AvoidanceByLCParameters> avoidance_by_lc_param_ptr_;
  std::shared_ptr<SideShiftParameters> side_shift_param_ptr_;
  std::shared_ptr<LaneChangeParameters> lane_change_param_ptr_;
  std::shared_ptr<PullOutParameters> pull_out_param_ptr_;
  std::shared_ptr<GoalPlannerParameters> goal_planner_param_ptr_;

  BehaviorPathPlannerParameters getCommonParam();

#ifdef USE_OLD_ARCHITECTURE
  BehaviorTreeManagerParam getBehaviorTreeManagerParam();
#endif

  AvoidanceParameters getAvoidanceParam();
  LaneChangeParameters getLaneChangeParam();
  SideShiftParameters getSideShiftParam();
  GoalPlannerParameters getGoalPlannerParam();
  PullOutParameters getPullOutParam();
  AvoidanceByLCParameters getAvoidanceByLCParam(
    const std::shared_ptr<AvoidanceParameters> & avoidance_param,
    const std::shared_ptr<LaneChangeParameters> & lane_change_param);

  // callback
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onAcceleration(const AccelWithCovarianceStamped::ConstSharedPtr msg);
  void onPerception(const PredictedObjects::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onCostMap(const OccupancyGrid::ConstSharedPtr msg);
  void onMap(const HADMapBin::ConstSharedPtr map_msg);
  void onRoute(const LaneletRoute::ConstSharedPtr route_msg);
  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void onLateralOffset(const LateralOffset::ConstSharedPtr msg);
  SetParametersResult onSetParam(const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Modify the path points near the goal to smoothly connect the lanelet and the goal point.
   */
  PathWithLaneId modifyPathForSmoothGoalConnection(
    const PathWithLaneId & path,
    const std::shared_ptr<PlannerData> & planner_data) const;  // (TODO) move to util
  OnSetParametersCallbackHandle::SharedPtr m_set_param_res;

  /**
   * @brief Execute behavior tree and publish planned data.
   */
  void run();

  /**
   * @brief extract path from behavior tree output
   */
#ifdef USE_OLD_ARCHITECTURE
  PathWithLaneId::SharedPtr getPath(
    const BehaviorModuleOutput & bt_out, const std::shared_ptr<PlannerData> & planner_data,
    const std::shared_ptr<BehaviorTreeManager> & bt_manager);
#else
  PathWithLaneId::SharedPtr getPath(
    const BehaviorModuleOutput & bt_out, const std::shared_ptr<PlannerData> & planner_data,
    const std::shared_ptr<PlannerManager> & planner_manager);
#endif

  bool keepInputPoints(const std::vector<std::shared_ptr<SceneModuleStatus>> & statuses) const;

  /**
   * @brief skip smooth goal connection
   */
  void computeTurnSignal(
    const std::shared_ptr<PlannerData> planner_data, const PathWithLaneId & path,
    const BehaviorModuleOutput & output);

  // debug
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_maximum_drivable_area_publisher_;
  rclcpp::Publisher<AvoidanceDebugMsgArray>::SharedPtr debug_avoidance_msg_array_publisher_;
  rclcpp::Publisher<LaneChangeDebugMsgArray>::SharedPtr debug_lane_change_msg_array_publisher_;

  /**
   * @brief publish steering factor from intersection
   */
  void publish_steering_factor(const TurnIndicatorsCommand & turn_signal);

  /**
   * @brief publish left and right bound
   */
  void publish_bounds(const PathWithLaneId & path);

  /**
   * @brief publish debug messages
   */
#ifdef USE_OLD_ARCHITECTURE
  void publishSceneModuleDebugMsg(
    const std::shared_ptr<SceneModuleVisitor> & debug_messages_data_ptr);
#endif

  /**
   * @brief publish path candidate
   */
#ifdef USE_OLD_ARCHITECTURE
  void publishPathCandidate(
    const std::vector<std::shared_ptr<SceneModuleInterface>> & scene_modules,
    const std::shared_ptr<PlannerData> & planner_data);
#else
  void publishPathCandidate(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);

  void publishPathReference(
    const std::vector<std::shared_ptr<SceneModuleManagerInterface>> & managers,
    const std::shared_ptr<PlannerData> & planner_data);
#endif

  /**
   * @brief convert path with lane id to path for publish path candidate
   */
  Path convertToPath(
    const std::shared_ptr<PathWithLaneId> & path_candidate_ptr, const bool is_ready,
    const std::shared_ptr<PlannerData> & planner_data);
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__BEHAVIOR_PATH_PLANNER_NODE_HPP_
