// Copyright 2019 Autoware Foundation
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "planner_manager.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <behavior_velocity_planner/srv/load_plugin.hpp>
#include <behavior_velocity_planner/srv/unload_plugin.hpp>
#include <behavior_velocity_planner_common/planner_data.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/path.hpp>
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace behavior_velocity_planner
{
using autoware_auto_mapping_msgs::msg::HADMapBin;
using behavior_velocity_planner::srv::LoadPlugin;
using behavior_velocity_planner::srv::UnloadPlugin;
using tier4_planning_msgs::msg::VelocityLimit;

class BehaviorVelocityPlannerNode : public rclcpp::Node
{
public:
  explicit BehaviorVelocityPlannerNode(const rclcpp::NodeOptions & node_options);

private:
  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // subscriber
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    sub_predicted_objects_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_no_ground_pointcloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_vehicle_odometry_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr sub_acceleration_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_lanelet_map_;
  rclcpp::Subscription<autoware_perception_msgs::msg::TrafficSignalArray>::SharedPtr
    sub_traffic_signals_;
  rclcpp::Subscription<tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>::SharedPtr
    sub_virtual_traffic_light_states_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_;

  void onTrigger(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);
  void onPredictedObjects(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
  void onNoGroundPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void onAcceleration(const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr msg);
  void onLaneletMap(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);
  void onTrafficSignals(
    const autoware_perception_msgs::msg::TrafficSignalArray::ConstSharedPtr msg);
  void onVirtualTrafficLightStates(
    const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr msg);
  void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void onExternalVelocityLimit(const VelocityLimit::ConstSharedPtr msg);
  void onParam();

  // publisher
  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;

  void publishDebugMarker(const autoware_auto_planning_msgs::msg::Path & path);

  //  parameter
  double forward_path_length_;
  double backward_path_length_;

  // member
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  bool is_driving_forward_{true};
  HADMapBin::ConstSharedPtr map_ptr_{nullptr};
  bool has_received_map_;

  rclcpp::Service<LoadPlugin>::SharedPtr srv_load_plugin_;
  rclcpp::Service<UnloadPlugin>::SharedPtr srv_unload_plugin_;
  void onUnloadPlugin(
    const UnloadPlugin::Request::SharedPtr request,
    const UnloadPlugin::Response::SharedPtr response);
  void onLoadPlugin(
    const LoadPlugin::Request::SharedPtr request, const LoadPlugin::Response::SharedPtr response);

  // mutex for planner_data_
  std::mutex mutex_;

  // function
  geometry_msgs::msg::PoseStamped getCurrentPose();
  bool isDataReady(const PlannerData planner_data, rclcpp::Clock clock) const;
  autoware_auto_planning_msgs::msg::Path generatePath(
    const autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
    const PlannerData & planner_data);

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace behavior_velocity_planner

#endif  // NODE_HPP_
