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
#include "tier4_autoware_utils/ros/polling_subscriber.hpp"

#include <autoware_behavior_velocity_planner/srv/load_plugin.hpp>
#include <autoware_behavior_velocity_planner/srv/unload_plugin.hpp>
#include <autoware_behavior_velocity_planner_common/planner_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/published_time_publisher.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_api_msgs/msg/crosswalk_status.hpp>
#include <tier4_api_msgs/msg/intersection_status.hpp>
#include <tier4_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_behavior_velocity_planner::srv::LoadPlugin;
using autoware_behavior_velocity_planner::srv::UnloadPlugin;
using autoware_map_msgs::msg::LaneletMapBin;
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
  rclcpp::Subscription<tier4_planning_msgs::msg::PathWithLaneId>::SharedPtr
    trigger_sub_path_with_lane_id_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_lanelet_map_;
  rclcpp::Subscription<VelocityLimit>::SharedPtr sub_external_velocity_limit_;

  // polling subscribers
  tier4_autoware_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::PredictedObjects>
    sub_predicted_objects_{this, "~/input/dynamic_objects"};

  tier4_autoware_utils::InterProcessPollingSubscriber<sensor_msgs::msg::PointCloud2>
    sub_no_ground_pointcloud_{
      this, "~/input/no_ground_pointcloud", tier4_autoware_utils::SingleDepthSensorQoS()};

  tier4_autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::Odometry>
    sub_vehicle_odometry_{this, "~/input/vehicle_odometry"};

  tier4_autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::AccelWithCovarianceStamped>
    sub_acceleration_{this, "~/input/accel"};

  tier4_autoware_utils::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_signals_{this, "~/input/traffic_signals"};

  tier4_autoware_utils::InterProcessPollingSubscriber<
    tier4_v2x_msgs::msg::VirtualTrafficLightStateArray>
    sub_virtual_traffic_light_states_{this, "~/input/virtual_traffic_light_states"};

  tier4_autoware_utils::InterProcessPollingSubscriber<nav_msgs::msg::OccupancyGrid>
    sub_occupancy_grid_{this, "~/input/occupancy_grid"};

  void onTrigger(const tier4_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg);

  void onParam();
  void onLaneletMap(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  void onExternalVelocityLimit(const VelocityLimit::ConstSharedPtr msg);

  void processNoGroundPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void processOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void processTrafficSignals(
    const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg);
  bool processData(rclcpp::Clock clock);

  // publisher
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_viz_pub_;

  void publishDebugMarker(const autoware_planning_msgs::msg::Path & path);

  //  parameter
  double forward_path_length_;
  double backward_path_length_;
  double behavior_output_path_interval_;

  // member
  PlannerData planner_data_;
  BehaviorVelocityPlannerManager planner_manager_;
  bool is_driving_forward_{true};
  LaneletMapBin::ConstSharedPtr map_ptr_{nullptr};
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
  bool isDataReady(rclcpp::Clock clock);
  autoware_planning_msgs::msg::Path generatePath(
    const tier4_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg,
    const PlannerData & planner_data);

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

  std::unique_ptr<tier4_autoware_utils::PublishedTimePublisher> published_time_publisher_;

  static constexpr int logger_throttle_interval = 3000;
};
}  // namespace autoware::behavior_velocity_planner

#endif  // NODE_HPP_
