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

#ifndef SURROUND_OBSTACLE_CHECKER__NODE_HPP_
#define SURROUND_OBSTACLE_CHECKER__NODE_HPP_

#include "surround_obstacle_checker/debug_marker.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"

#include <motion_utils/vehicle/vehicle_state_checker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_planning_msgs/msg/velocity_limit.hpp>
#include <tier4_planning_msgs/msg/velocity_limit_clear_command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace surround_obstacle_checker
{

using autoware_auto_perception_msgs::msg::PredictedObjects;
using autoware_auto_perception_msgs::msg::Shape;
using motion_utils::VehicleStopChecker;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_planning_msgs::msg::VelocityLimitClearCommand;
using vehicle_info_util::VehicleInfo;

using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public rclcpp::Node
{
public:
  explicit SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

  struct NodeParam
  {
    bool is_surround_obstacle;
    std::unordered_map<int, bool> enable_check_map;
    std::unordered_map<int, double> surround_check_front_distance_map;
    std::unordered_map<int, double> surround_check_side_distance_map;
    std::unordered_map<int, double> surround_check_back_distance_map;
    bool pointcloud_enable_check;
    double pointcloud_surround_check_front_distance;
    double pointcloud_surround_check_side_distance;
    double pointcloud_surround_check_back_distance;
    double surround_check_hysteresis_distance;
    double state_clear_time;
    bool publish_debug_footprints;
    std::string debug_footprint_label;
  };

private:
  rcl_interfaces::msg::SetParametersResult onParam(
    const std::vector<rclcpp::Parameter> & parameters);

  std::array<double, 3> getCheckDistances(const std::string & str_label) const;

  void onTimer();

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void onDynamicObjects(const PredictedObjects::ConstSharedPtr msg);

  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  std::optional<Obstacle> getNearestObstacle() const;

  std::optional<Obstacle> getNearestObstacleByPointCloud() const;

  std::optional<Obstacle> getNearestObstacleByDynamicObject() const;

  std::optional<geometry_msgs::msg::TransformStamped> getTransform(
    const std::string & source, const std::string & target, const rclcpp::Time & stamp,
    double duration_sec) const;

  bool isStopRequired(const bool is_obstacle_found, const bool is_stopped);

  // ros
  mutable tf2_ros::Buffer tf_buffer_{get_clock()};
  mutable tf2_ros::TransformListener tf_listener_{tf_buffer_};
  rclcpp::TimerBase::SharedPtr timer_;

  // publisher and subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_dynamic_objects_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr pub_stop_reason_;
  rclcpp::Publisher<VelocityLimitClearCommand>::SharedPtr pub_clear_velocity_limit_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_velocity_limit_;

  // parameter callback result
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // stop checker
  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;

  // debug
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;

  // parameter
  NodeParam node_param_;
  vehicle_info_util::VehicleInfo vehicle_info_;

  // data
  nav_msgs::msg::Odometry::ConstSharedPtr odometry_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  PredictedObjects::ConstSharedPtr object_ptr_;

  // State Machine
  State state_ = State::PASS;
  std::shared_ptr<const rclcpp::Time> last_obstacle_found_time_;

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;

  bool use_dynamic_object_;

  std::unordered_map<std::string, int> label_map_;
};
}  // namespace surround_obstacle_checker

#endif  // SURROUND_OBSTACLE_CHECKER__NODE_HPP_
