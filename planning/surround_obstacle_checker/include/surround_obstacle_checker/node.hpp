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

#include "autoware_utils/trajectory/tmp_conversion.hpp"
#include "surround_obstacle_checker/debug_marker.hpp"

#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

using Point2d = boost::geometry::model::d2::point_xy<double>;
using Polygon2d =
  boost::geometry::model::polygon<Point2d, false, false>;  // counter-clockwise, open
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

enum class State { PASS, STOP };

class SurroundObstacleCheckerNode : public rclcpp::Node
{
public:
  explicit SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  void pathCallback(const Trajectory::ConstSharedPtr input_msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);
  void dynamicObjectCallback(
    const autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr input_msg);
  void currentVelocityCallback(const nav_msgs::msg::Odometry::ConstSharedPtr input_msg);
  void insertStopVelocity(const size_t closest_idx, TrajectoryPoints * traj);
  bool convertPose(
    const geometry_msgs::msg::Pose & pose, const std::string & source, const std::string & target,
    const rclcpp::Time & time, geometry_msgs::msg::Pose & conv_pose);
  bool getPose(
    const std::string & source, const std::string & target, geometry_msgs::msg::Pose & pose);
  void getNearestObstacle(double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point);
  void getNearestObstacleByPointCloud(
    double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point);
  void getNearestObstacleByDynamicObject(
    double * min_dist_to_obj, geometry_msgs::msg::Point * nearest_obj_point);
  bool isObstacleFound(const double min_dist_to_obj);
  bool isStopRequired(const bool is_obstacle_found, const bool is_stopped);
  size_t getClosestIdx(const TrajectoryPoints & traj, const geometry_msgs::msg::Pose current_pose);
  bool checkStop(const TrajectoryPoint & closest_point);
  Polygon2d createSelfPolygon();
  Polygon2d createObjPolygon(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Vector3 & size);
  Polygon2d createObjPolygon(
    const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Polygon & footprint);
  diagnostic_msgs::msg::DiagnosticStatus makeStopReasonDiag(
    const std::string no_start_reason, const geometry_msgs::msg::Pose & stop_pose);
  std::string jsonDumpsPose(const geometry_msgs::msg::Pose & pose);

  /*
   * ROS
   */
  // publisher and subscriber
  rclcpp::Subscription<Trajectory>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<autoware_auto_perception_msgs::msg::PredictedObjects>::SharedPtr
    dynamic_object_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_velocity_sub_;
  rclcpp::Publisher<Trajectory>::SharedPtr path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr stop_reason_diag_pub_;
  std::shared_ptr<SurroundObstacleCheckerDebugNode> debug_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // parameter
  nav_msgs::msg::Odometry::ConstSharedPtr current_velocity_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_ptr_;
  autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr object_ptr_;
  vehicle_info_util::VehicleInfo vehicle_info_;
  Polygon2d self_poly_;
  bool use_pointcloud_;
  bool use_dynamic_object_;
  double surround_check_distance_;
  // For preventing chattering,
  // surround_check_recover_distance_ must be  bigger than surround_check_distance_
  double surround_check_recover_distance_;
  double state_clear_time_;
  double stop_state_ego_speed_;
  bool is_surround_obstacle_;

  // State Machine
  State state_ = State::PASS;
  std::shared_ptr<const rclcpp::Time> last_obstacle_found_time_;
};

#endif  // SURROUND_OBSTACLE_CHECKER__NODE_HPP_
