// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_
#define AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>
#include <autoware_auto_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::motion::control::autonomous_emergency_braking
{

using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_system_msgs::msg::AutowareState;
using autoware_auto_vehicle_msgs::msg::VelocityReport;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using vehicle_info_util::VehicleInfo;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using Path = std::vector<geometry_msgs::msg::Pose>;
using Vector3 = geometry_msgs::msg::Vector3;

struct ObjectData
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point position;
  double velocity{0.0};
  double rss{0.0};
  double distance_to_object{0.0};
};

class CollisionDataKeeper
{
public:
  explicit CollisionDataKeeper(rclcpp::Clock::SharedPtr clock) { clock_ = clock; }

  void setTimeout(const double timeout_sec) { timeout_sec_ = timeout_sec; }

  bool checkExpired()
  {
    if (data_ && (clock_->now() - data_->stamp).seconds() > timeout_sec_) {
      data_.reset();
    }
    return (data_ == nullptr);
  }

  void update(const ObjectData & data) { data_.reset(new ObjectData(data)); }

  ObjectData get()
  {
    if (data_) {
      return *data_;
    } else {
      return ObjectData();
    }
  }

private:
  std::unique_ptr<ObjectData> data_;
  double timeout_sec_{0.0};
  rclcpp::Clock::SharedPtr clock_;
};

class AEB : public rclcpp::Node
{
public:
  explicit AEB(const rclcpp::NodeOptions & node_options);

  // subscriber
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Subscription<VelocityReport>::SharedPtr sub_velocity_;
  rclcpp::Subscription<Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_predicted_traj_;
  rclcpp::Subscription<AutowareState>::SharedPtr sub_autoware_state_;

  // publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_pointcloud_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_ego_path_publisher_;  // debug

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  // callback
  void onPointCloud(const PointCloud2::ConstSharedPtr input_msg);
  void onVelocity(const VelocityReport::ConstSharedPtr input_msg);
  void onImu(const Imu::ConstSharedPtr input_msg);
  void onTimer();
  void onPredictedTrajectory(const Trajectory::ConstSharedPtr input_msg);
  void onAutowareState(const AutowareState::ConstSharedPtr input_msg);

  bool isDataReady();

  // main function
  void onCheckCollision(DiagnosticStatusWrapper & stat);
  bool checkCollision(MarkerArray & debug_markers);
  bool hasCollision(
    const double current_v, const Path & ego_path, const std::vector<ObjectData> & objects);

  void generateEgoPath(
    const double curr_v, const double curr_w, Path & path, std::vector<Polygon2d> & polygons);
  void generateEgoPath(
    const Trajectory & predicted_traj, Path & path, std::vector<Polygon2d> & polygons);
  void createObjectData(
    const Path & ego_path, const std::vector<Polygon2d> & ego_polys, const rclcpp::Time & stamp,
    std::vector<ObjectData> & objects);

  void addMarker(
    const rclcpp::Time & current_time, const Path & path, const std::vector<Polygon2d> & polygons,
    const std::vector<ObjectData> & objects, const double color_r, const double color_g,
    const double color_b, const double color_a, const std::string & ns,
    MarkerArray & debug_markers);

  void addCollisionMarker(const ObjectData & data, MarkerArray & debug_markers);

  PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr_{nullptr};
  VelocityReport::ConstSharedPtr current_velocity_ptr_{nullptr};
  Vector3::SharedPtr angular_velocity_ptr_{nullptr};
  Trajectory::ConstSharedPtr predicted_traj_ptr_{nullptr};
  AutowareState::ConstSharedPtr autoware_state_{nullptr};

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // vehicle info
  VehicleInfo vehicle_info_;

  // diag
  Updater updater_{this};

  // member variables
  bool publish_debug_pointcloud_;
  bool use_predicted_trajectory_;
  bool use_imu_path_;
  double voxel_grid_x_;
  double voxel_grid_y_;
  double voxel_grid_z_;
  double min_generated_path_length_;
  double expand_width_;
  double longitudinal_offset_;
  double t_response_;
  double a_ego_min_;
  double a_obj_min_;
  double imu_prediction_time_horizon_;
  double imu_prediction_time_interval_;
  double mpc_prediction_time_horizon_;
  double mpc_prediction_time_interval_;
  CollisionDataKeeper collision_data_keeper_;
};
}  // namespace autoware::motion::control::autonomous_emergency_braking

#endif  // AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_
