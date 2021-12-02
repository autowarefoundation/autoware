// Copyright 2015-2019 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
#define NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_

#define FMT_HEADER_ONLY

#include "ndt_scan_matcher/particle.hpp"

#include <ndt/omp.hpp>
#include <ndt/pcl_generic.hpp>
#include <ndt/pcl_modified.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_debug_msgs/msg/int32_stamped.hpp>
#include <autoware_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <array>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

enum class NDTImplementType { PCL_GENERIC = 0, PCL_MODIFIED = 1, OMP = 2 };

template <typename PointSource, typename PointTarget>
std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> getNDT(
  const NDTImplementType & ndt_mode)
{
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr;
  if (ndt_mode == NDTImplementType::PCL_GENERIC) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLGeneric<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::PCL_MODIFIED) {
    ndt_ptr.reset(new NormalDistributionsTransformPCLModified<PointSource, PointTarget>);
    return ndt_ptr;
  }
  if (ndt_mode == NDTImplementType::OMP) {
    ndt_ptr.reset(new NormalDistributionsTransformOMP<PointSource, PointTarget>);
    return ndt_ptr;
  }

  const std::string s = fmt::format("Unknown NDT type {}", static_cast<int>(ndt_mode));
  throw std::runtime_error(s);
}

class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;

  // TODO(Tier IV): move file
  struct OMPParams
  {
    OMPParams() : search_method(pclomp::NeighborSearchMethod::KDTREE), num_threads(1) {}
    pclomp::NeighborSearchMethod search_method;
    int num_threads;
  };

public:
  NDTScanMatcher();

private:
  void serviceNDTAlign(
    const autoware_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    autoware_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  void callbackMapPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callbackSensorPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void callbackInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr);

  geometry_msgs::msg::PoseWithCovarianceStamped alignUsingMonteCarlo(
    const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> & ndt_ptr,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);

  void updateTransforms();

  void publishTF(
    const std::string & child_frame_id, const geometry_msgs::msg::PoseStamped & pose_msg);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const geometry_msgs::msg::TransformStamped::SharedPtr & transform_stamped_ptr);

  void timerDiagnostic();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_with_covariance_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr exe_time_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr transform_probability_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Int32Stamped>::SharedPtr iteration_num_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_old_pub_;
  rclcpp::Publisher<autoware_debug_msgs::msg::Float32Stamped>::SharedPtr
    initial_to_result_distance_new_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ndt_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

  rclcpp::Service<autoware_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr service_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  NDTImplementType ndt_implement_type_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  std::string base_frame_;
  std::string ndt_base_frame_;
  std::string map_frame_;
  double converged_param_transform_probability_;
  float inversion_vector_threshold_;
  float oscillation_threshold_;

  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr>
    initial_pose_msg_ptr_array_;
  std::mutex ndt_map_mtx_;

  OMPParams omp_params_;

  std::thread diagnostic_thread_;
  std::map<std::string, std::string> key_value_stdmap_;
};

#endif  // NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_CORE_HPP_
