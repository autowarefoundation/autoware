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

#ifndef NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_
#define NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/tf2_listener_module.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tier4_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>

#include <map>
#include <memory>
#include <string>

class PoseInitializationModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::NormalDistributionsTransform<PointSource, PointTarget>;

public:
  PoseInitializationModule(
    rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
    std::shared_ptr<NormalDistributionsTransform> ndt_ptr,
    std::shared_ptr<Tf2ListenerModule> tf2_listener_module, std::string map_frame,
    rclcpp::CallbackGroup::SharedPtr main_callback_group,
    std::shared_ptr<std::map<std::string, std::string>> state_ptr);

private:
  void service_ndt_align(
    const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
    tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res);

  geometry_msgs::msg::PoseWithCovarianceStamped align_using_monte_carlo(
    const std::shared_ptr<NormalDistributionsTransform> & ndt_ptr,
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov);
  void publish_point_cloud(
    const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const std::shared_ptr<const pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr);

  rclcpp::Service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    ndt_monte_carlo_initial_pose_marker_pub_;
  int initial_estimate_particles_num_;

  std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;
  std::mutex * ndt_ptr_mutex_;
  std::string map_frame_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module_;
  std::shared_ptr<std::map<std::string, std::string>> state_ptr_;
};

#endif  // NDT_SCAN_MATCHER__POSE_INITIALIZATION_MODULE_HPP_
