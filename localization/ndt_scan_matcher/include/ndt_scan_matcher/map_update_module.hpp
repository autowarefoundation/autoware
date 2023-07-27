// Copyright 2022 Autoware Foundation
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

#ifndef NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
#define NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_

#include "ndt_scan_matcher/debug.hpp"
#include "ndt_scan_matcher/particle.hpp"
#include "ndt_scan_matcher/tf2_listener_module.hpp"
#include "ndt_scan_matcher/util_func.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>
#include <tier4_autoware_utils/transform/transforms.hpp>

#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fmt/format.h>
#include <multigrid_pclomp/multigrid_ndt_omp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class MapUpdateModule
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform =
    pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

public:
  MapUpdateModule(
    rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
    std::shared_ptr<NormalDistributionsTransform> ndt_ptr,
    std::shared_ptr<Tf2ListenerModule> tf2_listener_module, std::string map_frame,
    rclcpp::CallbackGroup::SharedPtr main_callback_group,
    std::shared_ptr<std::map<std::string, std::string>> state_ptr);

private:
  friend class NDTScanMatcher;
  void callback_ekf_odom(nav_msgs::msg::Odometry::ConstSharedPtr odom_ptr);
  void map_update_timer_callback();

  void update_ndt(
    const std::vector<autoware_map_msgs::msg::PointCloudMapCellWithID> & maps_to_add,
    const std::vector<std::string> & map_ids_to_remove);
  void update_map(const geometry_msgs::msg::Point & position);
  [[nodiscard]] bool should_update_map(const geometry_msgs::msg::Point & position) const;
  void publish_partial_pcd_map();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialPointCloudMap>::SharedPtr
    pcd_loader_client_;
  rclcpp::TimerBase::SharedPtr map_update_timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_odom_sub_;

  rclcpp::CallbackGroup::SharedPtr map_callback_group_;

  std::shared_ptr<NormalDistributionsTransform> ndt_ptr_;
  std::mutex * ndt_ptr_mutex_;
  std::string map_frame_;
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module_;
  std::shared_ptr<std::map<std::string, std::string>> state_ptr_;

  std::optional<geometry_msgs::msg::Point> last_update_position_ = std::nullopt;
  std::optional<geometry_msgs::msg::Point> current_position_ = std::nullopt;
  const double dynamic_map_loading_update_distance_;
  const double dynamic_map_loading_map_radius_;
  const double lidar_radius_;
};

#endif  // NDT_SCAN_MATCHER__MAP_UPDATE_MODULE_HPP_
