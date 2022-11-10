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

#include "ndt_scan_matcher/pose_initialization_module.hpp"

PoseInitializationModule::PoseInitializationModule(
  rclcpp::Node * node, std::mutex * ndt_ptr_mutex,
  std::shared_ptr<NormalDistributionsTransform> ndt_ptr,
  std::shared_ptr<Tf2ListenerModule> tf2_listener_module, std::string map_frame,
  rclcpp::CallbackGroup::SharedPtr main_callback_group,
  std::shared_ptr<std::map<std::string, std::string>> state_ptr)
: ndt_ptr_(ndt_ptr),
  ndt_ptr_mutex_(ndt_ptr_mutex),
  map_frame_(map_frame),
  logger_(node->get_logger()),
  clock_(node->get_clock()),
  tf2_listener_module_(tf2_listener_module),
  state_ptr_(state_ptr)
{
  initial_estimate_particles_num_ =
    node->declare_parameter("initial_estimate_particles_num", initial_estimate_particles_num_);

  sensor_aligned_pose_pub_ =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("monte_carlo_points_aligned", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  service_ = node->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
    "ndt_align_srv",
    std::bind(
      &PoseInitializationModule::service_ndt_align, this, std::placeholders::_1,
      std::placeholders::_2),
    rclcpp::ServicesQoS().get_rmw_qos_profile(), main_callback_group);
}

void PoseInitializationModule::service_ndt_align(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  auto TF_pose_to_map_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();
  tf2_listener_module_->get_transform(
    clock_->now(), map_frame_, req->pose_with_covariance.header.frame_id, TF_pose_to_map_ptr);

  // transform pose_frame to map_frame
  const auto mapTF_initial_pose_msg = transform(req->pose_with_covariance, *TF_pose_to_map_ptr);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    res->success = false;
    RCLCPP_WARN(logger_, "No InputTarget");
    return;
  }

  if (ndt_ptr_->getInputSource() == nullptr) {
    res->success = false;
    RCLCPP_WARN(logger_, "No InputSource");
    return;
  }

  // mutex Map
  std::lock_guard<std::mutex> lock(*ndt_ptr_mutex_);

  (*state_ptr_)["state"] = "Aligning";
  res->pose_with_covariance = align_using_monte_carlo(ndt_ptr_, mapTF_initial_pose_msg);
  (*state_ptr_)["state"] = "Sleeping";
  res->success = true;
  res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
}

geometry_msgs::msg::PoseWithCovarianceStamped PoseInitializationModule::align_using_monte_carlo(
  const std::shared_ptr<NormalDistributionsTransform> & ndt_ptr,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov)
{
  if (ndt_ptr->getInputTarget() == nullptr || ndt_ptr->getInputSource() == nullptr) {
    RCLCPP_WARN(logger_, "No Map or Sensor PointCloud");
    return geometry_msgs::msg::PoseWithCovarianceStamped();
  }

  // generateParticle
  const auto initial_poses =
    create_random_pose_array(initial_pose_with_cov, initial_estimate_particles_num_);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();

  for (unsigned int i = 0; i < initial_poses.size(); i++) {
    const auto & initial_pose = initial_poses[i];
    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
    ndt_ptr->align(*output_cloud, initial_pose_matrix);
    const pclomp::NdtResult ndt_result = ndt_ptr->getResult();

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose), ndt_result.transform_probability,
      ndt_result.iteration_num);
    particle_array.push_back(particle);
    const auto marker_array = make_debug_markers(
      clock_->now(), map_frame_, tier4_autoware_utils::createMarkerScale(0.3, 0.1, 0.1), particle,
      i);
    ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);

    auto sensor_points_mapTF_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    pcl::transformPointCloud(*ndt_ptr->getInputSource(), *sensor_points_mapTF_ptr, ndt_result.pose);
    publish_point_cloud(initial_pose_with_cov.header.stamp, map_frame_, sensor_points_mapTF_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;

  return result_pose_with_cov_msg;
}

void PoseInitializationModule::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const std::shared_ptr<const pcl::PointCloud<PointSource>> & sensor_points_mapTF_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);
}
