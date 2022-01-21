// Copyright 2020 Autoware Foundation
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

#include "pose_initializer/pose_initializer_core.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

double getGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap, const tf2::Vector3 & point)
{
  constexpr double radius = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = INFINITY;
  for (const auto & p : pcdmap->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }
  return std::isfinite(height) ? height : point.getZ();
}

PoseInitializer::PoseInitializer()
: Node("pose_initializer"), tf2_listener_(tf2_buffer_), map_frame_("map")
{
  enable_gnss_callback_ = this->declare_parameter("enable_gnss_callback", true);

  std::vector<double> initialpose_particle_covariance =
    this->declare_parameter<std::vector<double>>("initialpose_particle_covariance");
  for (std::size_t i = 0; i < initialpose_particle_covariance.size(); ++i) {
    initialpose_particle_covariance_[i] = initialpose_particle_covariance[i];
  }

  std::vector<double> gnss_particle_covariance =
    this->declare_parameter<std::vector<double>>("gnss_particle_covariance");
  for (std::size_t i = 0; i < gnss_particle_covariance.size(); ++i) {
    gnss_particle_covariance_[i] = gnss_particle_covariance[i];
  }

  std::vector<double> service_particle_covariance =
    this->declare_parameter<std::vector<double>>("service_particle_covariance");
  for (std::size_t i = 0; i < service_particle_covariance.size(); ++i) {
    service_particle_covariance_[i] = service_particle_covariance[i];
  }

  std::vector<double> output_pose_covariance =
    this->declare_parameter<std::vector<double>>("output_pose_covariance");
  for (std::size_t i = 0; i < output_pose_covariance.size(); ++i) {
    output_pose_covariance_[i] = output_pose_covariance[i];
  }

  // We can't use _1 because pcl leaks an alias to boost::placeholders::_1, so it would be ambiguous
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&PoseInitializer::callbackInitialPose, this, std::placeholders::_1));
  map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&PoseInitializer::callbackMapPoints, this, std::placeholders::_1));
  gnss_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", 1,
    std::bind(&PoseInitializer::callbackGNSSPoseCov, this, std::placeholders::_1));
  pose_initialization_request_sub_ =
    this->create_subscription<tier4_localization_msgs::msg::PoseInitializationRequest>(
      "pose_initialization_request", rclcpp::QoS{1}.transient_local(),
      std::bind(&PoseInitializer::callbackPoseInitializationRequest, this, std::placeholders::_1));

  initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose3d", 10);

  ndt_client_ =
    this->create_client<tier4_localization_msgs::srv::PoseWithCovarianceStamped>("ndt_align_srv");
  while (!ndt_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }

  initialize_pose_service_ =
    this->create_service<tier4_localization_msgs::srv::PoseWithCovarianceStamped>(
      "service/initialize_pose", std::bind(
                                   &PoseInitializer::serviceInitializePose, this,
                                   std::placeholders::_1, std::placeholders::_2));

  initialize_pose_auto_service_ =
    this->create_service<tier4_external_api_msgs::srv::InitializePoseAuto>(
      "service/initialize_pose_auto", std::bind(
                                        &PoseInitializer::serviceInitializePoseAuto, this,
                                        std::placeholders::_1, std::placeholders::_2));
}

void PoseInitializer::callbackMapPoints(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_points_msg_ptr)
{
  std::string map_frame_ = map_points_msg_ptr->header.frame_id;
  map_ptr_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_ptr_);
}

void PoseInitializer::serviceInitializePose(
  const tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  tier4_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  enable_gnss_callback_ = false;  // get only first topic

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(req->pose_with_covariance, add_height_pose_msg_ptr);

  add_height_pose_msg_ptr->pose.covariance = service_particle_covariance_;

  res->success = callAlignServiceAndPublishResult(add_height_pose_msg_ptr);
}

void PoseInitializer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr)
{
  enable_gnss_callback_ = false;  // get only first topic

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  add_height_pose_msg_ptr->pose.covariance = initialpose_particle_covariance_;

  callAlignServiceAndPublishResult(add_height_pose_msg_ptr);
}

// NOTE Still not usable callback
void PoseInitializer::callbackGNSSPoseCov(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr)
{
  if (!enable_gnss_callback_) {
    return;
  }

  // TODO(YamatoAndo) check service is available

  auto add_height_pose_msg_ptr = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  add_height_pose_msg_ptr->pose.covariance = gnss_particle_covariance_;

  callAlignServiceAndPublishResult(add_height_pose_msg_ptr);
}

void PoseInitializer::serviceInitializePoseAuto(
  const tier4_external_api_msgs::srv::InitializePoseAuto::Request::SharedPtr req,
  tier4_external_api_msgs::srv::InitializePoseAuto::Response::SharedPtr res)
{
  (void)req;

  RCLCPP_INFO(this->get_logger(), "Called Pose Initialize Service");
  enable_gnss_callback_ = true;
  res->status = tier4_api_utils::response_success();
}

void PoseInitializer::callbackPoseInitializationRequest(
  const tier4_localization_msgs::msg::PoseInitializationRequest::ConstSharedPtr request_msg_ptr)
{
  RCLCPP_INFO(this->get_logger(), "Called Pose Initialize");
  enable_gnss_callback_ = request_msg_ptr->data;
}

bool PoseInitializer::getHeight(
  const geometry_msgs::msg::PoseWithCovarianceStamped & input_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr output_pose_msg_ptr)
{
  std::string fixed_frame = input_pose_msg.header.frame_id;
  tf2::Vector3 point(
    input_pose_msg.pose.pose.position.x, input_pose_msg.pose.pose.position.y,
    input_pose_msg.pose.pose.position.z);

  if (map_ptr_) {
    tf2::Transform transform;
    try {
      const auto stamped = tf2_buffer_.lookupTransform(map_frame_, fixed_frame, tf2::TimePointZero);
      tf2::fromMsg(stamped.transform, transform);
    } catch (tf2::TransformException & exception) {
      RCLCPP_WARN_STREAM(get_logger(), "failed to lookup transform: " << exception.what());
    }

    point = transform * point;
    point.setZ(getGroundHeight(map_ptr_, point));
    point = transform.inverse() * point;
  }

  *output_pose_msg_ptr = input_pose_msg;
  output_pose_msg_ptr->pose.pose.position.x = point.getX();
  output_pose_msg_ptr->pose.pose.position.y = point.getY();
  output_pose_msg_ptr->pose.pose.position.z = point.getZ();

  return true;
}

bool PoseInitializer::callAlignServiceAndPublishResult(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr input_pose_msg)
{
  if (request_id_ != response_id_) {
    RCLCPP_ERROR(get_logger(), "Did not receive response for previous NDT Align Server call");
    return false;
  }
  auto req = std::make_shared<tier4_localization_msgs::srv::PoseWithCovarianceStamped::Request>();
  req->pose_with_covariance = *input_pose_msg;
  req->seq = ++request_id_;

  RCLCPP_INFO(get_logger(), "call NDT Align Server");

  ndt_client_->async_send_request(
    req,
    [this](rclcpp::Client<tier4_localization_msgs::srv::PoseWithCovarianceStamped>::SharedFuture
             result) {
      if (result.get()->success) {
        RCLCPP_INFO(get_logger(), "called NDT Align Server");
        response_id_ = result.get()->seq;
        // NOTE temporary cov
        geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_covariance =
          result.get()->pose_with_covariance;
        pose_with_covariance.pose.covariance = output_pose_covariance_;
        initial_pose_pub_->publish(pose_with_covariance);
        enable_gnss_callback_ = false;
      } else {
        RCLCPP_INFO(get_logger(), "failed NDT Align Server");
        response_id_ = result.get()->seq;
      }
    });
  return true;
}
