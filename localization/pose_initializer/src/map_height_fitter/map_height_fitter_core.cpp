// Copyright 2022 The Autoware Contributors
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

#include "map_height_fitter_core.hpp"

#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <memory>

MapHeightFitter::MapHeightFitter() : Node("map_height_fitter"), tf2_listener_(tf2_buffer_)
{
  enable_partial_map_load_ = declare_parameter<bool>("enable_partial_map_load", false);

  const auto durable_qos = rclcpp::QoS(1).transient_local();
  using std::placeholders::_1;
  using std::placeholders::_2;

  srv_fit_ = create_service<RequestHeightFitting>(
    "fit_map_height", std::bind(&MapHeightFitter::on_fit, this, _1, _2));

  if (!enable_partial_map_load_) {
    sub_map_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_map", durable_qos, std::bind(&MapHeightFitter::on_map, this, _1));
  } else {
    callback_group_service_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cli_get_partial_pcd_ = create_client<autoware_map_msgs::srv::GetPartialPointCloudMap>(
      "client_partial_map_load", rmw_qos_profile_default, callback_group_service_);
    while (!cli_get_partial_pcd_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Cannot find partial map loading interface. Please check the setting in "
        "pointcloud_map_loader to see if the interface is enabled.");
    }
  }
}

void MapHeightFitter::on_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *map_cloud_);
}

double MapHeightFitter::get_ground_height(const tf2::Vector3 & point) const
{
  const double x = point.getX();
  const double y = point.getY();

  // find distance d to closest point
  double min_dist2 = INFINITY;
  for (const auto & p : map_cloud_->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    min_dist2 = std::min(min_dist2, sd);
  }

  // find lowest height within radius (d+1.0)
  const double radius2 = std::pow(std::sqrt(min_dist2) + 1.0, 2.0);
  double height = INFINITY;
  for (const auto & p : map_cloud_->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius2) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }

  return std::isfinite(height) ? height : point.getZ();
}

void MapHeightFitter::get_partial_point_cloud_map(const geometry_msgs::msg::Point & point)
{
  if (!cli_get_partial_pcd_) {
    throw std::runtime_error{"Partial map loading in pointcloud_map_loader is not enabled"};
  }
  const auto req = std::make_shared<autoware_map_msgs::srv::GetPartialPointCloudMap::Request>();
  req->area.center = point;
  req->area.radius = 50;

  RCLCPP_INFO(this->get_logger(), "Send request to map_loader");
  auto res{cli_get_partial_pcd_->async_send_request(
    req, [](rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedFuture) {})};

  std::future_status status = res.wait_for(std::chrono::seconds(0));
  while (status != std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "waiting response");
    if (!rclcpp::ok()) {
      return;
    }
    status = res.wait_for(std::chrono::seconds(1));
  }

  RCLCPP_INFO(
    this->get_logger(), "Loaded partial pcd map from map_loader (grid size: %d)",
    static_cast<int>(res.get()->new_pointcloud_with_ids.size()));

  sensor_msgs::msg::PointCloud2 pcd_msg;
  for (const auto & pcd_with_id : res.get()->new_pointcloud_with_ids) {
    if (pcd_msg.width == 0) {
      pcd_msg = pcd_with_id.pointcloud;
    } else {
      pcd_msg.width += pcd_with_id.pointcloud.width;
      pcd_msg.row_step += pcd_with_id.pointcloud.row_step;
      pcd_msg.data.insert(
        pcd_msg.data.end(), pcd_with_id.pointcloud.data.begin(), pcd_with_id.pointcloud.data.end());
    }
  }

  map_frame_ = res.get()->header.frame_id;
  map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(pcd_msg, *map_cloud_);
}

void MapHeightFitter::on_fit(
  const RequestHeightFitting::Request::SharedPtr req,
  const RequestHeightFitting::Response::SharedPtr res)
{
  const auto & position = req->pose_with_covariance.pose.pose.position;
  tf2::Vector3 point(position.x, position.y, position.z);
  std::string req_frame = req->pose_with_covariance.header.frame_id;
  res->success = false;

  if (enable_partial_map_load_) {
    get_partial_point_cloud_map(req->pose_with_covariance.pose.pose.position);
  }

  if (map_cloud_) {
    try {
      const auto stamped = tf2_buffer_.lookupTransform(map_frame_, req_frame, tf2::TimePointZero);
      tf2::Transform transform{tf2::Quaternion{}, tf2::Vector3{}};
      tf2::fromMsg(stamped.transform, transform);
      point = transform * point;
      point.setZ(get_ground_height(point));
      point = transform.inverse() * point;
      res->success = true;
    } catch (tf2::TransformException & exception) {
      RCLCPP_WARN_STREAM(get_logger(), "failed to lookup transform: " << exception.what());
    }
  }

  res->pose_with_covariance = req->pose_with_covariance;
  res->pose_with_covariance.pose.pose.position.x = point.getX();
  res->pose_with_covariance.pose.pose.position.y = point.getY();
  res->pose_with_covariance.pose.pose.position.z = point.getZ();
}
