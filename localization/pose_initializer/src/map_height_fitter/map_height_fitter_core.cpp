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

MapHeightFitter::MapHeightFitter() : Node("map_height_fitter"), tf2_listener_(tf2_buffer_)
{
  const auto durable_qos = rclcpp::QoS(1).transient_local();
  using std::placeholders::_1;
  using std::placeholders::_2;

  sub_map_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "pointcloud_map", durable_qos, std::bind(&MapHeightFitter::on_map, this, _1));
  srv_fit_ = create_service<RequestHeightFitting>(
    "fit_map_height", std::bind(&MapHeightFitter::on_fit, this, _1, _2));
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

void MapHeightFitter::on_fit(
  const RequestHeightFitting::Request::SharedPtr req,
  const RequestHeightFitting::Response::SharedPtr res) const
{
  const auto & position = req->pose_with_covariance.pose.pose.position;
  tf2::Vector3 point(position.x, position.y, position.z);
  std::string req_frame = req->pose_with_covariance.header.frame_id;
  res->success = false;

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
