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

#include "autoware/map_height_fitter/map_height_fitter.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>

#include <memory>

namespace autoware::map_height_fitter
{

struct MapHeightFitter::Impl
{
  static constexpr char enable_partial_load[] = "enable_partial_load";

  explicit Impl(rclcpp::Node * node);
  void on_pcd_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void on_vector_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  bool get_partial_point_cloud_map(const Point & point);
  double get_ground_height(const Point & point) const;
  std::optional<Point> fit(const Point & position, const std::string & frame);

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  std::string map_frame_;
  rclcpp::Node * node_;

  std::string fit_target_;

  // for fitting by pointcloud_map_loader
  rclcpp::CallbackGroup::SharedPtr group_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  rclcpp::Client<autoware_map_msgs::srv::GetPartialPointCloudMap>::SharedPtr cli_pcd_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcd_map_;
  rclcpp::AsyncParametersClient::SharedPtr params_pcd_map_loader_;

  // for fitting by vector_map_loader
  lanelet::LaneletMapPtr vector_map_;
  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_vector_map_;
};

MapHeightFitter::Impl::Impl(rclcpp::Node * node) : tf2_listener_(tf2_buffer_), node_(node)
{
  fit_target_ = node->declare_parameter<std::string>("map_height_fitter.target");
  if (fit_target_ == "pointcloud_map") {
    const auto callback =
      [this](const std::shared_future<std::vector<rclcpp::Parameter>> & future) {
        bool partial_load = false;
        for (const auto & param : future.get()) {
          if (param.get_name() == enable_partial_load) {
            partial_load = param.as_bool();
          }
        }

        if (partial_load) {
          group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          cli_pcd_map_ = node_->create_client<autoware_map_msgs::srv::GetPartialPointCloudMap>(
            "~/partial_map_load", rmw_qos_profile_default, group_);
        } else {
          const auto durable_qos = rclcpp::QoS(1).transient_local();
          sub_pcd_map_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "~/pointcloud_map", durable_qos,
            std::bind(&MapHeightFitter::Impl::on_pcd_map, this, std::placeholders::_1));
        }
      };

    const auto map_loader_name =
      node->declare_parameter<std::string>("map_height_fitter.map_loader_name");
    params_pcd_map_loader_ = rclcpp::AsyncParametersClient::make_shared(node, map_loader_name);
    params_pcd_map_loader_->wait_for_service();
    params_pcd_map_loader_->get_parameters({enable_partial_load}, callback);

  } else if (fit_target_ == "vector_map") {
    const auto durable_qos = rclcpp::QoS(1).transient_local();
    sub_vector_map_ = node_->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "~/vector_map", durable_qos,
      std::bind(&MapHeightFitter::Impl::on_vector_map, this, std::placeholders::_1));

  } else {
    throw std::runtime_error("invalid fit_target");
  }
}

void MapHeightFitter::Impl::on_pcd_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *map_cloud_);
}

bool MapHeightFitter::Impl::get_partial_point_cloud_map(const Point & point)
{
  const auto logger = node_->get_logger();

  if (!cli_pcd_map_) {
    RCLCPP_WARN_STREAM(logger, "Partial map loading in pointcloud_map_loader is not enabled");
    return false;
  }
  if (!cli_pcd_map_->service_is_ready()) {
    RCLCPP_WARN_STREAM(logger, "Partial map loading in pointcloud_map_loader is not ready");
    return false;
  }

  const auto req = std::make_shared<autoware_map_msgs::srv::GetPartialPointCloudMap::Request>();
  req->area.center_x = static_cast<float>(point.x);
  req->area.center_y = static_cast<float>(point.y);
  req->area.radius = 50;

  RCLCPP_DEBUG(logger, "Send request to map_loader");
  auto future = cli_pcd_map_->async_send_request(req);
  auto status = future.wait_for(std::chrono::seconds(1));
  while (status != std::future_status::ready) {
    RCLCPP_DEBUG(logger, "waiting response");
    if (!rclcpp::ok()) {
      return false;
    }
    status = future.wait_for(std::chrono::seconds(1));
  }

  const auto res = future.get();
  RCLCPP_DEBUG(
    logger, "Loaded partial pcd map from map_loader (grid size: %lu)",
    res->new_pointcloud_with_ids.size());

  sensor_msgs::msg::PointCloud2 pcd_msg;
  for (const auto & pcd_with_id : res->new_pointcloud_with_ids) {
    if (pcd_msg.width == 0) {
      pcd_msg = pcd_with_id.pointcloud;
    } else {
      pcd_msg.width += pcd_with_id.pointcloud.width;
      pcd_msg.row_step += pcd_with_id.pointcloud.row_step;
      pcd_msg.data.insert(
        pcd_msg.data.end(), pcd_with_id.pointcloud.data.begin(), pcd_with_id.pointcloud.data.end());
    }
  }
  map_frame_ = res->header.frame_id;
  map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(pcd_msg, *map_cloud_);
  return true;
}

void MapHeightFitter::Impl::on_vector_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  vector_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, vector_map_);
  map_frame_ = msg->header.frame_id;
}

double MapHeightFitter::Impl::get_ground_height(const Point & point) const
{
  const auto logger = node_->get_logger();

  const double x = point.x;
  const double y = point.y;

  double height = INFINITY;
  if (fit_target_ == "pointcloud_map") {
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

    for (const auto & p : map_cloud_->points) {
      const double dx = x - p.x;
      const double dy = y - p.y;
      const double sd = (dx * dx) + (dy * dy);
      if (sd < radius2) {
        height = std::min(height, static_cast<double>(p.z));
      }
    }
  } else if (fit_target_ == "vector_map") {
    const auto closest_points = vector_map_->pointLayer.nearest(lanelet::BasicPoint2d{x, y}, 1);
    if (closest_points.empty()) {
      RCLCPP_WARN_STREAM(logger, "failed to get closest lanelet");
      return point.z;
    }
    height = closest_points.front().z();
  }

  return std::isfinite(height) ? height : point.z;
}

std::optional<Point> MapHeightFitter::Impl::fit(const Point & position, const std::string & frame)
{
  const auto logger = node_->get_logger();
  RCLCPP_INFO_STREAM(logger, "fit_target: " << fit_target_ << ", frame: " << frame);

  Point point;
  point.x = position.x;
  point.y = position.y;
  point.z = position.z;

  RCLCPP_DEBUG(logger, "original point: %.3f %.3f %.3f", point.x, point.y, point.z);

  // prepare data
  if (fit_target_ == "pointcloud_map") {
    if (cli_pcd_map_) {  // if cli_pcd_map_ is available, prepare pointcloud map by partial loading
      if (!get_partial_point_cloud_map(position)) {
        RCLCPP_WARN_STREAM(logger, "failed to get partial point cloud map");
        return std::nullopt;
      }
    }  // otherwise, pointcloud map should be already prepared by on_pcd_map
    if (!map_cloud_) {
      RCLCPP_WARN_STREAM(logger, "point cloud map is not ready");
      return std::nullopt;
    }
  } else if (fit_target_ == "vector_map") {
    // vector_map_ should be already prepared by on_vector_map
    if (!vector_map_) {
      RCLCPP_WARN_STREAM(logger, "vector map is not ready");
      return std::nullopt;
    }
  } else {
    throw std::runtime_error("invalid fit_target");
  }

  // transform frame to map_frame_
  try {
    const auto stamped = tf2_buffer_.lookupTransform(frame, map_frame_, tf2::TimePointZero);
    tf2::doTransform(point, point, stamped);
  } catch (tf2::TransformException & exception) {
    RCLCPP_WARN_STREAM(logger, "failed to lookup transform: " << exception.what());
    return std::nullopt;
  }

  // fit height on map_frame_
  point.z = get_ground_height(point);

  // transform map_frame_ to frame
  try {
    const auto stamped = tf2_buffer_.lookupTransform(map_frame_, frame, tf2::TimePointZero);
    tf2::doTransform(point, point, stamped);
  } catch (tf2::TransformException & exception) {
    RCLCPP_WARN_STREAM(logger, "failed to lookup transform: " << exception.what());
    return std::nullopt;
  }

  RCLCPP_DEBUG(logger, "modified point: %.3f %.3f %.3f", point.x, point.y, point.z);

  return point;
}

MapHeightFitter::MapHeightFitter(rclcpp::Node * node)
{
  impl_ = std::make_unique<Impl>(node);
}

MapHeightFitter::~MapHeightFitter() = default;

std::optional<Point> MapHeightFitter::fit(const Point & position, const std::string & frame)
{
  return impl_->fit(position, frame);
}

}  // namespace autoware::map_height_fitter
