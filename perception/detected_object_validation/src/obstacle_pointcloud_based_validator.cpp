// Copyright 2022 Tier IV, Inc.
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

#include "obstacle_pointcloud_based_validator/obstacle_pointcloud_based_validator.hpp"

#include <perception_utils/perception_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <boost/geometry.hpp>

#include <pcl/filters/crop_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace
{
inline pcl::PointXY toPCL(const double x, const double y)
{
  pcl::PointXY pcl_point;
  pcl_point.x = x;
  pcl_point.y = y;
  return pcl_point;
}

inline pcl::PointXY toPCL(const geometry_msgs::msg::Point & point)
{
  return toPCL(point.x, point.y);
}

inline pcl::PointXYZ toXYZ(const pcl::PointXY & point)
{
  return pcl::PointXYZ(point.x, point.y, 0.0);
}

inline pcl::PointCloud<pcl::PointXYZ>::Ptr toXYZ(
  const pcl::PointCloud<pcl::PointXY>::Ptr & pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pointcloud_xyz->reserve(pointcloud->size());
  for (const auto & point : *pointcloud) {
    pointcloud_xyz->push_back(toXYZ(point));
  }
  return pointcloud_xyz;
}

}  // namespace

namespace obstacle_pointcloud_based_validator
{
namespace bg = boost::geometry;
using Shape = autoware_auto_perception_msgs::msg::Shape;
using Polygon2d = tier4_autoware_utils::Polygon2d;

ObstaclePointCloudBasedValidator::ObstaclePointCloudBasedValidator(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("obstacle_pointcloud_based_validator", node_options),
  objects_sub_(this, "~/input/detected_objects", rclcpp::QoS{1}.get_rmw_qos_profile()),
  obstacle_pointcloud_sub_(
    this, "~/input/obstacle_pointcloud",
    rclcpp::SensorDataQoS{}.keep_last(1).get_rmw_qos_profile()),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  sync_(SyncPolicy(10), objects_sub_, obstacle_pointcloud_sub_)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_.registerCallback(
    std::bind(&ObstaclePointCloudBasedValidator::onObjectsAndObstaclePointCloud, this, _1, _2));
  objects_pub_ = create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "~/output/objects", rclcpp::QoS{1});

  min_pointcloud_num_ = declare_parameter<int>("min_pointcloud_num", 10);

  const bool enable_debugger = declare_parameter<bool>("enable_debugger", false);
  if (enable_debugger) debugger_ = std::make_shared<Debugger>(this);
}

void ObstaclePointCloudBasedValidator::onObjectsAndObstaclePointCloud(
  const autoware_auto_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_obstacle_pointcloud)
{
  autoware_auto_perception_msgs::msg::DetectedObjects output, removed_objects;
  output.header = input_objects->header;
  removed_objects.header = input_objects->header;

  // Transform to pointcloud frame
  autoware_auto_perception_msgs::msg::DetectedObjects transformed_objects;
  if (!perception_utils::transformObjects(
        *input_objects, input_obstacle_pointcloud->header.frame_id, tf_buffer_,
        transformed_objects)) {
    // objects_pub_->publish(*input_objects);
    return;
  }

  // Convert to PCL
  pcl::PointCloud<pcl::PointXY>::Ptr obstacle_pointcloud(new pcl::PointCloud<pcl::PointXY>);
  pcl::fromROSMsg(*input_obstacle_pointcloud, *obstacle_pointcloud);
  if (obstacle_pointcloud->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5, "cannot receive pointcloud");
    // objects_pub_->publish(*input_objects);
    return;
  }

  // Create Kd-tree to search neighbor pointcloud to reduce cost.
  pcl::search::Search<pcl::PointXY>::Ptr kdtree =
    pcl::make_shared<pcl::search::KdTree<pcl::PointXY>>(false);
  kdtree->setInputCloud(obstacle_pointcloud);

  for (size_t i = 0; i < transformed_objects.objects.size(); ++i) {
    const auto & transformed_object = transformed_objects.objects.at(i);
    const auto & object = input_objects->objects.at(i);
    const auto search_radius = getMaxRadius(transformed_object);
    if (!search_radius) {
      output.objects.push_back(object);
      continue;
    }

    // Search neighbor pointcloud to reduce cost.
    pcl::PointCloud<pcl::PointXY>::Ptr neighbor_pointcloud(new pcl::PointCloud<pcl::PointXY>);
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree->radiusSearch(
      toPCL(transformed_object.kinematics.pose_with_covariance.pose.position),
      search_radius.value(), indices, distances);
    for (const auto & index : indices) {
      neighbor_pointcloud->push_back(obstacle_pointcloud->at(index));
    }
    if (debugger_) debugger_->addNeighborPointcloud(neighbor_pointcloud);

    // Filter object that have few pointcloud in them.
    const auto num = getPointCloudNumWithinPolygon(transformed_object, neighbor_pointcloud);
    if (num) {
      (min_pointcloud_num_ <= num.value()) ? output.objects.push_back(object)
                                           : removed_objects.objects.push_back(object);
    } else {
      output.objects.push_back(object);
    }
  }

  objects_pub_->publish(output);
  if (debugger_) {
    debugger_->publishRemovedObjects(removed_objects);
    debugger_->publishNeighborPointcloud(input_obstacle_pointcloud->header);
    debugger_->publishPointcloudWithinPolygon(input_obstacle_pointcloud->header);
  }
}

std::optional<size_t> ObstaclePointCloudBasedValidator::getPointCloudNumWithinPolygon(
  const autoware_auto_perception_msgs::msg::DetectedObject & object,
  const pcl::PointCloud<pcl::PointXY>::Ptr pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> vertices_array;
  pcl::Vertices vertices;

  Polygon2d poly2d =
    tier4_autoware_utils::toPolygon2d(object.kinematics.pose_with_covariance.pose, object.shape);
  if (bg::is_empty(poly2d)) return std::nullopt;

  pcl::PointCloud<pcl::PointXYZ>::Ptr poly3d(new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < poly2d.outer().size(); ++i) {
    vertices.vertices.emplace_back(i);
    vertices_array.emplace_back(vertices);
    poly3d->emplace_back(poly2d.outer().at(i).x(), poly2d.outer().at(i).y(), 0.0);
  }

  pcl::CropHull<pcl::PointXYZ> cropper;  // don't be implemented PointXY by PCL
  cropper.setInputCloud(toXYZ(pointcloud));
  cropper.setDim(2);
  cropper.setHullIndices(vertices_array);
  cropper.setHullCloud(poly3d);
  cropper.setCropOutside(true);
  cropper.filter(*cropped_pointcloud);

  if (debugger_) debugger_->addPointcloudWithinPolygon(cropped_pointcloud);
  return cropped_pointcloud->size();
}

std::optional<float> ObstaclePointCloudBasedValidator::getMaxRadius(
  const autoware_auto_perception_msgs::msg::DetectedObject & object)
{
  if (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::CYLINDER) {
    return std::hypot(object.shape.dimensions.x * 0.5f, object.shape.dimensions.y * 0.5f);
  } else if (object.shape.type == Shape::POLYGON) {
    float max_dist = 0.0;
    for (const auto & point : object.shape.footprint.points) {
      const float dist = std::hypot(point.x, point.y);
      max_dist = max_dist < dist ? dist : max_dist;
    }
    return max_dist;
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "unknown shape type");
    return std::nullopt;
  }
}

}  // namespace obstacle_pointcloud_based_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  obstacle_pointcloud_based_validator::ObstaclePointCloudBasedValidator)
