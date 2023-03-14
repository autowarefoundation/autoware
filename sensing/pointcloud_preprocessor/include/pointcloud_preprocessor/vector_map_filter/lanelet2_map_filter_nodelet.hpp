// Copyright 2020 Tier IV, Inc.
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

#ifndef POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__LANELET2_MAP_FILTER_NODELET_HPP_
#define POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__LANELET2_MAP_FILTER_NODELET_HPP_

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/boost_geometry.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

using tier4_autoware_utils::LinearRing2d;
using tier4_autoware_utils::MultiPoint2d;
using tier4_autoware_utils::Point2d;

namespace pointcloud_preprocessor
{
class Lanelet2MapFilterComponent : public rclcpp::Node
{
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2Ptr = sensor_msgs::msg::PointCloud2::SharedPtr;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

public:
  explicit Lanelet2MapFilterComponent(const rclcpp::NodeOptions & options);

private:
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr filtered_pointcloud_pub_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::ConstLanelets road_lanelets_;

  float voxel_size_x_;
  float voxel_size_y_;

  void pointcloudCallback(const PointCloud2ConstPtr msg);

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg);

  bool transformPointCloud(
    const std::string & in_target_frame, const PointCloud2ConstPtr & in_cloud_ptr,
    PointCloud2 * out_cloud_ptr);

  LinearRing2d getConvexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud);

  lanelet::ConstLanelets getIntersectedLanelets(
    const LinearRing2d & convex_hull, const lanelet::ConstLanelets & road_lanelets_);

  pcl::PointCloud<pcl::PointXYZ> getLaneFilteredPointCloud(
    const lanelet::ConstLanelets & joint_lanelets,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

  bool pointWithinLanelets(const Point2d & point, const lanelet::ConstLanelets & joint_lanelets);

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);
};

}  // namespace pointcloud_preprocessor

#endif  // POINTCLOUD_PREPROCESSOR__VECTOR_MAP_FILTER__LANELET2_MAP_FILTER_NODELET_HPP_
