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
/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v1.0: amc-nu (abrahammonrroy@yahoo.com)
 */

#ifndef RAY_GROUND_FILTER__NODE_HPP_
#define RAY_GROUND_FILTER__NODE_HPP_

#include "autoware/universe_utils/system/time_keeper.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/transform_datatypes.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include "autoware/pointcloud_preprocessor/filter.hpp"
#include "gencolors.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/optional.hpp>

#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point>;

namespace autoware::ground_segmentation
{
class RayGroundFilterComponent : public autoware::pointcloud_preprocessor::Filter
{
  typedef pcl::PointXYZ PointType_;

  struct PointXYZRTColor
  {
    pcl::PointXYZ point;

    size_t ring;  // ring number if available

    float radius;  // cylindrical coords on XY Plane
    float theta;   // angle deg on XY plane

    size_t radial_div;  // index of the radial division to which this point belongs to

    size_t red;    // Red component  [0-255]
    size_t green;  // Green Component[0-255]
    size_t blue;   // Blue component [0-255]

    size_t original_index;  // index of this point in the source pointcloud
  };
  typedef std::vector<PointXYZRTColor> PointCloudXYZRTColor;

protected:
  void filter(
    const PointCloud2ConstPtr & input, const IndicesPtr & indices, PointCloud2 & output) override;

private:
  double general_max_slope_;            // degrees
  double local_max_slope_;              // degrees
  double initial_max_slope_;            // degrees
  double radial_divider_angle_;         // distance in rads between dividers
  double concentric_divider_distance_;  // distance in meters between concentric divisions
  double                                // minimum height threshold regardless the slope
    min_height_threshold_;              // useful for close points
  double
    reclass_distance_threshold_;  // distance between points at which re classification will occur

  size_t radial_dividers_num_;

  size_t grid_width_;
  size_t grid_height_;
  double grid_precision_;
  cv::Mat previous_occupancy_mat_;
  cv::Mat accumulated_occupancy_mat_;

  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;

  Polygon vehicle_footprint_;
  bool use_vehicle_footprint_;

  std::vector<cv::Scalar> colors_;
  const size_t color_num_ = 10;                          // different number of color to generate
  pcl::PointCloud<PointType_>::Ptr previous_cloud_ptr_;  // holds the previous groundless result of
                                                         // ground classification

  // time keeper related
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_;

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform succeeded
   * @retval false transform failed
   */

  /*!
   *
   * @param[in] in_cloud Input Point Cloud to be organized in radial segments
   * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
   * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each
   * radial segment
   * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the
   * points ordered
   */
  void ConvertXYZIToRTZColor(
    const pcl::PointCloud<PointType_>::Ptr in_cloud, PointCloudXYZRTColor & out_organized_points,
    std::vector<pcl::PointIndices> & out_radial_divided_indices,
    std::vector<PointCloudXYZRTColor> & out_radial_ordered_clouds);

  /*!
   * Classifies Points in the PointCloud as Ground and Not Ground
   * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance
   * from the origin
   * @param out_ground_indices Returns the indices of the points classified as ground in the
   * original PointCloud
   * @param out_no_ground_indices Returns the indices of the points classified as not ground in the
   * original PointCloud
   */
  void ClassifyPointCloud(
    std::vector<PointCloudXYZRTColor> & in_radial_ordered_clouds,
    pcl::PointIndices & out_ground_indices, pcl::PointIndices & out_no_ground_indices);

  /*!
   * Returns the resulting complementary PointCloud, one with the points kept and the other removed
   * as indicated in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_only_indices_cloud_ptr Resulting PointCloud with the indices kept
   * @param out_removed_indices_cloud_ptr Resulting PointCloud with the indices removed
   */
  void ExtractPointsIndices(
    const PointCloud2::ConstSharedPtr in_cloud_ptr, const pcl::PointIndices & in_indices,
    PointCloud2::SharedPtr ground_cloud_msg_ptr, PointCloud2::SharedPtr no_ground_cloud_msg_ptr);

  boost::optional<float> calcPointVehicleIntersection(const Point & point);

  void setVehicleFootprint(
    const double min_x, const double max_x, const double min_y, const double max_y);
  void initializePointCloud2(
    const PointCloud2::ConstSharedPtr & in_cloud_ptr,
    const PointCloud2::SharedPtr & out_cloud_msg_ptr);
  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & p);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit RayGroundFilterComponent(const rclcpp::NodeOptions & options);
};
}  // namespace autoware::ground_segmentation

#endif  // RAY_GROUND_FILTER__NODE_HPP_
