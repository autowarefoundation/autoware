// Copyright 2023-2026 the Autoware Foundation
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
//

#ifndef OCCLUSION_PREDICTOR_HPP_
#define OCCLUSION_PREDICTOR_HPP_

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/pcl_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#if __has_include(<image_geometry/pinhole_camera_model.hpp>)
#include <image_geometry/pinhole_camera_model.hpp>  // for ROS 2 Jazzy or newer
#else
#include <image_geometry/pinhole_camera_model.h>  // for ROS 2 Humble or older
#endif

#include <lanelet2_core/Forward.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

struct Ray
{
  float azimuth;
  float elevation;
  float dist;
};

class CloudOcclusionPredictor
{
public:
  CloudOcclusionPredictor(
    rclcpp::Node * node_ptr, float max_valid_pt_distance, float azimuth_occlusion_resolution_deg,
    float elevation_occlusion_resolution_deg);

  void predict(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr & rois_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const tf2_ros::Buffer & tf_buffer,
    const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
    std::vector<int> & occlusion_ratios);

private:
  uint32_t predict(const pcl::PointXYZ & roi_top_left, const pcl::PointXYZ & roi_bottom_right);

  void filterCloud(
    const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const std::vector<pcl::PointXYZ> & roi_tls,
    const std::vector<pcl::PointXYZ> & roi_brs, pcl::PointCloud<pcl::PointXYZ> & cloud_out) const;

  static void sampleTrafficLightRoi(
    const pcl::PointXYZ & top_left, const pcl::PointXYZ & bottom_right,
    uint32_t horizontal_sample_num, uint32_t vertical_sample_num,
    pcl::PointCloud<pcl::PointXYZ> & cloud_out);

  static void calcRoiVector3D(
    const tier4_perception_msgs::msg::TrafficLightRoi & roi,
    const image_geometry::PinholeCameraModel & pinhole_model,
    const std::map<lanelet::Id, tf2::Vector3> & traffic_light_position_map,
    const tf2::Transform & tf_camera2map, pcl::PointXYZ & top_left, pcl::PointXYZ & bottom_right);

  std::map<int, std::map<int, std::vector<Ray> > > lidar_rays_;
  rclcpp::Node * node_ptr_;
  float max_valid_pt_distance_;
  float azimuth_occlusion_resolution_deg_;
  float elevation_occlusion_resolution_deg_;
};

}  // namespace autoware::traffic_light

#endif  // OCCLUSION_PREDICTOR_HPP_
