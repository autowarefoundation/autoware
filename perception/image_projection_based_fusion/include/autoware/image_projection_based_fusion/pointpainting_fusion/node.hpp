// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__NODE_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__NODE_HPP_

#include "autoware/image_projection_based_fusion/fusion_node.hpp"
#include "autoware/image_projection_based_fusion/pointpainting_fusion/pointpainting_trt.hpp"
#include "autoware/lidar_centerpoint/postprocess/non_maximum_suppression.hpp"

#include <autoware/image_projection_based_fusion/utils/geometry.hpp>
#include <autoware/image_projection_based_fusion/utils/utils.hpp>
#include <autoware/lidar_centerpoint/centerpoint_trt.hpp>
#include <autoware/lidar_centerpoint/detection_class_remapper.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{
using Label = autoware_perception_msgs::msg::ObjectClassification;

inline bool isInsideBbox(
  float proj_x, float proj_y, sensor_msgs::msg::RegionOfInterest roi, float zc)
{
  // z_c is scaling to normalize projection point
  return proj_x >= roi.x_offset * zc && proj_x <= (roi.x_offset + roi.width) * zc &&
         proj_y >= roi.y_offset * zc && proj_y <= (roi.y_offset + roi.height) * zc;
}

class PointPaintingFusionNode
: public FusionNode<sensor_msgs::msg::PointCloud2, DetectedObjects, DetectedObjectsWithFeature>
{
public:
  explicit PointPaintingFusionNode(const rclcpp::NodeOptions & options);

protected:
  void preprocess(sensor_msgs::msg::PointCloud2 & pointcloud_msg) override;

  void fuseOnSingleImage(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const std::size_t image_id,
    const DetectedObjectsWithFeature & input_roi_msg,
    const sensor_msgs::msg::CameraInfo & camera_info,
    sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg) override;

  void postprocess(sensor_msgs::msg::PointCloud2 & painted_pointcloud_msg) override;

  rclcpp::Publisher<DetectedObjects>::SharedPtr obj_pub_ptr_;

  std::vector<double> tan_h_;  // horizontal field of view

  int omp_num_threads_{1};
  float score_threshold_{0.0};
  std::vector<std::string> class_names_;
  std::map<std::string, float> class_index_;
  std::map<std::string, std::function<bool(int)>> isClassTable_;
  std::vector<double> pointcloud_range;
  bool has_variance_{false};
  bool has_twist_{false};

  autoware::lidar_centerpoint::NonMaximumSuppression iou_bev_nms_;
  autoware::lidar_centerpoint::DetectionClassRemapper detection_class_remapper_;

  std::unique_ptr<image_projection_based_fusion::PointPaintingTRT> detector_ptr_{nullptr};

  bool out_of_scope(const DetectedObjects & obj);
};
}  // namespace autoware::image_projection_based_fusion
#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__POINTPAINTING_FUSION__NODE_HPP_
