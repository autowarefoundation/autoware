// Copyright 2023 TIER IV, Inc.
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

#include "image_projection_based_fusion/roi_pointcloud_fusion/node.hpp"

#include "image_projection_based_fusion/utils/geometry.hpp"
#include "image_projection_based_fusion/utils/utils.hpp"

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include "euclidean_cluster/utils.hpp"

namespace image_projection_based_fusion
{
RoiPointCloudFusionNode::RoiPointCloudFusionNode(const rclcpp::NodeOptions & options)
: FusionNode<PointCloud2, DetectedObjectWithFeature, DetectedObjectsWithFeature>(
    "roi_pointcloud_fusion", options)
{
  fuse_unknown_only_ = declare_parameter<bool>("fuse_unknown_only");
  min_cluster_size_ = declare_parameter<int>("min_cluster_size");
  max_cluster_size_ = declare_parameter<int>("max_cluster_size");
  cluster_2d_tolerance_ = declare_parameter<double>("cluster_2d_tolerance");
  pub_objects_ptr_ =
    this->create_publisher<DetectedObjectsWithFeature>("output_clusters", rclcpp::QoS{1});
  cluster_debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
}

void RoiPointCloudFusionNode::preprocess(
  __attribute__((unused)) sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  return;
}

void RoiPointCloudFusionNode::postprocess(
  __attribute__((unused)) sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  const auto objects_sub_count = pub_objects_ptr_->get_subscription_count() +
                                 pub_objects_ptr_->get_intra_process_subscription_count();
  if (objects_sub_count < 1) {
    return;
  }

  DetectedObjectsWithFeature output_msg;
  output_msg.header = pointcloud_msg.header;
  output_msg.feature_objects = output_fused_objects_;

  if (objects_sub_count > 0) {
    pub_objects_ptr_->publish(output_msg);
  }
  output_fused_objects_.clear();
  // publish debug cluster
  if (cluster_debug_pub_->get_subscription_count() > 0) {
    sensor_msgs::msg::PointCloud2 debug_cluster_msg;
    euclidean_cluster::convertObjectMsg2SensorMsg(output_msg, debug_cluster_msg);
    cluster_debug_pub_->publish(debug_cluster_msg);
  }
}
void RoiPointCloudFusionNode::fuseOnSingleImage(
  const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
  __attribute__((unused)) const std::size_t image_id,
  const DetectedObjectsWithFeature & input_roi_msg,
  const sensor_msgs::msg::CameraInfo & camera_info,
  __attribute__((unused)) sensor_msgs::msg::PointCloud2 & output_pointcloud_msg)
{
  if (input_pointcloud_msg.data.empty()) {
    return;
  }
  std::vector<DetectedObjectWithFeature> output_objs;
  // select ROIs for fusion
  for (const auto & feature_obj : input_roi_msg.feature_objects) {
    if (fuse_unknown_only_) {
      bool is_roi_label_unknown = feature_obj.object.classification.front().label ==
                                  autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
      if (is_roi_label_unknown) {
        output_objs.push_back(feature_obj);
      }
    } else {
      // TODO(badai-nguyen): selected class from a list
      output_objs.push_back(feature_obj);
    }
  }
  if (output_objs.empty()) {
    return;
  }

  // transform pointcloud to camera optical frame id
  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);

  geometry_msgs::msg::TransformStamped transform_stamped;
  {
    const auto transform_stamped_optional = getTransformStamped(
      tf_buffer_, input_roi_msg.header.frame_id, input_pointcloud_msg.header.frame_id,
      input_roi_msg.header.stamp);
    if (!transform_stamped_optional) {
      return;
    }
    transform_stamped = transform_stamped_optional.value();
  }
  int point_step = input_pointcloud_msg.point_step;
  int x_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "x")].offset;
  int y_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "y")].offset;
  int z_offset = input_pointcloud_msg.fields[pcl::getFieldIndex(input_pointcloud_msg, "z")].offset;

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(input_pointcloud_msg, transformed_cloud, transform_stamped);

  std::vector<sensor_msgs::msg::PointCloud2> clusters;
  std::vector<size_t> clusters_data_size;
  clusters.resize(output_objs.size());
  for (auto & cluster : clusters) {
    cluster.point_step = input_pointcloud_msg.point_step;
    cluster.height = input_pointcloud_msg.height;
    cluster.fields = input_pointcloud_msg.fields;
    cluster.data.resize(max_cluster_size_ * input_pointcloud_msg.point_step);
    clusters_data_size.push_back(0);
  }
  for (size_t offset = 0; offset < input_pointcloud_msg.data.size(); offset += point_step) {
    const float transformed_x =
      *reinterpret_cast<const float *>(&transformed_cloud.data[offset + x_offset]);
    const float transformed_y =
      *reinterpret_cast<const float *>(&transformed_cloud.data[offset + y_offset]);
    const float transformed_z =
      *reinterpret_cast<const float *>(&transformed_cloud.data[offset + z_offset]);
    if (transformed_z <= 0.0) {
      continue;
    }
    Eigen::Vector2d projected_point = calcRawImageProjectedPoint(
      pinhole_camera_model, cv::Point3d(transformed_x, transformed_y, transformed_z));
    for (std::size_t i = 0; i < output_objs.size(); ++i) {
      auto & feature_obj = output_objs.at(i);
      const auto & check_roi = feature_obj.feature.roi;
      auto & cluster = clusters.at(i);

      if (clusters_data_size.at(i) >= static_cast<size_t>(max_cluster_size_ * point_step)) {
        continue;
      }
      if (
        check_roi.x_offset <= projected_point.x() && check_roi.y_offset <= projected_point.y() &&
        check_roi.x_offset + check_roi.width >= projected_point.x() &&
        check_roi.y_offset + check_roi.height >= projected_point.y()) {
        std::memcpy(
          &cluster.data[clusters_data_size.at(i)], &input_pointcloud_msg.data[offset], point_step);
        clusters_data_size.at(i) += point_step;
      }
    }
  }

  // refine and update output_fused_objects_
  updateOutputFusedObjects(
    output_objs, clusters, clusters_data_size, input_pointcloud_msg, input_roi_msg.header,
    tf_buffer_, min_cluster_size_, max_cluster_size_, cluster_2d_tolerance_, output_fused_objects_);
}

bool RoiPointCloudFusionNode::out_of_scope(__attribute__((unused))
                                           const DetectedObjectWithFeature & obj)
{
  return false;
}
}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiPointCloudFusionNode)
