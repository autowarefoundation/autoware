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
: FusionNode<PointCloud2, DetectedObjectWithFeature>("roi_pointcloud_fusion", options)
{
  fuse_unknown_only_ = declare_parameter<bool>("fuse_unknown_only");
  min_cluster_size_ = declare_parameter<int>("min_cluster_size");
  cluster_2d_tolerance_ = declare_parameter<double>("cluster_2d_tolerance");
  pub_objects_ptr_ =
    this->create_publisher<DetectedObjectsWithFeature>("output_clusters", rclcpp::QoS{1});
  cluster_debug_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/clusters", 1);
}

void RoiPointCloudFusionNode::preprocess(__attribute__((unused))
                                         sensor_msgs::msg::PointCloud2 & pointcloud_msg)
{
  return;
}

void RoiPointCloudFusionNode::postprocess(__attribute__((unused))
                                          sensor_msgs::msg::PointCloud2 & pointcloud_msg)
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
                                  autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
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
  Eigen::Matrix4d projection;
  projection << camera_info.p.at(0), camera_info.p.at(1), camera_info.p.at(2), camera_info.p.at(3),
    camera_info.p.at(4), camera_info.p.at(5), camera_info.p.at(6), camera_info.p.at(7),
    camera_info.p.at(8), camera_info.p.at(9), camera_info.p.at(10), camera_info.p.at(11);
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

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  tf2::doTransform(input_pointcloud_msg, transformed_cloud, transform_stamped);

  std::vector<PointCloud> clusters;
  clusters.resize(output_objs.size());

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x"),
       iter_y(transformed_cloud, "y"), iter_z(transformed_cloud, "z"),
       iter_orig_x(input_pointcloud_msg, "x"), iter_orig_y(input_pointcloud_msg, "y"),
       iter_orig_z(input_pointcloud_msg, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z, ++iter_orig_x, ++iter_orig_y, ++iter_orig_z) {
    if (*iter_z <= 0.0) {
      continue;
    }
    Eigen::Vector4d projected_point = projection * Eigen::Vector4d(*iter_x, *iter_y, *iter_z, 1.0);
    Eigen::Vector2d normalized_projected_point = Eigen::Vector2d(
      projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());

    for (std::size_t i = 0; i < output_objs.size(); ++i) {
      auto & feature_obj = output_objs.at(i);
      const auto & check_roi = feature_obj.feature.roi;
      auto & cluster = clusters.at(i);

      if (
        check_roi.x_offset <= normalized_projected_point.x() &&
        check_roi.y_offset <= normalized_projected_point.y() &&
        check_roi.x_offset + check_roi.width >= normalized_projected_point.x() &&
        check_roi.y_offset + check_roi.height >= normalized_projected_point.y()) {
        cluster.push_back(pcl::PointXYZ(*iter_orig_x, *iter_orig_y, *iter_orig_z));
      }
    }
  }

  // refine and update output_fused_objects_
  updateOutputFusedObjects(
    output_objs, clusters, input_pointcloud_msg.header, input_roi_msg.header, tf_buffer_,
    min_cluster_size_, cluster_2d_tolerance_, output_fused_objects_);
}

bool RoiPointCloudFusionNode::out_of_scope(__attribute__((unused))
                                           const DetectedObjectWithFeature & obj)
{
  return false;
}
}  // namespace image_projection_based_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(image_projection_based_fusion::RoiPointCloudFusionNode)
