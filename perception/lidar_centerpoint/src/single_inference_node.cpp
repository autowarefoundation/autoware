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

#include "lidar_centerpoint/single_inference_node.hpp"

#include <lidar_centerpoint/centerpoint_config.hpp>
#include <lidar_centerpoint/preprocess/pointcloud_densification.hpp>
#include <lidar_centerpoint/ros_utils.hpp>
#include <lidar_centerpoint/utils.hpp>
#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/file_io.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <fstream>
#include <memory>
#include <sstream>

namespace centerpoint
{
SingleInferenceLidarCenterPointNode::SingleInferenceLidarCenterPointNode(
  const rclcpp::NodeOptions & node_options)
: Node("lidar_center_point", node_options), tf_buffer_(this->get_clock())
{
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("score_threshold", 0.35));
  const float circle_nms_dist_threshold =
    static_cast<float>(this->declare_parameter<double>("circle_nms_dist_threshold", 1.5));
  const auto yaw_norm_thresholds =
    this->declare_parameter<std::vector<double>>("yaw_norm_thresholds");
  const std::string densification_world_frame_id =
    this->declare_parameter("densification_world_frame_id", "map");
  const int densification_num_past_frames =
    this->declare_parameter("densification_num_past_frames", 1);
  const std::string trt_precision = this->declare_parameter("trt_precision", "fp16");
  const std::string encoder_onnx_path = this->declare_parameter<std::string>("encoder_onnx_path");
  const std::string encoder_engine_path =
    this->declare_parameter<std::string>("encoder_engine_path");
  const std::string head_onnx_path = this->declare_parameter<std::string>("head_onnx_path");
  const std::string head_engine_path = this->declare_parameter<std::string>("head_engine_path");
  class_names_ = this->declare_parameter<std::vector<std::string>>("class_names");
  has_twist_ = this->declare_parameter("has_twist", false);
  const std::size_t point_feature_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("point_feature_size"));
  const std::size_t max_voxel_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("max_voxel_size"));
  const auto point_cloud_range = this->declare_parameter<std::vector<double>>("point_cloud_range");
  const auto voxel_size = this->declare_parameter<std::vector<double>>("voxel_size");
  const std::size_t downsample_factor =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("downsample_factor"));
  const std::size_t encoder_in_feature_size =
    static_cast<std::size_t>(this->declare_parameter<std::int64_t>("encoder_in_feature_size"));
  const auto allow_remapping_by_area_matrix =
    this->declare_parameter<std::vector<int64_t>>("allow_remapping_by_area_matrix");
  const auto min_area_matrix = this->declare_parameter<std::vector<double>>("min_area_matrix");
  const auto max_area_matrix = this->declare_parameter<std::vector<double>>("max_area_matrix");
  const auto pcd_path = this->declare_parameter<std::string>("pcd_path");
  const auto detections_path = this->declare_parameter<std::string>("detections_path");

  detection_class_remapper_.setParameters(
    allow_remapping_by_area_matrix, min_area_matrix, max_area_matrix);

  NetworkParam encoder_param(encoder_onnx_path, encoder_engine_path, trt_precision);
  NetworkParam head_param(head_onnx_path, head_engine_path, trt_precision);
  DensificationParam densification_param(
    densification_world_frame_id, densification_num_past_frames);

  if (point_cloud_range.size() != 6) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("single_inference_lidar_centerpoint"),
      "The size of point_cloud_range != 6: use the default parameters.");
  }
  if (voxel_size.size() != 3) {
    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("single_inference_lidar_centerpoint"),
      "The size of voxel_size != 3: use the default parameters.");
  }
  CenterPointConfig config(
    class_names_.size(), point_feature_size, max_voxel_size, point_cloud_range, voxel_size,
    downsample_factor, encoder_in_feature_size, score_threshold, circle_nms_dist_threshold,
    yaw_norm_thresholds);
  detector_ptr_ =
    std::make_unique<CenterPointTRT>(encoder_param, head_param, densification_param, config);

  detect(pcd_path, detections_path);
  exit(0);
}

std::vector<Eigen::Vector3d> SingleInferenceLidarCenterPointNode::getVertices(
  const autoware_auto_perception_msgs::msg::Shape & shape, const Eigen::Affine3d & pose) const
{
  std::vector<Eigen::Vector3d> vertices;
  Eigen::Vector3d vertex;

  vertex.x() = -shape.dimensions.x / 2.0;
  vertex.y() = -shape.dimensions.y / 2.0;
  vertex.z() = shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = -shape.dimensions.x / 2.0;
  vertex.y() = shape.dimensions.y / 2.0;
  vertex.z() = shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = -shape.dimensions.x / 2.0;
  vertex.y() = shape.dimensions.y / 2.0;
  vertex.z() = -shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = shape.dimensions.x / 2.0;
  vertex.y() = shape.dimensions.y / 2.0;
  vertex.z() = shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = shape.dimensions.x / 2.0;
  vertex.y() = shape.dimensions.y / 2.0;
  vertex.z() = -shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = shape.dimensions.x / 2.0;
  vertex.y() = -shape.dimensions.y / 2.0;
  vertex.z() = shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = shape.dimensions.x / 2.0;
  vertex.y() = -shape.dimensions.y / 2.0;
  vertex.z() = -shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  vertex.x() = -shape.dimensions.x / 2.0;
  vertex.y() = -shape.dimensions.y / 2.0;
  vertex.z() = -shape.dimensions.z / 2.0;
  vertices.push_back(pose * vertex);

  return vertices;
}

void SingleInferenceLidarCenterPointNode::detect(
  const std::string & pcd_path, const std::string & detections_path)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::io::loadPCDFile(pcd_path, *pc_ptr);
  pcl::toROSMsg(*pc_ptr, msg);
  msg.header.frame_id = "lidar_frame";

  std::vector<Box3D> det_boxes3d;
  bool is_success = detector_ptr_->detect(msg, tf_buffer_, det_boxes3d);
  if (!is_success) {
    return;
  }

  autoware_auto_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = msg.header;
  for (const auto & box3d : det_boxes3d) {
    autoware_auto_perception_msgs::msg::DetectedObject obj;
    box3DToDetectedObject(box3d, class_names_, has_twist_, obj);
    output_msg.objects.emplace_back(obj);
  }

  detection_class_remapper_.mapClasses(output_msg);

  dumpDetectionsAsMesh(output_msg, detections_path);

  RCLCPP_INFO(
    rclcpp::get_logger("single_inference_lidar_centerpoint"),
    "The detection results were saved as meshes in %s", detections_path.c_str());
}

void SingleInferenceLidarCenterPointNode::dumpDetectionsAsMesh(
  const autoware_auto_perception_msgs::msg::DetectedObjects & objects_msg,
  const std::string & output_path) const
{
  std::ofstream ofs(output_path, std::ofstream::out);
  std::stringstream vertices_stream;
  std::stringstream faces_stream;
  int index = 0;
  int num_detections = static_cast<int>(objects_msg.objects.size());

  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;
  ofs << "comment created by lidar_centerpoint" << std::endl;
  ofs << "element vertex " << 8 * num_detections << std::endl;
  ofs << "property float x" << std::endl;
  ofs << "property float y" << std::endl;
  ofs << "property float z" << std::endl;
  ofs << "element face " << 12 * num_detections << std::endl;
  ofs << "property list uchar uint vertex_indices" << std::endl;
  ofs << "end_header" << std::endl;

  auto streamFace = [&faces_stream](int v1, int v2, int v3) {
    faces_stream << std::to_string(3) << " " << std::to_string(v1) << " " << std::to_string(v2)
                 << " " << std::to_string(v3) << std::endl;
  };

  for (const auto & object : objects_msg.objects) {
    Eigen::Affine3d pose_affine;
    tf2::fromMsg(object.kinematics.pose_with_covariance.pose, pose_affine);

    std::vector<Eigen::Vector3d> vertices = getVertices(object.shape, pose_affine);

    for (const auto & vertex : vertices) {
      vertices_stream << vertex.x() << " " << vertex.y() << " " << vertex.z() << std::endl;
    }

    streamFace(index + 1, index + 3, index + 4);
    streamFace(index + 3, index + 5, index + 6);
    streamFace(index + 0, index + 7, index + 5);
    streamFace(index + 7, index + 2, index + 4);
    streamFace(index + 5, index + 3, index + 1);
    streamFace(index + 7, index + 0, index + 2);
    streamFace(index + 2, index + 1, index + 4);
    streamFace(index + 4, index + 3, index + 6);
    streamFace(index + 5, index + 7, index + 6);
    streamFace(index + 6, index + 7, index + 4);
    streamFace(index + 0, index + 5, index + 1);
    index += 8;
  }

  ofs << vertices_stream.str();
  ofs << faces_stream.str();

  ofs.close();
}

}  // namespace centerpoint

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(centerpoint::SingleInferenceLidarCenterPointNode)
