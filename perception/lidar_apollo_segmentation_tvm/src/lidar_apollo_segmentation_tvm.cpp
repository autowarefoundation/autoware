// Copyright 2020-2022 Arm Ltd., TierIV
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

#include <baidu_cnn/inference_engine_tvm_config.hpp>
#include <common/types.hpp>
#include <lidar_apollo_segmentation_tvm/feature_map.hpp>
#include <lidar_apollo_segmentation_tvm/lidar_apollo_segmentation_tvm.hpp>
#include <tvm_utility/pipeline.hpp>

#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using model_zoo::perception::lidar_obstacle_detection::baidu_cnn::onnx_bcnn::config;

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
ApolloLidarSegmentationPreProcessor::ApolloLidarSegmentationPreProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config, int32_t range,
  bool8_t use_intensity_feature, bool8_t use_constant_feature, float32_t min_height,
  float32_t max_height)
: input_channels(config.network_inputs[0].second[1]),
  input_width(config.network_inputs[0].second[2]),
  input_height(config.network_inputs[0].second[3]),
  datatype_bytes(config.tvm_dtype_bits / 8),
  feature_generator(std::make_shared<FeatureGenerator>(
    input_width, input_height, range, use_intensity_feature, use_constant_feature, min_height,
    max_height))
{
  // Allocate input variable
  std::vector<int64_t> shape_x{1, input_channels, input_width, input_height};
  TVMArrayContainer x{
    shape_x,
    config.tvm_dtype_code,
    config.tvm_dtype_bits,
    config.tvm_dtype_lanes,
    config.tvm_device_type,
    config.tvm_device_id};
  output = x;
}

TVMArrayContainerVector ApolloLidarSegmentationPreProcessor::schedule(
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr)
{
  // generate feature map
  std::shared_ptr<FeatureMapInterface> feature_map_ptr = feature_generator->generate(pc_ptr);

  if (feature_map_ptr->channels < input_channels) {
    throw std::runtime_error("schedule: incorrect feature configuration");
  }

  TVMArrayCopyFromBytes(
    output.getArray(), feature_map_ptr->map_data.data(),
    input_channels * input_height * input_width * datatype_bytes);

  return {output};
}

ApolloLidarSegmentationPostProcessor::ApolloLidarSegmentationPostProcessor(
  const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
  const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr, int32_t range,
  float32_t objectness_thresh, float32_t score_threshold, float32_t height_thresh,
  int32_t min_pts_num)
: output_channels(config.network_outputs[0].second[1]),
  output_width(config.network_outputs[0].second[2]),
  output_height(config.network_outputs[0].second[3]),
  datatype_bytes(config.tvm_dtype_bits / 8),
  objectness_thresh_(objectness_thresh),
  score_threshold_(score_threshold),
  height_thresh_(height_thresh),
  min_pts_num_(min_pts_num),
  pc_ptr_(pc_ptr),
  cluster2d_(std::make_shared<Cluster2D>(output_width, output_height, range))
{
}

std::shared_ptr<DetectedObjectsWithFeature> ApolloLidarSegmentationPostProcessor::schedule(
  const TVMArrayContainerVector & input)
{
  pcl::PointIndices valid_idx;
  valid_idx.indices.resize(pc_ptr_->size());
  std::iota(valid_idx.indices.begin(), valid_idx.indices.end(), 0);
  std::vector<float32_t> feature(output_channels * output_width * output_height, 0);
  TVMArrayCopyToBytes(
    input[0].getArray(), feature.data(),
    output_channels * output_width * output_height * datatype_bytes);
  cluster2d_->cluster(
    feature.data(), pc_ptr_, valid_idx, objectness_thresh_, true /*use all grids for clustering*/);
  auto object_array = cluster2d_->getObjects(score_threshold_, height_thresh_, min_pts_num_);

  return object_array;
}

ApolloLidarSegmentation::ApolloLidarSegmentation(
  int32_t range, float32_t score_threshold, bool8_t use_intensity_feature,
  bool8_t use_constant_feature, float32_t z_offset, float32_t min_height, float32_t max_height,
  float32_t objectness_thresh, int32_t min_pts_num, float32_t height_thresh)
: range_(range),
  score_threshold_(score_threshold),
  z_offset_(z_offset),
  objectness_thresh_(objectness_thresh),
  min_pts_num_(min_pts_num),
  height_thresh_(height_thresh),
  pcl_pointcloud_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
  PreP(std::make_shared<PrePT>(
    config, range, use_intensity_feature, use_constant_feature, min_height, max_height)),
  IE(std::make_shared<IET>(config, "lidar_apollo_segmentation_tvm")),
  PostP(std::make_shared<PostPT>(
    config, pcl_pointcloud_ptr_, range, objectness_thresh, score_threshold, height_thresh,
    min_pts_num)),
  pipeline(
    std::make_shared<tvm_utility::pipeline::Pipeline<PrePT, IET, PostPT>>(*PreP, *IE, *PostP))
{
}

void ApolloLidarSegmentation::transformCloud(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & transformed_cloud,
  float32_t z_offset)
{
  if (target_frame_ == input.header.frame_id && z_offset == 0) {
    transformed_cloud = input;
  } else {
    pcl::PointCloud<pcl::PointXYZI> in_cluster, transformed_cloud_cluster;
    pcl::fromROSMsg(input, in_cluster);

    // transform pointcloud to target_frame
    if (target_frame_ != input.header.frame_id) {
      geometry_msgs::msg::TransformStamped transform_stamped;
      builtin_interfaces::msg::Time time_stamp = input.header.stamp;
      const tf2::TimePoint time_point = tf2::TimePoint(
        std::chrono::seconds(time_stamp.sec) + std::chrono::nanoseconds(time_stamp.nanosec));
      transform_stamped =
        tf_buffer_->lookupTransform(target_frame_, input.header.frame_id, time_point);
      Eigen::Matrix4f affine_matrix =
        tf2::transformToEigen(transform_stamped.transform).matrix().cast<float32_t>();
      pcl::transformPointCloud(in_cluster, transformed_cloud_cluster, affine_matrix);
      transformed_cloud_cluster.header.frame_id = target_frame_;
    } else {
      transformed_cloud_cluster = in_cluster;
    }

    // move pointcloud z_offset in z axis
    if (z_offset != 0) {
      Eigen::Affine3f z_up_translation(Eigen::Translation3f(0, 0, z_offset));
      Eigen::Matrix4f z_up_transform = z_up_translation.matrix();
      pcl::transformPointCloud(
        transformed_cloud_cluster, transformed_cloud_cluster, z_up_transform);
    }

    pcl::toROSMsg(transformed_cloud_cluster, transformed_cloud);
  }
}

std::shared_ptr<const DetectedObjectsWithFeature> ApolloLidarSegmentation::detectDynamicObjects(
  const sensor_msgs::msg::PointCloud2 & input)
{
  // move up pointcloud z_offset in z axis
  sensor_msgs::msg::PointCloud2 transformed_cloud;
  ApolloLidarSegmentation::transformCloud(input, transformed_cloud, z_offset_);
  // convert from ros to pcl
  pcl::fromROSMsg(transformed_cloud, *pcl_pointcloud_ptr_);

  // inference pipeline
  auto output = pipeline->schedule(pcl_pointcloud_ptr_);
  for (auto & feature_object : output->feature_objects) {
    feature_object.feature.cluster.header = input.header;
  }
  output->header = input.header;

  // move down pointcloud z_offset in z axis
  for (std::size_t i = 0; i < output->feature_objects.size(); i++) {
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    transformCloud(output->feature_objects.at(i).feature.cluster, transformed_cloud, -z_offset_);
    output->feature_objects.at(i).feature.cluster = transformed_cloud;
  }

  return output;
}

const std::string & ApolloLidarSegmentation::network_name() const { return config.network_name; }

tvm_utility::Version ApolloLidarSegmentation::version_check() const
{
  return IE->version_check(model_version_from);
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
