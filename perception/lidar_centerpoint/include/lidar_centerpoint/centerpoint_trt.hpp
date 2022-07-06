// Copyright 2021 TIER IV, Inc.
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

#ifndef LIDAR_CENTERPOINT__CENTERPOINT_TRT_HPP_
#define LIDAR_CENTERPOINT__CENTERPOINT_TRT_HPP_

#include <lidar_centerpoint/cuda_utils.hpp>
#include <lidar_centerpoint/network/network_trt.hpp>
#include <lidar_centerpoint/postprocess/postprocess_kernel.hpp>
#include <lidar_centerpoint/preprocess/voxel_generator.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace centerpoint
{
class NetworkParam
{
public:
  NetworkParam(std::string onnx_path, std::string engine_path, std::string trt_precision)
  : onnx_path_(std::move(onnx_path)),
    engine_path_(std::move(engine_path)),
    trt_precision_(std::move(trt_precision))
  {
  }

  std::string onnx_path() const { return onnx_path_; }
  std::string engine_path() const { return engine_path_; }
  std::string trt_precision() const { return trt_precision_; }

private:
  std::string onnx_path_;
  std::string engine_path_;
  std::string trt_precision_;
};

class CenterPointTRT
{
public:
  explicit CenterPointTRT(
    const NetworkParam & encoder_param, const NetworkParam & head_param,
    const DensificationParam & densification_param, const CenterPointConfig & config);

  ~CenterPointTRT();

  bool detect(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
    std::vector<Box3D> & det_boxes3d);

protected:
  void initPtr();

  virtual bool preprocess(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer);

  void inference();

  void postProcess(std::vector<Box3D> & det_boxes3d);

  std::unique_ptr<VoxelGeneratorTemplate> vg_ptr_{nullptr};
  std::unique_ptr<VoxelEncoderTRT> encoder_trt_ptr_{nullptr};
  std::unique_ptr<HeadTRT> head_trt_ptr_{nullptr};
  std::unique_ptr<PostProcessCUDA> post_proc_ptr_{nullptr};
  cudaStream_t stream_{nullptr};

  bool verbose_{false};
  std::size_t class_size_{0};
  CenterPointConfig config_;
  std::size_t num_voxels_{0};
  std::size_t encoder_in_feature_size_{0};
  std::size_t spatial_features_size_{0};
  std::vector<float> voxels_;
  std::vector<int> coordinates_;
  std::vector<float> num_points_per_voxel_;
  cuda::unique_ptr<float[]> voxels_d_{nullptr};
  cuda::unique_ptr<int[]> coordinates_d_{nullptr};
  cuda::unique_ptr<float[]> num_points_per_voxel_d_{nullptr};
  cuda::unique_ptr<float[]> encoder_in_features_d_{nullptr};
  cuda::unique_ptr<float[]> pillar_features_d_{nullptr};
  cuda::unique_ptr<float[]> spatial_features_d_{nullptr};
  cuda::unique_ptr<float[]> head_out_heatmap_d_{nullptr};
  cuda::unique_ptr<float[]> head_out_offset_d_{nullptr};
  cuda::unique_ptr<float[]> head_out_z_d_{nullptr};
  cuda::unique_ptr<float[]> head_out_dim_d_{nullptr};
  cuda::unique_ptr<float[]> head_out_rot_d_{nullptr};
  cuda::unique_ptr<float[]> head_out_vel_d_{nullptr};
};

}  // namespace centerpoint

#endif  // LIDAR_CENTERPOINT__CENTERPOINT_TRT_HPP_
