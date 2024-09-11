// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__LIDAR_TRANSFUSION__TRANSFUSION_TRT_HPP_
#define AUTOWARE__LIDAR_TRANSFUSION__TRANSFUSION_TRT_HPP_

#include "autoware/lidar_transfusion/cuda_utils.hpp"
#include "autoware/lidar_transfusion/network/network_trt.hpp"
#include "autoware/lidar_transfusion/postprocess/postprocess_kernel.hpp"
#include "autoware/lidar_transfusion/preprocess/pointcloud_densification.hpp"
#include "autoware/lidar_transfusion/preprocess/preprocess_kernel.hpp"
#include "autoware/lidar_transfusion/preprocess/voxel_generator.hpp"
#include "autoware/lidar_transfusion/utils.hpp"
#include "autoware/lidar_transfusion/visibility_control.hpp"

#include <autoware/universe_utils/system/stop_watch.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::lidar_transfusion
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

class LIDAR_TRANSFUSION_PUBLIC TransfusionTRT
{
public:
  explicit TransfusionTRT(
    const NetworkParam & network_param, const DensificationParam & densification_param,
    const TransfusionConfig & config);
  virtual ~TransfusionTRT();

  bool detect(
    const sensor_msgs::msg::PointCloud2 & msg, const tf2_ros::Buffer & tf_buffer,
    std::vector<Box3D> & det_boxes3d, std::unordered_map<std::string, double> & proc_timing);

protected:
  void initPtr();

  bool preprocess(const sensor_msgs::msg::PointCloud2 & msg, const tf2_ros::Buffer & tf_buffer);

  bool inference();

  bool postprocess(std::vector<Box3D> & det_boxes3d);

  std::unique_ptr<NetworkTRT> network_trt_ptr_{nullptr};
  std::unique_ptr<VoxelGenerator> vg_ptr_{nullptr};
  std::unique_ptr<autoware::universe_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<PreprocessCuda> pre_ptr_{nullptr};
  std::unique_ptr<PostprocessCuda> post_ptr_{nullptr};
  cudaStream_t stream_{nullptr};

  TransfusionConfig config_;

  // input of pre-process

  unsigned int voxel_features_size_{0};
  unsigned int voxel_num_size_{0};
  unsigned int voxel_idxs_size_{0};
  unsigned int cls_size_{0};
  unsigned int box_size_{0};
  unsigned int dir_cls_size_{0};
  cuda::unique_ptr<float[]> points_d_{nullptr};
  cuda::unique_ptr<float[]> points_aux_d_{nullptr};
  cuda::unique_ptr<unsigned int> params_input_d_{nullptr};
  cuda::unique_ptr<unsigned int[]> shuffle_indices_d_{nullptr};
  cuda::unique_ptr<float[]> voxel_features_d_{nullptr};
  cuda::unique_ptr<unsigned int[]> voxel_num_d_{nullptr};
  cuda::unique_ptr<unsigned int[]> voxel_idxs_d_{nullptr};
  cuda::unique_ptr<float[]> cls_output_d_{nullptr};
  cuda::unique_ptr<float[]> box_output_d_{nullptr};
  cuda::unique_ptr<float[]> dir_cls_output_d_{nullptr};
};

}  // namespace autoware::lidar_transfusion

#endif  // AUTOWARE__LIDAR_TRANSFUSION__TRANSFUSION_TRT_HPP_
