// Copyright 2021-2022 AutoCore Ltd., TIER IV, Inc., Arm Ltd.
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

#ifndef LIDAR_CENTERPOINT_TVM__CENTERPOINT_TVM_HPP_
#define LIDAR_CENTERPOINT_TVM__CENTERPOINT_TVM_HPP_

#include <lidar_centerpoint_tvm/postprocess/generate_detected_boxes.hpp>
#include <lidar_centerpoint_tvm/preprocess/voxel_generator.hpp>
#include <lidar_centerpoint_tvm/visibility_control.hpp>
#include <tvm_utility/pipeline.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tvm_vendor/dlpack/dlpack.h>
#include <tvm_vendor/tvm/runtime/module.h>
#include <tvm_vendor/tvm/runtime/packed_func.h>
#include <tvm_vendor/tvm/runtime/registry.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware
{
namespace perception
{
namespace lidar_centerpoint_tvm
{
using tvm_utility::pipeline::TVMArrayContainer;
using tvm_utility::pipeline::TVMArrayContainerVector;

struct MixedInputs
{
  // The number of non-empty voxel
  std::size_t num_voxels;
  // The voxel features (point info in each voxel) or pillar features.
  std::vector<float> features;
  // The number of points in each voxel.
  std::vector<float> num_points_per_voxel;
  // The index from voxel number to its 3D position
  std::vector<int32_t> coords;
};

class LIDAR_CENTERPOINT_TVM_LOCAL VoxelEncoderPreProcessor
: public tvm_utility::pipeline::PreProcessor<MixedInputs>
{
public:
  /// \brief Constructor.
  /// \param[in] config The TVM configuration.
  /// \param[in] config_mod The centerpoint model configuration.
  explicit VoxelEncoderPreProcessor(
    const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
    const CenterPointConfig & config_mod);

  /// \brief Convert the voxel_features to encoder_in_features (a TVM array).
  /// \param[in] voxel_inputs The voxel features related input
  /// \return A TVM array containing the encoder_in_features.
  /// \throw std::runtime_error If the features are incorrectly configured.
  TVMArrayContainerVector schedule(const MixedInputs & voxel_inputs);

private:
  const int64_t max_voxel_size;
  const int64_t max_point_in_voxel_size;
  const int64_t encoder_in_feature_size;
  const int64_t datatype_bytes;
  const CenterPointConfig config_detail;
  std::vector<float> encoder_in_features;
  TVMArrayContainer output;
};

class LIDAR_CENTERPOINT_TVM_LOCAL VoxelEncoderPostProcessor
: public tvm_utility::pipeline::PostProcessor<std::shared_ptr<std::vector<float>>>
{
public:
  /// \brief Constructor.
  /// \param[in] config The TVM configuration.
  explicit VoxelEncoderPostProcessor(
    const tvm_utility::pipeline::InferenceEngineTVMConfig & config);

  /// \brief Copy the inference result.
  /// \param[in] input The result of the voxel_encoder inference engine.
  /// \return The inferred data.
  std::shared_ptr<std::vector<float>> schedule(const TVMArrayContainerVector & input);

private:
  const int64_t max_voxel_size;
  const int64_t encoder_out_feature_size;
  const int64_t datatype_bytes;
  std::shared_ptr<std::vector<float>> pillar_features;
};

class LIDAR_CENTERPOINT_TVM_LOCAL BackboneNeckHeadPreProcessor
: public tvm_utility::pipeline::PreProcessor<MixedInputs>
{
public:
  /// \brief Constructor.
  /// \param[in] config The TVM configuration.
  /// \param[in] config_mod The centerpoint model configuration.
  explicit BackboneNeckHeadPreProcessor(
    const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
    const CenterPointConfig & config_mod);

  /// \brief Convert the pillar_features to spatial_features.
  /// \param[in] pillar_inputs The pillar features related input
  /// \return A TVM array containing the spatial_features.
  /// \throw std::runtime_error If the features are incorrectly configured.
  TVMArrayContainerVector schedule(const MixedInputs & pillar_inputs);

private:
  const int64_t input_channels;
  const int64_t input_height;
  const int64_t input_width;
  const int64_t datatype_bytes;
  const CenterPointConfig config_detail;
  std::vector<float> spatial_features;
  TVMArrayContainer output;
};

class LIDAR_CENTERPOINT_TVM_LOCAL BackboneNeckHeadPostProcessor
: public tvm_utility::pipeline::PostProcessor<std::vector<Box3D>>
{
public:
  /// \brief Constructor.
  /// \param[in] config The TVM configuration.
  /// \param[in] config_mod The centerpoint model configuration.
  explicit BackboneNeckHeadPostProcessor(
    const tvm_utility::pipeline::InferenceEngineTVMConfig & config,
    const CenterPointConfig & config_mod);

  /// \brief Copy the inference result.
  /// \param[in] input The result of the inference engine.
  /// \return The inferred data.
  std::vector<Box3D> schedule(const TVMArrayContainerVector & input);

private:
  const int64_t datatype_bytes;
  const CenterPointConfig config_detail;
  std::vector<float> head_out_heatmap;
  std::vector<float> head_out_offset;
  std::vector<float> head_out_z;
  std::vector<float> head_out_dim;
  std::vector<float> head_out_rot;
  std::vector<float> head_out_vel;
};

class LIDAR_CENTERPOINT_TVM_PUBLIC CenterPointTVM
{
public:
  /// \brief Constructor
  /// \param[in] dense_param The densification parameter used to constructing vg_ptr.
  /// \param[in] config The CenterPoint model configuration.
  explicit CenterPointTVM(
    const DensificationParam & densification_param, const CenterPointConfig & config);

  ~CenterPointTVM();

  bool detect(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer,
    std::vector<Box3D> & det_boxes3d);

protected:
  void initPtr();

  virtual bool preprocess(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg, const tf2_ros::Buffer & tf_buffer);

  using VE_PrePT = VoxelEncoderPreProcessor;
  using BNH_PrePT = BackboneNeckHeadPreProcessor;
  using IET = tvm_utility::pipeline::InferenceEngineTVM;
  using VE_PostPT = VoxelEncoderPostProcessor;
  using BNH_PostPT = BackboneNeckHeadPostProcessor;

  tvm_utility::pipeline::InferenceEngineTVMConfig config_ve;
  tvm_utility::pipeline::InferenceEngineTVMConfig config_bnh;

  // Voxel Encoder Pipeline.
  std::shared_ptr<VE_PrePT> VE_PreP;
  std::shared_ptr<IET> VE_IE;
  std::shared_ptr<VE_PostPT> VE_PostP;
  std::shared_ptr<tvm_utility::pipeline::Pipeline<VE_PrePT, IET, VE_PostPT>> ve_pipeline;

  // Backbone Neck Head Pipeline.
  std::shared_ptr<BNH_PrePT> BNH_PreP;
  std::shared_ptr<IET> BNH_IE;
  std::shared_ptr<BNH_PostPT> BNH_PostP;
  std::shared_ptr<tvm_utility::pipeline::Pipeline<BNH_PrePT, IET, BNH_PostPT>> bnh_pipeline;

  // Variables
  std::unique_ptr<VoxelGeneratorTemplate> vg_ptr_{nullptr};

  CenterPointConfig config_;
  std::size_t num_voxels_{0};
  std::shared_ptr<std::vector<float>> voxels_;
  std::shared_ptr<std::vector<int32_t>> coordinates_;
  std::shared_ptr<std::vector<float>> num_points_per_voxel_;
};

}  // namespace lidar_centerpoint_tvm
}  // namespace perception
}  // namespace autoware

#endif  // LIDAR_CENTERPOINT_TVM__CENTERPOINT_TVM_HPP_
