// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "autoware/shape_estimation/tensorrt_shape_estimator.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

#include <iostream>
#include <random>

namespace autoware::shape_estimation
{
TrtShapeEstimator::TrtShapeEstimator(
  const std::string & model_path, const std::string & precision,
  const tensorrt_common::BatchConfig & batch_config, const size_t max_workspace_size,
  const tensorrt_common::BuildConfig build_config)
{
  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(
    model_path, precision, nullptr, batch_config, max_workspace_size, build_config);

  trt_common_->setup();

  if (!trt_common_->isInitialized()) {
    std::cerr << "Failed to initialize TensorRT" << std::endl;
    return;
  }

  const auto pc_input_dims = trt_common_->getBindingDimensions(0);
  const auto pc_input_size = std::accumulate(
    pc_input_dims.d + 1, pc_input_dims.d + pc_input_dims.nbDims, 1, std::multiplies<int>());
  input_pc_d_ = cuda_utils::make_unique<float[]>(pc_input_size * batch_config[2]);
  batch_size_ = batch_config[2];
  const auto one_hot_input_dims = trt_common_->getBindingDimensions(1);
  const auto one_hot_input_size = std::accumulate(
    one_hot_input_dims.d + 1, one_hot_input_dims.d + one_hot_input_dims.nbDims, 1,
    std::multiplies<int>());
  input_one_hot_d_ = cuda_utils::make_unique<float[]>(one_hot_input_size * batch_config[2]);

  const auto stage1_center_out_dims = trt_common_->getBindingDimensions(2);
  out_s1center_elem_num_ = std::accumulate(
    stage1_center_out_dims.d + 1, stage1_center_out_dims.d + stage1_center_out_dims.nbDims, 1,
    std::multiplies<int>());
  out_s1center_elem_num_ = out_s1center_elem_num_ * batch_config[2];
  out_s1center_elem_num_per_batch_ = static_cast<size_t>(out_s1center_elem_num_ / batch_config[2]);
  out_s1center_prob_d_ = cuda_utils::make_unique<float[]>(out_s1center_elem_num_);
  out_s1center_prob_h_ =
    cuda_utils::make_unique_host<float[]>(out_s1center_elem_num_, cudaHostAllocPortable);

  const auto pred_out_dims = trt_common_->getBindingDimensions(3);
  out_pred_elem_num_ = std::accumulate(
    pred_out_dims.d + 1, pred_out_dims.d + pred_out_dims.nbDims, 1, std::multiplies<int>());
  out_pred_elem_num_ = out_pred_elem_num_ * batch_config[2];
  out_pred_elem_num_per_batch_ = static_cast<size_t>(out_pred_elem_num_ / batch_config[2]);
  out_pred_prob_d_ = cuda_utils::make_unique<float[]>(out_pred_elem_num_);
  out_pred_prob_h_ =
    cuda_utils::make_unique_host<float[]>(out_pred_elem_num_, cudaHostAllocPortable);

  g_type_mean_size_ = {{4.6344314, 1.9600292, 1.7375569},     {6.936331, 2.5178623, 2.8506238},
                       {11.194943, 2.9501154, 3.4918275},     {12.275775, 2.9231303, 3.87086},
                       {0.80057803, 0.5983815, 1.27450867},   {1.76282397, 0.59706367, 1.73698127},
                       {16.17150617, 2.53246914, 3.53079012}, {3.64300781, 1.54298177, 1.92320313}};
}

bool TrtShapeEstimator::inference(
  const DetectedObjectsWithFeature & input, DetectedObjectsWithFeature & output)
{
  if (!trt_common_->isInitialized()) {
    return false;
  }

  bool result = false;

  for (size_t i = 0; i < input.feature_objects.size(); i += batch_size_) {
    DetectedObjectsWithFeature input_batch;
    input_batch.header = input.header;

    for (size_t j = 0; j < batch_size_ && (i + j) < input.feature_objects.size(); ++j) {
      input_batch.feature_objects.push_back(input.feature_objects[i + j]);
    }

    preprocess(input_batch);
    DetectedObjectsWithFeature output_batch;
    result = feed_forward_and_decode(input_batch, output_batch);

    output.feature_objects.insert(
      output.feature_objects.end(), output_batch.feature_objects.begin(),
      output_batch.feature_objects.end());
  }

  return result;
}

void TrtShapeEstimator::preprocess(const DetectedObjectsWithFeature & input)
{
  auto input_dims_pc = trt_common_->getBindingDimensions(0);
  int batch_size = static_cast<int>(input.feature_objects.size());

  const auto input_chan = static_cast<float>(input_dims_pc.d[1]);
  const auto input_pc_size = static_cast<float>(input_dims_pc.d[2]);

  auto input_dims_one_hot = trt_common_->getBindingDimensions(1);
  const auto input_one_hot_size = static_cast<float>(input_dims_one_hot.d[1]);

  int volume_pc = batch_size * input_chan * input_pc_size;
  input_pc_h_.resize(volume_pc);

  int volume_one_hot = batch_size * input_one_hot_size;
  input_one_hot_h_.resize(volume_one_hot);

  // fill point cloud
  for (size_t i = 0; i < input.feature_objects.size(); i++) {
    const auto & feature_object = input.feature_objects[i];

    int point_size_of_cloud =
      feature_object.feature.cluster.width * feature_object.feature.cluster.height;

    if (point_size_of_cloud <= input_pc_size) {
      int offset = 0;
      for (sensor_msgs::PointCloud2ConstIterator<float> x_iter(feature_object.feature.cluster, "x"),
           y_iter(feature_object.feature.cluster, "y"), z_iter(feature_object.feature.cluster, "z");
           x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        input_pc_h_[i * input_chan * input_pc_size + 0 + offset] = -1 * (*y_iter);
        input_pc_h_[i * input_chan * input_pc_size + 512 + offset] = -1 * (*z_iter);
        input_pc_h_[i * input_chan * input_pc_size + 1024 + offset] = *x_iter;
        offset++;
      }

      int iter_count = static_cast<int>(input_pc_size) / point_size_of_cloud;
      int remainer_count = static_cast<int>(input_pc_size) % point_size_of_cloud;

      for (int j = 1; j < iter_count; j++) {
        for (int k = 0; k < point_size_of_cloud; k++) {
          input_pc_h_[i * input_chan * input_pc_size + 0 + j * point_size_of_cloud + k] =
            input_pc_h_[i * input_chan * input_pc_size + k];

          input_pc_h_[i * input_chan * input_pc_size + 512 + j * point_size_of_cloud + k] =
            input_pc_h_[i * input_chan * input_pc_size + 512 + k];

          input_pc_h_[i * input_chan * input_pc_size + 1024 + j * point_size_of_cloud + k] =
            input_pc_h_[i * input_chan * input_pc_size + 1024 + k];
        }
      }

      for (int j = 0; j < remainer_count; j++) {
        input_pc_h_[i * input_chan * input_pc_size + 0 + iter_count * point_size_of_cloud + j] =
          input_pc_h_[i * input_chan * input_pc_size + j];

        input_pc_h_[i * input_chan * input_pc_size + 512 + iter_count * point_size_of_cloud + j] =
          input_pc_h_[i * input_chan * input_pc_size + 512 + j];

        input_pc_h_[i * input_chan * input_pc_size + 1024 + iter_count * point_size_of_cloud + j] =
          input_pc_h_[i * input_chan * input_pc_size + 1024 + j];
      }

    } else {
      std::vector<float> sampled_points;
      std::vector<size_t> indices(
        feature_object.feature.cluster.width * feature_object.feature.cluster.height);
      std::iota(indices.begin(), indices.end(), 0);

      std::sample(
        indices.begin(), indices.end(), std::back_inserter(sampled_points), 512,
        std::mt19937{std::random_device{}()});

      // Create an iterator to read the points
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(feature_object.feature.cluster, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(feature_object.feature.cluster, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(feature_object.feature.cluster, "z");

      int offset = 0;
      for (const auto & index : sampled_points) {
        auto idx = index * feature_object.feature.cluster.point_step / sizeof(float);
        input_pc_h_[i * input_chan * input_pc_size + 0 + offset] = (-1 * iter_y[idx]);
        input_pc_h_[i * input_chan * input_pc_size + 512 + offset] = (-1 * iter_z[idx]);
        input_pc_h_[i * input_chan * input_pc_size + 1024 + offset] = iter_x[idx];
        offset++;
      }
    }

    const auto & label = feature_object.object.classification.front().label;

    // Initialize all elements to 0.0f
    input_one_hot_h_[i * 4 + 0] = 0.0f;
    input_one_hot_h_[i * 4 + 1] = 0.0f;
    input_one_hot_h_[i * 4 + 2] = 0.0f;
    input_one_hot_h_[i * 4 + 3] = 0.0f;

    if (label == Label::CAR) {
      input_one_hot_h_[i * 4 + 0] = 1.0f;
    } else if (label == Label::TRUCK) {
      input_one_hot_h_[i * 4 + 1] = 1.0f;
    } else if (label == Label::BUS) {
      input_one_hot_h_[i * 4 + 2] = 1.0f;
    } else if (label == Label::TRAILER) {
      input_one_hot_h_[i * 4 + 3] = 1.0f;
    }
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    input_pc_d_.get(), input_pc_h_.data(), input_pc_h_.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    input_one_hot_d_.get(), input_one_hot_h_.data(), input_one_hot_h_.size() * sizeof(float),
    cudaMemcpyHostToDevice));
}

bool TrtShapeEstimator::feed_forward_and_decode(
  const DetectedObjectsWithFeature & input, DetectedObjectsWithFeature & output)
{
  int batch_size = static_cast<int>(input.feature_objects.size());
  std::vector<void *> buffers = {
    input_pc_d_.get(), input_one_hot_d_.get(), out_s1center_prob_d_.get(), out_pred_prob_d_.get()};
  trt_common_->enqueueV2(buffers.data(), *stream_, nullptr);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_s1center_prob_h_.get(), out_s1center_prob_d_.get(), out_s1center_elem_num_ * sizeof(float),
    cudaMemcpyDeviceToHost, *stream_));

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    out_pred_prob_h_.get(), out_pred_prob_d_.get(), out_pred_elem_num_ * sizeof(float),
    cudaMemcpyDeviceToHost, *stream_));

  cudaStreamSynchronize(*stream_);

  float * out_stage1_net = out_s1center_prob_h_.get();
  float * out_pred_net = out_pred_prob_h_.get();

  output.feature_objects.resize(batch_size);
  for (int i = 0; i < batch_size; i++) {
    output.feature_objects.at(i) = input.feature_objects.at(i);

    autoware_perception_msgs::msg::Shape shape;
    geometry_msgs::msg::Pose pose;

    shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

    pose.position.x = out_stage1_net[i * out_s1center_elem_num_per_batch_ + 2] +
                      out_pred_net[i * out_pred_elem_num_per_batch_ + 2];
    pose.position.y =
      -(out_stage1_net[i * out_s1center_elem_num_per_batch_ + 0] +
        out_pred_net[i * out_pred_elem_num_per_batch_ + 0]);
    pose.position.z =
      -(out_stage1_net[i * out_s1center_elem_num_per_batch_ + 1] +
        out_pred_net[i * out_pred_elem_num_per_batch_ + 1]);

    int idx = 0;
    idx += 3;

    int NUM_HEADING_BIN = 12;
    int NUM_SIZE_CLUSTER = 8;

    int heading_idx = 0;
    float max = std::numeric_limits<float>::lowest();
    for (int j = idx; j < idx + NUM_HEADING_BIN; j++) {
      if (out_pred_net[i * out_pred_elem_num_per_batch_ + j] > max) {
        max = out_pred_net[i * out_pred_elem_num_per_batch_ + j];
        heading_idx = j - idx;
      }
    }

    idx += NUM_HEADING_BIN;

    float heading_residual =
      out_pred_net[i * out_pred_elem_num_per_batch_ + idx + heading_idx] * (M_PI / NUM_HEADING_BIN);

    idx += NUM_HEADING_BIN;

    float heading = class2angle(heading_idx, heading_residual, NUM_HEADING_BIN);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, -M_PI / 2 - heading);

    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();

    int size_idx = 0;
    max = std::numeric_limits<float>::lowest();
    for (int j = idx; j < idx + NUM_SIZE_CLUSTER; j++) {
      if (out_pred_net[i * out_pred_elem_num_per_batch_ + j] > max) {
        max = out_pred_net[i * out_pred_elem_num_per_batch_ + j];
        size_idx = j - idx;
      }
    }

    idx += NUM_SIZE_CLUSTER;
    float x_size = out_pred_net[i * out_pred_elem_num_per_batch_ + idx + size_idx * 3] *
                     g_type_mean_size_[size_idx][0] +
                   g_type_mean_size_[size_idx][0];

    float y_size = out_pred_net[i * out_pred_elem_num_per_batch_ + idx + size_idx * 3 + 1] *
                     g_type_mean_size_[size_idx][1] +
                   g_type_mean_size_[size_idx][1];
    float z_size = out_pred_net[i * out_pred_elem_num_per_batch_ + idx + size_idx * 3 + 2] *
                     g_type_mean_size_[size_idx][2] +
                   g_type_mean_size_[size_idx][2];

    shape.dimensions.x = x_size;
    shape.dimensions.y = y_size;
    shape.dimensions.z = z_size;

    output.feature_objects.at(i).object.shape = shape;
    output.feature_objects.at(i).object.kinematics.pose_with_covariance.pose = pose;
  }

  return true;
}

double TrtShapeEstimator::class2angle(int pred_cls, double residual, int num_class)
{
  double angle_per_class = 2.0 * M_PI / static_cast<double>(num_class);
  double angle_center = static_cast<double>(pred_cls) * angle_per_class;
  double angle = angle_center + residual;

  // Adjust angle to be within the range [-pi, pi]
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
};
}  // namespace autoware::shape_estimation
