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

#ifndef AUTOWARE__SHAPE_ESTIMATION__TENSORRT_SHAPE_ESTIMATOR_HPP_
#define AUTOWARE__SHAPE_ESTIMATION__TENSORRT_SHAPE_ESTIMATOR_HPP_

#include <cuda_utils/cuda_check_error.hpp>
#include <cuda_utils/cuda_unique_ptr.hpp>
#include <cuda_utils/stream_unique_ptr.hpp>
#include <pcl_ros/transforms.hpp>
#include <tensorrt_common/tensorrt_common.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::shape_estimation
{
using cuda_utils::CudaUniquePtr;
using cuda_utils::CudaUniquePtrHost;
using cuda_utils::makeCudaStream;
using cuda_utils::StreamUniquePtr;

using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;
using Label = autoware_perception_msgs::msg::ObjectClassification;

class TrtShapeEstimator
{
public:
  TrtShapeEstimator(
    const std::string & model_path, const std::string & precision,
    const tensorrt_common::BatchConfig & batch_config, const size_t max_workspace_size = (1 << 30),
    const tensorrt_common::BuildConfig build_config =
      tensorrt_common::BuildConfig("MinMax", -1, false, false, false, 0.0));

  ~TrtShapeEstimator() = default;

  bool inference(const DetectedObjectsWithFeature & input, DetectedObjectsWithFeature & output);

private:
  void preprocess(const DetectedObjectsWithFeature & input);

  bool feed_forward_and_decode(
    const DetectedObjectsWithFeature & input, DetectedObjectsWithFeature & output);

  void postprocess();

  static double class2angle(int pred_cls, double residual, int num_class);

  StreamUniquePtr stream_{makeCudaStream()};
  std::unique_ptr<tensorrt_common::TrtCommon> trt_common_;

  std::vector<float> input_pc_h_;
  CudaUniquePtr<float[]> input_pc_d_;

  std::vector<float> input_one_hot_h_;
  CudaUniquePtr<float[]> input_one_hot_d_;

  size_t out_s1center_elem_num_;
  size_t out_s1center_elem_num_per_batch_;
  CudaUniquePtr<float[]> out_s1center_prob_d_;
  CudaUniquePtrHost<float[]> out_s1center_prob_h_;

  size_t out_pred_elem_num_;
  size_t out_pred_elem_num_per_batch_;
  CudaUniquePtr<float[]> out_pred_prob_d_;
  CudaUniquePtrHost<float[]> out_pred_prob_h_;

  std::vector<std::vector<float>> g_type_mean_size_;
  size_t batch_size_;
};
}  // namespace autoware::shape_estimation
#endif  // AUTOWARE__SHAPE_ESTIMATION__TENSORRT_SHAPE_ESTIMATOR_HPP_
